// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Streaming live ingest: the low-memory replacement for the batch Collector ring.
// A single goroutine reads the UDP stream and, per block, fans the block out to
// every configured StreamChannel (which synthesises + windows incrementally).  No
// 140 s bin ring and no whole-window copy -- only each channel's one active-window
// 12 kHz audio buffer lives at a time (~hundreds of MB total vs ~19 GB batch).
//
// The board's per-record u64 seq is the ABSOLUTE block index.  We anchor the
// block<->UTC map once (t0 = wall_now - seq/binRate on the first block), then:
//   - a small forward gap (UDP loss) is zero-filled by StreamChannel.pushBlock so
//     the audio timeline stays UTC-locked (a dropout, not a shift);
//   - a backward/duplicate seq is ignored;
//   - a big jump (>= bigJump blocks, e.g. a board reset / sustained outage) re-
//     anchors t0 and resets the channels rather than zero-filling millions of cols.
package fcfb

import (
	"fmt"
	"net"
	"os"
	"sync"
	"time"
)

// bigJump: a seq gap this large is treated as a stream discontinuity (re-anchor),
// not a fillable dropout.  ~26 s of blocks at 40 kHz, matching ingest.py's "big".
const bigJump = 1 << 20

type feedCh struct {
	sc      *StreamChannel
	adc     int
	rowIdx  []int        // indices into the ADC's bin list (ka for adc 1/3, kb for adc 2)
	col     []complex128 // reused per-block extraction buffer
	rowIdxR []int        // adc==3 only: indices into ADC B's bin list (kb)
	colR    []complex128 // adc==3 only
}

type StreamCollector struct {
	meta  Meta
	feeds []feedCh
	clock func() float64

	haveT0  bool
	t0      float64
	lastSeq uint64
	lost    int64
	resyncs int
	reorder int64

	// parallel reconstruction (recon_workers > 1): the ingest goroutine keeps the
	// seq accounting here and batches blocks out to c.groups (see stream_parallel.go).
	// When parallel is false, onBlock reconstructs inline via feedAll (the default).
	parallel bool
	pgroups  int
	groups   []*synthGroup
	pool     *batchPool
	cur      *synthBatch
	wg       sync.WaitGroup

	stop chan struct{}
}

// newStreamCollector builds one StreamChannel per configured channel that resolves
// to a granted run, wiring each to the dispatch callback.
func newStreamCollector(cfg Config, meta Meta, dispatch func(audioL, audioR, qA, qB []float64, utc string, ch Channel, d Decoder)) *StreamCollector {
	sc := &StreamCollector{meta: meta, clock: nowSec, stop: make(chan struct{})}
	iq := cfg.WavFormat == "iq"
	for _, ch := range cfg.Channels {
		kc := roundBin(ch.FcHz)
		tune := ch.FcHz - float64(kc)*binW
		adc := ch.Adc
		if adc == 0 {
			adc = 1
		}
		dec, _ := cfg.decoder(ch.Decoder)
		if adc == 3 {
			// Diversity: resolve the bin on BOTH ADCs (requested with AdcBits=3).
			rowA, kaSubA, okA := resolveChannelRows(meta.Runs, meta.Ka, meta.Kb, kc, 1)
			rowB, kaSubB, okB := resolveChannelRows(meta.Runs, meta.Ka, meta.Kb, kc, 2)
			if !okA || !okB {
				fmt.Fprintf(os.Stderr, "streaming: dual channel %s (fc=%.4f MHz, adc=3) not granted on both ADCs (A=%v B=%v) -- skipped\n",
					ch.name(), ch.FcHz/1e6, okA, okB)
				continue
			}
			st := newStreamChannelDual(ch, dec, kc, tune, kaSubA, kaSubB, 0, dispatch)
			st.iq = iq
			sc.feeds = append(sc.feeds, feedCh{sc: st, adc: 3,
				rowIdx: rowA, col: make([]complex128, len(rowA)),
				rowIdxR: rowB, colR: make([]complex128, len(rowB))})
			continue
		}
		rowIdx, kaSub, ok := resolveChannelRows(meta.Runs, meta.Ka, meta.Kb, kc, adc)
		if !ok {
			fmt.Fprintf(os.Stderr, "streaming: channel %s (fc=%.4f MHz, adc=%d) is not in any granted run -- skipped\n",
				ch.name(), ch.FcHz/1e6, adc)
			continue
		}
		st := newStreamChannel(ch, dec, kc, tune, adc, kaSub, 0, dispatch)
		st.iq = iq
		sc.feeds = append(sc.feeds, feedCh{sc: st, adc: adc, rowIdx: rowIdx, col: make([]complex128, len(rowIdx))})
	}
	return sc
}

// newStreamCollectorForCfg builds the channels with no dispatch yet, so the caller
// can size the dispatch queue from the resolved channel count then wire it via
// setDispatch.
func newStreamCollectorForCfg(cfg Config, meta Meta) *StreamCollector {
	return newStreamCollector(cfg, meta, nil)
}

// setDispatch wires the dispatch callback into every channel (after construction).
func (c *StreamCollector) setDispatch(fn func(audioL, audioR, qA, qB []float64, utc string, ch Channel, d Decoder)) {
	for _, f := range c.feeds {
		f.sc.dispatch = fn
	}
}

func (c *StreamCollector) setAllT0(t0 float64) {
	for _, f := range c.feeds {
		f.sc.setT0(t0)
	}
}

// onBlock routes one decoded block (absolute index seq) to every channel, handling
// the block-clock anchor, forward gaps, reorders, and big-jump re-anchors.  The
// seq accounting is always sequential (on the ingest goroutine); the emit* calls
// reconstruct inline (default) or hand the block to the parallel groups.
func (c *StreamCollector) onBlock(seq uint64, colA, colB []complex128, now float64) {
	if !c.haveT0 {
		c.t0 = now - float64(seq)/binRate
		c.haveT0 = true
		c.lastSeq = seq
		c.emitSetT0(c.t0)
		c.emitBlock(seq, colA, colB)
		return
	}
	d := int64(seq) - int64(c.lastSeq)
	if d <= 0 {
		c.reorder++
		return // duplicate or reordered older block
	}
	if d >= bigJump {
		// Discontinuity: re-anchor and drop any in-flight windows (garbage).
		c.t0 = now - float64(seq)/binRate
		c.resyncs++
		c.emitResync(c.t0)
		c.lastSeq = seq
		c.emitBlock(seq, colA, colB)
		return
	}
	if d > 1 {
		c.lost += d - 1 // small gap: pushBlock zero-fills it
	}
	c.lastSeq = seq
	c.emitBlock(seq, colA, colB)
}

// emitSetT0 anchors the block<->UTC map on every channel (inline) or as a batch
// head control event (parallel).
func (c *StreamCollector) emitSetT0(t0 float64) {
	if c.parallel {
		c.openCtrl(ctrlSetT0, t0)
		return
	}
	c.setAllT0(t0)
}

// emitResync drops any in-flight windows and re-anchors (inline) or as a batch
// head control event (parallel).
func (c *StreamCollector) emitResync(t0 float64) {
	if c.parallel {
		c.openCtrl(ctrlResync, t0)
		return
	}
	for _, f := range c.feeds {
		f.sc.reset()
		f.sc.setT0(t0)
	}
}

// emitBlock reconstructs one block across every channel (inline) or appends it to
// the current batch for the groups (parallel).
func (c *StreamCollector) emitBlock(seq uint64, colA, colB []complex128) {
	if c.parallel {
		c.appendBlock(int64(seq), colA, colB)
		return
	}
	c.feedAll(seq, colA, colB)
}

func (c *StreamCollector) feedAll(seq uint64, colA, colB []complex128) {
	blk := int64(seq)
	for _, f := range c.feeds {
		if f.adc == 3 {
			// Same block feeds both antennas -> ADC A and ADC B stay time-locked.
			for i, j := range f.rowIdx {
				f.col[i] = colA[j]
			}
			for i, j := range f.rowIdxR {
				f.colR[i] = colB[j]
			}
			f.sc.pushBlockDual(blk, f.col, f.colR)
			continue
		}
		src := colA
		if f.adc == 2 {
			src = colB
		}
		for i, j := range f.rowIdx {
			f.col[i] = src[j]
		}
		f.sc.pushBlock(blk, f.col)
	}
}

// run ingests the UDP stream until Stop, locking onto the first run generation seen
// (ignoring stale pre-retune datagrams), exactly like the batch collector.
func (c *StreamCollector) run(us *net.UDPConn, timeout time.Duration) {
	buf := make([]byte, 1<<16)
	runGen := -1
	for {
		select {
		case <-c.stop:
			c.finishAll()
			return
		default:
		}
		_ = us.SetReadDeadline(time.Now().Add(timeout))
		n, _, err := us.ReadFromUDP(buf)
		if err != nil {
			if ne, ok := err.(net.Error); ok && ne.Timeout() {
				c.flushParallel() // no data for a while: don't strand a partial batch
				continue
			}
			c.finishAll()
			return
		}
		gen, _, decode, ok := parseDatagram(buf[:n], c.meta)
		if !ok {
			continue
		}
		if runGen < 0 {
			runGen = gen
		}
		if gen != runGen {
			continue
		}
		now := c.clock()
		decode(func(_ int, seq uint64, colA, colB []complex128) {
			c.onBlock(seq, colA, colB, now)
		})
	}
}

func (c *StreamCollector) finishAll() {
	c.stopParallel() // drain + join the worker goroutines (no-op when serial)
	for _, f := range c.feeds {
		f.sc.finish()
	}
}

func (c *StreamCollector) Stop() { close(c.stop) }

// stats returns transport accounting for the run summary.
func (c *StreamCollector) stats() (lastSeq uint64, lost int64, reorder int64, resyncs int) {
	return c.lastSeq, c.lost, c.reorder, c.resyncs
}
