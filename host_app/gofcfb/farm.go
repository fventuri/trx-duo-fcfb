// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// The farm: plan the union request, then stream-ingest the shared bin stream,
// reconstructing + decoding each channel per-channel with UTC window cutting (see
// stream_collector.go / stream_channel.go).
package fcfb

import (
	"encoding/binary"
	"fmt"
	"math"
	"net"
	"os"
	"sync"
	"time"
)

const rcvBuf = 128 << 20 // match the hardened Python ingest

func roundBin(fc float64) int { return int(math.Round(fc / binW)) }

// buildChans turns the config channels into per-channel occupied-band requests;
// the server pads its guard and unions overlapping bands into granted runs.
func buildChans(cfg Config) []Chan {
	out := make([]Chan, len(cfg.Channels))
	for i, c := range cfg.Channels {
		bw := c.BwHz
		if bw <= 0 {
			bw = 3000
		}
		out[i] = Chan{Flo: c.FcHz - bw/2, Fhi: c.FcHz + bw/2, AdcBits: c.Adc}
	}
	return out
}

var printMu sync.Mutex

func emit(s Spot) {
	printMu.Lock()
	fmt.Println(s.String())
	printMu.Unlock()
}

// verifyKernel queries the board's analysis params and checks the active synthesis
// kernel was fit for the same (R,T). A mismatch is fatal (the reconstruction would
// be silent garbage). Unknown params (old gateware/server) or an unparseable kernel
// filename downgrade to a warning -- we can't verify, but we don't block.
func verifyKernel(ip string) error {
	return verifyKernelAddr(fmt.Sprintf("%s:%d", ip, tcpPort))
}

// verifyKernelAddr is verifyKernel against an explicit host:port (used by tests).
func verifyKernelAddr(addr string) error {
	p, ok, err := queryParamsAddr(addr, 5*time.Second)
	if err != nil {
		// Couldn't run the query (e.g. connect failed); let dialBoard surface the
		// real board error instead of masking it here.
		fmt.Fprintf(os.Stderr, "kernel check: params query failed (%v); proceeding without verification\n", err)
		return nil
	}
	if !ok || p.ParamVer == paramVerUnknown {
		fmt.Fprintln(os.Stderr, "kernel check: board did not report analysis params (old server/gateware); cannot verify the kernel matches the board")
		return nil
	}
	if !kernelNameParsed {
		fmt.Fprintf(os.Stderr, "kernel check: kernel filename not in dual_R<R>_T<T>_K<K>.f64 form; skipping board-match check (board reports R=%d T=%d)\n", p.R, p.T)
		return nil
	}
	if p.R != kernelR || p.T != kernelT {
		return fmt.Errorf("kernel mismatch: kernel says R=%d T=%d but board reports R=%d T=%d -- wrong synthesis kernel for this bitstream", kernelR, kernelT, p.R, p.T)
	}
	fmt.Fprintf(os.Stderr, "kernel check OK: R=%d T=%d match the board (gateware %q, proto v%d, param_ver %d)\n",
		kernelR, kernelT, p.GatewareName, p.ProtoVer, p.ParamVer)
	return nil
}

func runLive(cfg Config) error {
	// Startup diagnostic: dump all board info (temp/voltages/model/fpga/hw_rev) up
	// front so a support log captures the board's identity + health.  Never fatal.
	LogBoardInfo(os.Stderr, cfg.BoardIP)
	if err := checkHostMTU(cfg.BoardIP, cfg.UDPPort, cfg.MTU); err != nil {
		return err
	}
	if err := verifyKernel(cfg.BoardIP); err != nil {
		return err
	}
	mon := startBoardMonitor(cfg.BoardIP, time.Duration(cfg.MonitorInterval)*time.Second)
	defer mon.Stop()
	chans := buildChans(cfg)
	us, ts, meta, err := dialBoard(cfg.BoardIP, chans, cfg.UDPPort, -1, cfg.Guard,
		rcvBuf, 10*time.Second)
	if err != nil {
		return err
	}
	defer us.Close()
	defer ts.Close()

	fmt.Fprintf(os.Stderr, "accepted: nruns=%d W_a=%d W_b=%d binw=%d rec=%dB bin_scale=%.0f\n",
		len(meta.Runs), meta.Wa, meta.Wb, meta.BinWidth, meta.Rec, meta.BinScale)
	fmt.Fprintf(os.Stderr, "runs: %v\n", meta.Runs)

	return runLiveStreaming(cfg, us, meta)
}

// runLiveStreaming is the low-memory live path: a single ingest goroutine fans each
// block out to per-channel StreamChannels (no bin ring, no whole-window copy).  The
// dispatcher runs decoders off the ingest thread and drops a window rather than
// block ingest if the workers fall behind.
func runLiveStreaming(cfg Config, us *net.UDPConn, meta Meta) error {
	col := newStreamCollectorForCfg(cfg, meta) // build channels first to size the queue
	if len(col.feeds) == 0 {
		return fmt.Errorf("streaming: no configured channel resolved to a granted run")
	}
	// Queue generously: every channel can finalise at the same boundary, and a
	// decode (~1-2 s) is far shorter than the window period, so the pool drains it.
	disp := newDispatcher(cfg, false, 4*len(col.feeds))
	col.setDispatch(disp.submit)
	if cfg.ReconWorkers > 1 {
		col.startParallel(cfg.ReconWorkers)
	}
	reconMode := "single-threaded"
	if col.parallel {
		reconMode = fmt.Sprintf("%d parallel groups", col.pgroups)
	}
	fmt.Fprintf(os.Stderr, "streaming reconstruction: %d channels, recon=%s, workers=%d, queue=%d (low-memory path)\n",
		len(col.feeds), reconMode, maxInt(1, cfg.Workers), 4*len(col.feeds))

	done := make(chan struct{})
	go func() { col.run(us, 5*time.Second); close(done) }()
	waitForSignal()
	col.Stop()
	<-done
	disp.wait()
	lastSeq, lost, reorder, resyncs := col.stats()
	fmt.Fprintf(os.Stderr, "streaming done: last_seq=%d lost=%d reorder=%d resyncs=%d dispatch_drops=%d\n",
		lastSeq, lost, reorder, resyncs, disp.dropped())
	return nil
}

// runReplay decodes a captured v3 .bin through the streaming pipeline: it reads the
// file, then feeds each channel's bins block-by-block into a StreamChannel.  The whole
// file is treated as one window (utc 000000).  (The bin buffer here is the whole
// Captured -- offline replay is not memory-constrained; true streaming ingest from the
// wire is the live path.)
func runReplay(cfg Config) error {
	cap, err := readV3File(cfg.Replay)
	if err != nil {
		return err
	}
	fmt.Fprintf(os.Stderr, "replay(streaming) %s: runs=%v nb=%d\n", cfg.Replay, cap.Runs, cap.Nb)
	disp := newDispatcher(cfg, true, len(cfg.Channels)) // block=true: lossless offline diff
	dur := float64(cap.Nb)/binRate + 1                  // one window covering the whole file

	type feeder struct {
		sc      *StreamChannel
		rowIdx  []int
		S       [][]complex128
		col     []complex128
		rowIdxR []int          // adc==3 only
		SR      [][]complex128 // adc==3 only (ADC B rows)
		colR    []complex128   // adc==3 only
	}
	var feeders []feeder
	for _, ch := range cfg.Channels {
		kc := roundBin(ch.FcHz)
		tune := ch.FcHz - float64(kc)*binW
		adc := ch.Adc
		if adc == 0 {
			adc = 1
		}
		dec, _ := cfg.decoder(ch.Decoder)
		if adc == 3 {
			rowA, kaSubA, SA, okA := channelRows(cap, kc, 1)
			rowB, kaSubB, SB, okB := channelRows(cap, kc, 2)
			if !okA || !okB {
				continue
			}
			sc := newStreamChannelDual(ch, dec, kc, tune, kaSubA, kaSubB, 0, disp.submit)
			sc.period, sc.capture = dur, dur
			feeders = append(feeders, feeder{sc: sc, rowIdx: rowA, S: SA, col: make([]complex128, len(rowA)),
				rowIdxR: rowB, SR: SB, colR: make([]complex128, len(rowB))})
			continue
		}
		rowIdx, kaSub, S, ok := channelRows(cap, kc, adc)
		if !ok {
			continue
		}
		sc := newStreamChannel(ch, dec, kc, tune, adc, kaSub, 0, disp.submit)
		sc.period, sc.capture = dur, dur
		feeders = append(feeders, feeder{sc: sc, rowIdx: rowIdx, S: S, col: make([]complex128, len(rowIdx))})
	}
	for blk := 0; blk < cap.Nb; blk++ {
		for _, f := range feeders {
			if f.sc.dual {
				for i, j := range f.rowIdx {
					f.col[i] = f.S[j][blk]
				}
				for i, j := range f.rowIdxR {
					f.colR[i] = f.SR[j][blk]
				}
				f.sc.pushBlockDual(int64(blk), f.col, f.colR)
				continue
			}
			for i, j := range f.rowIdx {
				f.col[i] = f.S[j][blk]
			}
			f.sc.pushBlock(int64(blk), f.col)
		}
	}
	for _, f := range feeders {
		f.sc.finish()
	}
	disp.wait()
	return nil
}

// readV3File parses a v3 stream .bin into a Captured (all blocks, bin_scale applied).
func readV3File(path string) (*Captured, error) {
	d, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	if len(d) < 56 || string(d[:8]) != "FCFBv1\x00\x00" {
		return nil, fmt.Errorf("%s: not a v3 stream file", path)
	}
	ver := binary.LittleEndian.Uint32(d[8:12])
	if ver < 3 {
		return nil, fmt.Errorf("%s: version %d not supported (need v3)", path, ver)
	}
	m := Meta{
		N:        int(binary.LittleEndian.Uint32(d[12:16])),
		Fs:       math.Float64frombits(binary.LittleEndian.Uint64(d[16:24])),
		AdcMask:  int(binary.LittleEndian.Uint32(d[24:28])),
		Wa:       int(binary.LittleEndian.Uint32(d[28:32])),
		Wb:       int(binary.LittleEndian.Uint32(d[32:36])),
		BinScale: math.Float64frombits(binary.LittleEndian.Uint64(d[40:48])),
		BinWidth: int(binary.LittleEndian.Uint32(d[48:52])),
	}
	nruns := int(binary.LittleEndian.Uint32(d[52:56]))
	off := 56
	for i := 0; i < nruns; i++ {
		m.Runs = append(m.Runs, Run{
			K0:  int(binary.LittleEndian.Uint32(d[off : off+4])),
			W:   int(binary.LittleEndian.Uint32(d[off+4 : off+8])),
			Adc: int(binary.LittleEndian.Uint32(d[off+8 : off+12])),
		})
		off += 12
	}
	m.Ka, m.Kb = runsToBins(m.Runs)
	m.Rec = recordSizeAB(m.Wa, m.Wb, m.BinWidth)
	body := d[off:]
	nb := len(body) / m.Rec
	bb := binBytes(m.BinWidth)
	runA := m.Wa * bb * 2
	cap := &Captured{Sa: alloc2(m.Wa, nb), Sb: alloc2(m.Wb, nb), Ka: m.Ka, Kb: m.Kb, Runs: m.Runs, Nb: nb}
	for rec := 0; rec < nb; rec++ {
		p := rec*m.Rec + 8
		for j := 0; j < m.Wa; j++ {
			iv := decodeSignedLE(body[p:p+bb], bb)
			qv := decodeSignedLE(body[p+bb:p+2*bb], bb)
			cap.Sa[j][rec] = complex(iv*m.BinScale, qv*m.BinScale)
			p += 2 * bb
		}
		p = rec*m.Rec + 8 + runA
		for j := 0; j < m.Wb; j++ {
			iv := decodeSignedLE(body[p:p+bb], bb)
			qv := decodeSignedLE(body[p+bb:p+2*bb], bb)
			cap.Sb[j][rec] = complex(iv*m.BinScale, qv*m.BinScale)
			p += 2 * bb
		}
	}
	return cap, nil
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}
