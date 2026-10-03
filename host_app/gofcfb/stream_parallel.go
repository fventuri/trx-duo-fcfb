// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Optional multi-core reconstruction for the streaming ingest path.
//
// The default farm reconstructs every channel inline on the single ingest
// goroutine (feedAll in stream_collector.go), which caps out at ~110 channels of
// synth+resample per core.  When recon_workers > 1, the ingest goroutine keeps
// doing ONLY the sequential seq accounting (t0 anchor, forward-gap zero-fill,
// reorder drop, big-jump resync) and hands the decoded bin-columns to G worker
// goroutines, each owning a fixed subset of the channels and running their
// pushBlock (NCO + scatter FIR + resampler + window cut + dispatch) on its own
// core.  See NOTE_GOFARM_PARALLEL_RECONSTRUCTION_20260927.md.
//
// Bit-exactness: each channel is independent and deterministic given its own
// ordered stream of setT0/reset/pushBlock calls.  Splitting channels across
// goroutines does not reorder any single channel's calls, so every channel
// produces byte-identical audio (hence identical spots) to the serial path.  The
// only shared, order-sensitive state -- the u64 seq accounting -- stays on the
// ingest goroutine; it emits a clean ordered event stream that the workers replay.
//
// Hand-off is BATCHED: blocks arrive at 40 kHz (25 us each), far faster than a
// goroutine wakeup, so per-block fan-out would be dominated by sync overhead.
// The ingest goroutine fills a batch of synthBatchBlocks blocks, then releases the
// batch (one shared, read-only copy) to all G groups at once; each group extracts
// its own channels' bins from it.  A control event (anchor / resync) flushes the
// current batch first so ordering versus the blocks is exact.  Batches come from a
// bounded pool: when all are in flight the ingest goroutine back-pressures (the
// same failure mode as a full dispatch queue, but with headroom it never triggers).
package fcfb

import (
	"sync"
	"sync/atomic"
)

const (
	// synthBatchBlocks: blocks accumulated before the batch is released to the
	// groups.  ~3.2 ms of audio at 40 kHz -- amortises the goroutine hand-off ~100x
	// while staying negligible against the 7.5-120 s decode windows.
	synthBatchBlocks = 128
	// synthPoolDepth: batches in flight before ingest back-pressures.  8 x 128
	// blocks ~= 25 ms buffered; the pool also bounds RAM (depth x blocks x (Wa+Wb)).
	synthPoolDepth = 8
)

// batchCtrl is a per-batch control event applied to the group's channels BEFORE
// the batch's blocks (mirrors setAllT0 / reset+setT0 in the serial path).
type batchCtrl uint8

const (
	ctrlNone   batchCtrl = iota
	ctrlSetT0            // anchor: setT0 on all channels
	ctrlResync           // big-jump: reset + setT0 on all channels
)

// synthBatch is one hand-off unit: an optional head control event followed by n
// decoded bin-columns.  cols[k] is [Wa+Wb] with ADC A = cols[k][:wa] and ADC B =
// cols[k][wa:].  It is shared read-only by all G groups; refc counts groups still
// to consume it, and the last one returns it to the pool.
type synthBatch struct {
	ctrl batchCtrl
	t0   float64
	n    int
	blk  []int64        // len synthBatchBlocks; blk[:n] valid
	cols [][]complex128 // len synthBatchBlocks; each [wa+wb]
	refc int32          // groups remaining to consume (set at send)
}

// batchPool is a fixed set of preallocated batches recycled between the ingest
// goroutine (get) and the groups (put).  free is buffered to the pool depth so put
// never blocks (at most depth-1 batches are ever idle at once).
type batchPool struct {
	free chan *synthBatch
}

func newBatchPool(depth, wa, wb int) *batchPool {
	p := &batchPool{free: make(chan *synthBatch, depth)}
	for i := 0; i < depth; i++ {
		b := &synthBatch{
			blk:  make([]int64, synthBatchBlocks),
			cols: make([][]complex128, synthBatchBlocks),
		}
		for k := range b.cols {
			b.cols[k] = make([]complex128, wa+wb)
		}
		p.free <- b
	}
	return p
}

// get returns a reset batch, blocking (back-pressure) until one is free.
func (p *batchPool) get() *synthBatch {
	b := <-p.free
	b.n, b.ctrl = 0, ctrlNone
	return b
}

func (p *batchPool) put(b *synthBatch) { p.free <- b }

// synthGroup owns a disjoint subset of the collector's channels and reconstructs
// them from the shared batches on its own goroutine.
type synthGroup struct {
	feeds  []*feedCh // subset of the collector's feeds (disjoint scratch buffers)
	in     chan *synthBatch
	wa, wb int
	pool   *batchPool
}

// run consumes batches until in is closed, replaying each channel's ordered
// setT0/reset/pushBlock calls exactly as the serial feedAll would.
func (g *synthGroup) run(wg *sync.WaitGroup) {
	defer wg.Done()
	for b := range g.in {
		switch b.ctrl {
		case ctrlSetT0:
			for _, f := range g.feeds {
				f.sc.setT0(b.t0)
			}
		case ctrlResync:
			for _, f := range g.feeds {
				f.sc.reset()
				f.sc.setT0(b.t0)
			}
		}
		for k := 0; k < b.n; k++ {
			blk := b.blk[k]
			colA := b.cols[k][:g.wa]
			colB := b.cols[k][g.wa:]
			for _, f := range g.feeds {
				if f.adc == 3 {
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
		if atomic.AddInt32(&b.refc, -1) == 0 {
			g.pool.put(b)
		}
	}
}

// startParallel switches the collector into multi-core reconstruction with
// groups worker goroutines (capped at the channel count).  The ingest goroutine
// keeps onBlock's seq accounting; emitSetT0/emitResync/emitBlock now route to the
// batch pipeline instead of the inline feedAll.  Must be called after setDispatch
// and before run().
func (c *StreamCollector) startParallel(groups int) {
	if groups > len(c.feeds) {
		groups = len(c.feeds)
	}
	if groups <= 1 {
		return // nothing to parallelise; stay on the inline path
	}
	c.parallel = true
	c.pgroups = groups
	c.pool = newBatchPool(synthPoolDepth, c.meta.Wa, c.meta.Wb)
	c.groups = make([]*synthGroup, groups)
	for g := 0; g < groups; g++ {
		c.groups[g] = &synthGroup{
			in:   make(chan *synthBatch, synthPoolDepth),
			wa:   c.meta.Wa,
			wb:   c.meta.Wb,
			pool: c.pool,
		}
	}
	// Channel-count-balanced round-robin (the note's recommended first cut).  Groups
	// reference the collector's feeds directly; each feed lands in exactly one group,
	// so their scratch col buffers are never shared across goroutines.
	for i := range c.feeds {
		g := c.groups[i%groups]
		g.feeds = append(g.feeds, &c.feeds[i])
	}
	c.wg.Add(groups)
	for _, g := range c.groups {
		go g.run(&c.wg)
	}
	c.cur = c.pool.get()
}

// appendBlock copies one decoded block's columns into the current batch, releasing
// the batch to the groups when it fills.  colB may be nil / empty when Wb == 0.
func (c *StreamCollector) appendBlock(blk int64, colA, colB []complex128) {
	b := c.cur
	b.blk[b.n] = blk
	dst := b.cols[b.n]
	copy(dst[:c.meta.Wa], colA)
	if c.meta.Wb > 0 {
		copy(dst[c.meta.Wa:], colB)
	}
	b.n++
	if b.n == synthBatchBlocks {
		c.sendCur()
	}
}

// openCtrl marks a control event (anchor / resync) at the head of a batch.  Any
// buffered blocks are flushed first so the event applies strictly after them and
// strictly before the blocks that follow (exactly the serial ordering).
func (c *StreamCollector) openCtrl(ctrl batchCtrl, t0 float64) {
	if c.cur.n > 0 {
		c.sendCur()
	}
	c.cur.ctrl = ctrl
	c.cur.t0 = t0
}

// sendCur fans the current batch out to every group (shared read-only) and takes a
// fresh batch from the pool (back-pressuring if all are in flight).
func (c *StreamCollector) sendCur() {
	b := c.cur
	atomic.StoreInt32(&b.refc, int32(c.pgroups))
	for _, g := range c.groups {
		g.in <- b
	}
	c.cur = c.pool.get()
}

// flushParallel releases a partial batch (window boundary is handled per-channel,
// but this bounds latency on a stream lull and on shutdown).
func (c *StreamCollector) flushParallel() {
	if c.parallel && c.cur != nil && c.cur.n > 0 {
		c.sendCur()
	}
}

// stopParallel flushes the last partial batch, closes the group queues, and waits
// for the workers to drain.  After it returns, no goroutine touches the channels,
// so finishAll can flush the open windows sequentially.
func (c *StreamCollector) stopParallel() {
	if !c.parallel {
		return
	}
	c.flushParallel()
	for _, g := range c.groups {
		close(g.in)
	}
	c.wg.Wait()
}
