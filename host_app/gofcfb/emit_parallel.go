// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Optional multi-core reconstruction for the fcfbhpsdr P2 emitter.
//
// The emitter reconstructs every enabled DDC on a SINGLE worker goroutine
// (Emitter.worker), so per-DDC synth+resample is serialized on one core.  When
// recon_workers > 1 the dispatcher (worker) keeps doing ONLY the single-goroutine
// control work -- snapshot, settle/rebuild, BoardStream.recv -- and hands each
// decoded datagram's bin-columns to G worker goroutines, each owning a fixed,
// disjoint subset of the pipes and running their processPipe (idx-gather + phased
// synth + resample + frame + send) on its own core.
//
// Bit-exactness: each pipe is an independent, deterministic reconstruction chain
// over its own mutable state (synth ring + block counter + phase tables, resampler
// buffers, framing remainder + seq) and a UDP socket unique to (radio, ddc).  The
// only inputs are the datagram's read-only colsA/colsB and radio.clientEP (atomic).
// Splitting pipes across goroutines reorders work ACROSS pipes, never WITHIN a pipe,
// so every pipe's I/Q is byte-for-byte identical to the serial path.  This mirrors
// the farm's proven recon_workers invariant (stream_parallel.go).
//
// Hand-off is per-datagram fork-join (not the farm's per-block batching): recv
// returns ~40-60 blocks (~1.4 ms) per datagram, already ~50x coarser than a block,
// so a plain G channel-sends + one WaitGroup.Wait per datagram is cheap and needs no
// batch pool / refcount machinery.  Workers are idle between datagrams, so the pipe
// set can be repartitioned on retune with zero extra synchronization.
package fcfb

import (
	"sort"
	"sync"
)

// emitPool is the fixed worker pool the dispatcher fans each datagram out to.
type emitPool struct {
	e       *Emitter
	n       int
	workers []*emitWorker
	wg      sync.WaitGroup // worker goroutines (joined by stop)
}

// emitWorker owns a disjoint subset of the pipes (reassigned on rebuild) and runs
// their DSP for each datagram.  sub is this worker's private idx-gather scratch.
type emitWorker struct {
	e     *Emitter
	pipes []*pipe
	in    chan emitJob // one job per datagram (unbuffered: worker parks between them)
	sub   []complex128
}

// emitJob is one datagram's columns, shared read-only across all workers, plus the
// barrier the dispatcher waits on.
type emitJob struct {
	colsA, colsB [][]complex128
	wg           *sync.WaitGroup
}

func newEmitPool(e *Emitter, n int) *emitPool {
	p := &emitPool{e: e, n: n}
	p.workers = make([]*emitWorker, n)
	for i := range p.workers {
		w := &emitWorker{e: e, in: make(chan emitJob)}
		p.workers[i] = w
		p.wg.Add(1)
		go w.run(&p.wg)
	}
	return p
}

// run processes this worker's pipes for each datagram until its queue is closed.
func (w *emitWorker) run(wg *sync.WaitGroup) {
	defer wg.Done()
	for job := range w.in {
		for _, p := range w.pipes {
			w.e.processPipe(p, job.colsA, job.colsB, &w.sub)
		}
		job.wg.Done()
	}
}

// repartition reassigns the pipes across workers by a stable sorted key, so the
// assignment is deterministic across rebuilds.  It MUST be called only when every
// worker is parked on <-in (i.e. between fork-joins, which the dispatcher guarantees
// by calling it right after a rebuild and before the next dispatch); the subsequent
// send on w.in establishes happens-before so the worker sees the new slice.
func (p *emitPool) repartition(pipes map[pipeKey]*pipe) {
	for _, w := range p.workers {
		w.pipes = w.pipes[:0]
	}
	keys := make([]pipeKey, 0, len(pipes))
	for k := range pipes {
		keys = append(keys, k)
	}
	sort.Slice(keys, func(i, j int) bool {
		if keys[i].radio != keys[j].radio {
			return keys[i].radio < keys[j].radio
		}
		return keys[i].ddc < keys[j].ddc
	})
	for i, k := range keys {
		w := p.workers[i%p.n]
		w.pipes = append(w.pipes, pipes[k])
	}
}

// dispatch fans one datagram out to every worker and blocks until all have finished.
// The barrier preserves ordering: no pipe advances to the next datagram until this
// one is fully processed across all pipes.
func (p *emitPool) dispatch(colsA, colsB [][]complex128) {
	var wg sync.WaitGroup
	wg.Add(p.n)
	job := emitJob{colsA: colsA, colsB: colsB, wg: &wg}
	for _, w := range p.workers {
		w.in <- job // workers are parked on <-in (previous Wait returned)
	}
	wg.Wait()
}

// stop closes the worker queues and waits for the goroutines to drain.  Called after
// the dispatcher loop exits, when no dispatch is in flight.
func (p *emitPool) stop() {
	for _, w := range p.workers {
		close(w.in)
	}
	p.wg.Wait()
}
