// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Micro-benchmark for the hot scatter FIR in StreamSynth.push (the farm's single
// biggest CPU line).  Shape matches live data: W=7 bins, Dv=8 taps.  Run:
//
//	go test -bench=StreamSynthPush -benchmem
//
// Baseline on this machine is ~92 ns/op, 0 allocs.  Use this to gate any FIR
// change before wiring it into the farm.
//
// Negative result (measured 2026-09-26, not adopted): folding gt into the
// phasors as split real/imag SoA with two independent complex accumulators
// ("Option A") reached only ~87 ns/op -- ~5%, i.e. ~2% of total farm CPU (push
// is ~40% of the farm and this only touches its FIR part).  It also reassociates
// the tap/bin sum (~1e-12 off push), giving up the guaranteed bit-exact match to
// the batch reference for a marginal win.  The loop is ILP/memory-bound, so pure
// Go tops out here; real gains need SIMD (breaks the single static binary) or an
// FFT-based synthesis (algorithmic).
package fcfb

import (
	"math"
	"testing"
)

// benchW is the typical run width seen on-air (accepts are mostly {k,7,1}).
const benchW = 7

// newBenchSynth builds a StreamSynth with a realistic (W=benchW, Dv=8) shape and
// a ring of nb precomputed columns to cycle through.
func newBenchSynth(nb int) (*StreamSynth, [][]complex128) {
	cap := makeWindow(100, benchW, nb, 42)
	kaSub := make([]int, benchW)
	for j := range kaSub {
		kaSub[j] = cap.Ka[j]
	}
	kc := cap.Ka[benchW/2]
	ss := newStreamSynth(kaSub, kc, 3.0)
	cols := make([][]complex128, nb)
	for m := 0; m < nb; m++ {
		cols[m] = streamColumn(cap, m, make([]complex128, benchW))
	}
	return ss, cols
}

func BenchmarkStreamSynthPush(b *testing.B) {
	const nb = 512
	ss, cols := newBenchSynth(nb)
	b.ReportAllocs()
	b.ResetTimer()
	var sink complex128
	for i := 0; i < b.N; i++ {
		sink += ss.push(cols[i%nb])
	}
	sinkC = sink
}

// BenchmarkStreamResamplerPush measures the 40k->12k resampler (StreamResampler,
// the production streaming path) in ns per INPUT sample.  Cost scales with the
// filter length (resampHalf) -- this is the gate for Option D (shorten the filter).
func BenchmarkStreamResamplerPush(b *testing.B) {
	rs := newStreamResampler(binRate, 12000)
	out := make([]float64, 0, 4)
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		out = out[:0]
		rs.push(math.Sin(float64(i)*0.01), &out)
	}
	sinkF = float64(len(out))
}

// sinkC/sinkF keep benchmark results live so the compiler can't elide the work.
var (
	sinkC complex128
	sinkF float64
)
