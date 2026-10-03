// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Shared capture-window types and block<->UTC helpers used by the streaming
// collector (stream_collector.go), the offline replay reader (readV3File), the
// reconstruction reference (synth.go), and the bit-exact tests.
package fcfb

import "time"

// Captured is a fully-buffered window of bin-blocks, bin-major and bin_scale-applied
// ([W][nb]).  Produced by readV3File for offline replay and used as the golden
// reference by the reconstruction math + tests.
type Captured struct {
	Sa, Sb [][]complex128 // [Wa][nb] / [Wb][nb]
	Ka, Kb []int
	Runs   []Run
	Nb     int
}

func alloc2(rows, cols int) [][]complex128 {
	out := make([][]complex128, rows)
	for i := range out {
		out[i] = make([]complex128, cols)
	}
	return out
}

// nowSec is the wall clock in seconds (the streaming collector's t0 anchor source).
func nowSec() float64 { return float64(time.Now().UnixNano()) / 1e9 }

// blockRange maps a UTC window [start, start+dur] to block indices given anchor t0.
func blockRange(t0, start, dur float64) (int64, int64) {
	m0 := int64((start - t0) * binRate)
	m1 := int64((start + dur - t0) * binRate)
	if m0 < 0 {
		m0 = 0
	}
	if m1 < 0 {
		m1 = 0
	}
	return m0, m1
}
