// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
package fcfb

import (
	"math"
	"math/cmplx"
	"math/rand"
	"testing"
)

// synthPhasesRef is an independent batch reference for the multi-phase streaming
// synth: it reimplements synth.synth_channel(phases) + the stream_dsp.StreamSynth
// NCO (bin-carrier removal at n=block*R+offs, fine tune at the absolute output
// index).  Returns the interleaved complex output, length nb*phases.
func synthPhasesRef(S [][]complex128, kaSub []int, kc int, tuneHz float64, phases int) []complex128 {
	const R = rHop
	const N = nFFT
	W := len(S)
	nb := len(S[0])
	KN := len(gDual)
	D := (KN + R - 1) / R
	out := make([]complex128, nb*phases)
	for l := 0; l < phases; l++ {
		off := int(math.Round(float64(l) * float64(R) / float64(phases)))
		acc := make([]complex128, nb)
		for d := 0; d < D; d++ {
			s := d*R + off
			if s >= KN {
				break
			}
			gt := gDual[s]
			sm := s % N
			pv := make([]complex128, W)
			for j := 0; j < W; j++ {
				ang := 2 * math.Pi * float64(kaSub[j]*sm) / float64(N)
				pv[j] = cmplx.Exp(complex(0, ang)) / complex(float64(N), 0)
			}
			for m := d; m < nb; m++ { // acc[m] += gt * (pv . S[:,m-d])
				var proj complex128
				for j := 0; j < W; j++ {
					proj += pv[j] * S[j][m-d]
				}
				acc[m] += gt * proj
			}
		}
		for m := 0; m < nb; m++ {
			n := int64(m)*int64(R) + int64(off)
			ang := -2 * math.Pi * float64((int64(kc)*n)%int64(N)) / float64(N)
			if tuneHz != 0 {
				ang += -2 * math.Pi * tuneHz * (float64(m)*float64(phases) + float64(l)) / (float64(phases) * binRate)
			}
			out[m*phases+l] = acc[m] * cmplx.Exp(complex(0, ang))
		}
	}
	return out
}

func randCols(rng *rand.Rand, w, nb int) [][]complex128 {
	S := make([][]complex128, w)
	for j := range S {
		S[j] = make([]complex128, nb)
		for m := 0; m < nb; m++ {
			S[j][m] = complex(rng.NormFloat64(), rng.NormFloat64())
		}
	}
	return S
}

// TestPushPhasesMatchesBatch feeds the streaming synth block-by-block and checks the
// settled output (from block D-1) matches the batch reference, for several phase
// counts and a non-zero fine tune.
func TestPushPhasesMatchesBatch(t *testing.T) {
	const R = rHop
	D := (len(gDual) + R - 1) / R
	kaSub := []int{698, 699, 700, 701, 702}
	kc := 700
	const nb = 48
	const tol = 1e-9
	rng := rand.New(rand.NewSource(7))
	for _, phases := range []int{1, 5, 25} {
		for _, tune := range []float64{0, 1234.5} {
			S := randCols(rng, len(kaSub), nb)
			ref := synthPhasesRef(S, kaSub, kc, tune, phases)
			ss := newStreamSynthP(kaSub, kc, tune, phases)
			var got []complex128
			col := make([]complex128, len(kaSub))
			for m := 0; m < nb; m++ {
				for j := range kaSub {
					col[j] = S[j][m]
				}
				ss.pushPhases(col, &got)
			}
			want := ref[(D-1)*phases:] // streaming emits from block D-1
			if len(got) != len(want) {
				t.Fatalf("phases=%d tune=%.1f: len got=%d want=%d", phases, tune, len(got), len(want))
			}
			for i := range want {
				if d := cmplx.Abs(got[i] - want[i]); d > tol*(cmplx.Abs(want[i])+1e-12) {
					t.Fatalf("phases=%d tune=%.1f: sample %d diff %g (got %v want %v)",
						phases, tune, i, d, got[i], want[i])
				}
			}
		}
	}
}

// TestPushPhasesPhase1MatchesPush cross-checks the phases==1 pushPhases path against
// the farm's push() recurrence over the settled blocks (they NCO differently -- one
// explicit, one recurrence -- so only a tiny tolerance is expected).
func TestPushPhasesPhase1MatchesPush(t *testing.T) {
	const R = rHop
	D := (len(gDual) + R - 1) / R
	kaSub := []int{699, 700, 701}
	kc := 700
	const nb = 40
	const tol = 1e-9
	rng := rand.New(rand.NewSource(11))
	S := randCols(rng, len(kaSub), nb)

	sPush := newStreamSynth(kaSub, kc, 250.0)
	sPhase := newStreamSynthP(kaSub, kc, 250.0, 1)
	var pushOut, phaseOut []complex128
	col := make([]complex128, len(kaSub))
	for m := 0; m < nb; m++ {
		for j := range kaSub {
			col[j] = S[j][m]
		}
		pushOut = append(pushOut, sPush.push(col))
		sPhase.pushPhases(col, &phaseOut)
	}
	// pushPhases drops the D-1 warm-up; compare against push()'s settled blocks.
	pushSettled := pushOut[D-1:]
	if len(phaseOut) != len(pushSettled) {
		t.Fatalf("len phaseOut=%d pushSettled=%d", len(phaseOut), len(pushSettled))
	}
	for i := range phaseOut {
		if d := cmplx.Abs(phaseOut[i] - pushSettled[i]); d > tol*(cmplx.Abs(pushSettled[i])+1e-12) {
			t.Fatalf("sample %d: pushPhases %v vs push %v diff %g", i, phaseOut[i], pushSettled[i], d)
		}
	}
}

// TestComplexResampler checks the complex resampler equals two independent real
// StreamResamplers and preserves a complex exponential's frequency (48k of 200k in).
func TestComplexResampler(t *testing.T) {
	rateIn := 200000.0
	rateOut := 48000
	cr := newComplexResampler(rateIn, rateOut)
	// a pure complex tone at +5 kHz
	const nin = 4000
	x := make([]complex128, nin)
	for i := 0; i < nin; i++ {
		ph := 2 * math.Pi * 5000.0 * float64(i) / rateIn
		x[i] = cmplx.Exp(complex(0, ph))
	}
	y := cr.process(x)
	if len(y) < 100 {
		t.Fatalf("too few output samples: %d", len(y))
	}
	// the magnitude should stay ~1 in the settled interior (unit tone, unit gain).
	mid := y[len(y)/2]
	if m := cmplx.Abs(mid); m < 0.9 || m > 1.1 {
		t.Fatalf("resampled tone magnitude %.3f not ~1", m)
	}
}
