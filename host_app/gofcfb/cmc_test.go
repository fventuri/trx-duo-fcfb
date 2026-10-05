// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
package fcfb

import (
	"math"
	"math/cmplx"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

// TestCommonModeCancelBeta checks the β math and real(C) for both reference choices
// on a hand-computable 2-frame I/Q vector:
//
//	A = {1+0j, 0+1j}   B = {3+0j, 0+1j}   (4ch [I_A,Q_A,I_B,Q_B])
//
// ref A: β = Σconj(A)B/Σ|A|² = (3+1)/2 = 2 ; C = B-2A = {1, -j}   -> real {1, 0}
// ref B: β = Σconj(B)A/Σ|B|² = (3+1)/10 = 0.4 ; C = A-0.4B = {-0.2, 0.6j} -> real {-0.2, 0}
func TestCommonModeCancelBeta(t *testing.T) {
	d := &WavData{FormatTag: wavFmtFloat, NumCh: 4, Rate: 12000,
		F32: []float32{1, 0, 3, 0, 0, 1, 0, 1}}

	monoA, bA, err := commonModeCancel(d, "A")
	if err != nil {
		t.Fatal(err)
	}
	if cmplx.Abs(bA-complex(2, 0)) > 1e-9 {
		t.Fatalf("ref A: β = %v, want 2+0j", bA)
	}
	if want := PeakNormalizeInt16([]float64{1, 0}); monoA[0] != want[0] || monoA[1] != want[1] {
		t.Fatalf("ref A: real(C) int16 = %v, want %v", monoA, want)
	}

	monoB, bB, err := commonModeCancel(d, "B")
	if err != nil {
		t.Fatal(err)
	}
	if cmplx.Abs(bB-complex(0.4, 0)) > 1e-9 {
		t.Fatalf("ref B: β = %v, want 0.4+0j", bB)
	}
	if want := PeakNormalizeInt16([]float64{-0.2, 0}); monoB[0] != want[0] || monoB[1] != want[1] {
		t.Fatalf("ref B: real(C) int16 = %v, want %v", monoB, want)
	}
}

// TestCommonModeCancelComplexBeta exercises a genuinely complex β (A and B differ by
// a complex factor plus a residual), confirming real(C) = real(O) - real(β·R).
func TestCommonModeCancelComplexBeta(t *testing.T) {
	// A = {1+0j, 0+1j}, B = {0+1j, -1+0j} = (0+1j)*A exactly -> β = j, C = 0.
	d := &WavData{FormatTag: wavFmtFloat, NumCh: 4, Rate: 12000,
		F32: []float32{1, 0, 0, 1, 0, 1, -1, 0}}
	_, b, err := commonModeCancel(d, "A")
	if err != nil {
		t.Fatal(err)
	}
	if cmplx.Abs(b-complex(0, 1)) > 1e-6 {
		t.Fatalf("β = %v, want 0+1j", b)
	}
	// B is exactly β·A, so real(C) must be ~0 everywhere (peak-normalise of zeros).
	n := len(d.F32) / 4
	for k := 0; k < n; k++ {
		rr, ri := float64(d.F32[4*k]), float64(d.F32[4*k+1])
		or := float64(d.F32[4*k+2])
		if resid := or - (real(b)*rr - imag(b)*ri); math.Abs(resid) > 1e-6 {
			t.Fatalf("frame %d residual real(C)=%g, want ~0", k, resid)
		}
	}
}

// TestCommonModeCancelThreeDecodes: a 4-ch dual I/Q capture decodes A, B and CMC, in
// that order, each tagged and run on its own prefixed temp file.
func TestCommonModeCancelThreeDecodes(t *testing.T) {
	p := filepath.Join(t.TempDir(), "iq4.wav")
	start := time.Date(2026, 9, 27, 18, 30, 15, 0, time.UTC)
	a := Auxi{Start: start, Stop: start.Add(15 * time.Second), CenterHz: 14074000, ADHz: 12000, BwHz: 3000}
	// 2 frames so β is well-defined; A nonzero.
	if err := writeWAVFloat32(p, 12000, 4, []float32{0.1, 0.2, 0.3, 0.4, 0.5, -0.1, 0.2, 0.2}, a); err != nil {
		t.Fatal(err)
	}
	out, rc, err := runCommonModeCancelling(stubArgs(p), "A", false)
	if err != nil || rc != 0 {
		t.Fatalf("rc=%d err=%v", rc, err)
	}
	ia := strings.Index(out, "#ANT A")
	ib := strings.Index(out, "#ANT B")
	ic := strings.Index(out, "#ANT CMC")
	if ia < 0 || ib < 0 || ic < 0 || !(ia < ib && ib < ic) {
		t.Fatalf("want #ANT A, then B, then CMC: %q", out)
	}
	// fast window (15s): HHMMSS stamp from auxi; A-/B-/CMC- prefixed temp names.
	for _, want := range []string{"A-260927_183015.wav", "B-260927_183015.wav", "CMC-260927_183015.wav"} {
		if !strings.Contains(out, want) {
			t.Fatalf("missing temp decode %q in: %q", want, out)
		}
	}
}

// TestCommonModeCancelRejectsNonDualIQ: every non-(4ch float) format hard-fails.
func TestCommonModeCancelRejectsNonDualIQ(t *testing.T) {
	dir := t.TempDir()
	start := time.Date(2026, 9, 27, 18, 30, 15, 0, time.UTC)
	a := Auxi{Start: start, Stop: start.Add(15 * time.Second), CenterHz: 14074000, ADHz: 12000, BwHz: 3000}

	mono := filepath.Join(dir, "mono.wav")
	if err := WriteWAVInt16(mono, 12000, 1, []int16{0, 1, -1, 2}); err != nil {
		t.Fatal(err)
	}
	stereo := filepath.Join(dir, "stereo.wav")
	if err := WriteWAVInt16(stereo, 12000, 2, []int16{10, 20, 11, 21}); err != nil {
		t.Fatal(err)
	}
	iq2 := filepath.Join(dir, "iq2.wav") // single-ADC I/Q (2ch float)
	if err := writeWAVFloat32(iq2, 12000, 2, []float32{0, 0, 0.5, -0.5}, a); err != nil {
		t.Fatal(err)
	}

	for _, p := range []string{mono, stereo, iq2} {
		_, rc, err := runCommonModeCancelling(stubArgs(p), "A", false)
		if err == nil || rc == 0 {
			t.Fatalf("%s: expected hard-fail, got rc=%d err=%v", filepath.Base(p), rc, err)
		}
		if !strings.Contains(err.Error(), "dual-ADC I/Q") {
			t.Fatalf("%s: error should mention dual-ADC I/Q, got %v", filepath.Base(p), err)
		}
	}
}

// TestCommonModeCancelBadArgs: missing decoder command and a bad --ref are rejected.
func TestCommonModeCancelBadArgs(t *testing.T) {
	if _, rc, err := runCommonModeCancelling(nil, "A", false); err == nil || rc == 0 {
		t.Fatalf("empty args should fail, got rc=%d err=%v", rc, err)
	}
	p := filepath.Join(t.TempDir(), "iq4.wav")
	start := time.Date(2026, 9, 27, 18, 30, 15, 0, time.UTC)
	a := Auxi{Start: start, Stop: start.Add(15 * time.Second), CenterHz: 14074000, ADHz: 12000, BwHz: 3000}
	if err := writeWAVFloat32(p, 12000, 4, []float32{0.1, 0.2, 0.3, 0.4}, a); err != nil {
		t.Fatal(err)
	}
	if _, rc, err := runCommonModeCancelling(stubArgs(p), "X", false); err == nil || rc == 0 {
		t.Fatalf("bad --ref should fail, got rc=%d err=%v", rc, err)
	}
}
