// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
package fcfb

import (
	"math"
	"math/cmplx"
	"testing"
)

func TestFreqPhaseRoundTrip(t *testing.T) {
	for _, ph := range []uint32{0, 1, 1 << 20, 0x7FFFFFFF, 0xFFFFFFFF} {
		if got := phaseFromFreq(freqFromPhase(ph)); got != ph {
			t.Fatalf("phase %d -> hz -> phase %d", ph, got)
		}
	}
	// Hz round-trips within one phase LSB (~0.0286 Hz).
	for _, hz := range []float64{0, 7.04e6, 14.074e6, 21.36e6, 28.5e6, 61.44e6} {
		got := freqFromPhase(phaseFromFreq(hz))
		if math.Abs(got-hz) > 1.0/phaseScale {
			t.Fatalf("hz %.3f -> %.3f (diff %.4f > LSB)", hz, got, got-hz)
		}
	}
}

func TestDiscoveryRoundTrip(t *testing.T) {
	mac := [6]byte{0x02, 0, 0, 0xFC, 0xFB, 0x07}
	b := buildDiscoveryReply(mac, boardAngelia, 3, 39, 40, 6)
	if len(b) != 60 {
		t.Fatalf("discovery reply len %d != 60", len(b))
	}
	d, ok := parseDiscoveryReply(b)
	if !ok || d.status != 3 || d.boardID != boardAngelia || d.device != 1003 ||
		d.p2Version != 39 || d.fwVersion != 40 || d.nDDC != 6 || d.mac != mac {
		t.Fatalf("discovery round-trip mismatch: %+v ok=%v", d, ok)
	}
	// A discovery request must be recognised; the reply must NOT be.
	req := make([]byte, 60)
	req[4] = 0x02
	if !isDiscovery(req) || isDiscovery(b) {
		t.Fatalf("isDiscovery: req=%v reply=%v", isDiscovery(req), isDiscovery(b))
	}
}

func TestHighPriorityRoundTrip(t *testing.T) {
	freqs := []float64{14.074e6, 7.074e6, 21.074e6}
	b := buildHighPriority(1234, true, freqs)
	hp := parseHighPriority(b)
	if hp.seq != 1234 || !hp.running {
		t.Fatalf("hp seq/running: %+v", hp)
	}
	for i, f := range freqs {
		if math.Abs(hp.ddcFreqHz[i]-f) > 1.0/phaseScale {
			t.Fatalf("ddc %d freq %.3f != %.3f", i, hp.ddcFreqHz[i], f)
		}
	}
}

func TestReceiveSpecificRoundTrip(t *testing.T) {
	ddcs := map[int]ddcConfig{
		0: {adc: 0, rate: 48000, bits: 24},
		2: {adc: 1, rate: 192000, bits: 24},
	}
	b := buildReceiveSpecific(7, ddcs, 2)
	rs := parseReceiveSpecific(b)
	if rs.seq != 7 || rs.nAdc != 2 || rs.enable != (1<<0|1<<2) {
		t.Fatalf("rs header: %+v", rs)
	}
	if len(rs.ddcs) != 2 {
		t.Fatalf("rs ddcs len %d", len(rs.ddcs))
	}
	for d, want := range ddcs {
		got := rs.ddcs[d]
		if got.adc != want.adc || got.rate != want.rate || got.bits != want.bits {
			t.Fatalf("ddc %d: got %+v want %+v", d, got, want)
		}
	}
}

func TestRxIQRoundTrip(t *testing.T) {
	iq := make([]complex128, samplesPerFrame)
	for i := range iq {
		ph := 2 * math.Pi * float64(i) / 17.0
		iq[i] = 0.5 * cmplx.Exp(complex(0, ph)) // |.|=0.5, well inside full scale
	}
	pkt := buildRxIQ(99, iq, 24, 0xDEADBEEF)
	if len(pkt) != 1444 {
		t.Fatalf("rx-iq packet len %d != 1444", len(pkt))
	}
	r, ok := parseRxIQ(pkt)
	if !ok || r.seq != 99 || r.timestamp != 0xDEADBEEF || r.bitsPerSample != 24 || r.n != samplesPerFrame {
		t.Fatalf("rx-iq header: %+v ok=%v", r, ok)
	}
	const q = 1.0 / s24Scale // one 24-bit LSB
	for i := range iq {
		if math.Abs(real(r.iq[i])-real(iq[i])) > q || math.Abs(imag(r.iq[i])-imag(iq[i])) > q {
			t.Fatalf("sample %d: got %v want %v (> 1 LSB)", i, r.iq[i], iq[i])
		}
	}
}
