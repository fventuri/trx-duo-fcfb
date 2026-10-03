// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
package fcfb

import "testing"

func TestAdmitBinMatchesServerFormula(t *testing.T) {
	// admitBin must be (long)(f/BINW+0.5), the server's fcfb_admit rounding.
	for _, k := range []int{1, 234, 700, 1500, 2048, 4095} {
		f := float64(k) * binW
		if got := admitBin(f); got != k {
			t.Fatalf("admitBin(%d*binW)=%d != %d", k, got, k)
		}
		// just below a bin centre still rounds to k (half-up); well below to k-1.
		if got := admitBin(f - 0.4*binW); got != k {
			t.Fatalf("admitBin(k-0.4)=%d != %d", got, k)
		}
		if k > 1 {
			if got := admitBin(f - 0.6*binW); got != k-1 {
				t.Fatalf("admitBin(k-0.6)=%d != %d", got, k-1)
			}
		}
	}
}

func TestPhasesForBW(t *testing.T) {
	cases := map[float64]int{
		3000: 1, 40000: 1, 48000: 5, 96000: 5, 192000: 5,
		200000: 5, 250000: 25, 1000000: 25, 1200000: 125,
	}
	for bw, want := range cases {
		if got := phasesForBW(bw); got != want {
			t.Fatalf("phasesForBW(%.0f)=%d want %d", bw, got, want)
		}
	}
}

func TestBinBudgetMTU(t *testing.T) {
	cases := []struct {
		bw, mtu, want int
	}{
		{24, 1500, 242}, {24, 3980, 655}, {16, 1500, 363}, {24, 40, 0},
	}
	for _, c := range cases {
		if got := binBudgetMTU(c.bw, c.mtu); got != c.want {
			t.Fatalf("binBudgetMTU(%d,%d)=%d want %d", c.bw, c.mtu, got, c.want)
		}
	}
}

func TestPlanChannelsUnionAndCost(t *testing.T) {
	// Two narrow channels 4 bins apart share/merge; a distant one is its own run.
	reqs := []ChannelReq{
		{FcHz: 700 * binW, BwHz: 3000, Adc: 1, Name: "A"},
		{FcHz: 704 * binW, BwHz: 3000, Adc: 1, Name: "B"},
		{FcHz: 1500 * binW, BwHz: 3000, Adc: 1, Name: "C"},
	}
	p := planChannels(reqs, guardBins, budgetDefault)
	if len(p.RunsB) != 0 {
		t.Fatalf("no ADC-B channels but RunsB=%v", p.RunsB)
	}
	// A occupies [700 - g, 700 + g], B [704 - g, 704 + g] with g=3 (bw<1 bin) -> merge
	// into one run [697, 707] = 11 bins; C -> [1497, 1503] = 7 bins.
	if len(p.RunsA) != 2 {
		t.Fatalf("expected 2 merged runs on A, got %v", p.RunsA)
	}
	if p.RunsA[0] != (run{K0: 697, W: 11}) {
		t.Fatalf("merged AB run = %+v want {697,11}", p.RunsA[0])
	}
	if p.RunsA[1] != (run{K0: 1497, W: 7}) {
		t.Fatalf("C run = %+v want {1497,7}", p.RunsA[1])
	}
	if p.CostA != 18 || p.CostB != 0 || !p.OK {
		t.Fatalf("cost A=%d B=%d ok=%v want 18/0/true", p.CostA, p.CostB, p.OK)
	}
	// per-channel recon recipe: kc, tune, phases.
	if p.Channels[0].Kc != 700 || p.Channels[0].Phases != 1 {
		t.Fatalf("chan A kc/phases = %d/%d", p.Channels[0].Kc, p.Channels[0].Phases)
	}
	// fc exactly on the bin grid -> tune ~0.
	if got := p.Channels[0].TuneHz; got > 1e-6 || got < -1e-6 {
		t.Fatalf("chan A tune = %v want ~0", got)
	}
}

func TestPlanDualADC(t *testing.T) {
	reqs := []ChannelReq{{FcHz: 900 * binW, BwHz: 3000, Adc: 3, Name: "div"}}
	p := planChannels(reqs, guardBins, budgetDefault)
	if len(p.RunsA) != 1 || len(p.RunsB) != 1 {
		t.Fatalf("adc=3 should occupy both A and B: A=%v B=%v", p.RunsA, p.RunsB)
	}
	if p.RunsA[0] != p.RunsB[0] {
		t.Fatalf("adc=3 runs should match: A=%+v B=%+v", p.RunsA[0], p.RunsB[0])
	}
	if p.CostA != p.RunsA[0].W || p.CostB != p.RunsB[0].W {
		t.Fatalf("dual cost A=%d B=%d", p.CostA, p.CostB)
	}
}

func TestToIngestChannels(t *testing.T) {
	reqs := []ChannelReq{{FcHz: 14.074e6, BwHz: 3000, Adc: 2, Name: "x"}}
	ch := toIngestChannels(reqs, nil)
	if len(ch) != 1 {
		t.Fatalf("len %d", len(ch))
	}
	if ch[0].Flo != 14.074e6-1500 || ch[0].Fhi != 14.074e6+1500 || ch[0].AdcBits != 2 {
		t.Fatalf("chan = %+v", ch[0])
	}
}
