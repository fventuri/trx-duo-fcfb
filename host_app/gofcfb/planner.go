// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Channel planner: {(fc, BW, adc)} -> board request + per-channel synth plan.  Go
// port of host_app/planner.py (the subset the HPSDR emitter needs).
//
// Each requested channel occupies the bins spanning [fc-BW/2, fc+BW/2], padded by
// `guard` bins each side (measured margin the windowed-dual needs to reach the
// reconstruction floor at a band edge; matches the server's fcfb_admit).  The board
// admits each channel to that padded run and ORs them into a per-ADC keep bitmap, so
// nearby channels SHARE bins -- the streamed cost is the union popcount, not the sum.
// The wire limit is bins (spectral span), not channels.
package fcfb

import "sort"

const (
	guardBins     = guardDef // 3, bins each side of the occupied band (matches server)
	budgetDefault = 500      // total streamed bins (line-rate ceiling); server hard cap 655
)

// phaseLadder holds the divisors of R=3125 (=5^5) -> jitter-free decimation.  Output
// rate = phases*40 kHz.  `phases` must divide R so the `phases` equally-spaced
// decimation instants round(l*R/phases) are EXACT integer full-rate positions (else
// ~-60 dBc jitter spurs on off-centre content).  Bonus: every ladder rate resamples
// to 48 kHz exactly (40k->48k x6/5, 200k->48k x6/25, 1M->48k x6/125).
var phaseLadder = []int{1, 5, 25, 125, 625, 3125}

// phasesForBW returns the smallest jitter-free output multiple (divisor of R) whose
// rate covers bwHz.
func phasesForBW(bwHz float64) int {
	for _, p := range phaseLadder {
		if float64(p)*binRate >= bwHz {
			return p
		}
	}
	return phaseLadder[len(phaseLadder)-1]
}

// binBudgetMTU is the largest W_total (W_a+W_b) whose record still fits one
// unfragmented datagram -- a byte-identical mirror of the server's
// fcfb_bin_budget_mtu (host/fcfb_net.h).  Overhead = IP(20)+UDP(8)+fcfb_udp_hdr(12)+
// seq(8) = 48 B; each bin costs 2*bin_bytes (I+Q).  int24 -> 242 @1500 / 655 @3980.
// `mtu` MUST match the -u the server was started with.
func binBudgetMTU(binWidth, mtu int) int {
	bb := 2
	if binWidth == 24 {
		bb = 3
	}
	const overhead = 20 + 8 + 12 + 8
	if mtu <= overhead {
		return 0
	}
	return (mtu - overhead) / (2 * bb)
}

// ChannelReq is a requested receiver channel.
type ChannelReq struct {
	FcHz    float64 // centre frequency
	BwHz    float64 // occupied bandwidth (signal span to preserve)
	Adc     int     // 1=ADC A, 2=ADC B, 3=both (diversity)
	AudioHz float64 // place fc at this output frequency (default DC)
	Name    string
}

// ChannelPlan is the reconstruction recipe for one channel.
type ChannelPlan struct {
	Req      ChannelReq
	Kc       int     // reference bin = round(fc/BINW)
	Klo, Khi int     // covering run (occupied + guard), inclusive
	TuneHz   float64 // fractional fine-tune -> AudioHz
	Phases   int     // output rate = phases*40 kHz
}

// run is one merged contiguous bin run (K0, W).
type run struct{ K0, W int }

// Plan is the result of planChannels.
type Plan struct {
	Channels     []ChannelPlan
	RunsA, RunsB []run
	CostA, CostB int
	Budget       int
	OK           bool
}

// admitBin is round-half-up Hz->bin, matching the server's fcfb_admit ((long)(f/BINW+0.5)).
func admitBin(fHz float64) int { return int(fHz/binW + 0.5) }

// occupiedRun is the padded inclusive bin range [klo, khi], matching fcfb_admit:
// round each band edge to a bin, then add `guard` bins each side (klo>=1, khi<=N-1).
func occupiedRun(req ChannelReq, guard int) (klo, khi int) {
	klo = admitBin(req.FcHz-req.BwHz/2) - guard
	khi = admitBin(req.FcHz+req.BwHz/2) + guard
	if klo < 1 {
		klo = 1
	}
	if khi > nFFT-1 {
		khi = nFFT - 1
	}
	return
}

// mergeRuns merges inclusive [lo,hi] intervals into sorted (K0, W) contiguous runs.
func mergeRuns(ivs [][2]int) []run {
	if len(ivs) == 0 {
		return nil
	}
	s := make([][2]int, len(ivs))
	copy(s, ivs)
	sort.Slice(s, func(i, j int) bool {
		if s[i][0] != s[j][0] {
			return s[i][0] < s[j][0]
		}
		return s[i][1] < s[j][1]
	})
	out := [][2]int{{s[0][0], s[0][1]}}
	for _, iv := range s[1:] {
		last := &out[len(out)-1]
		if iv[0] <= last[1]+1 { // overlap or touch -> merge
			if iv[1] > last[1] {
				last[1] = iv[1]
			}
		} else {
			out = append(out, [2]int{iv[0], iv[1]})
		}
	}
	runs := make([]run, len(out))
	for i, o := range out {
		runs[i] = run{K0: o[0], W: o[1] - o[0] + 1}
	}
	return runs
}

// planChannels turns channel requests into a Plan (union runs, cost, per-channel recon).
func planChannels(reqs []ChannelReq, guard, budget int) Plan {
	var chans []ChannelPlan
	var ivA, ivB [][2]int
	for _, req := range reqs {
		klo, khi := occupiedRun(req, guard)
		kc := admitBin(req.FcHz)
		tune := (req.FcHz - float64(kc)*binW) - req.AudioHz
		chans = append(chans, ChannelPlan{
			Req: req, Kc: kc, Klo: klo, Khi: khi, TuneHz: tune, Phases: phasesForBW(req.BwHz),
		})
		if req.Adc&1 != 0 {
			ivA = append(ivA, [2]int{klo, khi})
		}
		if req.Adc&2 != 0 {
			ivB = append(ivB, [2]int{klo, khi})
		}
	}
	runsA := mergeRuns(ivA)
	runsB := mergeRuns(ivB)
	costA, costB := 0, 0
	for _, r := range runsA {
		costA += r.W
	}
	for _, r := range runsB {
		costB += r.W
	}
	return Plan{
		Channels: chans, RunsA: runsA, RunsB: runsB,
		CostA: costA, CostB: costB, Budget: budget, OK: costA+costB <= budget,
	}
}

// ddsInject maps a request index -> (dds_k, dds_phase_inc) for bench tone injection.
type ddsInject map[int]struct {
	K     int
	PhInc uint32
}

// toIngestChannels builds the []Chan for dialBoard: each channel's OCCUPIED band
// (fc +- BW/2); the server adds its own guard and unions.  `dds` optionally maps a
// channel index -> injection tone.
func toIngestChannels(reqs []ChannelReq, dds ddsInject) []Chan {
	out := make([]Chan, len(reqs))
	for i, req := range reqs {
		c := Chan{Flo: req.FcHz - req.BwHz/2, Fhi: req.FcHz + req.BwHz/2, AdcBits: req.Adc}
		if dds != nil {
			if inj, ok := dds[i]; ok {
				c.DdsK = inj.K
				c.DdsPhInc = inj.PhInc
			}
		}
		out[i] = c
	}
	return out
}
