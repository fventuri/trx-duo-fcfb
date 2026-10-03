// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Acceptance gate for multi-core reconstruction (recon_workers > 1): the parallel
// group path must dispatch windows BYTE-IDENTICAL to the single-threaded inline
// path over the same seq stream -- for mono (both ADCs) and dual/diversity
// channels, and across clean streams, forward gaps, reorders and big-jump resyncs.
// Because each channel's calls are never reordered, parallelising channels across
// goroutines cannot change any channel's audio; these tests lock that invariant in.
// Run with -race to also check the batch hand-off is data-race free.
package fcfb

import (
	"fmt"
	"math"
	"math/rand"
	"sort"
	"sync"
	"testing"
)

// multiSrc builds a two-run capture (ADC A run + ADC B run over the same bins), so
// a config can place mono-A, mono-B and dual channels on it.  A and B carry
// different data + amplitude so a swap or cross-feed would fail the comparison.
func multiSrc(k0, w, nb int) *Captured {
	fill := func(S [][]complex128, seed int64, amp float64) {
		rng := rand.New(rand.NewSource(seed))
		for j := 0; j < w; j++ {
			f := 0.011*float64(j+1) + 0.0005
			for m := 0; m < nb; m++ {
				re := math.Cos(2*math.Pi*f*float64(m)) + 0.3*(rng.Float64()-0.5)
				im := math.Sin(2*math.Pi*f*float64(m)) + 0.3*(rng.Float64()-0.5)
				S[j][m] = complex(re*amp, im*amp)
			}
		}
	}
	src := &Captured{
		Sa: alloc2(w, nb), Sb: alloc2(w, nb),
		Runs: []Run{{K0: k0, W: w, Adc: 1}, {K0: k0, W: w, Adc: 2}}, Nb: nb,
	}
	fill(src.Sa, 1, 1000)
	fill(src.Sb, 2, 640)
	src.Ka, src.Kb = runsToBins(src.Runs)
	return src
}

// multiCfg places four channels (mono A x2, mono B, dual) on the src run, all with
// the same small period/capture so a few thousand blocks span several windows.
func multiCfg(src *Captured, period, capture float64) (Config, Meta) {
	k0 := src.Runs[0].K0
	bin := func(off int) float64 { return float64(k0+off) * binW }
	cfg := Config{Workers: 1, ReconWorkers: 1,
		Decoders: []Decoder{{Name: "d", Cmd: "x {wav}", Parser: "jt9", PeriodS: period, CaptureS: capture}},
		Channels: []Channel{
			{FcHz: bin(3) + 137.5, Decoder: "d", Adc: 1, Name: "a3"},
			{FcHz: bin(10) - 61.0, Decoder: "d", Adc: 1, Name: "a10"},
			{FcHz: bin(6) + 12.0, Decoder: "d", Adc: 2, Name: "b6"},
			{FcHz: bin(8), Decoder: "d", Adc: 3, Name: "dual8"},
		}}
	meta := Meta{Runs: src.Runs, Ka: src.Ka, Kb: src.Kb, Wa: len(src.Sa), Wb: len(src.Sb)}
	return cfg, meta
}

// winRec is one dispatched window flattened to a comparable string (channel, utc,
// and the peak-normalised int16 L/R samples the decoder would actually see).
func winRec(audioL, audioR []float64, utc string, ch Channel) string {
	l := peakNormalizeInt16(audioL)
	var r []int16
	if audioR != nil {
		r = peakNormalizeInt16(audioR)
	}
	return fmt.Sprintf("%s|%s|L%v|R%v", ch.name(), utc, l, r)
}

// recorder is a concurrency-safe dispatch sink (workers call it from many
// goroutines in the parallel case).
type recorder struct {
	mu   sync.Mutex
	recs []string
}

func (rc *recorder) dispatch(audioL, audioR []float64, utc string, ch Channel, d Decoder) {
	s := winRec(audioL, audioR, utc, ch)
	rc.mu.Lock()
	rc.recs = append(rc.recs, s)
	rc.mu.Unlock()
}

func (rc *recorder) sorted() []string {
	rc.mu.Lock()
	defer rc.mu.Unlock()
	out := append([]string(nil), rc.recs...)
	sort.Strings(out)
	return out
}

// feedSeq drives a collector with one onBlock per seq in the supplied order
// (skipping any seq in drop), reading columns from src at blk = seq % nb so a
// big-jump seq still has data.
func feedSeq(col *StreamCollector, src *Captured, seqs []uint64, drop map[uint64]bool) {
	w := len(src.Sa)
	colA := make([]complex128, w)
	colB := make([]complex128, w)
	for _, seq := range seqs {
		if drop[seq] {
			continue
		}
		blk := int(seq) % src.Nb
		for j := 0; j < w; j++ {
			colA[j] = src.Sa[j][blk]
			colB[j] = src.Sb[j][blk]
		}
		col.onBlock(seq, colA, colB, 0) // now=0 -> t0=0 (deterministic windows)
	}
	col.finishAll()
}

// runBoth feeds the identical seq stream through a serial (recon_workers=1) and a
// parallel (recon_workers=groups) collector and returns both sorted window sets
// plus both stats tuples.
func runBoth(t *testing.T, src *Captured, period, capture float64, groups int, seqs []uint64, drop map[uint64]bool) (serial, parallel []string) {
	t.Helper()
	cfg, meta := multiCfg(src, period, capture)

	var rcS recorder
	colS := newStreamCollector(cfg, meta, rcS.dispatch)
	feedSeq(colS, src, seqs, drop)
	sSeq, sLost, sReorder, sResync := colS.stats()

	var rcP recorder
	colP := newStreamCollector(cfg, meta, rcP.dispatch)
	colP.startParallel(groups)
	if !colP.parallel {
		t.Fatalf("startParallel(%d) did not enable the parallel path (feeds=%d)", groups, len(colP.feeds))
	}
	feedSeq(colP, src, seqs, drop)
	pSeq, pLost, pReorder, pResync := colP.stats()

	// The seq accounting runs on the (single) ingest goroutine in both modes, so it
	// must be identical regardless of how the synth is split.
	if sSeq != pSeq || sLost != pLost || sReorder != pReorder || sResync != pResync {
		t.Fatalf("stats differ serial{seq=%d lost=%d reorder=%d resync=%d} parallel{seq=%d lost=%d reorder=%d resync=%d}",
			sSeq, sLost, sReorder, sResync, pSeq, pLost, pReorder, pResync)
	}
	return rcS.sorted(), rcP.sorted()
}

func assertRecsEqual(t *testing.T, serial, parallel []string) {
	t.Helper()
	if len(serial) == 0 {
		t.Fatal("no windows dispatched -- test would be vacuous")
	}
	if len(serial) != len(parallel) {
		t.Fatalf("window count: serial=%d parallel=%d", len(serial), len(parallel))
	}
	for i := range serial {
		if serial[i] != parallel[i] {
			// Trim the audio dump so the failure is readable.
			s, p := serial[i], parallel[i]
			if len(s) > 80 {
				s = s[:80]
			}
			if len(p) > 80 {
				p = p[:80]
			}
			t.Fatalf("window %d differs:\n serial=%s...\n parallel=%s...", i, s, p)
		}
	}
}

// TestParallelMatchesSerialClean: on a clean contiguous stream the parallel groups
// produce byte-identical windows to the serial path, for mono-A, mono-B and dual.
func TestParallelMatchesSerialClean(t *testing.T) {
	const period, capture = 0.02, 0.015
	src := multiSrc(200, 16, 3000)
	seqs := make([]uint64, src.Nb)
	for i := range seqs {
		seqs[i] = uint64(i)
	}
	for _, groups := range []int{2, 3, 4, 8} { // 8 > 4 channels: caps to 4
		serial, parallel := runBoth(t, src, period, capture, groups, seqs, nil)
		assertRecsEqual(t, serial, parallel)
		t.Logf("groups=%d: %d windows byte-identical to serial", groups, len(serial))
	}
}

// TestParallelMatchesSerialGapReorderResync: forward gaps (loss), a backward
// reorder, and a big-jump resync must all reconstruct byte-identically in the
// batched parallel path (the control events flush the batch in order).
func TestParallelMatchesSerialGapReorderResync(t *testing.T) {
	const period, capture = 0.02, 0.015
	src := multiSrc(200, 16, 4000)

	var seqs []uint64
	drop := map[uint64]bool{}
	// window 0 + a bit, with a lost datagram [500,520) inside it
	for s := 0; s <= 900; s++ {
		seqs = append(seqs, uint64(s))
	}
	for s := uint64(500); s < 520; s++ {
		drop[s] = true
	}
	// a backward/duplicate seq (reordered) -- must be ignored
	seqs = append(seqs, 700)
	// continue into window 1
	for s := 901; s <= 1300; s++ {
		seqs = append(seqs, uint64(s))
	}
	// big jump: re-anchor, drop the in-flight window, then a fresh run of blocks
	base := uint64(1300 + bigJump + 5000)
	for s := base; s <= base+900; s++ {
		seqs = append(seqs, s)
	}

	serial, parallel := runBoth(t, src, period, capture, 3, seqs, drop)
	assertRecsEqual(t, serial, parallel)
	t.Logf("gap+reorder+resync: %d windows byte-identical to serial", len(serial))
}
