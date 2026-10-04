// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// M1 validation: the streaming synth + resampler must be BIT-EXACT with the batch
// synthChannel / resamplePoly / toWav12k path over the same fixed window.
package fcfb

import (
	"math"
	"math/rand"
	"testing"
)

// makeWindow builds a synthetic single-run Captured window on ADC A: bins
// k0..k0+w-1, nb columns of deterministic complex data (a couple of tones + noise).
func makeWindow(k0, w, nb int, seed int64) *Captured {
	rng := rand.New(rand.NewSource(seed))
	Sa := alloc2(w, nb)
	for j := 0; j < w; j++ {
		// each bin gets a distinct slow phasor plus noise, so cross-bin structure
		// exercises the phvec dot product.
		f := 0.013*float64(j+1) + 0.0007
		for m := 0; m < nb; m++ {
			re := math.Cos(2*math.Pi*f*float64(m)) + 0.3*(rng.Float64()-0.5)
			im := math.Sin(2*math.Pi*f*float64(m)) + 0.3*(rng.Float64()-0.5)
			Sa[j][m] = complex(re*1000, im*1000)
		}
	}
	runs := []Run{{K0: k0, W: w, Adc: 1}}
	ka, kb := runsToBins(runs)
	return &Captured{Sa: Sa, Sb: alloc2(0, nb), Ka: ka, Kb: kb, Runs: runs, Nb: nb}
}

// streamColumn extracts column m of the ADC-A run as an ascending-ka slice, i.e.
// exactly what channelBaseband feeds synthChannel row-wise.
func streamColumn(cap *Captured, m int, dst []complex128) []complex128 {
	for j := range cap.Sa {
		dst[j] = cap.Sa[j][m]
	}
	return dst
}

func runStream(cap *Captured, kc int, tune float64) (bb []complex128, audio []float64) {
	w := len(cap.Sa)
	kaSub := make([]int, w)
	for j := range kaSub {
		kaSub[j] = cap.Ka[j] // single run: full ka, ascending
	}
	ss := newStreamSynth(kaSub, kc, tune)
	rs := newStreamResampler(binRate, 12000)
	col := make([]complex128, w)
	bb = make([]complex128, cap.Nb)
	for m := 0; m < cap.Nb; m++ {
		streamColumn(cap, m, col)
		s := ss.push(col)
		bb[m] = s
		rs.push(real(s), &audio)
	}
	rs.flush(&audio)
	return bb, audio
}

// TestStreamSynthMatchesBatch: the streaming synth NCOs by a phasor recurrence
// rather than a per-block cmplx.Exp, so it is no longer bit-identical to
// synthChannel -- but must stay within a tiny tolerance (accumulated phasor drift
// is ~1e-13 over a window, far below the int16 audio quantisation; the byte-exact
// audio tests confirm the WAVs, hence the spots, are unchanged).
func TestStreamSynthMatchesBatch(t *testing.T) {
	const tol = 1e-9 // >> the ~1e-13 recurrence drift, << anything that moves a spot
	cases := []struct {
		k0, w, kc int
		tune      float64
	}{
		{k0: 100, w: 5, kc: 102, tune: 137.5},    // kc mid-run, fine tune
		{k0: 100, w: 5, kc: 100, tune: 0},        // kc at run start, no tune
		{k0: 2040, w: 7, kc: 2043, tune: -412.3}, // higher bins, negative tune
		{k0: 10, w: 1, kc: 10, tune: 51.0},       // single-bin run
	}
	for _, c := range cases {
		cap := makeWindow(c.k0, c.w, 500, 42)
		ref := channelBaseband(cap, c.kc, 1, c.tune)
		if ref == nil {
			t.Fatalf("channelBaseband returned nil for %+v", c)
		}
		bb, _ := runStream(cap, c.kc, c.tune)
		if len(bb) != len(ref) {
			t.Fatalf("%+v: len bb=%d ref=%d", c, len(bb), len(ref))
		}
		for m := range ref {
			d := bb[m] - ref[m]
			if math.Hypot(real(d), imag(d)) > tol*(math.Hypot(real(ref[m]), imag(ref[m]))+1e-12) {
				t.Fatalf("%+v: synth diff at block %d exceeds tol: stream=%v batch=%v", c, m, bb[m], ref[m])
			}
		}
	}
}

func TestStreamResamplerBitExact(t *testing.T) {
	// Feed the batch baseband through both resamplers and require identical output.
	cap := makeWindow(100, 5, 500, 7)
	ref := channelBaseband(cap, 102, 1, 137.5)
	x := make([]float64, len(ref))
	for i := range ref {
		x[i] = real(ref[i])
	}
	want := resamplePoly(x, 3, 10)

	rs := newStreamResampler(binRate, 12000)
	var got []float64
	for _, v := range x {
		rs.push(v, &got)
	}
	rs.flush(&got)

	if len(got) != len(want) {
		t.Fatalf("resampler length: stream=%d batch=%d", len(got), len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("resampler mismatch at %d: stream=%v batch=%v", i, got[i], want[i])
		}
	}
}

func TestStreamAudioBitExact(t *testing.T) {
	// End to end: streaming int16 WAV audio must byte-match the batch toWav12k.
	cap := makeWindow(2040, 7, 500, 99)
	kc, tune := 2043, -412.3
	ref := channelBaseband(cap, kc, 1, tune)
	wantAudio := toWav12k(ref, binRate)

	_, audio := runStream(cap, kc, tune)
	gotAudio := peakNormalizeInt16(audio)

	if len(gotAudio) != len(wantAudio) {
		t.Fatalf("audio length: stream=%d batch=%d", len(gotAudio), len(wantAudio))
	}
	for i := range wantAudio {
		if gotAudio[i] != wantAudio[i] {
			t.Fatalf("audio mismatch at %d: stream=%d batch=%d", i, gotAudio[i], wantAudio[i])
		}
	}
}

// subWindow slices blocks [m0,m1) of a Captured into a fresh single-run Captured
// (the batch reference for one window).
func subWindow(cap *Captured, m0, m1 int64) *Captured {
	nb := int(m1 - m0)
	Sa := alloc2(len(cap.Sa), nb)
	for j := range cap.Sa {
		copy(Sa[j], cap.Sa[j][m0:m1])
	}
	return &Captured{Sa: Sa, Sb: alloc2(0, nb), Ka: cap.Ka, Kb: cap.Kb, Runs: cap.Runs, Nb: nb}
}

// TestStreamChannelWindowCut: a StreamChannel with a small custom period/capture
// must cut the stream into windows and produce, for each, audio byte-identical to
// the batch channelBaseband -> toWav12k over exactly that window's [m0,m1) blocks.
func TestStreamChannelWindowCut(t *testing.T) {
	const period, capture = 0.02, 0.015 // 800-block period, 600-block capture
	const kc, tune = 102, 137.5
	cap := makeWindow(100, 5, 2400, 5) // spans windows 0,1,2 plus dead zones

	var got [][]int16
	disp := func(audio, _, _, _ []float64, utc string, ch Channel, d Decoder) {
		got = append(got, peakNormalizeInt16(audio))
	}
	_, kaSub, _, ok := channelRows(cap, kc, 1)
	if !ok {
		t.Fatal("channelRows failed")
	}
	sc := newStreamChannel(Channel{FcHz: 0}, Decoder{}, kc, tune, 1, kaSub, 0, disp)
	sc.period, sc.capture = period, capture

	col := make([]complex128, len(kaSub))
	for blk := 0; blk < cap.Nb; blk++ {
		for i := range kaSub {
			col[i] = cap.Sa[i][blk]
		}
		sc.pushBlock(int64(blk), col)
	}
	sc.finish()

	// expected windows 0,1,2
	var want [][]int16
	for wi := int64(0); wi < 3; wi++ {
		m0, m1 := blockRange(0, float64(wi)*period, capture)
		sub := subWindow(cap, m0, m1)
		bb := channelBaseband(sub, kc, 1, tune)
		want = append(want, toWav12k(bb, binRate))
	}
	if len(got) != len(want) {
		t.Fatalf("window count: got %d want %d", len(got), len(want))
	}
	for wi := range want {
		if len(got[wi]) != len(want[wi]) {
			t.Fatalf("window %d length: got %d want %d", wi, len(got[wi]), len(want[wi]))
		}
		for i := range want[wi] {
			if got[wi][i] != want[wi][i] {
				t.Fatalf("window %d sample %d: got %d want %d", wi, i, got[wi][i], want[wi][i])
			}
		}
	}
}

// TestStreamChannelWindowContiguous checks the process_only_active_cycle=false
// path (capture == period): consecutive windows must tile the block axis with no
// gap and no overlap, so every block falls inside exactly one capture window and
// window wi ends on the exact block where wi+1 begins.
func TestStreamChannelWindowContiguous(t *testing.T) {
	const period, capture = 0.02, 0.02 // 800-block period, no dead time
	const kc, tune = 102, 137.5
	sc := newStreamChannel(Channel{FcHz: 0}, Decoder{}, kc, tune, 1, nil, 0, nil)
	sc.period, sc.capture = period, capture

	blocksPerWin := int64(period * binRate) // 800
	for blk := int64(0); blk < 3*blocksPerWin+1; blk++ {
		wi, m0, m1, in := sc.windowOf(blk)
		if !in {
			t.Fatalf("block %d: in=false, but full-period windows leave no dead time", blk)
		}
		if want := blk / blocksPerWin; wi != want {
			t.Fatalf("block %d: window %d, want %d", blk, wi, want)
		}
		// The window carrying blk must be contiguous with its successor: this
		// window's m1 is the next window's m0 (half-open [m0, m1)).
		nextM0, _ := blockRange(sc.t0, float64(wi+1)*period, capture)
		if m1 != nextM0 {
			t.Fatalf("window %d: m1=%d but next m0=%d (gap/overlap)", wi, m1, nextM0)
		}
		if blk < m0 || blk >= m1 {
			t.Fatalf("block %d not within its window [%d,%d)", blk, m0, m1)
		}
	}
}

// TestStreamReplayMatchesBatch: on a real captured fixture, the streaming single-
// window path must produce audio byte-identical to the batch channelBaseband ->
// toWav12k for the same channel (this is what runReplay does, minus the
// decoder subprocess, so spots are identical too).
func TestStreamReplayMatchesBatch(t *testing.T) {
	cap, err := readV3File("../fixtures/board_v3_2ch.bin")
	if err != nil {
		t.Skipf("fixture unavailable: %v", err)
	}
	if len(cap.Runs) == 0 || cap.Nb == 0 {
		t.Skip("fixture has no runs/blocks")
	}
	r := cap.Runs[0]
	adc := r.Adc
	if adc == 0 {
		adc = 1
	}
	kc := r.K0 + r.W/2 // mid-run bin
	const tune = 0.0

	// batch reference
	refBB := channelBaseband(cap, kc, adc, tune)
	if refBB == nil {
		t.Fatalf("channelBaseband nil for kc=%d adc=%d", kc, adc)
	}
	want := toWav12k(refBB, binRate)

	// streaming single window over the whole file (as runReplay)
	rowIdx, kaSub, S, ok := channelRows(cap, kc, adc)
	if !ok {
		t.Fatal("channelRows failed on fixture")
	}
	var got []int16
	disp := func(audio, _, _, _ []float64, utc string, ch Channel, d Decoder) { got = peakNormalizeInt16(audio) }
	dur := float64(cap.Nb)/binRate + 1
	sc := newStreamChannel(Channel{}, Decoder{}, kc, tune, adc, kaSub, 0, disp)
	sc.period, sc.capture = dur, dur
	col := make([]complex128, len(rowIdx))
	for blk := 0; blk < cap.Nb; blk++ {
		for i, j := range rowIdx {
			col[i] = S[j][blk]
		}
		sc.pushBlock(int64(blk), col)
	}
	sc.finish()

	if len(got) != len(want) {
		t.Fatalf("audio length: streaming=%d batch=%d", len(got), len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("fixture audio mismatch at %d: streaming=%d batch=%d", i, got[i], want[i])
		}
	}
	t.Logf("fixture nb=%d kc=%d adc=%d -> %d audio samples, byte-identical", cap.Nb, kc, adc, len(got))
}

// --- M3: streaming collector fan-out + drop/gap handling ----------------------

// fastCollector builds a StreamCollector for a single ADC-A channel over a
// makeWindow-style dataset, with the window period/capture overridden small so a
// few thousand blocks span several windows.  Returns the collector, the source
// data, and a pointer to the captured (utc, audio) windows in dispatch order.
func fastCollector(cap *Captured, kc int, tune, period, capture float64) (*StreamCollector, *[][]int16) {
	w := len(cap.Sa)
	meta := Meta{Runs: cap.Runs, Ka: cap.Ka, Kb: cap.Kb, Wa: w, Wb: 0}
	cfg := Config{Workers: 1,
		Decoders: []Decoder{{Name: "ft4", Cmd: "jt9 --ft4 {wav}", Parser: "jt9", PeriodS: period, CaptureS: capture}},
		Channels: []Channel{
			{FcHz: float64(kc)*binW + tune, Decoder: "ft4", Adc: 1, Name: "x"},
		}}
	got := &[][]int16{}
	disp := func(audio, _, _, _ []float64, utc string, ch Channel, d Decoder) {
		*got = append(*got, peakNormalizeInt16(audio))
	}
	col := newStreamCollector(cfg, meta, disp)
	for i := range col.feeds {
		col.feeds[i].sc.period = period
		col.feeds[i].sc.capture = capture
	}
	return col, got
}

// batchWindows returns the batch reference audio for windows 0..n-1 of src.
func batchWindows(src *Captured, kc int, tune, period, capture float64, n int) [][]int16 {
	var want [][]int16
	for wi := 0; wi < n; wi++ {
		m0, m1 := blockRange(0, float64(wi)*period, capture)
		sub := subWindow(src, m0, m1)
		want = append(want, toWav12k(channelBaseband(sub, kc, 1, tune), binRate))
	}
	return want
}

func assertWindowsEqual(t *testing.T, got, want [][]int16) {
	t.Helper()
	if len(got) != len(want) {
		t.Fatalf("window count: got %d want %d", len(got), len(want))
	}
	for wi := range want {
		if len(got[wi]) != len(want[wi]) {
			t.Fatalf("window %d length: got %d want %d", wi, len(got[wi]), len(want[wi]))
		}
		for i := range want[wi] {
			if got[wi][i] != want[wi][i] {
				t.Fatalf("window %d sample %d: got %d want %d", wi, i, got[wi][i], want[wi][i])
			}
		}
	}
}

// TestStreamCollectorFanout: a contiguous seq stream through onBlock reconstructs
// each window byte-identical to the batch path (validates t0 anchor + fan-out).
func TestStreamCollectorFanout(t *testing.T) {
	const period, capture = 0.02, 0.015
	const kc, tune = 102, 137.5
	src := makeWindow(100, 5, 2400, 5)
	col, got := fastCollector(src, kc, tune, period, capture)

	colA := make([]complex128, 5)
	for blk := 0; blk < src.Nb; blk++ {
		for j := 0; j < 5; j++ {
			colA[j] = src.Sa[j][blk]
		}
		col.onBlock(uint64(blk), colA, nil, 0) // now=0 -> t0=0 (aligns to batch windows)
	}
	col.finishAll()

	assertWindowsEqual(t, *got, batchWindows(src, kc, tune, period, capture, 3))
	if _, lost, reorder, resyncs := col.stats(); lost != 0 || reorder != 0 || resyncs != 0 {
		t.Fatalf("clean stream should have no loss: lost=%d reorder=%d resyncs=%d", lost, reorder, resyncs)
	}
}

// TestStreamCollectorZeroFill: a forward seq gap (UDP loss) inside a window is
// zero-filled, keeping the timeline aligned -> audio matches the batch path with
// those columns zeroed, and lost is accounted.
func TestStreamCollectorZeroFill(t *testing.T) {
	const period, capture = 0.02, 0.015
	const kc, tune = 102, 137.5
	src := makeWindow(100, 5, 2400, 5)
	col, got := fastCollector(src, kc, tune, period, capture)

	const gapLo, gapHi = 200, 210 // lose blocks [200,210) in window 0
	colA := make([]complex128, 5)
	for blk := 0; blk < src.Nb; blk++ {
		if blk >= gapLo && blk < gapHi {
			continue // dropped datagram
		}
		for j := 0; j < 5; j++ {
			colA[j] = src.Sa[j][blk]
		}
		col.onBlock(uint64(blk), colA, nil, 0)
	}
	col.finishAll()

	// reference: same data but the lost columns zeroed.
	ref := makeWindow(100, 5, 2400, 5)
	for j := 0; j < 5; j++ {
		for blk := gapLo; blk < gapHi; blk++ {
			ref.Sa[j][blk] = 0
		}
	}
	assertWindowsEqual(t, *got, batchWindows(ref, kc, tune, period, capture, 3))
	if _, lost, _, _ := col.stats(); lost != gapHi-gapLo {
		t.Fatalf("lost accounting: got %d want %d", lost, gapHi-gapLo)
	}
}

// TestStreamCollectorReorderAndBigJump: a backward seq is ignored; a big seq jump
// re-anchors and drops the in-flight window without emitting garbage.
func TestStreamCollectorReorderAndBigJump(t *testing.T) {
	const period, capture = 0.02, 0.015
	const kc, tune = 102, 137.5
	src := makeWindow(100, 5, 4000, 5)
	col, got := fastCollector(src, kc, tune, period, capture)
	colA := make([]complex128, 5)
	feed := func(seq uint64, now float64) {
		blk := int(seq) % src.Nb
		for j := 0; j < 5; j++ {
			colA[j] = src.Sa[j][blk]
		}
		col.onBlock(seq, colA, nil, now)
	}

	// window 0 fully (blocks 0..599), plus a couple dead-zone blocks to finalize it.
	for blk := 0; blk <= 620; blk++ {
		feed(uint64(blk), 0)
	}
	if len(*got) != 1 {
		t.Fatalf("expected window 0 dispatched, got %d windows", len(*got))
	}
	// reorder: a backward seq is ignored.
	feed(300, 0)
	if _, _, reorder, _ := col.stats(); reorder != 1 {
		t.Fatalf("reorder not counted: %d", reorder)
	}
	// big jump: re-anchor (resync), no extra window emitted from the partial state.
	before := len(*got)
	feed(uint64(620+bigJump+5000), 0)
	if _, _, _, resyncs := col.stats(); resyncs != 1 {
		t.Fatalf("big jump should resync once: %d", resyncs)
	}
	if len(*got) != before {
		t.Fatalf("re-anchor must not emit a window: before=%d after=%d", before, len(*got))
	}
}

// subWindowAB copies BOTH ADCs' rows (subWindow only copies ADC A) so a dual
// channel's ADC-B reference can be reconstructed by the batch path.
func subWindowAB(cap *Captured, m0, m1 int64) *Captured {
	nb := int(m1 - m0)
	Sa := alloc2(len(cap.Sa), nb)
	for j := range cap.Sa {
		copy(Sa[j], cap.Sa[j][m0:m1])
	}
	Sb := alloc2(len(cap.Sb), nb)
	for j := range cap.Sb {
		copy(Sb[j], cap.Sb[j][m0:m1])
	}
	return &Captured{Sa: Sa, Sb: Sb, Ka: cap.Ka, Kb: cap.Kb, Runs: cap.Runs, Nb: nb}
}

// TestStreamDualChannelSync is the diversity guarantee: a dual (adc=3) channel's
// left output must be BIT-IDENTICAL to a mono ADC-A decode and its right output
// bit-identical to a mono ADC-B decode of the same bin, window for window, with
// left and right the same length -- i.e. the two antennas are perfectly
// time-locked and each is correctly reconstructed.  ADC A and ADC B carry
// deliberately different data and amplitude so a swap or cross-feed would fail.
func TestStreamDualChannelSync(t *testing.T) {
	const period, capture = 0.02, 0.015
	const kc, tune = 102, 137.5
	const w, nb = 5, 2400
	k0 := kc - 2

	fill := func(S [][]complex128, seed int64, amp float64) {
		rng := rand.New(rand.NewSource(seed))
		for j := 0; j < w; j++ {
			f := 0.013*float64(j+1) + 0.0007
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
	fill(src.Sa, 1, 1000) // ADC A
	fill(src.Sb, 2, 700)  // ADC B: different data AND amplitude
	src.Ka, src.Kb = runsToBins(src.Runs)

	meta := Meta{Runs: src.Runs, Ka: src.Ka, Kb: src.Kb, Wa: w, Wb: w}
	cfg := Config{Workers: 1,
		Decoders: []Decoder{{Name: "d", Cmd: "x {wav}", Parser: "jt9", PeriodS: period, CaptureS: capture}},
		Channels: []Channel{{FcHz: float64(kc)*binW + tune, Decoder: "d", Adc: 3, Name: "dual"}},
	}
	type win struct{ l, r []float64 }
	var got []win
	disp := func(audioL, audioR, _, _ []float64, utc string, ch Channel, d Decoder) {
		got = append(got, win{append([]float64(nil), audioL...), append([]float64(nil), audioR...)})
	}
	col := newStreamCollector(cfg, meta, disp)
	if len(col.feeds) != 1 || !col.feeds[0].sc.dual {
		t.Fatalf("expected one dual feed, got %d feeds", len(col.feeds))
	}
	for i := range col.feeds {
		col.feeds[i].sc.period, col.feeds[i].sc.capture = period, capture
	}

	colA := make([]complex128, w)
	colB := make([]complex128, w)
	for blk := 0; blk < nb; blk++ {
		for j := 0; j < w; j++ {
			colA[j] = src.Sa[j][blk]
			colB[j] = src.Sb[j][blk]
		}
		col.onBlock(uint64(blk), colA, colB, 0) // now=0 -> t0=0, aligns to batch windows
	}
	col.finishAll()

	if len(got) < 2 {
		t.Fatalf("expected multiple dual windows, got %d", len(got))
	}
	for wi := range got {
		if len(got[wi].l) != len(got[wi].r) {
			t.Fatalf("window %d: left/right length differ (%d vs %d) -- not time-locked",
				wi, len(got[wi].l), len(got[wi].r))
		}
		m0, m1 := blockRange(0, float64(wi)*period, capture)
		sub := subWindowAB(src, m0, m1)
		wantL := toWav12k(channelBaseband(sub, kc, 1, tune), binRate) // mono ADC A
		wantR := toWav12k(channelBaseband(sub, kc, 2, tune), binRate) // mono ADC B
		gotL := peakNormalizeInt16(got[wi].l)
		gotR := peakNormalizeInt16(got[wi].r)
		if len(gotL) != len(wantL) || len(gotR) != len(wantR) {
			t.Fatalf("window %d length vs batch: L %d/%d R %d/%d", wi, len(gotL), len(wantL), len(gotR), len(wantR))
		}
		for i := range wantL {
			if gotL[i] != wantL[i] {
				t.Fatalf("window %d L sample %d: dual=%d monoA=%d", wi, i, gotL[i], wantL[i])
			}
		}
		for i := range wantR {
			if gotR[i] != wantR[i] {
				t.Fatalf("window %d R sample %d: dual=%d monoB=%d", wi, i, gotR[i], wantR[i])
			}
		}
	}
	t.Logf("dual: %d windows, L==monoADC-A and R==monoADC-B byte-identical, lengths matched", len(got))
}

// TestStreamResamplerBounded checks trimming keeps the input buffer small over a
// long stream (the whole point of streaming) while staying bit-exact.
func TestStreamResamplerBounded(t *testing.T) {
	rng := rand.New(rand.NewSource(1))
	const nin = 200000
	x := make([]float64, nin)
	for i := range x {
		x[i] = rng.NormFloat64()
	}
	want := resamplePoly(x, 3, 10)

	rs := newStreamResampler(binRate, 12000)
	var got []float64
	maxBuf := 0
	for _, v := range x {
		rs.push(v, &got)
		if len(rs.buf) > maxBuf {
			maxBuf = len(rs.buf)
		}
	}
	rs.flush(&got)

	if len(got) != len(want) {
		t.Fatalf("length: stream=%d batch=%d", len(got), len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("mismatch at %d over long stream", i)
		}
	}
	// history should stay a small multiple of L/up, never grow with nin.
	if maxBuf > 4*rs.l/rs.up+rs.down+16 {
		t.Fatalf("input buffer not bounded: maxBuf=%d (L=%d up=%d)", maxBuf, rs.l, rs.up)
	}
}

// TestResampleRatio locks the reduced up/down for several rates, including the
// invariant that 12000 stays 3/10 (so the default remains bit-exact vs batch).
func TestResampleRatio(t *testing.T) {
	cases := []struct{ out, up, down int }{
		{12000, 3, 10}, // default: unchanged
		{48000, 6, 5},
		{24000, 3, 5},
		{8000, 1, 5},
	}
	for _, c := range cases {
		up, down := resampleRatio(binRate, c.out)
		if up != c.up || down != c.down {
			t.Fatalf("resampleRatio(%v,%d)=%d/%d want %d/%d", binRate, c.out, up, down, c.up, c.down)
		}
	}
}

// TestStreamChannelRate48k: a decoder with rate=48000 resamples to 48 kHz, and the
// streaming path stays byte-identical to the batch toWavRate at that rate.
func TestStreamChannelRate48k(t *testing.T) {
	cap := makeWindow(100, 5, 500, 7)
	kc, tune := 102, 137.5
	const rate = 48000
	want := toWavRate(channelBaseband(cap, kc, 1, tune), binRate, rate)

	_, kaSub, _, ok := channelRows(cap, kc, 1)
	if !ok {
		t.Fatal("channelRows failed")
	}
	var got []int16
	disp := func(audio, _, _, _ []float64, utc string, ch Channel, d Decoder) { got = peakNormalizeInt16(audio) }
	dur := float64(cap.Nb)/binRate + 1
	dec := Decoder{Name: "x", Rate: rate, PeriodS: dur, CaptureS: dur}
	sc := newStreamChannel(Channel{}, dec, kc, tune, 1, kaSub, 0, disp)
	col := make([]complex128, len(kaSub))
	for m := 0; m < cap.Nb; m++ {
		for j := range kaSub {
			col[j] = cap.Sa[j][m]
		}
		sc.pushBlock(int64(m), col)
	}
	sc.finish()

	if len(got) != len(want) {
		t.Fatalf("48k audio length: streaming=%d batch=%d", len(got), len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("48k audio mismatch at %d", i)
		}
	}
}
