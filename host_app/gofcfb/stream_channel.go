// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Streaming per-channel reconstruction: the incremental equivalents of
// synth.synthChannel and synth.resamplePoly, so a channel can be synthesised and
// resampled block-by-block as bin-columns arrive -- no whole-window bin buffer and
// no per-channel acc[nb].  Both pieces are BIT-EXACT with the batch path over the
// same input (see stream_channel_test.go):
//
//   - StreamSynth  -- holds the last D=ceil(len(g)/R) bin-columns and emits one
//     complex baseband sample per pushed block, NCO'd to DC + fine-tuned using the
//     ABSOLUTE block index (phase continuous across blocks).  The scatter FIR
//     accumulates taps d=dmax..0 (high->low), matching synthChannel's acc[p] order.
//
//   - StreamResampler -- exact-ratio polyphase resampler (rateIn->12 kHz) that
//     emits an output sample only once every input it needs has arrived (no
//     clamping), then flush() emits the implicit-zero tail exactly as resamplePoly
//     does over the finite buffer.  Input history is bounded to ~L/up samples.
//
// This file has no window/UTC/boundary logic yet -- that (StreamChannel proper +
// the collector fan-out) is the next milestone.  These are the validated kernels
// it will build on.
package fcfb

import (
	"fmt"
	"math"
	"math/cmplx"
	"os"
	"sync"
	"sync/atomic"
	"time"
)

// StreamSynth reconstructs one channel's complex baseband at binRate (phases=1),
// one output sample per pushed bin-column.
// StreamSynth is a gapless per-channel windowed-dual synthesiser.  It emits
// `phases` output samples per input block at rate phases*40 kHz (phases must divide
// R so the decimation is jitter-free -- see phasesForBW).  Two consumers:
//   - the decoder farm uses phases==1 via push(): a single tap table + a phasor-
//     recurrence NCO (one complex multiply/block), the hot-path fast form.
//   - the HPSDR emitter uses phases>=1 via pushPhases(): a tap table per phase and
//     a per-output-sample NCO from the absolute position, matching the offline
//     synthChannel / stream_dsp.StreamSynth (the reference the farm was validated
//     against).  The FIR projection is shared by both (projAt).
type StreamSynth struct {
	// phase-0 tables (aliases of gtP[0]/phvecP[0]); push()'s fast path uses these.
	gt      []complex128   // Dv taps g[d*R]
	phvec   [][]complex128 // Dv x W: exp(2i*pi*ka[j]*(d*R % N)/N)/N
	w       int
	dv      int            // len(phase-0 taps) == len(ring)
	ring    [][]complex128 // D x W ring of recent columns; ring[p%D] is block p
	p       int64          // absolute index of the NEXT block to push
	nco     complex128     // phases==1 NCO phasor for block p: exp(-2i*pi*p*phi)
	ncoStep complex128     // per-block advance: exp(-2i*pi*phi)

	// multi-phase state (pushPhases).
	phases  int
	offs    []int            // offs[l] = round(l*R/phases); offs[0]==0
	gtP     [][]complex128   // per phase l: taps g[d*R+offs[l]]
	phvecP  [][][]complex128 // per phase l, per tap d: (1/N)exp(2i*pi*ka[j]*((d*R+offs[l])%N)/N)
	kc      int
	tuneHz  float64
	rate    float64 // phases*binRate (output sample rate)
	emitBlk int64   // first block whose FIR window is full (D-1); warm-up dropped below it
}

// newStreamSynth builds the phases==1 synthesis state (the farm's rate): centred on
// bin kc with fine tune tuneHz, on bins kaSub (absolute bin indices, ascending --
// exactly the rows channelBaseband feeds synthChannel).
func newStreamSynth(kaSub []int, kc int, tuneHz float64) *StreamSynth {
	return newStreamSynthP(kaSub, kc, tuneHz, 1)
}

// newStreamSynthP builds the synthesis state for `phases` output samples per block
// (output rate phases*40 kHz).  phases==1 reproduces newStreamSynth exactly.
func newStreamSynthP(kaSub []int, kc int, tuneHz float64, phases int) *StreamSynth {
	const R = rHop
	const N = nFFT
	if phases < 1 {
		phases = 1
	}
	w := len(kaSub)
	D := (len(gDual) + R - 1) / R
	offs := make([]int, phases)
	gtP := make([][]complex128, phases)
	phvecP := make([][][]complex128, phases)
	for l := 0; l < phases; l++ {
		off := int(math.Round(float64(l) * float64(R) / float64(phases)))
		offs[l] = off
		var gt []complex128
		var phvec [][]complex128
		for d := 0; d < D; d++ {
			s := d*R + off
			if s >= len(gDual) {
				break
			}
			gt = append(gt, gDual[s])
			pv := make([]complex128, w)
			sm := s % N
			for j := 0; j < w; j++ {
				ang := 2 * math.Pi * float64(kaSub[j]*sm) / float64(N)
				pv[j] = cmplx.Exp(complex(0, ang)) / complex(float64(N), 0)
			}
			phvec = append(phvec, pv)
		}
		gtP[l] = gt
		phvecP[l] = phvec
	}
	ring := make([][]complex128, D) // D >= taps of any phase
	for i := range ring {
		ring[i] = make([]complex128, w)
	}
	// phases==1 recurrence NCO: per-block phase -2*pi*phi, phi = kc*R/N + tune/binRate.
	phi := float64(kc)*float64(R)/float64(N) + tuneHz/binRate
	ncoStep := cmplx.Exp(complex(0, -2*math.Pi*phi))
	return &StreamSynth{
		gt: gtP[0], phvec: phvecP[0], w: w, dv: len(gtP[0]), ring: ring,
		nco: complex(1, 0), ncoStep: ncoStep,
		phases: phases, offs: offs, gtP: gtP, phvecP: phvecP,
		kc: kc, tuneHz: tuneHz, rate: float64(phases) * binRate, emitBlk: int64(D - 1),
	}
}

// projAt returns the settled projection sum_d gt[d] * (phvec[d] . ring[p-d]) for
// absolute block p, walking d high->low (bit-identical to the original push loop).
func (s *StreamSynth) projAt(gt []complex128, phvec [][]complex128, p int64) complex128 {
	dmax := len(gt) - 1
	if int64(dmax) > p {
		dmax = int(p) // cold start: only d<=p have real history
	}
	rl := int64(len(s.ring))
	var bb complex128
	for d := dmax; d >= 0; d-- {
		c := s.ring[int((p-int64(d))%rl)]
		pv := phvec[d]
		var proj complex128
		for j := 0; j < s.w; j++ {
			proj += pv[j] * c[j]
		}
		bb += gt[d] * proj
	}
	return bb
}

// push stores column col (w complex bins for this block, ka-ascending) and returns
// the settled baseband sample for the current block (phases==1).  col is copied, so
// the caller may reuse the slice.
func (s *StreamSynth) push(col []complex128) complex128 {
	copy(s.ring[int(s.p%int64(s.dv))], col)
	bb := s.projAt(s.gt, s.phvec, s.p)
	// NCO by phasor recurrence (bin carrier + fine tune = exp(-2i*pi*p*phi)); one
	// complex multiply per block instead of a transcendental.
	bb *= s.nco
	s.nco *= s.ncoStep
	s.p++
	return bb
}

// pushPhases stores column col and appends the `phases` settled output samples for
// the current block to *out (nothing during the D-1 OLA warm-up, matching
// stream_dsp.StreamSynth).  Each output has the bin carrier removed and the fine
// tune applied at its absolute position.  col is copied, so it may be reused.
func (s *StreamSynth) pushPhases(col []complex128, out *[]complex128) {
	const N = nFFT
	p := s.p
	copy(s.ring[int(p%int64(len(s.ring)))], col)
	s.p++
	if p < s.emitBlk {
		return // warm-up: retained as history, emits nothing
	}
	for l := 0; l < s.phases; l++ {
		bb := s.projAt(s.gtP[l], s.phvecP[l], p)
		// bin carrier at absolute full-rate position n = p*R + offs[l]; reduce n mod N
		// first (overflow-safe over long streams, bit-identical to (kc*n)%N).
		n := (p*int64(rHop) + int64(s.offs[l])) % int64(N)
		ang := -2 * math.Pi * float64((int64(s.kc)*n)%int64(N)) / float64(N)
		if s.tuneHz != 0 {
			ang += -2 * math.Pi * s.tuneHz * (float64(p)*float64(s.phases) + float64(l)) / s.rate
		}
		*out = append(*out, bb*cmplx.Exp(complex(0, ang)))
	}
}

// complexResampler is a phase-continuous exact-ratio resampler for complex input,
// running two StreamResamplers (real, imag) in lockstep -- identical ratio + state
// progression, so they emit the same count each call.  Used by the HPSDR emitter to
// take a DDC's baseband from phases*40 kHz to the client's requested rate.
type complexResampler struct{ re, im *StreamResampler }

func newComplexResampler(rateIn float64, rateOut int) *complexResampler {
	return &complexResampler{re: newStreamResampler(rateIn, rateOut), im: newStreamResampler(rateIn, rateOut)}
}

func (c *complexResampler) process(x []complex128) []complex128 {
	var orr, oii []float64
	for _, v := range x {
		c.re.push(real(v), &orr)
		c.im.push(imag(v), &oii)
	}
	n := len(orr)
	if len(oii) < n {
		n = len(oii)
	}
	out := make([]complex128, n)
	for i := 0; i < n; i++ {
		out[i] = complex(orr[i], oii[i])
	}
	return out
}

// StreamResampler is a phase-continuous exact-ratio resampler (rateIn -> 12 kHz)
// that reproduces resamplePoly over the growing input, sample-exact.
type StreamResampler struct {
	up, down int
	h        []float64
	l        int       // len(h)
	buf      []float64 // input; buf[0] is absolute input index base
	base     int       // absolute index of buf[0]
	ninRecv  int       // total inputs appended (absolute)
	nextN    int       // next output index to emit
}

// newStreamResampler resamples rateIn -> rateOut using the exact reduced ratio
// (same up/down as toWavRate), so streaming and batch stay bit-identical.
func newStreamResampler(rateIn float64, rateOut int) *StreamResampler {
	up, down := resampleRatio(rateIn, rateOut)
	h := lowpassFIR(up, down)
	return &StreamResampler{up: up, down: down, h: h, l: len(h)}
}

// jloOf mirrors resamplePoly's lower tap bound for output n, before the >=0 clamp.
func (r *StreamResampler) jloOf(n int) int {
	base := n * r.down
	jlo := (base - (r.l - 1) + r.up - 1) / r.up
	if jlo < 0 {
		jlo = 0
	}
	return jlo
}

// compute evaluates output sample n from the current buffer (jhi clamped to the
// last received input, exactly like resamplePoly clamps to nin-1).
func (r *StreamResampler) compute(n int) float64 {
	base := n * r.down
	jlo := r.jloOf(n)
	jhi := base / r.up
	if jhi > r.ninRecv-1 {
		jhi = r.ninRecv - 1
	}
	var acc float64
	for j := jlo; j <= jhi; j++ {
		acc += r.h[base-r.up*j] * r.buf[j-r.base]
	}
	return acc
}

// push appends one input sample and appends every now-settled output to *out.
// An output is settled once all inputs it needs (jhi, unclamped) have arrived, so
// its value can never change -> identical to the batch result.
func (r *StreamResampler) push(x float64, out *[]float64) {
	r.buf = append(r.buf, x)
	r.ninRecv++
	// largest n with jhi(n)=floor(n*down/up) <= ninRecv-1
	maxN := ((r.ninRecv - 1) * r.up) / r.down
	for r.nextN <= maxN {
		*out = append(*out, r.compute(r.nextN))
		r.nextN++
	}
	r.trim()
}

// flush emits the remaining outputs up to nout, using the implicit-zero tail
// (jhi clamped) -- the finite-window edge resamplePoly produces at the end.
func (r *StreamResampler) flush(out *[]float64) {
	nout := (r.ninRecv*r.up + r.down - 1) / r.down
	for r.nextN < nout {
		*out = append(*out, r.compute(r.nextN))
		r.nextN++
	}
}

// trim drops input history no future output can reach (jlo is non-decreasing in n).
func (r *StreamResampler) trim() {
	jlo := r.jloOf(r.nextN)
	if jlo > r.base && jlo-r.base <= len(r.buf) {
		r.buf = r.buf[jlo-r.base:]
		r.base = jlo
	}
}

// --- StreamChannel: UTC window cutting on top of the streaming kernels ---------
//
// A StreamChannel is fed one bin-column per block (via pushBlock, absolute block
// index) and produces one peak-normalised int16 WAV per UTC capture window, handed
// to a dispatch callback.  It runs in RESET mode: a fresh StreamSynth + StreamResampler
// per window, fed exactly the window's [m0,m1) blocks, so each window's audio is
// BIT-IDENTICAL to the batch channelBaseband -> toWav12k over the same block range
// (and therefore identical spots to the validated batch farm).  No 140 s bin ring
// and no whole-window acc[nb] -- only the tiny 12 kHz audio buffer of the one active
// window lives at a time.

// StreamChannel synthesises + windows one configured channel.  In dual mode
// (adc=3, diversity) it carries a second synthesis chain fed from ADC B on the
// same bin: both chains share this one channel's t0, window boundaries, block
// sequence and gap zero-fill, so ADC A (left) and ADC B (right) come out
// sample-for-sample time-locked and phase-coherent -- written as one stereo WAV.
type StreamChannel struct {
	ch       Channel
	dec      Decoder
	kc       int
	tune     float64
	adc      int
	kaSub    []int
	t0       float64 // shared anchor: UTC = t0 + blk/binRate
	period   float64 // window period (s)
	capture  float64 // window capture length (s)
	dispatch func(audioL, audioR []float64, utc string, ch Channel, d Decoder)

	// dual (adc=3) diversity: second chain on ADC B, same kc/tune.
	dual   bool
	kaSubR []int

	// active window state (nil/false between windows)
	active  bool
	curWi   int64
	m0, m1  int64
	nextBlk int64 // next absolute block expected inside the active window
	synth   *StreamSynth
	rsmp    *StreamResampler
	audio   []float64
	synthR  *StreamSynth     // dual: ADC B chain
	rsmpR   *StreamResampler // dual
	audioR  []float64        // dual
	zero    []complex128     // reusable zero column for gap fill (ADC A)
	zeroR   []complex128     // reusable zero column for gap fill (ADC B, dual)

	// windowOf cache: the last resolved window covers blocks [cM0, cNext) (capture
	// [cM0, cM1) then dead time up to the next window), so windowOf recomputes the
	// Floor + blockRange only once per period instead of once per block.
	cValid          bool
	cWi             int64
	cM0, cM1, cNext int64
}

func newStreamChannel(ch Channel, dec Decoder, kc int, tune float64, adc int, kaSub []int,
	t0 float64, dispatch func(audioL, audioR []float64, utc string, ch Channel, d Decoder)) *StreamChannel {
	return &StreamChannel{
		ch: ch, dec: dec, kc: kc, tune: tune, adc: adc, kaSub: kaSub,
		t0: t0, period: dec.PeriodS, capture: dec.CaptureS, dispatch: dispatch,
		zero: make([]complex128, len(kaSub)),
	}
}

// newStreamChannelDual builds a diversity channel (adc=3): ADC A on kaSubA and
// ADC B on kaSubB, same bin kc and fine tune.  finalize hands the worker both
// audio streams for one stereo WAV.
func newStreamChannelDual(ch Channel, dec Decoder, kc int, tune float64, kaSubA, kaSubB []int,
	t0 float64, dispatch func(audioL, audioR []float64, utc string, ch Channel, d Decoder)) *StreamChannel {
	return &StreamChannel{
		ch: ch, dec: dec, kc: kc, tune: tune, adc: 3, kaSub: kaSubA,
		dual: true, kaSubR: kaSubB,
		t0: t0, period: dec.PeriodS, capture: dec.CaptureS, dispatch: dispatch,
		zero: make([]complex128, len(kaSubA)), zeroR: make([]complex128, len(kaSubB)),
	}
}

// windowOf maps an absolute block index to its capture window (start block m0,
// end block m1, and whether the block falls inside the capture, using the exact
// same block<->UTC math as the batch blockRange).  Cached per period: a block in
// the last window's [cM0, cNext) span is classified without recomputing.
func (s *StreamChannel) windowOf(blk int64) (wi, m0, m1 int64, in bool) {
	if s.cValid && blk >= s.cM0 && blk < s.cNext {
		return s.cWi, s.cM0, s.cM1, blk < s.cM1
	}
	utc := s.t0 + float64(blk)/binRate
	wi = int64(math.Floor(utc / s.period))
	start := float64(wi) * s.period
	m0, m1 = blockRange(s.t0, start, s.capture)
	nextM0, _ := blockRange(s.t0, float64(wi+1)*s.period, s.capture)
	s.cValid, s.cWi, s.cM0, s.cM1, s.cNext = true, wi, m0, m1, nextM0
	in = blk >= m0 && blk < m1
	return
}

// pushBlock feeds column col (the channel's kaSub bins for absolute block blk).
// col is copied by the synth, so the caller may reuse it.
func (s *StreamChannel) pushBlock(blk int64, col []complex128) {
	s.push(blk, col, nil)
}

// pushBlockDual feeds a diversity channel: colA (ADC A) and colB (ADC B) for the
// SAME absolute block blk.  Because both chains advance on this one block sequence,
// they stay perfectly time-aligned.  colA/colB are copied by the synths.
func (s *StreamChannel) pushBlockDual(blk int64, colA, colB []complex128) {
	s.push(blk, colA, colB)
}

// push is the shared window/gap logic for both mono (colB == nil) and dual channels.
func (s *StreamChannel) push(blk int64, colA, colB []complex128) {
	wi, m0, m1, in := s.windowOf(blk)
	if s.active && (!in || wi != s.curWi) {
		s.finalize() // previous window ended (gap after capture, or a new window)
	}
	if !in {
		return // block lies in the dead time between capture end and next period
	}
	if !s.active {
		s.start(wi, m0, m1) // nextBlk = m0; a mid-window join zero-fills the lead-in
	}
	if blk < s.nextBlk {
		return // stale/reordered block already past -> ignore
	}
	// Forward gap (UDP loss) or mid-window join: zero-fill BOTH chains so the audio
	// timeline stays UTC-aligned and the two antennas stay sample-locked.
	for s.nextBlk < blk {
		s.feed(s.zero, s.zeroR)
	}
	s.feed(colA, colB)
	if s.nextBlk >= s.m1 {
		s.finalize()
	}
}

// setT0 re-anchors the block<->UTC map (used on a stream discontinuity).  Only
// valid between windows (the caller resets active windows first).
func (s *StreamChannel) setT0(t0 float64) {
	s.t0 = t0
	s.cValid = false // block<->UTC map changed; drop the windowOf cache
}

// reset drops the active window without dispatching it (garbage after a big seq
// jump / re-anchor).
func (s *StreamChannel) reset() {
	s.active = false
	s.synth, s.rsmp, s.audio = nil, nil, nil
	s.synthR, s.rsmpR, s.audioR = nil, nil, nil
}

func (s *StreamChannel) start(wi, m0, m1 int64) {
	s.active = true
	s.curWi, s.m0, s.m1, s.nextBlk = wi, m0, m1, m0
	s.synth = newStreamSynth(s.kaSub, s.kc, s.tune)
	s.rsmp = newStreamResampler(binRate, s.dec.rate())
	s.audio = s.audio[:0]
	if s.dual {
		s.synthR = newStreamSynth(s.kaSubR, s.kc, s.tune)
		s.rsmpR = newStreamResampler(binRate, s.dec.rate())
		s.audioR = s.audioR[:0]
	}
}

// feed advances both chains by one block (colB ignored when not dual).  The two
// chains are identical apart from their input bins, so they emit the same number
// of audio samples per block -> left/right stay length-matched and time-locked.
func (s *StreamChannel) feed(colA, colB []complex128) {
	bb := s.synth.push(colA)
	s.rsmp.push(real(bb), &s.audio)
	if s.dual {
		bbR := s.synthR.push(colB)
		s.rsmpR.push(real(bbR), &s.audioR)
	}
	s.nextBlk++
}

func (s *StreamChannel) finalize() {
	if !s.active {
		return
	}
	s.rsmp.flush(&s.audio)
	audio := s.audio // hand ownership to the worker; peak-normalise off the ingest thread
	var audioR []float64
	if s.dual {
		s.rsmpR.flush(&s.audioR)
		audioR = s.audioR
	}
	start := float64(s.curWi) * s.period
	// Full window-start stamp YYMMDD_HHMMSS (e.g. 260927_183000): used verbatim for
	// the decoder WAV filename; trimmed to HHMMSS for the spot display and {utc}.
	utc := time.Unix(int64(start), 0).UTC().Format("060102_150405")
	s.active = false
	s.synth, s.rsmp, s.audio = nil, nil, nil
	s.synthR, s.rsmpR, s.audioR = nil, nil, nil
	if s.dispatch != nil {
		s.dispatch(audio, audioR, utc, s.ch, s.dec)
	}
}

// finish flushes the currently open window (end of stream / replay file).
func (s *StreamChannel) finish() { s.finalize() }

// resolveChannelRows finds the covering run for (kc,adc) and returns the row
// indices into the ADC's bin list (ka for adc 1, kb for adc 2) plus those bins'
// absolute indices, exactly as channelBaseband selects them.  Works off raw
// runs/ka/kb so it serves both the offline Captured and the live Meta.
func resolveChannelRows(runs []Run, ka, kb []int, kc, adc int) (rowIdx, kaSub []int, ok bool) {
	k0, w, ok := coveringRun(runs, kc, adc)
	if !ok {
		return nil, nil, false
	}
	kl := ka
	if adc == 2 {
		kl = kb
	}
	for j, k := range kl {
		if k >= k0 && k < k0+w {
			rowIdx = append(rowIdx, j)
			kaSub = append(kaSub, k)
		}
	}
	return rowIdx, kaSub, len(rowIdx) > 0
}

// channelRows resolves a channel against a captured window (the Sa/Sb rows and
// their absolute bin indices), exactly as channelBaseband selects them.
func channelRows(cap *Captured, kc, adc int) (rowIdx, kaSub []int, S [][]complex128, ok bool) {
	rowIdx, kaSub, ok = resolveChannelRows(cap.Runs, cap.Ka, cap.Kb, kc, adc)
	if !ok {
		return nil, nil, nil, false
	}
	S = cap.Sa
	if adc == 2 {
		S = cap.Sb
	}
	return rowIdx, kaSub, S, true
}

// --- Dispatcher: a bounded job QUEUE drained by a fixed worker pool ------------
//
// All the channels of a mode finalise at the same UTC boundary (and FT8/FT4/WSPR
// boundaries coincide at :00), so up to N windows land at once.  A decode takes
// ~1-2 s while the shortest period is 7.5-15 s, so a queue drains comfortably --
// far better than dropping the instant every worker is momentarily busy.  Live
// ingest never blocks: submit drops only if the queue is genuinely full (real
// overload).  Replay blocks on submit so the offline diff is lossless.

type winJob struct {
	audio  []float64 // ADC A / left
	audioR []float64 // dual (adc=3): ADC B / right; nil for mono
	utc    string
	ch     Channel
	dec    Decoder
}

type Dispatcher struct {
	cfg   Config
	jobs  chan winJob
	wg    sync.WaitGroup
	block bool  // replay: block on submit (never drop); live: false
	drops int64 // windows dropped on queue overflow (atomic)
}

func newDispatcher(cfg Config, block bool, queueDepth int) *Dispatcher {
	nw := maxInt(1, cfg.Workers)
	d := &Dispatcher{cfg: cfg, jobs: make(chan winJob, maxInt(queueDepth, nw)), block: block}
	for i := 0; i < nw; i++ {
		d.wg.Add(1)
		go d.worker()
	}
	return d
}

func (d *Dispatcher) worker() {
	defer d.wg.Done()
	for j := range d.jobs {
		// off the ingest thread: peak-normalise, then one mono or stereo WAV.
		var pcm []int16
		numCh := 1
		if j.audioR != nil {
			pcm = peakNormalizeStereoInt16(j.audio, j.audioR) // L=ADC A, R=ADC B
			numCh = 2
		} else {
			pcm = peakNormalizeInt16(j.audio)
		}
		out, err := runDecoder(pcm, numCh, j.ch, j.dec, j.utc, d.cfg.SaveWav)
		if err != nil && out == "" {
			continue
		}
		for _, s := range j.dec.parse(out, j.ch, j.utc) {
			emit(s)
		}
	}
}

func (d *Dispatcher) submit(audio, audioR []float64, utc string, ch Channel, dec Decoder) {
	job := winJob{audio: audio, audioR: audioR, utc: utc, ch: ch, dec: dec}
	if d.block {
		d.jobs <- job
		return
	}
	select {
	case d.jobs <- job:
	default:
		atomic.AddInt64(&d.drops, 1)
		fmt.Fprintf(os.Stderr, "dispatch: queue full, dropping %s %s window\n", ch.name(), utc)
	}
}

// wait closes the queue and waits for the workers to drain it.
func (d *Dispatcher) wait() {
	close(d.jobs)
	d.wg.Wait()
}

func (d *Dispatcher) dropped() int64 { return atomic.LoadInt64(&d.drops) }
