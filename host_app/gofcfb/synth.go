// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Per-channel synthesis: reconstruct a channel's complex baseband from the kept
// FFT bins, NCO it to DC, and resample to 12 kHz real audio for jt9/wsprd.
// Port of host_app/decode.channel_baseband + synth.synth_channel (phases=1, the
// narrow-channel case FT8/WSPR use) + decode.to_wav12k.
//
// The synth is written as a scatter FIR so no full-window tap matrix is ever
// materialised (the Python whole-window path cost ~2.5 GB for a WSPR window; this
// peaks at one output buffer).
package fcfb

import (
	"math"
	"math/cmplx"
)

// coveringRun returns the (k0,W) run on ADC `adc` (1=A,2=B) containing bin kc.
func coveringRun(runs []Run, kc, adc int) (k0, w int, ok bool) {
	for _, r := range runs {
		ab := r.Adc
		if ab == 0 {
			ab = 1
		}
		if ab&adc != 0 && kc >= r.K0 && kc < r.K0+r.W {
			return r.K0, r.W, true
		}
	}
	return 0, 0, false
}

// channelBaseband reconstructs the channel centred on bin kc from a captured
// window on one ADC, returning complex baseband at 40 kHz (phases=1), or nil if
// the channel's run is not present.
func channelBaseband(cap *Captured, kc, adc int, tuneHz float64) []complex128 {
	k0, w, ok := coveringRun(cap.Runs, kc, adc)
	if !ok {
		return nil
	}
	ka := cap.Ka
	S := cap.Sa
	if adc == 2 {
		ka = cap.Kb
		S = cap.Sb
	}
	var rows [][]complex128
	var kaSub []int
	for j, k := range ka {
		if k >= k0 && k < k0+w {
			rows = append(rows, S[j])
			kaSub = append(kaSub, k)
		}
	}
	if len(rows) == 0 {
		return nil
	}
	return synthChannel(rows, kaSub, kc, tuneHz)
}

func synthChannel(S [][]complex128, kaSub []int, kc int, tuneHz float64) []complex128 {
	const R = rHop
	const N = nFFT
	W := len(S)
	nb := len(S[0])
	D := (len(gDual) + R - 1) / R // 8

	// taps: s[d] = d*R (phi=0); gt[d]=g[s]; phvec[d][j]=exp(2i*pi*ka[j]*(s%N)/N)/N
	var gt []complex128
	var phvec [][]complex128
	for d := 0; d < D; d++ {
		s := d * R
		if s >= len(gDual) {
			break
		}
		gt = append(gt, gDual[s])
		pv := make([]complex128, W)
		sm := s % N
		for j := 0; j < W; j++ {
			ang := 2 * math.Pi * float64(kaSub[j]*sm) / float64(N)
			pv[j] = cmplx.Exp(complex(0, ang)) / complex(float64(N), 0)
		}
		phvec = append(phvec, pv)
	}
	Dv := len(gt)

	// scatter FIR: acc[m'+d] += gt[d] * (phvec[d] . S[:,m'])
	acc := make([]complex128, nb)
	col := make([]complex128, W)
	for m := 0; m < nb; m++ {
		for j := 0; j < W; j++ {
			col[j] = S[j][m]
		}
		for d := 0; d < Dv; d++ {
			if m+d >= nb {
				break
			}
			var proj complex128
			pv := phvec[d]
			for j := 0; j < W; j++ {
				proj += pv[j] * col[j]
			}
			acc[m+d] += gt[d] * proj
		}
	}

	// NCO: remove the bin carrier exp(-2i*pi*(kc*m*R mod N)/N), then fine tune.
	for m := 0; m < nb; m++ {
		n := (int64(kc) * int64(m) * int64(R)) % int64(N)
		ang := -2 * math.Pi * float64(n) / float64(N)
		if tuneHz != 0 {
			ang += -2 * math.Pi * tuneHz * float64(m) / binRate
		}
		acc[m] *= cmplx.Exp(complex(0, ang))
	}
	return acc
}

// gcdInt is the greatest common divisor (Euclid).
func gcdInt(a, b int) int {
	for b != 0 {
		a, b = b, a%b
	}
	if a < 0 {
		a = -a
	}
	return a
}

// resampleRatio reduces rateOut/rateIn to lowest terms (up/down), so the resampler
// hits rateOut exactly for any integer rate.  For 12000 from 40000 this is 3/10 --
// identical to the original fixed choice, so the default stays bit-exact.
func resampleRatio(rateIn float64, rateOut int) (up, down int) {
	ri := int(math.Round(rateIn))
	g := gcdInt(rateOut, ri)
	if g == 0 {
		g = 1
	}
	return rateOut / g, ri / g
}

// toWavRate takes complex baseband at rateIn (a multiple of 40 kHz) and returns
// real int16 audio at rateOut (peak-normalised).
func toWavRate(bb []complex128, rateIn float64, rateOut int) []int16 {
	up, down := resampleRatio(rateIn, rateOut)
	x := make([]float64, len(bb))
	for i := range bb {
		x[i] = real(bb[i])
	}
	y := resamplePoly(x, up, down)
	return peakNormalizeInt16(y)
}

// toWav12k is toWavRate at the default 12 kHz (used by tests and the batch path's
// bit-exact reference).
func toWav12k(bb []complex128, rateIn float64) []int16 {
	return toWavRate(bb, rateIn, 12000)
}

// peakNormalizeInt16 peak-normalises a real signal to int16 (peak -> 24000), the
// final step of to_wav12k.  Shared with the streaming path so both produce byte-
// identical WAV audio.
func clampInt16(s float64) int16 {
	if s > 32767 {
		s = 32767
	} else if s < -32768 {
		s = -32768
	}
	return int16(math.Round(s))
}

func peakNormalizeInt16(y []float64) []int16 {
	peak := 1e-30
	for _, v := range y {
		if a := math.Abs(v); a > peak {
			peak = a
		}
	}
	out := make([]int16, len(y))
	for i, v := range y {
		out[i] = clampInt16(v / peak * 24000)
	}
	return out
}

// peakNormalizeStereoInt16 interleaves two time-aligned audio streams (L=ADC A,
// R=ADC B) into one stereo int16 buffer, normalised JOINTLY by the single peak
// across both channels.  A shared scale factor preserves the relative amplitude
// (and, with the phase-coherent synthesis, the relative phase) between the two
// antennas -- the information a future diversity combiner needs.  jt9/wsprd
// renormalise each mono split internally, so a weaker channel still decodes.
func peakNormalizeStereoInt16(l, r []float64) []int16 {
	n := len(l)
	if len(r) < n {
		n = len(r) // equal by construction; guard against a truncated flush
	}
	peak := 1e-30
	for i := 0; i < n; i++ {
		if a := math.Abs(l[i]); a > peak {
			peak = a
		}
		if a := math.Abs(r[i]); a > peak {
			peak = a
		}
	}
	out := make([]int16, 2*n)
	for i := 0; i < n; i++ {
		out[2*i] = clampInt16(l[i] / peak * 24000)
		out[2*i+1] = clampInt16(r[i] / peak * 24000)
	}
	return out
}

// interleaveF32 interleaves equal-length channel streams into one float32 buffer
// (channel-minor): out[i*n+c] = chans[c][i].  Used to build the farm's I/Q WAV --
// [I,Q] for a single ADC, [I_A,Q_A,I_B,Q_B] for dual -- with no normalisation, so
// inter-antenna amplitude and phase are preserved.
func interleaveF32(chans ...[]float64) []float32 {
	nc := len(chans)
	n := len(chans[0])
	for _, c := range chans {
		if len(c) < n {
			n = len(c) // equal by construction; guard a truncated flush
		}
	}
	out := make([]float32, n*nc)
	for i := 0; i < n; i++ {
		for c := 0; c < nc; c++ {
			out[i*nc+c] = float32(chans[c][i])
		}
	}
	return out
}

// resamplePoly is an efficient rational resampler (upfirdn without materialising
// the zero-stuffed signal): y[n] = sum_j h[n*down - up*j] * x[j].
func resamplePoly(x []float64, up, down int) []float64 {
	h := lowpassFIR(up, down)
	L := len(h)
	nin := len(x)
	nout := (nin*up + down - 1) / down
	y := make([]float64, nout)
	for n := 0; n < nout; n++ {
		base := n * down
		jlo := (base - (L - 1) + up - 1) / up // ceil((base-L+1)/up)
		if jlo < 0 {
			jlo = 0
		}
		jhi := base / up
		if jhi > nin-1 {
			jhi = nin - 1
		}
		var acc float64
		for j := jlo; j <= jhi; j++ {
			acc += h[base-up*j] * x[j]
		}
		y[n] = acc
	}
	return y
}

// resampHalf sets the resampler filter length: L = 2*resampHalf*max(up,down)+1
// taps (per-output cost ~= L/up MACs).  The 4-term Blackman-Harris window fixes
// the stopband (~-92 dB) regardless of length; resampHalf only trades transition-
// band width (near the output Nyquist, well above the ~1500 Hz signal band) for CPU.
// 10 -> 201 taps @ 40k->12k (was 16/321).  Verified safe: the 0-4.5 kHz response
// is identical to the 321-tap filter to <1e-4 dB (they diverge only above 4.5 kHz,
// out of band for FT8/WSPR), and a board A/B (weak -20 dB FT8 replayed through both
// lengths) decoded bit-identically.  Cuts ~23% off the resampler (~4-5% farm CPU).
const resampHalf = 10

// lowpassFIR designs a windowed-sinc anti-imaging/anti-alias filter for a
// rational resampler (cutoff = 1/max(up,down) of the upsampled Nyquist, gain=up).
// 4-term Blackman-Harris window (~-92 dB stopband) -- keeps weak signals clean.
func lowpassFIR(up, down int) []float64 {
	m := up
	if down > m {
		m = down
	}
	fc := 1.0 / float64(m) // fraction of the upsampled Nyquist
	L := 2*resampHalf*m + 1
	c := float64(L-1) / 2
	h := make([]float64, L)
	var sum float64
	for i := 0; i < L; i++ {
		t := float64(i) - c
		s := fc * sinc(fc*t)
		w := blackmanHarris(i, L)
		h[i] = s * w
		sum += h[i]
	}
	// normalise to unity DC gain, then scale by up (upsampling energy).
	g := float64(up) / sum
	for i := range h {
		h[i] *= g
	}
	return h
}

func sinc(x float64) float64 {
	if x == 0 {
		return 1
	}
	px := math.Pi * x
	return math.Sin(px) / px
}

func blackmanHarris(i, L int) float64 {
	const a0, a1, a2, a3 = 0.35875, 0.48829, 0.14128, 0.01168
	w := 2 * math.Pi * float64(i) / float64(L-1)
	return a0 - a1*math.Cos(w) + a2*math.Cos(2*w) - a3*math.Cos(3*w)
}
