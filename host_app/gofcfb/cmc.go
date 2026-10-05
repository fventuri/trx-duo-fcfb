// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// common-mode-cancelling: like wav-to-decoder, but for a dual-ADC diversity capture
// it decodes a THIRD channel in addition to the two antennas -- the frequency-flat
// common-mode-cancelled channel
//
//	C = B - β·A       (--ref A, default)   β = Σ conj(A)·B / Σ|A|²
//	C = A - β·B       (--ref B)            β = Σ conj(B)·A / Σ|B|²
//
// β is a single complex scalar estimated over the whole window.  The two TRX-duo RX
// channels share a largely common-mode noise/interference (the broadband bins that
// dominate the β sum), so B - β·A nulls that common mode while leaving spatially
// diverse signals; on real captures this roughly doubles the weak-signal yield.
// Because β is one LTI complex scalar (not per-bin, not time-varying) the operation
// preserves absolute phase and timing, so it is safe for every coherent decoder
// (jt9 FT8/FT4/FST4, wsprd) -- the canceller knows nothing about the modulation.
//
// --trim F (default 0.05) estimates β from noise-only cells: the top F fraction of
// STFT cells by power |R|²+|O|² (where strong signals concentrate) are excluded from
// the β sums, so on signal-rich bands β tracks the broadband common mode instead of
// being pulled by a bright wanted signal that would then land in the null (doc 13
// enhancement A).  β is still one scalar and C = B - β·A is still formed in the time
// domain; only the estimate is cleaned.  --trim 0 restores the exact all-sample
// time-domain β (Σ conj(R)·O / Σ|R|², identical by Parseval to the untrimmed STFT).
//
// It emits all three decodes tagged with "#ANT A", "#ANT B", "#ANT CMC", so
// fcfbfarm's parser merges them (A, B and CMC land as one channel's spots).  The
// canceller ALONE is band-dependent (it can null a band's strong signal), so CMC is
// always ADDITIVE to A/B here, never a replacement.
//
// It needs a dual-ADC I/Q WAV (32-bit float, 4 channels [I_A,Q_A,I_B,Q_B]) -- the
// only format with two antennas to cancel -- and hard-fails on anything else.
//
// Used as an fcfbfarm [decoder] cmd (the farm substitutes {wav}), exactly like
// wav-to-decoder:
//
//	cmd = common-mode-cancelling jt9 --ft8 {wav}
//	cmd = common-mode-cancelling --ref B jt9 --ft4 {wav}
//	cmd = common-mode-cancelling wsprd -f {fmhz} {wav}
//
// or by hand on a saved file.  See cmd/common-mode-cancelling and wavdecode.go (the
// WAV read, I-channel extraction, peak-normalise and per-antenna decode it reuses).
package fcfb

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
)

// CommonModeCancellingMain is the common-mode-cancelling entry point (see
// cmd/common-mode-cancelling).  It parses the leading --ref/--verbose options and
// passes the rest (the decoder argv, with {wav} already substituted by the farm) on.
func CommonModeCancellingMain() {
	args := os.Args[1:]
	ref := "A"        // reference antenna: A => C = B-β·A ; B => C = A-β·B
	trim := 0.05      // signal-excluded β: drop top-5% STFT cells by power (doc 13 enh. A); 0 = legacy time-domain β
	verbose := false
	parseTrim := func(s string) float64 {
		v, err := strconv.ParseFloat(s, 64)
		if err != nil || v < 0 || v >= 1 {
			fmt.Fprintf(os.Stderr, "common-mode-cancelling: --trim needs a fraction in [0,1), got %q\n", s)
			os.Exit(2)
		}
		return v
	}
	for len(args) > 0 && strings.HasPrefix(args[0], "-") && args[0] != "--" {
		switch {
		case args[0] == "--ref":
			if len(args) < 2 {
				fmt.Fprintln(os.Stderr, "common-mode-cancelling: --ref needs an argument (A or B)")
				os.Exit(2)
			}
			ref = strings.ToUpper(args[1])
			args = args[2:]
		case strings.HasPrefix(args[0], "--ref="):
			ref = strings.ToUpper(strings.TrimPrefix(args[0], "--ref="))
			args = args[1:]
		case args[0] == "--trim":
			if len(args) < 2 {
				fmt.Fprintln(os.Stderr, "common-mode-cancelling: --trim needs an argument (e.g. 0.05)")
				os.Exit(2)
			}
			trim = parseTrim(args[1])
			args = args[2:]
		case strings.HasPrefix(args[0], "--trim="):
			trim = parseTrim(strings.TrimPrefix(args[0], "--trim="))
			args = args[1:]
		case args[0] == "-v" || args[0] == "--verbose":
			verbose = true
			args = args[1:]
		default:
			fmt.Fprintf(os.Stderr, "common-mode-cancelling: unknown option %q\n", args[0])
			os.Exit(2)
		}
	}
	out, rc, err := runCommonModeCancelling(args, ref, trim, verbose)
	if err != nil {
		fmt.Fprintln(os.Stderr, "common-mode-cancelling:", err)
		os.Exit(2)
	}
	fmt.Print(out)
	os.Exit(rc)
}

// cmcForm renders the canceller expression for messages.
func cmcForm(ref string) string {
	if ref == "B" {
		return "A - β·B"
	}
	return "B - β·A"
}

// STFT parameters for the signal-excluded β estimator (match tools/ft8dual.py).
const (
	cmcNFFT = 1024 // samples per frame
	cmcHop  = 256  // frame step (nperseg-noverlap)
)

// fft computes the in-place radix-2 decimation-in-time FFT of x; len(x) must be a
// power of two.  Self-contained (stdlib math only) so the package keeps zero deps.
func fft(x []complex128) {
	n := len(x)
	for i, j := 1, 0; i < n; i++ { // bit-reversal permutation
		bit := n >> 1
		for ; j&bit != 0; bit >>= 1 {
			j ^= bit
		}
		j ^= bit
		if i < j {
			x[i], x[j] = x[j], x[i]
		}
	}
	for length := 2; length <= n; length <<= 1 {
		ang := -2 * math.Pi / float64(length)
		half := length >> 1
		for i := 0; i < n; i += length {
			for k := 0; k < half; k++ {
				s, c := math.Sincos(ang * float64(k))
				v := x[i+k+half] * complex(c, s)
				u := x[i+k]
				x[i+k] = u + v
				x[i+k+half] = u - v
			}
		}
	}
}

// hannWindow returns the length-n periodic Hann window (scipy get_window('hann', n)).
func hannWindow(n int) []float64 {
	w := make([]float64, n)
	for i := range w {
		w[i] = 0.5 - 0.5*math.Cos(2*math.Pi*float64(i)/float64(n))
	}
	return w
}

// betaTrimmed estimates the frequency-flat β from an STFT, excluding the top `trim`
// fraction of cells by power |R|²+|O|² (where strong signals concentrate), so β
// tracks the broadband common mode rather than a bright wanted signal (doc 13
// enhancement A).  R/O are the reference/other antennas at interleave offsets
// rOff/oOff.  β is still a single complex scalar; the caller forms C = O - β·R in the
// time domain exactly as for the untrimmed estimator.
func betaTrimmed(d *WavData, rOff, oOff int, trim float64) (complex128, error) {
	n := len(d.F32) / 4
	if n < cmcNFFT {
		return 0, fmt.Errorf("window too short for STFT β (%d < %d samples)", n, cmcNFFT)
	}
	win := hannWindow(cmcNFFT)
	m := 1 + (n-cmcNFFT)/cmcHop // number of frames
	ncell := m * cmcNFFT
	pow := make([]float64, ncell)    // |R|²+|O|² per cell (for the trim threshold)
	cRO := make([]complex128, ncell) // conj(R)·O per cell
	rr := make([]float64, ncell)     // |R|² per cell
	R := make([]complex128, cmcNFFT)
	O := make([]complex128, cmcNFFT)
	for f := 0; f < m; f++ {
		s := f * cmcHop
		for i := 0; i < cmcNFFT; i++ {
			b := 4 * (s + i)
			w := win[i]
			R[i] = complex(float64(d.F32[b+rOff])*w, float64(d.F32[b+rOff+1])*w)
			O[i] = complex(float64(d.F32[b+oOff])*w, float64(d.F32[b+oOff+1])*w)
		}
		fft(R)
		fft(O)
		for k := 0; k < cmcNFFT; k++ {
			idx := f*cmcNFFT + k
			rk, ok := R[k], O[k]
			rrk := real(rk)*real(rk) + imag(rk)*imag(rk)
			ook := real(ok)*real(ok) + imag(ok)*imag(ok)
			rr[idx] = rrk
			// conj(R)·O = (Re R·Re O + Im R·Im O) + j(Re R·Im O - Im R·Re O)
			cRO[idx] = complex(real(rk)*real(ok)+imag(rk)*imag(ok),
				real(rk)*imag(ok)-imag(rk)*real(ok))
			pow[idx] = rrk + ook
		}
	}
	// threshold at the (1-trim) quantile of cell power; keep cells strictly below it
	sorted := make([]float64, ncell)
	copy(sorted, pow)
	sort.Float64s(sorted)
	thr := sorted[int(float64(ncell-1)*(1-trim))]
	var sumRR float64
	var sumRO complex128
	for idx := 0; idx < ncell; idx++ {
		if pow[idx] < thr {
			sumRR += rr[idx]
			sumRO += cRO[idx]
		}
	}
	if sumRR == 0 {
		return 0, fmt.Errorf("no cells survived trim=%g; cannot estimate β", trim)
	}
	return complex(real(sumRO)/sumRR, imag(sumRO)/sumRR), nil
}

// commonModeCancel forms the common-mode-cancelled channel of a 4-ch dual-ADC I/Q
// WavData and returns real(C) peak-normalised to int16 (the audio the decoder reads)
// together with the estimated β.  ref="A": C = B - β·A, β = Σ conj(A)·B / Σ|A|².
// ref="B": C = A - β·B, β = Σ conj(B)·A / Σ|B|².  With trim<=0 the sums run over every
// sample (the frequency-flat β is a time-domain complex dot product; no FFT needed);
// with trim>0 β comes from betaTrimmed (signal-excluded STFT estimate, doc 13 enh. A),
// but C = O - β·R is still synthesised in the time domain either way.
func commonModeCancel(d *WavData, ref string, trim float64) ([]int16, complex128, error) {
	if d.F32 == nil || d.NumCh != 4 {
		return nil, 0, fmt.Errorf("common-mode cancelling needs 4-channel float32 I/Q")
	}
	// Interleaving is [I_A, Q_A, I_B, Q_B]; R = reference antenna, O = the other.
	rOff, oOff := 0, 2 // ref A: R=A (I at 0), O=B (I at 2)
	if ref == "B" {
		rOff, oOff = 2, 0
	}
	n := len(d.F32) / 4

	var bRe, bIm float64
	if trim > 0 && n >= cmcNFFT {
		beta, err := betaTrimmed(d, rOff, oOff, trim)
		if err != nil {
			return nil, 0, err
		}
		bRe, bIm = real(beta), imag(beta)
	} else {
		var sumRR, reRO, imRO float64 // Σ|R|², Re(Σ conj(R)·O), Im(Σ conj(R)·O)
		for k := 0; k < n; k++ {
			b := 4 * k
			rr, ri := float64(d.F32[b+rOff]), float64(d.F32[b+rOff+1])
			or, oi := float64(d.F32[b+oOff]), float64(d.F32[b+oOff+1])
			sumRR += rr*rr + ri*ri
			reRO += rr*or + ri*oi // Re{(rr-j ri)(or+j oi)}
			imRO += rr*oi - ri*or // Im{(rr-j ri)(or+j oi)}
		}
		if sumRR == 0 {
			return nil, 0, fmt.Errorf("reference antenna %s is all zero; cannot estimate β", ref)
		}
		bRe, bIm = reRO/sumRR, imRO/sumRR
	}
	// real(C) = real(O) - real(β·R) = or - (bRe*rr - bIm*ri)
	out := make([]float64, n)
	for k := 0; k < n; k++ {
		b := 4 * k
		rr, ri := float64(d.F32[b+rOff]), float64(d.F32[b+rOff+1])
		out[k] = float64(d.F32[b+oOff]) - (bRe*rr - bIm*ri)
	}
	return PeakNormalizeInt16(out), complex(bRe, bIm), nil
}

// runCommonModeCancelling decodes the dual-ADC I/Q WAV named in args three times --
// antenna A, antenna B, and the common-mode-cancelled channel C -- each tagged with
// an "#ANT" marker, and returns the combined decoder output, an aggregate return
// code, and an error for setup problems.  ref selects the canceller form (see
// commonModeCancel); verbose prints the estimated β to stderr.
func runCommonModeCancelling(args []string, ref string, trim float64, verbose bool) (string, int, error) {
	if len(args) == 0 {
		return "", 2, fmt.Errorf("no decoder command given")
	}
	if ref != "A" && ref != "B" {
		return "", 2, fmt.Errorf("--ref must be A or B, got %q", ref)
	}
	wavIdx := findWavArg(args)
	if wavIdx < 0 {
		return "", 2, fmt.Errorf("no .wav argument found in: %s", strings.Join(args, " "))
	}
	wavPath, err := filepath.Abs(args[wavIdx])
	if err != nil {
		return "", 2, err
	}
	d, err := ReadWAV(wavPath)
	if err != nil {
		return "", 2, err
	}
	if d.FormatTag != wavFmtFloat || d.F32 == nil || d.NumCh != 4 {
		return "", 2, fmt.Errorf("%s: common-mode cancelling needs dual-ADC I/Q "+
			"(32-bit float, 4 channels [I_A,Q_A,I_B,Q_B]); got format tag %d, %d channel(s)",
			wavPath, d.FormatTag, d.NumCh)
	}

	cmcMono, beta, cerr := commonModeCancel(d, ref, trim)
	if cerr != nil {
		return "", 2, cerr
	}
	if verbose {
		fmt.Fprintf(os.Stderr, "common-mode-cancelling %s: C = %s, trim = %.3f, β = %.4f%+.4fj\n",
			filepath.Base(wavPath), cmcForm(ref), trim, real(beta), imag(beta))
	}

	dir, derr := os.MkdirTemp("", "cmc_")
	if derr != nil {
		return "", 2, derr
	}
	defer os.RemoveAll(dir)
	base := tempBase(d, wavPath) // the "...HHMM[SS].wav" tail the decoder parses for the window time

	// A, B, then CMC -- each split/synthesised into its own mono 16-bit temp WAV
	// (named so the decoder still reads the window time from the filename tail),
	// decoded in sequence and tagged with its #ANT marker.
	decodes := []struct {
		tag  string
		mono []int16
	}{
		{"A", extractMono(d, antSel{ant: "A", ch: 0, iq: true})},
		{"B", extractMono(d, antSel{ant: "B", ch: 2, iq: true})},
		{"CMC", cmcMono},
	}

	var sb strings.Builder
	rc := 0
	for _, dec := range decodes {
		tmp := filepath.Join(dir, dec.tag+"-"+base) // e.g. A-260927_1830.wav, CMC-260927_1830.wav
		if werr := WriteWAVInt16(tmp, d.Rate, 1, dec.mono); werr != nil {
			return sb.String(), 2, werr
		}
		fmt.Fprintf(&sb, "#ANT %s\n", dec.tag)
		out, orc := runOne(args, wavIdx, tmp, dir)
		sb.WriteString(out)
		if orc != 0 {
			rc = orc
		}
	}
	return sb.String(), rc, nil
}
