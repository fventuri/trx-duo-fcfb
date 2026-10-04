// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// wav-to-decoder: run a WSJT-X-style decoder (jt9/wsprd/...) on a WAV that fcfbfarm
// wrote, in either format.  It autodetects from the header:
//
//	PCM int16  1ch  -> mono real audio: run the decoder once, unchanged
//	PCM int16  2ch  -> dual real audio (L=ADC A, R=ADC B): split, decode each, #ANT
//	float32    2ch  -> single-ADC I/Q [I,Q]: decode real(I/Q)=I
//	float32    4ch  -> dual-ADC I/Q [I_A,Q_A,I_B,Q_B]: decode I_A and I_B, #ANT
//
// The decoders read mono 16-bit audio, so for I/Q we take the I channel (= the
// real part the farm's audio path would have produced) and peak-normalise it, and
// for dual we split into two sequential decodes -- exactly as the retired Python
// dual-decoder did, emitting the same "#ANT A/B" markers fcfbfarm's parser reads.
//
// Used as an fcfbfarm [decoder] cmd (the farm substitutes {wav}):
//
//	cmd = wav-to-decoder jt9 --ft8 {wav}
//	cmd = wav-to-decoder wsprd -f {fmhz} {wav}
//
// or by hand on a saved file.  --iq / --audio force the interpretation.
package fcfb

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"time"
)

// WavToDecoderMain is the wav-to-decoder entry point (see cmd/wav-to-decoder).
func WavToDecoderMain() {
	args := os.Args[1:]
	mode := "" // "" = autodetect, "iq", "audio"
	for len(args) > 0 && strings.HasPrefix(args[0], "--") && args[0] != "--" {
		switch args[0] {
		case "--iq":
			mode = "iq"
		case "--audio":
			mode = "audio"
		default:
			fmt.Fprintf(os.Stderr, "wav-to-decoder: unknown option %q\n", args[0])
			os.Exit(2)
		}
		args = args[1:]
	}
	out, rc, err := runWavToDecoder(args, mode)
	if err != nil {
		fmt.Fprintln(os.Stderr, "wav-to-decoder:", err)
		os.Exit(2)
	}
	fmt.Print(out)
	os.Exit(rc)
}

// findWavArg returns the index of the last argument that is an existing .wav file.
func findWavArg(args []string) int {
	idx := -1
	for i, a := range args {
		if strings.HasSuffix(strings.ToLower(a), ".wav") {
			if _, err := os.Stat(a); err == nil {
				idx = i
			}
		}
	}
	return idx
}

// antSel names one decode: which antenna tag to emit ("" = none) and how to pull its
// mono audio from the parsed WAV.
type antSel struct {
	ant string
	ch  int  // channel index into the interleaved data (audio: the channel; iq: the I channel)
	iq  bool // iq: take float32 I and peak-normalise; else int16 audio passthrough
}

// runWavToDecoder decodes the WAV named in args, returning the combined decoder
// output (with #ANT markers), an aggregate return code, and an error for setup
// problems.  mode forces "iq"/"audio"; "" autodetects from the format tag.
func runWavToDecoder(args []string, mode string) (string, int, error) {
	if len(args) == 0 {
		return "", 2, fmt.Errorf("no decoder command given")
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

	// Decide interpretation: explicit mode, else the format tag.
	iq := d.FormatTag == wavFmtFloat
	switch mode {
	case "iq":
		iq = true
	case "audio":
		iq = false
	}
	if iq && d.F32 == nil {
		return "", 2, fmt.Errorf("%s: --iq but not float32 I/Q", wavPath)
	}
	if !iq && d.I16 == nil {
		return "", 2, fmt.Errorf("%s: --audio but not 16-bit PCM", wavPath)
	}

	// Map (interpretation, channels) -> the per-antenna decodes.
	var sels []antSel
	switch {
	case !iq && d.NumCh == 1:
		sels = []antSel{{ant: "", ch: 0}} // mono passthrough (ch unused)
	case !iq && d.NumCh == 2:
		sels = []antSel{{ant: "A", ch: 0}, {ant: "B", ch: 1}}
	case iq && d.NumCh == 2:
		sels = []antSel{{ant: "", ch: 0, iq: true}}
	case iq && d.NumCh == 4:
		sels = []antSel{{ant: "A", ch: 0, iq: true}, {ant: "B", ch: 2, iq: true}}
	default:
		kind := "audio"
		if iq {
			kind = "I/Q"
		}
		return "", 2, fmt.Errorf("%s: unsupported %s channel count %d", wavPath, kind, d.NumCh)
	}

	// Mono passthrough: run the decoder once on the file as-is.
	if len(sels) == 1 && sels[0].ant == "" && !sels[0].iq {
		dir, derr := os.MkdirTemp("", "wav2dec_")
		if derr != nil {
			return "", 2, derr
		}
		defer os.RemoveAll(dir)
		out, rc := runOne(args, wavIdx, wavPath, dir)
		return out, rc, nil
	}

	// Split/convert each antenna to a mono 16-bit temp WAV, named so the decoders
	// still read the window time from the filename tail, then decode in sequence.
	dir, derr := os.MkdirTemp("", "wav2dec_")
	if derr != nil {
		return "", 2, derr
	}
	defer os.RemoveAll(dir)
	base := tempBase(d, wavPath) // the "...HHMM[SS].wav" tail the decoder parses

	var sb strings.Builder
	rc := 0
	for _, s := range sels {
		mono := extractMono(d, s)
		prefix := "" // antenna tag as an A-/B- filename prefix, keeping the time tail intact
		if s.ant != "" {
			prefix = s.ant + "-"
		}
		tmp := filepath.Join(dir, prefix+base) // e.g. A-260927_1830.wav (no prefix for single)
		if werr := WriteWAVInt16(tmp, d.Rate, 1, mono); werr != nil {
			return sb.String(), 2, werr
		}
		if s.ant != "" {
			fmt.Fprintf(&sb, "#ANT %s\n", s.ant) // tag the following decode lines
		}
		out, orc := runOne(args, wavIdx, tmp, dir)
		sb.WriteString(out)
		if orc != 0 {
			rc = orc
		}
	}
	return sb.String(), rc, nil
}

// extractMono pulls one antenna's mono 16-bit audio: the int16 channel for audio, or
// the peak-normalised I channel for I/Q (= the real part the audio path would use).
func extractMono(d *WavData, s antSel) []int16 {
	if s.iq {
		n := len(d.F32) / maxInt(d.NumCh, 1)
		i := make([]float64, n)
		for k := 0; k < n; k++ {
			i[k] = float64(d.F32[k*d.NumCh+s.ch])
		}
		return PeakNormalizeInt16(i)
	}
	n := len(d.I16) / maxInt(d.NumCh, 1)
	out := make([]int16, n)
	for k := 0; k < n; k++ {
		out[k] = d.I16[k*d.NumCh+s.ch]
	}
	return out
}

// tempBase is the temp WAV's "...HHMM[SS].wav" tail.  For I/Q it is synthesised from
// the auxi window bounds (StartTime, and HHMM vs HHMMSS from StopTime-StartTime); for
// audio (no auxi) it preserves the input filename's tail, as dual-decoder did.
func tempBase(d *WavData, wavPath string) string {
	if d.Auxi != nil && !d.Auxi.Start.IsZero() {
		slow := d.Auxi.Stop.Sub(d.Auxi.Start) >= 60*time.Second
		return WSJTXStamp(d.Auxi.Start, slow) + ".wav"
	}
	return filepath.Base(wavPath)
}

// runOne runs the decoder with args[wavIdx] replaced by wavArg, in working dir
// (where the decoder's own scratch files land), and returns its stdout and rc.
func runOne(args []string, wavIdx int, wavArg, dir string) (string, int) {
	argv := make([]string, len(args))
	copy(argv, args)
	argv[wavIdx] = wavArg
	ctx, cancel := ctxTimeout(120 * time.Second)
	defer cancel()
	cmd := exec.CommandContext(ctx, argv[0], argv[1:]...)
	cmd.Dir = dir
	cmd.Stderr = os.Stderr
	out, err := cmd.Output()
	rc := 0
	if err != nil {
		rc = 1
		if ee, ok := err.(*exec.ExitError); ok {
			rc = ee.ExitCode()
		}
	}
	return string(out), rc
}
