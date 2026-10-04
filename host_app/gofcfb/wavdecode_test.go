// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
package fcfb

import (
	"path/filepath"
	"strings"
	"testing"
	"time"
)

// stubArgs builds a wav-to-decoder arg list whose "decoder" is a shell stub that
// prints a marker line and the basename of the WAV it was handed, so tests can see
// which temp file each decode ran on.  The last element is the real wav path.
func stubArgs(wavPath string) []string {
	return []string{"sh", "-c", `echo ANT-LINE; basename "$1"`, "sh", wavPath}
}

func TestWavToDecoderAudioMono(t *testing.T) {
	p := filepath.Join(t.TempDir(), "260927_183015.wav")
	if err := WriteWAVInt16(p, 12000, 1, []int16{0, 1, -1, 2}); err != nil {
		t.Fatal(err)
	}
	out, rc, err := runWavToDecoder(stubArgs(p), "")
	if err != nil || rc != 0 {
		t.Fatalf("rc=%d err=%v", rc, err)
	}
	if strings.Contains(out, "#ANT") {
		t.Fatalf("mono should emit no #ANT marker: %q", out)
	}
	if !strings.Contains(out, "260927_183015.wav") { // ran on the original file
		t.Fatalf("mono passthrough didn't run on the original file: %q", out)
	}
}

func TestWavToDecoderAudioStereo(t *testing.T) {
	p := filepath.Join(t.TempDir(), "260927_1830.wav")
	if err := WriteWAVInt16(p, 12000, 2, []int16{10, 20, 11, 21}); err != nil { // L=10,11 R=20,21
		t.Fatal(err)
	}
	out, rc, err := runWavToDecoder(stubArgs(p), "")
	if err != nil || rc != 0 {
		t.Fatalf("rc=%d err=%v", rc, err)
	}
	ia, ib := strings.Index(out, "#ANT A"), strings.Index(out, "#ANT B")
	if ia < 0 || ib < 0 || ia > ib {
		t.Fatalf("want #ANT A before #ANT B: %q", out)
	}
	// Temp WAVs keep the input tail with an A-/B- prefix (so the decoder still parses
	// the time), and each decode ran on its own split.
	if !strings.Contains(out, "A-260927_1830.wav") || !strings.Contains(out, "B-260927_1830.wav") {
		t.Fatalf("stereo split temp names wrong: %q", out)
	}
}

func TestWavToDecoderIQSingle(t *testing.T) {
	p := filepath.Join(t.TempDir(), "40mF8iq-whatever.wav") // name is irrelevant; auxi drives naming
	start := time.Date(2026, 9, 27, 18, 30, 15, 0, time.UTC)
	a := Auxi{Start: start, Stop: start.Add(15 * time.Second), CenterHz: 7074000, ADHz: 12000, BwHz: 3000}
	if err := writeWAVFloat32(p, 12000, 2, []float32{0, 0, 0.5, -0.5}, a); err != nil {
		t.Fatal(err)
	}
	out, rc, err := runWavToDecoder(stubArgs(p), "")
	if err != nil || rc != 0 {
		t.Fatalf("rc=%d err=%v", rc, err)
	}
	if strings.Contains(out, "#ANT") {
		t.Fatalf("single-ADC I/Q should emit no #ANT marker: %q", out)
	}
	if !strings.Contains(out, "260927_183015.wav") { // fast: HHMMSS, synthesised from auxi
		t.Fatalf("I/Q temp name not WSJT-X fast stamp from auxi: %q", out)
	}
}

func TestWavToDecoderIQDualSlow(t *testing.T) {
	p := filepath.Join(t.TempDir(), "iq4.wav")
	start := time.Date(2026, 9, 27, 18, 30, 0, 0, time.UTC)
	a := Auxi{Start: start, Stop: start.Add(120 * time.Second), CenterHz: 7038600, ADHz: 12000, BwHz: 3000}
	// 4ch [I_A,Q_A,I_B,Q_B], 1 frame.
	if err := writeWAVFloat32(p, 12000, 4, []float32{0.1, 0.2, 0.3, 0.4}, a); err != nil {
		t.Fatal(err)
	}
	out, rc, err := runWavToDecoder(stubArgs(p), "")
	if err != nil || rc != 0 {
		t.Fatalf("rc=%d err=%v", rc, err)
	}
	ia, ib := strings.Index(out, "#ANT A"), strings.Index(out, "#ANT B")
	if ia < 0 || ib < 0 || ia > ib {
		t.Fatalf("want #ANT A before #ANT B: %q", out)
	}
	// slow mode (period 120s): HHMM, no seconds; A-/B- prefixed.
	if !strings.Contains(out, "A-260927_1830.wav") || !strings.Contains(out, "B-260927_1830.wav") {
		t.Fatalf("dual I/Q slow temp names wrong: %q", out)
	}
}

// TestExtractMono: I/Q extraction returns the peak-normalised I channel; audio
// extraction picks the requested interleaved channel.
func TestExtractMono(t *testing.T) {
	// I/Q 4ch: I_A = {1, 3}, I_B = {2, 4} (ch 0 and ch 2).
	iq := &WavData{FormatTag: wavFmtFloat, NumCh: 4, F32: []float32{1, 9, 2, 9, 3, 9, 4, 9}}
	gotA := extractMono(iq, antSel{ch: 0, iq: true})
	wantA := PeakNormalizeInt16([]float64{1, 3})
	if len(gotA) != 2 || gotA[0] != wantA[0] || gotA[1] != wantA[1] {
		t.Fatalf("I_A = %v, want %v", gotA, wantA)
	}
	gotB := extractMono(iq, antSel{ch: 2, iq: true})
	wantB := PeakNormalizeInt16([]float64{2, 4})
	if gotB[0] != wantB[0] || gotB[1] != wantB[1] {
		t.Fatalf("I_B = %v, want %v", gotB, wantB)
	}
	// audio 2ch: R channel (ch 1) = {20, 21}.
	au := &WavData{FormatTag: wavFmtPCM, NumCh: 2, I16: []int16{10, 20, 11, 21}}
	if r := extractMono(au, antSel{ch: 1}); r[0] != 20 || r[1] != 21 {
		t.Fatalf("R channel = %v, want [20 21]", r)
	}
}
