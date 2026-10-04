// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
package fcfb

import (
	"path/filepath"
	"testing"
	"time"
)

// TestFloat32WAVRoundTrip writes a float32 I/Q WAV with an auxi chunk and reads it
// back, checking the format, samples and every auxi field we populate.
func TestFloat32WAVRoundTrip(t *testing.T) {
	p := filepath.Join(t.TempDir(), "iq.wav")
	start := time.Date(2026, 9, 27, 18, 30, 0, 0, time.UTC)
	a := Auxi{Start: start, Stop: start.Add(15 * time.Second), CenterHz: 7074000, ADHz: 12000, BwHz: 3000}
	// 2-channel [I,Q] interleave, 3 frames.
	in := []float32{0, 1, -0.5, 0.25, 0.75, -1}
	if err := writeWAVFloat32(p, 12000, 2, in, a); err != nil {
		t.Fatalf("writeWAVFloat32: %v", err)
	}
	d, err := ReadWAV(p)
	if err != nil {
		t.Fatalf("ReadWAV: %v", err)
	}
	if d.FormatTag != wavFmtFloat || d.NumCh != 2 || d.Rate != 12000 || d.Bits != 32 {
		t.Fatalf("header: tag=%d ch=%d rate=%d bits=%d", d.FormatTag, d.NumCh, d.Rate, d.Bits)
	}
	if len(d.F32) != len(in) {
		t.Fatalf("got %d samples, want %d", len(d.F32), len(in))
	}
	for i := range in {
		if d.F32[i] != in[i] {
			t.Fatalf("sample %d = %v, want %v", i, d.F32[i], in[i])
		}
	}
	if d.Auxi == nil {
		t.Fatal("auxi not read back")
	}
	if !d.Auxi.Start.Equal(a.Start) || !d.Auxi.Stop.Equal(a.Stop) {
		t.Fatalf("auxi times: start=%v stop=%v, want %v / %v", d.Auxi.Start, d.Auxi.Stop, a.Start, a.Stop)
	}
	if d.Auxi.CenterHz != a.CenterHz || d.Auxi.ADHz != a.ADHz || d.Auxi.BwHz != a.BwHz {
		t.Fatalf("auxi fields: %+v, want center=%d ad=%d bw=%d", d.Auxi, a.CenterHz, a.ADHz, a.BwHz)
	}
}

// TestReadInt16WAV confirms ReadWAV parses the 16-bit PCM the decoder path writes.
func TestReadInt16WAV(t *testing.T) {
	p := filepath.Join(t.TempDir(), "audio.wav")
	in := []int16{0, 100, -100, 32767, -32768, 7}
	if err := WriteWAVInt16(p, 12000, 2, in); err != nil {
		t.Fatalf("WriteWAVInt16: %v", err)
	}
	d, err := ReadWAV(p)
	if err != nil {
		t.Fatalf("ReadWAV: %v", err)
	}
	if d.FormatTag != wavFmtPCM || d.NumCh != 2 || d.Bits != 16 {
		t.Fatalf("header: tag=%d ch=%d bits=%d", d.FormatTag, d.NumCh, d.Bits)
	}
	if d.Auxi != nil {
		t.Fatal("int16 audio WAV should have no auxi")
	}
	if len(d.I16) != len(in) {
		t.Fatalf("got %d samples, want %d", len(d.I16), len(in))
	}
	for i := range in {
		if d.I16[i] != in[i] {
			t.Fatalf("sample %d = %d, want %d", i, d.I16[i], in[i])
		}
	}
}

// TestWSJTXStamp checks fast (HHMMSS) vs slow (HHMM) naming.
func TestWSJTXStamp(t *testing.T) {
	tm := time.Date(2026, 9, 27, 18, 30, 15, 0, time.UTC)
	if got := WSJTXStamp(tm, false); got != "260927_183015" {
		t.Fatalf("fast stamp = %q, want 260927_183015", got)
	}
	if got := WSJTXStamp(tm, true); got != "260927_1830" {
		t.Fatalf("slow stamp = %q, want 260927_1830", got)
	}
}
