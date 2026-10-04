// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
package fcfb

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

// TestExampleConfigParses keeps examples/farm.example.ini valid, and confirms the
// fst4w decoder/channel wire up (parser "fst4w" is accepted, channel resolves it).
func TestExampleConfigParses(t *testing.T) {
	cfg, err := parseConfig("examples/farm.example.ini")
	if err != nil {
		t.Fatalf("parseConfig(examples/farm.example.ini): %v", err)
	}
	d, ok := cfg.decoder("fst4w120")
	if !ok {
		t.Fatal("fst4w120 decoder not found in example config")
	}
	if d.Parser != "fst4w" || d.PeriodS != 120 {
		t.Errorf("fst4w120: parser=%q period=%v, want fst4w/120", d.Parser, d.PeriodS)
	}
	var found bool
	for _, ch := range cfg.Channels {
		if ch.Decoder == "fst4w120" {
			found = true
		}
	}
	if !found {
		t.Error("no channel references the fst4w120 decoder")
	}
}

// TestIQExampleConfigParses keeps examples/farm.example.iq.ini valid and confirms it
// selects the I/Q WAV format.
func TestIQExampleConfigParses(t *testing.T) {
	cfg, err := parseConfig("examples/farm.example.iq.ini")
	if err != nil {
		t.Fatalf("parseConfig(examples/farm.example.iq.ini): %v", err)
	}
	if cfg.WavFormat != "iq" {
		t.Fatalf("WavFormat = %q, want iq", cfg.WavFormat)
	}
	if cfg.SaveWav == "" {
		t.Error("iq example should set savewav")
	}
}

// TestParseFST4W checks the fst4w parser against real jt9 --fst4w output.  The
// line is the jt9 layout with a 4-digit HHMM timestamp (captured from
// `jt9 --fst4w -p 120 -f 1500` on an fst4sim-generated WSPR-payload signal):
//
//	0001   0  0.0 1500 `  K1JT EN50 30
func TestParseFST4W(t *testing.T) {
	const out = "0001   0  0.0 1500 `  K1JT EN50 30                     \n" +
		"<DecodeFinished>   0   1        0\n"
	ch := Channel{FcHz: 7.0386e6, Name: "40mFST4W"}
	d := Decoder{Name: "fst4w120", Parser: "fst4w"}
	spots := d.parse(out, ch, "0001")
	if len(spots) != 1 {
		t.Fatalf("got %d spots, want 1: %+v", len(spots), spots)
	}
	s := spots[0]
	if s.Snr != 0 || s.Dt != 0 || s.FreqHz != 1500 {
		t.Errorf("fields: snr=%v dt=%v freq=%v, want 0/0/1500", s.Snr, s.Dt, s.FreqHz)
	}
	if s.Message != "K1JT EN50 30" {
		t.Errorf("message = %q, want %q", s.Message, "K1JT EN50 30")
	}
	if s.FcHz != ch.FcHz || s.Channel != "40mFST4W" || s.Mode != "fst4w120" {
		t.Errorf("meta: fc=%v ch=%q mode=%q", s.FcHz, s.Channel, s.Mode)
	}

	// The jt9 parser (6-digit HHMMSS) must NOT match a 4-digit fst4w line --
	// that mismatch is exactly why fst4w needs its own parser.
	dj := Decoder{Name: "x", Parser: "jt9"}
	if got := dj.parse(out, ch, "0001"); len(got) != 0 {
		t.Errorf("jt9 parser matched fst4w output (%d spots); should not", len(got))
	}
}

// jt9 appends a decode-type annotation ("a1"/"a2"/...) and a low-confidence "?" to
// some lines; those are decoder metadata, not the message, and must be stripped so
// the same signal decoded with vs without the flag dedupes to one message.
func TestParseJT9StripsAnnotation(t *testing.T) {
	const out = "134630 -13  0.1 1465 ~  CQ KC3WOX FN00 a1\n" +
		"141630 -21  0.2 1049 ~  CQ W1WWB EM95 ? a1\n" +
		"142100 -17  0.1 1439 ~  CQ NM1G FN41\n" + // no annotation
		"140000 -05  0.0  800 ~  W1AW K1JT R-08\n" // report must survive
	ch := Channel{FcHz: 7.074e6, Name: "40mFT8"}
	d := Decoder{Name: "ft8", Parser: "jt9"}
	spots := d.parse(out, ch, "134630")
	want := []string{"CQ KC3WOX FN00", "CQ W1WWB EM95", "CQ NM1G FN41", "W1AW K1JT R-08"}
	if len(spots) != len(want) {
		t.Fatalf("got %d spots, want %d: %+v", len(spots), len(want), spots)
	}
	for i, w := range want {
		if spots[i].Message != w {
			t.Errorf("spot %d message = %q, want %q", i, spots[i].Message, w)
		}
	}
}

// TestWavNameIsCycleStamp: runDecoder must name the WAV for the cycle-start UTC
// (YYMMDD_HHMMSS.wav), so wsprd -- which reads the 4 chars before ".wav" as its
// timestamp (wsprd.c 3.0.0) -- sees digits.  A no-op decoder that lists the cwd
// reveals the filename runDecoder created.
func TestWavNameIsCycleStamp(t *testing.T) {
	d := Decoder{Name: "x", Cmd: "sh -c ls", Parser: "jt9"}
	out, err := runDecoder([]int16{0, 0, 0, 0}, 1, Channel{FcHz: 7.074e6}, d, "260927_183000", "")
	if err != nil {
		t.Fatalf("runDecoder: %v", err)
	}
	if !strings.Contains(out, "260927_183000.wav") {
		t.Fatalf("WAV not named for the cycle stamp; ls output = %q", out)
	}
	// wsprd reads the 4 chars before ".wav" as its HHMM (wsprd.c): here "3000",
	// which are digits, so wsprRE will match (the farm supplies the real UTC).
}

// TestWavNameSlowModeHHMM: slow modes (period >= 60 s: WSPR, FST4W) drop the
// seconds -> YYMMDD_HHMM.wav (WSJT-X convention), so wsprd's 4-char field and jt9's
// slow-mode branch read the true HHMM.
func TestWavNameSlowModeHHMM(t *testing.T) {
	d := Decoder{Name: "wspr", Cmd: "sh -c ls", Parser: "wspr", PeriodS: 120, CaptureS: 114}
	out, err := runDecoder([]int16{0, 0, 0, 0}, 1, Channel{FcHz: 7.0386e6}, d, "260927_223000", "")
	if err != nil {
		t.Fatalf("runDecoder: %v", err)
	}
	if !strings.Contains(out, "260927_2230.wav") {
		t.Fatalf("slow-mode WAV should be YYMMDD_HHMM; ls output = %q", out)
	}
	if strings.Contains(out, "260927_223000.wav") {
		t.Fatalf("slow-mode WAV must not keep seconds; ls output = %q", out)
	}
}

// TestSpotDisplayStaysHHMMSS: the WAV filename carries the full YYMMDD_HHMMSS, but
// the spot display and the {utc} placeholder must stay HHMMSS (unchanged behaviour).
func TestSpotDisplayStaysHHMMSS(t *testing.T) {
	if got := hhmmss("260927_183000"); got != "183000" {
		t.Fatalf("hhmmss = %q, want 183000", got)
	}
	if got := hhmmss("134630"); got != "134630" { // no underscore -> unchanged
		t.Fatalf("hhmmss(no _) = %q, want 134630", got)
	}
	d := Decoder{Name: "ft8", Parser: "jt9", Cmd: "jt9 --ft8 {utc} {wav}"}
	sp := d.parse("134630 -13  0.1 1465 ~  CQ K1ABC FN42\n", Channel{FcHz: 7.074e6, Name: "x"}, "260927_134630")
	if len(sp) != 1 || sp[0].UTC != "134630" {
		t.Fatalf("Spot.UTC = %v, want 134630", sp)
	}
	av := strings.Join(d.argv("f.wav", Channel{FcHz: 7.074e6}, "260927_134630"), " ")
	if !strings.Contains(av, " 134630 ") || strings.Contains(av, "260927_134630") {
		t.Fatalf("{utc} not trimmed to HHMMSS in argv: %q", av)
	}
}

// TestParseAntennaMarker checks that wav-to-decoder's "#ANT A"/"#ANT B" markers tag
// each following spot with its antenna, and that String() shows it (e.g. "20mF8 A").
func TestParseAntennaMarker(t *testing.T) {
	d := Decoder{Name: "ft8", Parser: "jt9", Cmd: "wav-to-decoder jt9 --ft8 {wav}"}
	ch := Channel{FcHz: 14.074e6, Name: "20mF8"}
	out := "#ANT A\n" +
		"134630 -18  0.1 1246 ~  HB9ETH N4MA EM60\n" +
		"#ANT B\n" +
		"134630 -11  0.1 1246 ~  HB9ETH N4MA EM60\n"
	sp := d.parse(out, ch, "260927_134630")
	if len(sp) != 2 {
		t.Fatalf("want 2 spots, got %d", len(sp))
	}
	if sp[0].Ant != "A" || sp[1].Ant != "B" {
		t.Fatalf("antennas: %q, %q (want A, B)", sp[0].Ant, sp[1].Ant)
	}
	if !strings.Contains(sp[0].String(), "20mF8 A") {
		t.Fatalf("String() missing antenna tag: %q", sp[0].String())
	}
	// No markers (mono / non-dual): Ant stays empty and the display is untagged.
	mono := d.parse("134630 -18  0.1 1246 ~  HB9ETH N4MA EM60\n", ch, "260927_134630")
	if len(mono) != 1 || mono[0].Ant != "" || strings.Contains(mono[0].String(), " A ") {
		t.Fatalf("mono spot should have no antenna tag: ant=%q str=%q", mono[0].Ant, mono[0].String())
	}
}

// TestParseFcfbfarmSection: the [fcfbfarm] section fills cfg.SaveWav, is optional
// (default ""), and rejects unknown keys.
func TestParseFcfbfarmSection(t *testing.T) {
	base := "[decoder]\nname=ft8\ncmd=jt9 --ft8 {wav}\nparser=jt9\nperiod=15\ncapture=13.5\n" +
		"[channel]\nfc=7.074e6\ndecoder=ft8\n"
	write := func(body string) string {
		p := filepath.Join(t.TempDir(), "farm.ini")
		if err := os.WriteFile(p, []byte(body), 0o644); err != nil {
			t.Fatal(err)
		}
		return p
	}

	cfg, err := parseConfig(write("[board]\nreplay=x.bin\n" + base +
		"[fcfbfarm]\nsavewav = ~/fcfb/{date}/{channel}/{antenna}-{timestamp}.wav\n"))
	if err != nil {
		t.Fatalf("parseConfig: %v", err)
	}
	if want := "~/fcfb/{date}/{channel}/{antenna}-{timestamp}.wav"; cfg.SaveWav != want {
		t.Fatalf("SaveWav = %q, want %q", cfg.SaveWav, want)
	}

	// Optional: absent section -> empty (temp-WAV behaviour).
	if cfg, err = parseConfig(write("[board]\nreplay=x.bin\n" + base)); err != nil || cfg.SaveWav != "" {
		t.Fatalf("no [fcfbfarm]: SaveWav=%q err=%v, want \"\" nil", cfg.SaveWav, err)
	}

	// Unknown key is rejected.
	if _, err = parseConfig(write("[board]\nreplay=x.bin\n" + base + "[fcfbfarm]\nbogus = 1\n")); err == nil {
		t.Fatal("expected error for unknown [fcfbfarm] key")
	}

	// wav_format: default audio, accepts iq, rejects anything else.
	if cfg, err = parseConfig(write("[board]\nreplay=x.bin\n" + base)); err != nil || cfg.WavFormat != "audio" {
		t.Fatalf("default wav_format=%q err=%v, want audio nil", cfg.WavFormat, err)
	}
	if cfg, err = parseConfig(write("[board]\nreplay=x.bin\n" + base + "[fcfbfarm]\nwav_format = iq\n")); err != nil || cfg.WavFormat != "iq" {
		t.Fatalf("wav_format=iq -> %q err=%v", cfg.WavFormat, err)
	}
	if _, err = parseConfig(write("[board]\nreplay=x.bin\n" + base + "[fcfbfarm]\nwav_format = int24\n")); err == nil {
		t.Fatal("expected error for invalid wav_format")
	}
}

// TestInterleaveF32 checks channel-minor interleave with a short-channel guard.
func TestInterleaveF32(t *testing.T) {
	got := interleaveF32([]float64{1, 2}, []float64{3, 4}) // single-ADC [I,Q]
	want := []float32{1, 3, 2, 4}
	if len(got) != 4 || got[0] != want[0] || got[1] != want[1] || got[2] != want[2] || got[3] != want[3] {
		t.Fatalf("interleave 2ch = %v, want %v", got, want)
	}
	// 4 channels, one short -> truncate to the min length (1 frame).
	g4 := interleaveF32([]float64{1, 9}, []float64{2}, []float64{3, 9}, []float64{4})
	if len(g4) != 4 || g4[0] != 1 || g4[1] != 2 || g4[2] != 3 || g4[3] != 4 {
		t.Fatalf("interleave 4ch (short) = %v, want [1 2 3 4]", g4)
	}
}

// TestRunDecoderIQSaveWav: iq mode writes a float32 WAV (with auxi) to the savewav
// path, keeps it, and it reads back with the right format, channels and metadata.
func TestRunDecoderIQSaveWav(t *testing.T) {
	dir := t.TempDir()
	tmpl := filepath.Join(dir, "{antenna}-{timestamp}.wav")
	d := Decoder{Name: "ft8", Cmd: "sh -c true", Parser: "jt9", PeriodS: 15, CaptureS: 13.5}
	ch := Channel{FcHz: 7074000, Name: "40m", Adc: 1, BwHz: 3000}
	iq := []float32{0.1, 0.3, 0.2, 0.4} // [I0,Q0,I1,Q1]
	if _, err := runDecoderIQ(iq, 2, ch, d, "260927_183015", tmpl); err != nil {
		t.Fatalf("runDecoderIQ: %v", err)
	}
	want := filepath.Join(dir, "A-20260927_183015.wav")
	wd, err := ReadWAV(want)
	if err != nil {
		t.Fatalf("saved I/Q WAV not readable at %q: %v", want, err)
	}
	if wd.FormatTag != wavFmtFloat || wd.NumCh != 2 || wd.Rate != 12000 {
		t.Fatalf("header: tag=%d ch=%d rate=%d", wd.FormatTag, wd.NumCh, wd.Rate)
	}
	for i := range iq {
		if wd.F32[i] != iq[i] {
			t.Fatalf("sample %d = %v, want %v", i, wd.F32[i], iq[i])
		}
	}
	if wd.Auxi == nil || wd.Auxi.CenterHz != 7074000 || wd.Auxi.BwHz != 3000 || wd.Auxi.ADHz != 12000 {
		t.Fatalf("auxi = %+v", wd.Auxi)
	}
	wantStart := time.Date(2026, 9, 27, 18, 30, 15, 0, time.UTC)
	if !wd.Auxi.Start.Equal(wantStart) || !wd.Auxi.Stop.Equal(wantStart.Add(15*time.Second)) {
		t.Fatalf("auxi times: start=%v stop=%v", wd.Auxi.Start, wd.Auxi.Stop)
	}
}

// TestExpandSaveWav checks each {placeholder} in a [fcfbfarm] savewav= template,
// the fast/slow {timestamp} forms, the A/B/AB {antenna} mapping, and ~ expansion.
func TestExpandSaveWav(t *testing.T) {
	home, _ := os.UserHomeDir()
	ft8 := Decoder{Name: "ft8", PeriodS: 15, CaptureS: 13.5}
	wspr := Decoder{Name: "wspr", PeriodS: 120, CaptureS: 114}
	tmpl := "/data/{date}/{channel}/{antenna}-{frequency}-{decoder}-{timestamp}.wav"

	got, err := expandSaveWav(tmpl, Channel{FcHz: 14.074e6, Name: "20mF8", Adc: 1}, ft8, "260927_183015", false)
	if err != nil {
		t.Fatalf("expandSaveWav (fast): %v", err)
	}
	if want := "/data/20260927/20mF8/A-14074000-ft8-20260927_183015.wav"; got != want {
		t.Fatalf("fast: got %q, want %q", got, want)
	}

	// Slow mode: {timestamp} drops the seconds (YYYYMMDD_HHMM).
	got, err = expandSaveWav(tmpl, Channel{FcHz: 7.0386e6, Name: "40mW", Adc: 2}, wspr, "260927_223000", true)
	if err != nil {
		t.Fatalf("expandSaveWav (slow): %v", err)
	}
	if want := "/data/20260927/40mW/B-7038600-wspr-20260927_2230.wav"; got != want {
		t.Fatalf("slow: got %q, want %q", got, want)
	}

	// adc=3 -> {antenna} "AB"; leading ~ -> home dir.
	got, err = expandSaveWav("~/w/{antenna}.wav", Channel{FcHz: 14.074e6, Name: "x", Adc: 3}, ft8, "260927_183015", false)
	if err != nil {
		t.Fatalf("expandSaveWav (~): %v", err)
	}
	if want := home + "/w/AB.wav"; got != want {
		t.Fatalf("~/AB: got %q, want %q", got, want)
	}
}

// TestRunDecoderSaveWav: with savewav set, runDecoder writes the WAV to the expanded
// path and keeps it after the decoder runs (instead of a deleted temp file).
func TestRunDecoderSaveWav(t *testing.T) {
	dir := t.TempDir()
	tmpl := filepath.Join(dir, "{channel}", "{antenna}-{timestamp}.wav")
	d := Decoder{Name: "ft8", Cmd: "sh -c true", Parser: "jt9", PeriodS: 15, CaptureS: 13.5}
	_, err := runDecoder([]int16{0, 0, 0, 0}, 1, Channel{FcHz: 7.074e6, Name: "40m", Adc: 1}, d, "260927_183015", tmpl)
	if err != nil {
		t.Fatalf("runDecoder: %v", err)
	}
	want := filepath.Join(dir, "40m", "A-20260927_183015.wav")
	if _, serr := os.Stat(want); serr != nil {
		t.Fatalf("saved WAV %q not kept: %v", want, serr)
	}
}
