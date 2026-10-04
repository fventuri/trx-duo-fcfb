// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Decoders, WAV output, and running/parsing the external decoders.  A [decoder] in
// the config is a named external command (jt9, wsprd, ...) plus the window timing
// and output parser it needs; channels reference one by name.
package fcfb

import (
	"bufio"
	"context"
	"encoding/binary"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
	"time"
)

func ctxTimeout(d time.Duration) (context.Context, context.CancelFunc) {
	return context.WithTimeout(context.Background(), d)
}

// Decoder is a user-defined [decoder]: a named external command plus the window
// timing and output parser it needs.  Channels reference one by name.
type Decoder struct {
	Name     string  // referenced by [channel] decoder=
	Cmd      string  // command template; placeholders {wav} {fmhz} {fhz} {utc}
	Parser   string  // output format: "jt9" | "fst4w" | "wspr"
	PeriodS  float64 // UTC window period (s)
	CaptureS float64 // capture length within the period (s)
	Rate     int     // WAV sample rate the synth resamples to (0 -> defaultWavRate)
}

// defaultWavRate is the WAV rate when a [decoder] sets no rate (jt9/wsprd use 12 kHz).
const defaultWavRate = 12000

// rate is the decoder's WAV sample rate, defaulting to 12 kHz.
func (d Decoder) rate() int {
	if d.Rate <= 0 {
		return defaultWavRate
	}
	return d.Rate
}

// splitCmd splits a command template into argv, honouring single/double quotes so
// an option value may contain spaces.
func splitCmd(s string) []string {
	var toks []string
	var cur strings.Builder
	inTok := false
	quote := rune(0)
	for _, r := range s {
		switch {
		case quote != 0:
			if r == quote {
				quote = 0
			} else {
				cur.WriteRune(r)
			}
			inTok = true
		case r == '\'' || r == '"':
			quote = r
			inTok = true
		case r == ' ' || r == '\t':
			if inTok {
				toks = append(toks, cur.String())
				cur.Reset()
				inTok = false
			}
		default:
			cur.WriteRune(r)
			inTok = true
		}
	}
	if inTok {
		toks = append(toks, cur.String())
	}
	return toks
}

type Spot struct {
	UTC     string
	Mode    string
	Channel string
	Ant     string // "A"/"B" for a dual-ADC diversity decode (which antenna); "" otherwise
	FcHz    float64
	Snr     float64
	Dt      float64
	FreqHz  float64
	Message string
}

func (s Spot) String() string {
	chv := s.Channel
	if s.Ant != "" {
		chv += " " + s.Ant // e.g. "20mF8 A" -- which antenna a diversity decode came from
	}
	return fmt.Sprintf("%s %-4s %-12s %10.4fMHz %+4.0fdB %+5.1f %5.0fHz  %s",
		s.UTC, s.Mode, chv, s.FcHz/1e6, s.Snr, s.Dt, s.FreqHz, s.Message)
}

var (
	jt9RE = regexp.MustCompile(`^\d{6}\s+(-?\d+)\s+(-?\d+\.\d+)\s+(\d+)\s+(.*)$`)
	// fst4wRE matches jt9 --fst4w output, which is the jt9 layout with a 4-digit
	// HHMM timestamp (FST4W periods are all >= 120 s) instead of HHMMSS:
	//   0001   0  0.0 1500 `  K1JT EN50 30
	fst4wRE = regexp.MustCompile(`^\d{4}\s+(-?\d+)\s+(-?\d+\.\d+)\s+(\d+)\s+(.*)$`)
	wsprRE  = regexp.MustCompile(`^\s*\d+\s+(-?\d+)\s+(-?\d+\.\d+)\s+(\d+\.\d+)\s+(-?\d+)\s+(.*)$`)
	syncRE  = regexp.MustCompile("^[~`?*#$]\\s+")
	// jt9AnnotRE strips jt9's trailing decode annotation: a low-confidence "?" and/or
	// a decode-type code like "a1"/"a2"/"q0" (a lowercase letter + digits). These are
	// decoder metadata, not part of the transmitted message. No valid FT8/FT4/FST4W
	// message token is a lowercase letter followed by digits (calls, grids and reports
	// are uppercase/digits/+/-), so this never removes real content -- but leaving it
	// in makes the same signal decoded with vs without the flag look like two messages.
	jt9AnnotRE = regexp.MustCompile(`(?:\s+(?:\?|[a-z]\d+))+\s*$`)
)

// hhmmss returns the HHMMSS portion of a YYMMDD_HHMMSS window stamp (the part
// after the underscore); used for the spot display and the {utc} placeholder so
// those stay HHMMSS even though the WAV filename now carries the full date+time.
func hhmmss(stamp string) string {
	if i := strings.LastIndex(stamp, "_"); i >= 0 && i+1 < len(stamp) {
		return stamp[i+1:]
	}
	return stamp
}

// argv renders the decoder's command template into argv, substituting the
// per-window placeholders.
func (d Decoder) argv(wav string, ch Channel, utc string) []string {
	rep := strings.NewReplacer(
		"{wav}", wav,
		"{fmhz}", fmt.Sprintf("%.6f", ch.FcHz/1e6),
		"{fhz}", fmt.Sprintf("%.0f", ch.FcHz),
		"{utc}", hhmmss(utc),
	)
	toks := splitCmd(d.Cmd)
	out := make([]string, len(toks))
	for i, t := range toks {
		out[i] = rep.Replace(t)
	}
	return out
}

func (d Decoder) parse(stdout string, ch Channel, utc string) []Spot {
	var spots []Spot
	name := ch.name()
	short := hhmmss(utc) // spot display keeps HHMMSS even though utc is now YYMMDD_HHMMSS
	ant := ""            // set by "#ANT A"/"#ANT B" marker lines wav-to-decoder emits per antenna
	for _, line := range strings.Split(stdout, "\n") {
		if rest, ok := strings.CutPrefix(strings.TrimSpace(line), "#ANT "); ok {
			ant = strings.TrimSpace(rest)
			continue
		}
		if d.Parser == "wspr" {
			g := wsprRE.FindStringSubmatch(line)
			if g == nil {
				continue
			}
			snr, _ := strconv.ParseFloat(g[1], 64)
			dt, _ := strconv.ParseFloat(g[2], 64)
			fmhz, _ := strconv.ParseFloat(g[3], 64)
			spots = append(spots, Spot{short, d.Name, name, ant, ch.FcHz, snr, dt,
				(fmhz - ch.FcHz/1e6) * 1e6, strings.TrimSpace(g[5])})
		} else {
			re := jt9RE
			if d.Parser == "fst4w" {
				re = fst4wRE
			}
			g := re.FindStringSubmatch(line)
			if g == nil {
				continue
			}
			snr, _ := strconv.ParseFloat(g[1], 64)
			dt, _ := strconv.ParseFloat(g[2], 64)
			freq, _ := strconv.ParseFloat(g[3], 64)
			msg := strings.TrimSpace(syncRE.ReplaceAllString(g[4], ""))
			msg = strings.TrimSpace(jt9AnnotRE.ReplaceAllString(msg, ""))
			spots = append(spots, Spot{short, d.Name, name, ant, ch.FcHz, snr, dt, freq, msg})
		}
	}
	return spots
}

// runDecoder writes real int16 audio to a WAV and runs the decoder on it.
func runDecoder(pcm []int16, numCh int, ch Channel, d Decoder, utc, saveWav string) (string, error) {
	return decodeWAV(ch, d, utc, saveWav, func(path string) error {
		return writeWAV(path, d.rate(), numCh, pcm)
	})
}

// runDecoderIQ writes complex float32 I/Q (with an auxi chunk) to a WAV and runs the
// decoder on it -- the decoder cmd must route through wav-to-decoder, which converts
// the I/Q to the audio the decoder reads.  iq is interleaved ([I,Q] single / 4-ch
// dual); numCh is 2 or 4.
func runDecoderIQ(iq []float32, numCh int, ch Channel, d Decoder, utc, saveWav string) (string, error) {
	a := auxiFor(ch, d, utc)
	return decodeWAV(ch, d, utc, saveWav, func(path string) error {
		return writeWAVFloat32(path, d.rate(), numCh, iq, a)
	})
}

// auxiFor builds the auxi metadata for a window: UTC start (parsed from utc) and stop
// (start + period), the channel centre frequency and bandwidth, and the file rate.
func auxiFor(ch Channel, d Decoder, utc string) Auxi {
	start, _ := time.Parse("060102_150405", utc)
	return Auxi{
		Start:    start,
		Stop:     start.Add(time.Duration(d.PeriodS * float64(time.Second))),
		CenterHz: uint32(ch.FcHz),
		ADHz:     uint32(d.rate()),
		BwHz:     uint32(ch.BwHz),
	}
}

// decodeWAV writes one window's WAV via write (int16 audio or float32 I/Q) and runs
// the decoder on it, returning stdout.  A temp dir is always created for the
// decoder's working directory (jt9/wsprd drop scratch files in cwd).  When saveWav is
// "" the WAV lives in that temp dir and is deleted on return; when saveWav is set it
// is written to the expanded path template instead and kept (see expandSaveWav).
func decodeWAV(ch Channel, d Decoder, utc, saveWav string, write func(path string) error) (string, error) {
	dir, err := os.MkdirTemp("", "fcfbfarm_")
	if err != nil {
		return "", err
	}
	defer os.RemoveAll(dir)
	// Slow modes (period >= 60 s: WSPR, FST4W) use a seconds-less stamp, matching
	// WSJT-X's convention (see the temp-WAV branch below).
	slow := d.PeriodS >= 60

	var wavArg string // the {wav} value handed to the decoder
	if saveWav != "" {
		// Persistent WAV: expand the user's [fcfbfarm] savewav= template, create its
		// parent directory, and write the WAV there.  It is not under dir, so it
		// survives the defer above -- the file is kept after decoding.
		p, perr := expandSaveWav(saveWav, ch, d, utc, slow)
		if perr != nil {
			return "", perr
		}
		if perr := os.MkdirAll(filepath.Dir(p), 0o755); perr != nil {
			return "", fmt.Errorf("savewav: %w", perr)
		}
		if werr := write(p); werr != nil {
			return "", werr
		}
		wavArg = p
	} else {
		// Temp WAV, deleted with dir when this returns.  Name it for the cycle-start
		// UTC so the file -- and any decoder that reads its timestamp from the
		// filename -- reflects the real window (not a fixed 000000_000000).
		// jt9/wsprd extract the time by offset from ".wav" (jt9.f90, wsprd.c), and
		// WSJT-X's own convention is YYMMDD_HHMMSS for fast modes and YYMMDD_HHMM
		// (no seconds) for slow modes (T/R >= 60 s: WSPR, FST4W).  Matching that
		// makes wsprd read the true HHMM and jt9 take its 4-digit slow-mode branch.
		stamp := utc
		if slow && len(stamp) == len("060102_150405") {
			stamp = stamp[:len("060102_1504")] // YYMMDD_HHMMSS -> YYMMDD_HHMM
		}
		wavArg = stamp + ".wav"
		if werr := write(filepath.Join(dir, wavArg)); werr != nil {
			return "", werr
		}
	}

	argv := d.argv(wavArg, ch, utc)
	if len(argv) == 0 {
		return "", fmt.Errorf("decoder %q: empty cmd", d.Name)
	}
	ctx, cancel := ctxTimeout(120 * time.Second)
	defer cancel()
	cmd := exec.CommandContext(ctx, argv[0], argv[1:]...)
	cmd.Dir = dir
	out, err := cmd.Output()
	return string(out), err
}

// expandSaveWav renders a [fcfbfarm] savewav= path template for one decode window.
// Placeholders (each written {name}): {date} YYYYMMDD, {timestamp} YYYYMMDD_HHMMSS
// (or YYYYMMDD_HHMM for slow modes), {channel} the channel name, {antenna} A/B/AB
// (adc 1/2/3), {frequency} the centre frequency in whole Hz (fractions truncated),
// {decoder} the decoder name.  A leading ~ is expanded to the user's home directory.
// The window stamp is the same YYMMDD_HHMMSS used elsewhere; its 2-digit year maps
// to 2000-2069.
func expandSaveWav(tmpl string, ch Channel, d Decoder, utc string, slow bool) (string, error) {
	t, err := time.Parse("060102_150405", utc)
	if err != nil {
		return "", fmt.Errorf("savewav: bad window stamp %q: %w", utc, err)
	}
	tsLayout := "20060102_150405"
	if slow {
		tsLayout = "20060102_1504"
	}
	path := strings.NewReplacer(
		"{date}", t.Format("20060102"),
		"{timestamp}", t.Format(tsLayout),
		"{channel}", ch.name(),
		"{antenna}", ch.antenna(),
		"{frequency}", strconv.FormatInt(int64(ch.FcHz), 10),
		"{decoder}", d.Name,
	).Replace(tmpl)
	if strings.HasPrefix(path, "~") {
		home, herr := os.UserHomeDir()
		if herr != nil {
			return "", fmt.Errorf("savewav: cannot expand ~: %w", herr)
		}
		path = home + path[1:]
	}
	return path, nil
}

// writeWAV writes 16-bit PCM. samples are interleaved when numCh > 1 (L,R,L,R...
// for stereo), so a dual-ADC channel's two antennas land in one time-locked file.
func writeWAV(path string, rate, numCh int, samples []int16) error {
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()
	w := bufio.NewWriter(f)
	dataLen := len(samples) * 2
	blockAlign := numCh * 2
	// RIFF/WAVE, PCM 16-bit
	w.WriteString("RIFF")
	binary.Write(w, binary.LittleEndian, uint32(36+dataLen))
	w.WriteString("WAVE")
	w.WriteString("fmt ")
	binary.Write(w, binary.LittleEndian, uint32(16))
	binary.Write(w, binary.LittleEndian, uint16(1))               // PCM
	binary.Write(w, binary.LittleEndian, uint16(numCh))           // channels
	binary.Write(w, binary.LittleEndian, uint32(rate))            // sample rate
	binary.Write(w, binary.LittleEndian, uint32(rate*blockAlign)) // byte rate
	binary.Write(w, binary.LittleEndian, uint16(blockAlign))      // block align
	binary.Write(w, binary.LittleEndian, uint16(16))              // bits/sample
	w.WriteString("data")
	binary.Write(w, binary.LittleEndian, uint32(dataLen))
	for _, s := range samples {
		binary.Write(w, binary.LittleEndian, s)
	}
	return w.Flush()
}
