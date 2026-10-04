// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// WAV I/O beyond the 16-bit-PCM decoder audio in decode.go: a float32
// (WAVE_FORMAT_IEEE_FLOAT) writer for complex I/Q output, the RFSpace/SpectraVue
// "auxi" SDR-metadata chunk, and a reader that handles both formats.  These are the
// exported surface the wav-to-decoder utility builds on (it lives in its own package
// but imports fcfb, like the other commands).
package fcfb

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"os"
	"time"
)

// WAVE format tags.
const (
	wavFmtPCM   = 1 // 16-bit integer PCM (decoder audio)
	wavFmtFloat = 3 // IEEE float32 (complex I/Q)
)

// Auxi is the "auxi" chunk SDR tools (RFSpace/SpectraVue lineage) use to carry
// capture metadata a WAV header cannot.  We populate Start/Stop (UTC window bounds),
// CenterHz (channel fc), ADHz (the file's complex sample rate) and BwHz (channel
// bandwidth); the remaining standard fields are written as zero.  The exact on-disk
// layout is documented in the repo's I/Q-format note.
type Auxi struct {
	Start, Stop time.Time
	CenterHz    uint32 // channel centre frequency (dial), Hz
	ADHz        uint32 // sample rate of this file, Hz
	BwHz        uint32 // channel occupied bandwidth, Hz
}

// auxiBytes is the on-disk size of the auxi chunk body: two SYSTEMTIMEs (16 B each)
// + nine DWORDs (CenterFreq, ADFrequency, IFFrequency, Bandwidth, IQOffset, and four
// unused).
const auxiBytes = 16 + 16 + 9*4

// writeSystemTime writes a Win32 SYSTEMTIME (eight little-endian uint16s), in UTC.
func writeSystemTime(w io.Writer, t time.Time) {
	t = t.UTC()
	for _, v := range []uint16{
		uint16(t.Year()), uint16(t.Month()), uint16(t.Weekday()), uint16(t.Day()),
		uint16(t.Hour()), uint16(t.Minute()), uint16(t.Second()), uint16(t.Nanosecond() / 1_000_000),
	} {
		binary.Write(w, binary.LittleEndian, v)
	}
}

// readSystemTime reads a Win32 SYSTEMTIME back into a UTC time.Time.
func readSystemTime(b []byte) time.Time {
	u := func(i int) int { return int(binary.LittleEndian.Uint16(b[i:])) }
	return time.Date(u(0), time.Month(u(2)), u(6), u(8), u(10), u(12), u(14)*1_000_000, time.UTC)
	// b[4:6] is DayOfWeek (ignored on read); b[6:8] is Day.
}

// writeAuxi writes the "auxi" chunk (id + size + body).
func writeAuxi(w io.Writer, a Auxi) {
	w.Write([]byte("auxi"))
	binary.Write(w, binary.LittleEndian, uint32(auxiBytes))
	writeSystemTime(w, a.Start)
	writeSystemTime(w, a.Stop)
	for _, v := range []uint32{a.CenterHz, a.ADHz, 0 /*IF*/, a.BwHz, 0 /*IQOffset*/, 0, 0, 0, 0} {
		binary.Write(w, binary.LittleEndian, v)
	}
}

// writeWAVFloat32 writes interleaved 32-bit IEEE-float samples as a
// WAVE_FORMAT_IEEE_FLOAT file with a fact chunk and an auxi metadata chunk.  For I/Q
// the interleave is [I,Q] (numCh=2, single ADC) or [I_A,Q_A,I_B,Q_B] (numCh=4, dual).
func writeWAVFloat32(path string, rate, numCh int, samples []float32, a Auxi) error {
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()
	w := bufio.NewWriter(f)

	dataLen := len(samples) * 4
	blockAlign := numCh * 4
	// RIFF size covers everything after the "RIFF"+size pair: "WAVE" + each chunk's
	// 8-byte header + body (fmt=18, fact=4, auxi=auxiBytes, data=dataLen).
	riff := 4 + (8 + 18) + (8 + 4) + (8 + auxiBytes) + (8 + dataLen)

	w.WriteString("RIFF")
	binary.Write(w, binary.LittleEndian, uint32(riff))
	w.WriteString("WAVE")

	// fmt chunk (18-byte extended form required for non-PCM: trailing cbSize=0).
	w.WriteString("fmt ")
	binary.Write(w, binary.LittleEndian, uint32(18))
	binary.Write(w, binary.LittleEndian, uint16(wavFmtFloat))     // wFormatTag
	binary.Write(w, binary.LittleEndian, uint16(numCh))           // nChannels
	binary.Write(w, binary.LittleEndian, uint32(rate))            // nSamplesPerSec
	binary.Write(w, binary.LittleEndian, uint32(rate*blockAlign)) // nAvgBytesPerSec
	binary.Write(w, binary.LittleEndian, uint16(blockAlign))      // nBlockAlign
	binary.Write(w, binary.LittleEndian, uint16(32))              // wBitsPerSample
	binary.Write(w, binary.LittleEndian, uint16(0))               // cbSize

	// fact chunk: samples per channel (required for non-PCM).
	w.WriteString("fact")
	binary.Write(w, binary.LittleEndian, uint32(4))
	binary.Write(w, binary.LittleEndian, uint32(len(samples)/maxInt(numCh, 1)))

	writeAuxi(w, a)

	w.WriteString("data")
	binary.Write(w, binary.LittleEndian, uint32(dataLen))
	for _, s := range samples {
		binary.Write(w, binary.LittleEndian, s)
	}
	return w.Flush()
}

// WavData is a parsed WAV file: the format, and the samples as int16 (PCM) or float32
// (IEEE float), plus the auxi metadata when present.
type WavData struct {
	FormatTag uint16
	NumCh     int
	Rate      int
	Bits      int
	I16       []int16   // set when FormatTag==wavFmtPCM and Bits==16
	F32       []float32 // set when FormatTag==wavFmtFloat and Bits==32
	Auxi      *Auxi
}

// ReadWAV parses a RIFF/WAVE file, reading the fmt, data and (if present) auxi
// chunks.  It supports 16-bit PCM and 32-bit IEEE-float data -- the two formats
// fcfbfarm writes -- and loads the samples into memory (decode windows are small).
func ReadWAV(path string) (*WavData, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	if len(raw) < 12 || string(raw[0:4]) != "RIFF" || string(raw[8:12]) != "WAVE" {
		return nil, fmt.Errorf("%s: not a RIFF/WAVE file", path)
	}
	d := &WavData{}
	var data []byte
	off := 12
	for off+8 <= len(raw) {
		id := string(raw[off : off+4])
		sz := int(binary.LittleEndian.Uint32(raw[off+4 : off+8]))
		body := off + 8
		if body+sz > len(raw) {
			sz = len(raw) - body // tolerate a truncated final chunk
		}
		switch id {
		case "fmt ":
			if sz < 16 {
				return nil, fmt.Errorf("%s: short fmt chunk", path)
			}
			d.FormatTag = binary.LittleEndian.Uint16(raw[body:])
			d.NumCh = int(binary.LittleEndian.Uint16(raw[body+2:]))
			d.Rate = int(binary.LittleEndian.Uint32(raw[body+4:]))
			d.Bits = int(binary.LittleEndian.Uint16(raw[body+14:]))
		case "data":
			data = raw[body : body+sz]
		case "auxi":
			if sz >= auxiBytes {
				d.Auxi = &Auxi{
					Start:    readSystemTime(raw[body:]),
					Stop:     readSystemTime(raw[body+16:]),
					CenterHz: binary.LittleEndian.Uint32(raw[body+32:]),
					ADHz:     binary.LittleEndian.Uint32(raw[body+36:]),
					BwHz:     binary.LittleEndian.Uint32(raw[body+44:]),
				}
			}
		}
		off = body + sz
		if sz%2 == 1 {
			off++ // chunks are word-aligned
		}
	}
	if d.FormatTag == 0 {
		return nil, fmt.Errorf("%s: no fmt chunk", path)
	}
	switch {
	case d.FormatTag == wavFmtPCM && d.Bits == 16:
		d.I16 = make([]int16, len(data)/2)
		for i := range d.I16 {
			d.I16[i] = int16(binary.LittleEndian.Uint16(data[2*i:]))
		}
	case d.FormatTag == wavFmtFloat && d.Bits == 32:
		d.F32 = make([]float32, len(data)/4)
		for i := range d.F32 {
			d.F32[i] = math.Float32frombits(binary.LittleEndian.Uint32(data[4*i:]))
		}
	default:
		return nil, fmt.Errorf("%s: unsupported format tag %d / %d-bit", path, d.FormatTag, d.Bits)
	}
	return d, nil
}

// WriteWAVInt16 is the exported wrapper over the internal 16-bit-PCM writer, for the
// wav-to-decoder utility (which writes the mono temp WAVs the decoders read).
func WriteWAVInt16(path string, rate, numCh int, samples []int16) error {
	return writeWAV(path, rate, numCh, samples)
}

// PeakNormalizeInt16 is the exported wrapper over the internal peak-normaliser, so
// wav-to-decoder produces decoder audio identical to the farm's.
func PeakNormalizeInt16(y []float64) []int16 { return peakNormalizeInt16(y) }

// WSJTXStamp formats a window-start time as WSJT-X names its WAVs: YYMMDD_HHMMSS for
// fast modes, YYMMDD_HHMM (no seconds) for slow modes (T/R >= 60 s).  jt9/wsprd read
// the window time from this tail, so wav-to-decoder names its temp WAVs with it.
func WSJTXStamp(t time.Time, slow bool) string {
	if slow {
		return t.UTC().Format("060102_1504")
	}
	return t.UTC().Format("060102_150405")
}
