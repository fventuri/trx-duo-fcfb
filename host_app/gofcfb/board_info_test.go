// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Tests for the BINF board-info query round-trip + decode (temperature/voltages/
// identity), the NaN sentinel, and the safe "unavailable" degradation.
package fcfb

import (
	"encoding/binary"
	"math"
	"testing"
	"time"
)

// encodeBoardInfo builds a 156-byte fcfb_board_info reply matching host/fcfb_net.h.
func encodeBoardInfo(temp float64, volts []float64, fpgaid int, fs float64, model, gw, hwrev string) []byte {
	b := make([]byte, boardInfoRespLen)
	copy(b[0:8], boardInfoTag)
	binary.LittleEndian.PutUint16(b[8:10], boardInfoVer)
	binary.LittleEndian.PutUint16(b[10:12], uint16(len(volts)))
	binary.LittleEndian.PutUint32(b[12:16], math.Float32bits(float32(temp)))
	for i, v := range volts {
		binary.LittleEndian.PutUint32(b[16+4*i:20+4*i], math.Float32bits(float32(v)))
	}
	binary.LittleEndian.PutUint32(b[48:52], uint32(fpgaid))
	binary.LittleEndian.PutUint64(b[52:60], math.Float64bits(fs))
	copy(b[60:92], model)
	copy(b[92:124], gw)
	copy(b[124:156], hwrev)
	return b
}

const boardInfoVer = 1

func TestQueryBoardInfoRoundTrip(t *testing.T) {
	volts := []float64{1.000, 1.800, 1.000, 1.000, 1.800, 1.500, 1.250, 0.000}
	addr, stop := rawReplyServer(encodeBoardInfo(46.8, volts, 2, 125e6, "TRX-duo SDR", "fcfb_stage1", "STEM_125-14_LN_v1.1"))
	defer stop()
	bi, ok, err := queryBoardInfoAddr(addr, 2*time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if !ok {
		t.Fatal("expected ok=true")
	}
	if math.Abs(bi.TempC-46.8) > 1e-3 {
		t.Errorf("temp %.3f, want 46.8", bi.TempC)
	}
	if bi.FpgaID != 2 || bi.FpgaName() != "xc7z010" {
		t.Errorf("fpga: id=%d name=%q, want 2/xc7z010", bi.FpgaID, bi.FpgaName())
	}
	if bi.Fs != 125e6 || bi.Model != "TRX-duo SDR" || bi.Gateware != "fcfb_stage1" ||
		bi.HwRev != "STEM_125-14_LN_v1.1" {
		t.Errorf("identity: fs=%.0f model=%q gw=%q hwrev=%q", bi.Fs, bi.Model, bi.Gateware, bi.HwRev)
	}
	if len(bi.Volt) != len(volts) {
		t.Fatalf("volt count %d, want %d", len(bi.Volt), len(volts))
	}
	for i, v := range volts {
		if math.Abs(bi.Volt[i]-v) > 1e-3 {
			t.Errorf("volt[%d] (%s) = %.3f, want %.3f", i, bi.VoltName[i], bi.Volt[i], v)
		}
		if bi.VoltName[i] != voltNames[i] {
			t.Errorf("volt[%d] name %q, want %q", i, bi.VoltName[i], voltNames[i])
		}
	}
}

func TestQueryBoardInfoNaN(t *testing.T) {
	// A sensor the board could not read comes back NaN; the client must preserve it.
	volts := []float64{math.NaN(), 1.8, 1.0, 1.0, 1.8, 1.5, 1.25, 0.0}
	addr, stop := rawReplyServer(encodeBoardInfo(math.NaN(), volts, 2, 125e6, "TRX-duo SDR", "fcfb_stage1", "STEM_125-14_LN_v1.1"))
	defer stop()
	bi, ok, err := queryBoardInfoAddr(addr, 2*time.Second)
	if err != nil || !ok {
		t.Fatalf("query: ok=%v err=%v", ok, err)
	}
	if !math.IsNaN(bi.TempC) {
		t.Errorf("temp should be NaN, got %v", bi.TempC)
	}
	if !math.IsNaN(bi.Volt[0]) {
		t.Errorf("volt[0] should be NaN, got %v", bi.Volt[0])
	}
	if got := tempStr(bi.TempC); got != "n/a" {
		t.Errorf("tempStr(NaN) = %q, want n/a", got)
	}
	_ = bi.Summary() // must not panic with a NaN present
}

func TestQueryBoardInfoUnavailable(t *testing.T) {
	// Old server (no info port) drops the connection without replying -> ok=false.
	addr, stop := rawReplyServer(nil)
	defer stop()
	_, ok, err := queryBoardInfoAddr(addr, 2*time.Second)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if ok {
		t.Error("expected ok=false against a silent (old) server")
	}
}

func TestQueryBoardInfoNoConnect(t *testing.T) {
	// Nothing listening -> unavailable (ok=false), no error (safe degradation).
	_, ok, err := queryBoardInfoAddr("127.0.0.1:1", 500*time.Millisecond)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if ok {
		t.Error("expected ok=false when the info port is unreachable")
	}
}
