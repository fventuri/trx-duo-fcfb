// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Tests for the gateware param handshake: kernel-filename parsing, the FPRM
// params query round-trip, and the verifyKernel match/mismatch/skip logic.
package fcfb

import (
	"encoding/binary"
	"io"
	"math"
	"net"
	"strings"
	"testing"
	"time"
)

func TestParseKernelName(t *testing.T) {
	cases := []struct {
		name    string
		r, t, k int
		ok      bool
	}{
		{"dual_R3125_T4_K6.f64", 3125, 4, 6, true},
		{"dual_R9999_T4_K6.f64", 9999, 4, 6, true},
		{"dual_R3125_T8_K12.f64", 3125, 8, 12, true},
		{"kernel.f64", 0, 0, 0, false},
		{"dual_R3125_T4.f64", 0, 0, 0, false},
		{"dual_R3125_T4_K6.bin", 0, 0, 0, false},
	}
	for _, c := range cases {
		r, tt, k, ok := parseKernelName(c.name)
		if ok != c.ok || r != c.r || tt != c.t || k != c.k {
			t.Errorf("parseKernelName(%q) = (%d,%d,%d,%v), want (%d,%d,%d,%v)",
				c.name, r, tt, k, ok, c.r, c.t, c.k, c.ok)
		}
	}
}

// encodeParams builds a 100-byte fcfb_params reply matching host/fcfb_net.h.
func encodeParams(p Params) []byte {
	b := make([]byte, paramsRespLen)
	copy(b[0:8], paramsTag)
	binary.LittleEndian.PutUint16(b[8:10], uint16(p.ProtoVer))
	binary.LittleEndian.PutUint16(b[10:12], uint16(p.ParamVer))
	binary.LittleEndian.PutUint32(b[12:16], uint32(p.R))
	binary.LittleEndian.PutUint32(b[16:20], uint32(p.T))
	binary.LittleEndian.PutUint32(b[20:24], uint32(p.N))
	binary.LittleEndian.PutUint32(b[24:28], uint32(p.Wmax))
	binary.LittleEndian.PutUint32(b[28:32], uint32(p.BinWidth))
	binary.LittleEndian.PutUint32(b[32:36], uint32(p.NDDS))
	binary.LittleEndian.PutUint64(b[36:44], math.Float64bits(p.Fs))
	binary.LittleEndian.PutUint32(b[44:48], uint32(p.GuardDefault))
	binary.LittleEndian.PutUint32(b[48:52], p.Features)
	copy(b[52:68], p.BuildID)
	copy(b[68:100], p.GatewareName)
	return b
}

// rawReplyServer serves one connection: it reads the 36-byte request and, if resp
// is non-nil, writes it back. A nil resp mimics an old server that drops an
// unknown-magic connection without replying. Returns host:port.
func rawReplyServer(resp []byte) (string, func()) {
	ln, _ := net.Listen("tcp", "127.0.0.1:0")
	go func() {
		conn, err := ln.Accept()
		if err != nil {
			return
		}
		defer conn.Close()
		buf := make([]byte, 36)
		io.ReadFull(conn, buf)
		if resp != nil {
			conn.Write(resp)
		}
	}()
	return ln.Addr().String(), func() { ln.Close() }
}

func TestQueryParamsRoundTrip(t *testing.T) {
	want := Params{
		ProtoVer: 4, ParamVer: 1, R: 3125, T: 4, N: 4096,
		Wmax: 512, BinWidth: 24, NDDS: 0, Fs: 125e6, GuardDefault: 3,
		GatewareName: "fcfb_stage1",
	}
	addr, stop := rawReplyServer(encodeParams(want))
	defer stop()
	got, ok, err := queryParamsAddr(addr, 2*time.Second)
	if err != nil {
		t.Fatal(err)
	}
	if !ok {
		t.Fatal("expected ok=true")
	}
	if got != want {
		t.Errorf("decoded %+v, want %+v", got, want)
	}
}

func TestQueryParamsOldServer(t *testing.T) {
	// Old server drops the connection without replying -> ok=false, no error.
	addr, stop := rawReplyServer(nil)
	defer stop()
	_, ok, err := queryParamsAddr(addr, 2*time.Second)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if ok {
		t.Error("expected ok=false against an old (silent) server")
	}
}

func TestVerifyKernelLogic(t *testing.T) {
	saveR, saveT, saveParsed := kernelR, kernelT, kernelNameParsed
	defer func() { kernelR, kernelT, kernelNameParsed = saveR, saveT, saveParsed }()

	match := encodeParams(Params{ProtoVer: 4, ParamVer: 1, R: 3125, T: 4, N: 4096, GatewareName: "fcfb_stage1"})
	mism := encodeParams(Params{ProtoVer: 4, ParamVer: 1, R: 9999, T: 4, N: 4096})
	unknown := encodeParams(Params{ProtoVer: 4, ParamVer: 0, R: 3125, T: 4, N: 4096})

	// Matching board + parsed kernel -> nil.
	kernelR, kernelT, kernelNameParsed = 3125, 4, true
	if err := verifyKernelAddr(mustServe(t, match)); err != nil {
		t.Errorf("match: unexpected error %v", err)
	}
	// Mismatched board -> fatal error.
	kernelR, kernelT, kernelNameParsed = 3125, 4, true
	if err := verifyKernelAddr(mustServe(t, mism)); err == nil {
		t.Error("mismatch: expected an error, got nil")
	} else if !strings.Contains(err.Error(), "kernel mismatch") {
		t.Errorf("mismatch: wrong error %v", err)
	}
	// Unknown params -> warn, nil.
	kernelR, kernelT, kernelNameParsed = 3125, 4, true
	if err := verifyKernelAddr(mustServe(t, unknown)); err != nil {
		t.Errorf("unknown params: expected nil, got %v", err)
	}
	// Unparseable kernel filename -> skip check, nil (even vs a mismatched board).
	kernelR, kernelT, kernelNameParsed = 0, 0, false
	if err := verifyKernelAddr(mustServe(t, mism)); err != nil {
		t.Errorf("unparseable kernel: expected nil, got %v", err)
	}
}

func mustServe(t *testing.T, resp []byte) string {
	t.Helper()
	addr, stop := rawReplyServer(resp)
	t.Cleanup(stop)
	return addr
}
