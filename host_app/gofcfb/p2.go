// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// openHPSDR Ethernet Protocol 2 ("new protocol" / P2) packet layer.  Pure,
// socket-free framing so the fcfb host app can present itself to any P2 client
// (piHPSDR, Thetis, linhpsdr, ...) as an HPSDR radio: answer discovery, parse the
// client's control packets (which DDC/receiver, centre frequency, sample rate), and
// build the RX DDC I/Q data packets the radio emits.
//
// Byte layouts are taken verbatim from piHPSDR's new_protocol.c / new_discovery.c
// (the canonical reference); every builder here has an inverse parser and the two
// are round-trip tested in p2_test.go.  All multi-byte fields are big-endian.  Go
// port of host_app/hpsdr_p2.py.
package fcfb

import (
	"encoding/binary"
	"math"
)

// ports (new_protocol.h): client -> radio.
const (
	generalFromHostPort = 1024 // also discovery + programming
	rxSpecFromHostPort  = 1025
	txSpecFromHostPort  = 1026
	highPriFromHostPort = 1027
	audioFromHostPort   = 1028
	txIQFromHostPort    = 1029
	rxIQToHostPort0     = 1035 // DDC k -> rxIQToHostPort0 + k

	maxDDC = 8 // we parse/serve up to this many DDCs
)

// HPSDR P2 board ids (discovery byte[11]; clients compute device = board_id + 1000,
// see new_discovery.c).  We advertise ANGELIA because the TRX-duo has two distinct
// ADCs: Angelia (ANAN-100D) is the canonical dual-ADC P2 model, so clients expose
// ADC1 + diversity and let a receiver select adc=1 (our ADC B); Hermes is single-ADC
// and would hide the second ADC entirely.
const (
	boardAtlas   = 0
	boardHermes  = 1
	boardHermes2 = 2
	boardAngelia = 3
	boardOrion   = 4
	boardOrion2  = 5
)

// HPSDR P2 uses a 122.88 MHz reference clock; a DDC centre frequency is carried as a
// 32-bit phase word = freq * 2^32 / 122.88e6.  (Our board samples at 125 MHz, but we
// honour the client's Hz value: decode phase -> Hz here, then ask the board for that
// RF frequency.  Client max ~61.44 MHz < our 62.5 MHz Nyquist, so all reachable.)
const p2Clock = 122880000.0

var phaseScale = float64(int64(1)<<32) / p2Clock // 34.9525333...

const (
	s24Scale = float64(int64(1) << 23) // 24-bit sample full scale (piHPSDR /2^23)
	s24Min   = -(1 << 23)
	s24Max   = (1 << 23) - 1

	// samples per RX I/Q frame for the standard 1444-byte UDP payload (16 + 6*238).
	samplesPerFrame = (1444 - 16) / 6 // 238
)

// ---------------------------------------------------------------- frequency
func freqFromPhase(phase uint32) float64 { return float64(phase) / phaseScale }

func phaseFromFreq(hz float64) uint32 { return uint32(int64(math.Round(hz*phaseScale)) & 0xFFFFFFFF) }

// ---------------------------------------------------------------- discovery
// isDiscovery reports whether pkt is a P2 discovery request ([0:4]=0, [4]=0x02).
func isDiscovery(pkt []byte) bool {
	return len(pkt) >= 5 && pkt[0] == 0 && pkt[1] == 0 && pkt[2] == 0 && pkt[3] == 0 && pkt[4] == 0x02
}

// buildDiscoveryReply builds the 60-byte discovery reply.  status 2=available,
// 3=in use.  board_id 3=Angelia (device = board_id+1000 in piHPSDR), the dual-ADC
// model so clients expose ADC1 + diversity.  Fields consumed by new_discovery.c:
// status@4, MAC@5..10, board_id@11, p2_version@12, fw_version@13, n_ddc@20.
func buildDiscoveryReply(mac [6]byte, boardID, status, p2Version, fwVersion, nDDC int) []byte {
	b := make([]byte, 60)
	b[4] = byte(status)
	copy(b[5:11], mac[:])
	b[11] = byte(boardID)
	b[12] = byte(p2Version)
	b[13] = byte(fwVersion)
	b[20] = byte(nDDC)
	return b
}

type discoveryReply struct {
	status, boardID, device, p2Version, fwVersion, nDDC int
	mac                                                 [6]byte
}

// parseDiscoveryReply inverts buildDiscoveryReply (client harness / tests).
func parseDiscoveryReply(pkt []byte) (discoveryReply, bool) {
	if len(pkt) < 21 || pkt[0] != 0 || pkt[1] != 0 || pkt[2] != 0 || pkt[3] != 0 {
		return discoveryReply{}, false
	}
	var d discoveryReply
	d.status = int(pkt[4])
	copy(d.mac[:], pkt[5:11])
	d.boardID = int(pkt[11])
	d.device = int(pkt[11]) + 1000
	d.p2Version = int(pkt[12])
	d.fwVersion = int(pkt[13])
	d.nDDC = int(pkt[20])
	return d, true
}

// ---------------------------------------------------------------- control (client -> radio)

type highPriority struct {
	seq       uint32
	running   bool
	ddcFreqHz []float64
}

// parseHighPriority parses a high-priority packet (port 1027): run bit + per-DDC
// centre frequency.  seq@0..3, running=bit0 of byte 4, DDC k phase word at 9+4k.
func parseHighPriority(pkt []byte) highPriority {
	hp := highPriority{}
	if len(pkt) >= 4 {
		hp.seq = binary.BigEndian.Uint32(pkt[0:4])
	}
	if len(pkt) >= 5 {
		hp.running = pkt[4]&0x01 != 0
	}
	for k := 0; k < maxDDC; k++ {
		off := 9 + 4*k
		if off+4 > len(pkt) {
			break
		}
		hp.ddcFreqHz = append(hp.ddcFreqHz, freqFromPhase(binary.BigEndian.Uint32(pkt[off:off+4])))
	}
	return hp
}

// buildHighPriority builds a high-priority packet (client side; harness/tests).
func buildHighPriority(seq uint32, running bool, ddcFreqHz []float64) []byte {
	b := make([]byte, 1444)
	binary.BigEndian.PutUint32(b[0:4], seq)
	if running {
		b[4] = 1
	}
	for k, hz := range ddcFreqHz {
		binary.BigEndian.PutUint32(b[9+4*k:9+4*k+4], phaseFromFreq(hz))
	}
	return b
}

type ddcConfig struct{ adc, rate, bits int }

type receiveSpecific struct {
	seq    uint32
	nAdc   int
	enable int
	ddcs   map[int]ddcConfig
}

// parseReceiveSpecific parses a receiver-specific packet (port 1025): which DDCs are
// enabled and, per DDC, the ADC, sample rate and bit depth.  n_adc@4, enable
// bitmap@7, per-DDC block at 17+6*ddc = {adc@+0, rate/1000 (u16 BE)@+1..+2, bits@+5}.
func parseReceiveSpecific(pkt []byte) receiveSpecific {
	rs := receiveSpecific{ddcs: map[int]ddcConfig{}}
	if len(pkt) >= 4 {
		rs.seq = binary.BigEndian.Uint32(pkt[0:4])
	}
	if len(pkt) >= 5 {
		rs.nAdc = int(pkt[4])
	}
	if len(pkt) >= 8 {
		rs.enable = int(pkt[7])
	}
	for ddc := 0; ddc < maxDDC; ddc++ {
		if rs.enable&(1<<ddc) == 0 {
			continue
		}
		base := 17 + 6*ddc
		if base+6 > len(pkt) {
			break
		}
		rs.ddcs[ddc] = ddcConfig{
			adc:  int(pkt[base]),
			rate: int(binary.BigEndian.Uint16(pkt[base+1:base+3])) * 1000,
			bits: int(pkt[base+5]),
		}
	}
	return rs
}

// buildReceiveSpecific builds a receiver-specific packet (client side).
func buildReceiveSpecific(seq uint32, ddcs map[int]ddcConfig, nAdc int) []byte {
	b := make([]byte, 1444)
	binary.BigEndian.PutUint32(b[0:4], seq)
	b[4] = byte(nAdc)
	enable := 0
	for ddc, c := range ddcs {
		enable |= 1 << ddc
		base := 17 + 6*ddc
		b[base] = byte(c.adc)
		binary.BigEndian.PutUint16(b[base+1:base+3], uint16(c.rate/1000))
		bits := c.bits
		if bits == 0 {
			bits = 24
		}
		b[base+5] = byte(bits)
	}
	b[7] = byte(enable)
	return b
}

// ---------------------------------------------------------------- RX I/Q data (radio -> client)

// enc24BE writes x in [-1,1] as a big-endian signed 24-bit two's-complement sample.
func enc24BE(dst []byte, x float64) {
	v := int32(math.Round(x * s24Scale))
	if v < s24Min {
		v = s24Min
	} else if v > s24Max {
		v = s24Max
	}
	u := uint32(v) & 0xFFFFFF
	dst[0] = byte(u >> 16)
	dst[1] = byte(u >> 8)
	dst[2] = byte(u)
}

// dec24BE decodes a big-endian signed 24-bit sample to a float /2^23 (piHPSDR's
// process_iq_data decode).
func dec24BE(b []byte) float64 {
	v := int32(b[0])<<16 | int32(b[1])<<8 | int32(b[2])
	return float64((v<<8)>>8) / s24Scale // sign-extend bit 23
}

// buildRxIQ builds ONE RX DDC I/Q packet: seq(4)+timestamp(8)+bitspersample(2)+
// samplesperframe(2) then, per sample, 3-byte BE I (left) + 3-byte BE Q (right).
// iq: complex samples with |.|<=1.  Standard 1444-byte payload = 238 samples.
func buildRxIQ(seq uint32, iq []complex128, bitsPerSample int, timestamp uint64) []byte {
	if bitsPerSample == 0 {
		bitsPerSample = 24
	}
	n := len(iq)
	b := make([]byte, 16+6*n)
	binary.BigEndian.PutUint32(b[0:4], seq)
	binary.BigEndian.PutUint64(b[4:12], timestamp)
	binary.BigEndian.PutUint16(b[12:14], uint16(bitsPerSample))
	binary.BigEndian.PutUint16(b[14:16], uint16(n))
	for i, s := range iq {
		off := 16 + 6*i
		enc24BE(b[off:off+3], real(s))
		enc24BE(b[off+3:off+6], imag(s))
	}
	return b
}

type rxIQ struct {
	seq           uint32
	timestamp     uint64
	bitsPerSample int
	n             int
	iq            []complex128
}

// parseRxIQ inverts buildRxIQ, decoding exactly as piHPSDR's process_iq_data.
func parseRxIQ(pkt []byte) (rxIQ, bool) {
	if len(pkt) < 16 {
		return rxIQ{}, false
	}
	var r rxIQ
	r.seq = binary.BigEndian.Uint32(pkt[0:4])
	r.timestamp = binary.BigEndian.Uint64(pkt[4:12])
	r.bitsPerSample = int(binary.BigEndian.Uint16(pkt[12:14]))
	r.n = int(binary.BigEndian.Uint16(pkt[14:16]))
	if 16+6*r.n > len(pkt) {
		return rxIQ{}, false
	}
	r.iq = make([]complex128, r.n)
	for i := 0; i < r.n; i++ {
		off := 16 + 6*i
		r.iq[i] = complex(dec24BE(pkt[off:off+3]), dec24BE(pkt[off+3:off+6]))
	}
	return r, true
}
