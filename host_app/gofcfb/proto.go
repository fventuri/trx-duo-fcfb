// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// v3/v4 wire protocol for the fcfb board: TCP control (request + accept + run
// list) and the UDP record stream.  A direct port of host_app/ingest.py /
// host/fcfb_net.h.  Both peers are little-endian; structs go on the wire packed.
package fcfb

import (
	"encoding/binary"
	"fmt"
	"io"
	"math"
	"net"
	"time"
)

const (
	tcpPort  = 7373
	fs       = 125e6
	nFFT     = 4096
	binW     = fs / nFFT // 30517.578125 Hz
	binRate  = 40000.0   // FS/R blocks per second (R=3125)
	rHop     = 3125
	guardDef = 3

	reqV3    = 3
	reqV4    = 4
	streamV3 = 3

	protoVer        = 4 // request-protocol version we speak (params query)
	paramVerUnknown = 0 // board did not report analysis params (old gw/server)
	paramsRespLen   = 100
)

var (
	reqMagic  = [4]byte{'F', 'R', 'E', 'Q'}
	udpMagic  = [4]byte{'F', 'U', 'D', 'P'}
	acceptTag = []byte("FCFBv1\x00\x00")
	prmMagic  = [4]byte{'F', 'P', 'R', 'M'}
	paramsTag = []byte("FCFBP\x00\x00\x00")
)

// Params is the board's fixed analysis parameters, from the FPRM params query.
// A host synthesis kernel must be fit for the same R/T. ParamVer == 0 means the
// board could not report them (old gateware/server) -> the caller cannot verify.
type Params struct {
	ProtoVer, ParamVer   int
	R, T, N              int
	Wmax, BinWidth, NDDS int
	Fs                   float64
	GuardDefault         int
	Features             uint32
	BuildID              string
	GatewareName         string
}

// queryParams opens a short TCP connection, sends the 36-byte FPRM params query,
// and reads the fcfb_params reply. ok=false with a nil error means the server is
// old / rejected the query (params unavailable) -> the caller warns and proceeds
// without a kernel check. A non-nil error is a connect/IO failure.
func queryParams(ip string, timeout time.Duration) (Params, bool, error) {
	return queryParamsAddr(fmt.Sprintf("%s:%d", ip, tcpPort), timeout)
}

// queryParamsAddr is queryParams against an explicit host:port (used by tests).
func queryParamsAddr(addr string, timeout time.Duration) (Params, bool, error) {
	ts, err := net.DialTimeout("tcp", addr, timeout)
	if err != nil {
		return Params{}, false, err
	}
	defer ts.Close()
	_ = ts.SetDeadline(time.Now().Add(timeout))
	// A 36-byte fcfb_req-shaped frame with FPRM magic + our proto version; the
	// server's first read is sizeof(fcfb_req)=36 bytes, so we must send all 36.
	req := make([]byte, 36)
	copy(req[0:4], prmMagic[:])
	binary.LittleEndian.PutUint16(req[4:6], uint16(protoVer))
	if _, err := ts.Write(req); err != nil {
		return Params{}, false, err
	}
	resp := make([]byte, paramsRespLen)
	if _, err := io.ReadFull(ts, resp); err != nil {
		// Old server dropped the unknown-magic connection -> params unavailable.
		return Params{}, false, nil
	}
	if string(resp[0:8]) != string(paramsTag) {
		return Params{}, false, nil
	}
	u16 := func(o int) int { return int(binary.LittleEndian.Uint16(resp[o : o+2])) }
	u32 := func(o int) int { return int(binary.LittleEndian.Uint32(resp[o : o+4])) }
	cstr := func(o, n int) string {
		b := resp[o : o+n]
		if i := indexByte(b, 0); i >= 0 {
			b = b[:i]
		}
		return string(b)
	}
	p := Params{
		ProtoVer:     u16(8),
		ParamVer:     u16(10),
		R:            u32(12),
		T:            u32(16),
		N:            u32(20),
		Wmax:         u32(24),
		BinWidth:     u32(28),
		NDDS:         u32(32),
		Fs:           math.Float64frombits(binary.LittleEndian.Uint64(resp[36:44])),
		GuardDefault: u32(44),
		Features:     binary.LittleEndian.Uint32(resp[48:52]),
		BuildID:      cstr(52, 16),
		GatewareName: cstr(68, 32),
	}
	return p, true, nil
}

func indexByte(b []byte, c byte) int {
	for i := range b {
		if b[i] == c {
			return i
		}
	}
	return -1
}

// Chan is one requested channel's occupied band + optional bench DDS tone.
type Chan struct {
	Flo, Fhi float64
	AdcBits  int
	DdsK     int
	DdsPhInc uint32
}

// Run is one granted contiguous bin range (k0, W, adc_bits) from the accept.
type Run struct{ K0, W, Adc int }

// Meta is everything the accept tells us about the granted stream.
type Meta struct {
	N        int
	Fs       float64
	AdcMask  int
	Wa, Wb   int
	BinScale float64
	BinWidth int
	Runs     []Run
	Ka, Kb   []int // absolute-k bin lists per ADC (ascending)
	Rec      int   // bytes per record
}

func binBytes(bw int) int { return bw / 8 }

// recordSizeAB mirrors sim/fcfb_stream_read.record_size_ab.
func recordSizeAB(wa, wb, bw int) int {
	payload := (wa + wb) * binBytes(bw) * 2
	return 8 + ((payload + 3) &^ 3) // u64 seq + {I,Q} runs, padded to 32-bit
}

func runsToBins(runs []Run) (ka, kb []int) {
	seenA := map[int]bool{}
	seenB := map[int]bool{}
	for _, r := range runs {
		ab := r.Adc
		if ab == 0 {
			ab = 1
		}
		for k := r.K0; k < r.K0+r.W; k++ {
			if ab&1 != 0 && !seenA[k] {
				seenA[k] = true
				ka = append(ka, k)
			}
			if ab&2 != 0 && !seenB[k] {
				seenB[k] = true
				kb = append(kb, k)
			}
		}
	}
	sortInts(ka)
	sortInts(kb)
	return
}

func sortInts(a []int) {
	for i := 1; i < len(a); i++ {
		for j := i; j > 0 && a[j-1] > a[j]; j-- {
			a[j-1], a[j] = a[j], a[j-1]
		}
	}
}

// sendRequest writes the v3 (or v4 for a custom guard) selection request.
func sendRequest(ts net.Conn, chans []Chan, udpPort, shift, guard int) error {
	if guard < 0 {
		guard = 0
	}
	ver := reqV3
	if guard != guardDef {
		ver = reqV4
	}
	ch0 := chans[0]
	buf := make([]byte, 0, 36+2+2)
	buf = append(buf, reqMagic[:]...)
	buf = le16(buf, uint16(ver))
	buf = le16(buf, uint16(ch0.AdcBits&0x3))
	buf = le64f(buf, ch0.Flo)
	buf = le64f(buf, ch0.Fhi)
	buf = le16(buf, uint16(udpPort))
	buf = le16(buf, uint16(ch0.DdsK))
	buf = le32(buf, uint32(int32(shift)))
	buf = le32(buf, ch0.DdsPhInc)
	buf = le16(buf, uint16(len(chans)-1)) // nextra
	if ver == reqV4 {
		buf = le16(buf, uint16(guard&0xffff))
	}
	if _, err := ts.Write(buf); err != nil {
		return err
	}
	for _, c := range chans[1:] {
		cb := make([]byte, 0, 24)
		cb = le64f(cb, c.Flo)
		cb = le64f(cb, c.Fhi)
		cb = le16(cb, uint16(c.AdcBits&0x3))
		cb = le16(cb, uint16(c.DdsK))
		cb = le32(cb, c.DdsPhInc)
		if _, err := ts.Write(cb); err != nil {
			return err
		}
	}
	return nil
}

// readAccept reads the 56-byte v3 accept + the run list.
func readAccept(ts net.Conn) (Meta, error) {
	var m Meta
	hdr := make([]byte, 56)
	if _, err := io.ReadFull(ts, hdr); err != nil {
		return m, fmt.Errorf("short accept: %w", err)
	}
	if string(hdr[:8]) != string(acceptTag) {
		return m, fmt.Errorf("bad accept magic %q (admission rejected?)", hdr[:8])
	}
	ver := int(binary.LittleEndian.Uint32(hdr[8:12]))
	if ver != streamV3 {
		return m, fmt.Errorf("unexpected stream version %d", ver)
	}
	m.N = int(binary.LittleEndian.Uint32(hdr[12:16]))
	m.Fs = math.Float64frombits(binary.LittleEndian.Uint64(hdr[16:24]))
	m.AdcMask = int(binary.LittleEndian.Uint32(hdr[24:28]))
	m.Wa = int(binary.LittleEndian.Uint32(hdr[28:32]))
	m.Wb = int(binary.LittleEndian.Uint32(hdr[32:36]))
	// hdr[36:40] = nb (unused)
	m.BinScale = math.Float64frombits(binary.LittleEndian.Uint64(hdr[40:48]))
	m.BinWidth = int(binary.LittleEndian.Uint32(hdr[48:52]))
	nruns := int(binary.LittleEndian.Uint32(hdr[52:56]))
	for i := 0; i < nruns; i++ {
		rb := make([]byte, 12)
		if _, err := io.ReadFull(ts, rb); err != nil {
			return m, fmt.Errorf("short run list: %w", err)
		}
		m.Runs = append(m.Runs, Run{
			K0:  int(binary.LittleEndian.Uint32(rb[0:4])),
			W:   int(binary.LittleEndian.Uint32(rb[4:8])),
			Adc: int(binary.LittleEndian.Uint32(rb[8:12])),
		})
	}
	m.Ka, m.Kb = runsToBins(m.Runs)
	m.Rec = recordSizeAB(m.Wa, m.Wb, m.BinWidth)
	return m, nil
}

// decodeSignedLE reads a little-endian signed integer of bb bytes.  The common
// wire widths (24-bit int24 streams, 16-bit) get a branch-free fast path; the
// generic loop covers any other width.
func decodeSignedLE(b []byte, bb int) float64 {
	switch bb {
	case 3: // int24: assemble then sign-extend bit 23 with an arithmetic shift
		v := int32(b[0]) | int32(b[1])<<8 | int32(b[2])<<16
		return float64((v << 8) >> 8)
	case 2:
		return float64(int16(uint16(b[0]) | uint16(b[1])<<8))
	}
	var v int64
	for i := 0; i < bb; i++ {
		v |= int64(b[i]) << (8 * i)
	}
	sbit := int64(1) << (8*bb - 1)
	v = (v ^ sbit) - sbit // sign-extend
	return float64(v)
}

// parseDatagram validates a UDP datagram and returns its generation and the
// per-record decoded complex bins (A-run then B-run), one column per record.
// cols is [nrec][Wa+Wb] complex; caller splits into A/B and applies bin_scale.
func parseDatagram(pkt []byte, m Meta) (gen int, ncols int, decode func(colFn func(rec int, seq uint64, colA, colB []complex128)), ok bool) {
	if len(pkt) < 12 || pkt[0] != 'F' || pkt[1] != 'U' || pkt[2] != 'D' || pkt[3] != 'P' {
		return 0, 0, nil, false
	}
	hgen := int(binary.LittleEndian.Uint16(pkt[6:8]))
	hrec := int(binary.LittleEndian.Uint16(pkt[8:10]))
	nrec := int(binary.LittleEndian.Uint16(pkt[10:12]))
	if hrec != m.Rec {
		return 0, 0, nil, false
	}
	bb := binBytes(m.BinWidth)
	runA := m.Wa * bb * 2
	body := pkt[12:]
	// cap nrec to what actually arrived
	if avail := len(body) / m.Rec; nrec > avail {
		nrec = avail
	}
	decode = func(colFn func(rec int, seq uint64, colA, colB []complex128)) {
		colA := make([]complex128, m.Wa)
		colB := make([]complex128, m.Wb)
		for r := 0; r < nrec; r++ {
			seq := binary.LittleEndian.Uint64(body[r*m.Rec : r*m.Rec+8])
			off := r*m.Rec + 8 // past the u64 seq
			p := off
			for j := 0; j < m.Wa; j++ {
				iv := decodeSignedLE(body[p:p+bb], bb)
				qv := decodeSignedLE(body[p+bb:p+2*bb], bb)
				colA[j] = complex(iv*m.BinScale, qv*m.BinScale)
				p += 2 * bb
			}
			p = off + runA
			for j := 0; j < m.Wb; j++ {
				iv := decodeSignedLE(body[p:p+bb], bb)
				qv := decodeSignedLE(body[p+bb:p+2*bb], bb)
				colB[j] = complex(iv*m.BinScale, qv*m.BinScale)
				p += 2 * bb
			}
			colFn(r, seq, colA, colB)
		}
	}
	return hgen, nrec, decode, true
}

// dialBoard opens the UDP data socket (bound first) and the TCP control socket,
// sends the request, and reads the accept.  Returns both sockets + meta.
func dialBoard(ip string, chans []Chan, udpPort, shift, guard, rcvbuf int, timeout time.Duration) (*net.UDPConn, net.Conn, Meta, error) {
	uaddr, err := net.ResolveUDPAddr("udp", fmt.Sprintf(":%d", udpPort))
	if err != nil {
		return nil, nil, Meta{}, err
	}
	us, err := net.ListenUDP("udp", uaddr)
	if err != nil {
		return nil, nil, Meta{}, err
	}
	_ = us.SetReadBuffer(rcvbuf)
	ts, err := net.DialTimeout("tcp", fmt.Sprintf("%s:%d", ip, tcpPort), timeout)
	if err != nil {
		us.Close()
		return nil, nil, Meta{}, err
	}
	if err := sendRequest(ts, chans, udpPort, shift, guard); err != nil {
		us.Close()
		ts.Close()
		return nil, nil, Meta{}, err
	}
	m, err := readAccept(ts)
	if err != nil {
		us.Close()
		ts.Close()
		return nil, nil, Meta{}, err
	}
	return us, ts, m, nil
}

func le16(b []byte, v uint16) []byte { return binary.LittleEndian.AppendUint16(b, v) }
func le32(b []byte, v uint32) []byte { return binary.LittleEndian.AppendUint32(b, v) }
func le64f(b []byte, v float64) []byte {
	return binary.LittleEndian.AppendUint64(b, math.Float64bits(v))
}
