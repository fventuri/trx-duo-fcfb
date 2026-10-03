// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// End-to-end loopback test: a fake fcfb board streams a constant centre-bin through
// the real emitter to the real P2 client harness.  A constant single bin at kc is a
// tone at that bin's centre frequency; a DDC tuned to kc must recover it at DC.  This
// exercises the whole wire path offline -- discovery, control parse, planner union
// request, board stream decode, multi-phase synth, resample, RX I/Q framing, client
// demux -- none of which the pure unit tests cover.
package fcfb

import (
	"encoding/binary"
	"io"
	"math"
	"math/cmplx"
	"net"
	"sync"
	"testing"
	"time"
)

// fakeBoard is a minimal fcfb v3 board server for the loopback test.
type fakeBoard struct {
	ln       net.Listener
	binValue int32 // I value written into the centre bin each block
	stop     chan struct{}
	wg       sync.WaitGroup
}

func startFakeBoard(t *testing.T, binValue int32) *fakeBoard {
	t.Helper()
	ln, err := net.Listen("tcp", "127.0.0.1:7373")
	if err != nil {
		t.Skipf("cannot bind board TCP :7373 (%v); skipping loopback", err)
	}
	fb := &fakeBoard{ln: ln, binValue: binValue, stop: make(chan struct{})}
	fb.wg.Add(1)
	go fb.acceptLoop()
	return fb
}

func (fb *fakeBoard) close() {
	close(fb.stop)
	fb.ln.Close()
	fb.wg.Wait()
}

func (fb *fakeBoard) acceptLoop() {
	defer fb.wg.Done()
	for {
		if tl, ok := fb.ln.(*net.TCPListener); ok {
			_ = tl.SetDeadline(time.Now().Add(300 * time.Millisecond))
		}
		conn, err := fb.ln.Accept()
		if err != nil {
			select {
			case <-fb.stop:
				return
			default:
				continue
			}
		}
		fb.wg.Add(1)
		go fb.handle(conn)
	}
}

func (fb *fakeBoard) handle(conn net.Conn) {
	defer fb.wg.Done()
	defer conn.Close()
	// v3 request fixed part is 36 bytes; nextra==0 for the single test channel.
	hdr := make([]byte, 36)
	if _, err := io.ReadFull(conn, hdr); err != nil {
		return
	}
	flo := math.Float64frombits(binary.LittleEndian.Uint64(hdr[8:16]))
	fhi := math.Float64frombits(binary.LittleEndian.Uint64(hdr[16:24]))
	udpPort := int(binary.LittleEndian.Uint16(hdr[24:26]))
	adcBitsReq := int(binary.LittleEndian.Uint16(hdr[6:8]))
	nextra := int(binary.LittleEndian.Uint16(hdr[34:36]))
	// drain any extra channel blocks (24 B each) -- test uses none.
	if nextra > 0 {
		io.CopyN(io.Discard, conn, int64(nextra*24))
	}

	// Grant the same run the server would: occupied band + guard(3), ADC A.
	const guard = 3
	klo := admitBin(flo) - guard
	khi := admitBin(fhi) + guard
	if klo < 1 {
		klo = 1
	}
	adc := adcBitsReq & 0x3
	if adc == 0 {
		adc = 1
	}
	wa := khi - klo + 1
	wb := 0
	binWidth := 24
	binScale := 1.0

	// accept (56 B) + one run (12 B).
	acc := make([]byte, 56)
	copy(acc[0:8], acceptTag)
	binary.LittleEndian.PutUint32(acc[8:12], streamV3)
	binary.LittleEndian.PutUint32(acc[12:16], nFFT)
	binary.LittleEndian.PutUint64(acc[16:24], math.Float64bits(fs))
	binary.LittleEndian.PutUint32(acc[24:28], uint32(adc))
	binary.LittleEndian.PutUint32(acc[28:32], uint32(wa))
	binary.LittleEndian.PutUint32(acc[32:36], uint32(wb))
	binary.LittleEndian.PutUint32(acc[36:40], 0) // nb (unused)
	binary.LittleEndian.PutUint64(acc[40:48], math.Float64bits(binScale))
	binary.LittleEndian.PutUint32(acc[48:52], uint32(binWidth))
	binary.LittleEndian.PutUint32(acc[52:56], 1) // nruns
	run := make([]byte, 12)
	binary.LittleEndian.PutUint32(run[0:4], uint32(klo))
	binary.LittleEndian.PutUint32(run[4:8], uint32(wa))
	binary.LittleEndian.PutUint32(run[8:12], uint32(adc))
	if _, err := conn.Write(append(acc, run...)); err != nil {
		return
	}

	// Stop streaming when this TCP connection closes (emitter reopened) or the board
	// shuts down.
	connClosed := make(chan struct{})
	go func() {
		buf := make([]byte, 64)
		for {
			if _, err := conn.Read(buf); err != nil {
				close(connClosed)
				return
			}
		}
	}()

	dstIP := conn.RemoteAddr().(*net.TCPAddr).IP
	us, err := net.DialUDP("udp", nil, &net.UDPAddr{IP: dstIP, Port: udpPort})
	if err != nil {
		return
	}
	defer us.Close()

	rec := recordSizeAB(wa, wb, binWidth)
	kcIdx := 700 - klo // centre bin index within the A-run
	const nrecPerDgram = 40
	var seq uint64
	var gen uint16
	for {
		select {
		case <-fb.stop:
			return
		case <-connClosed:
			return
		default:
		}
		dg := make([]byte, 12+nrecPerDgram*rec)
		copy(dg[0:4], udpMagic[:])
		binary.LittleEndian.PutUint16(dg[6:8], gen)
		binary.LittleEndian.PutUint16(dg[8:10], uint16(rec))
		binary.LittleEndian.PutUint16(dg[10:12], nrecPerDgram)
		for r := 0; r < nrecPerDgram; r++ {
			base := 12 + r*rec
			binary.LittleEndian.PutUint64(dg[base:base+8], seq)
			seq++
			// A-run: (I,Q) per bin, int24 LE.  Only the centre bin is non-zero (I).
			if kcIdx >= 0 && kcIdx < wa {
				off := base + 8 + kcIdx*2*3
				putInt24LE(dg[off:off+3], fb.binValue)
			}
		}
		if _, err := us.Write(dg); err != nil {
			return
		}
		time.Sleep(time.Millisecond) // ~40k blocks/s, roughly real time
	}
}

func putInt24LE(b []byte, v int32) {
	b[0] = byte(v)
	b[1] = byte(v >> 8)
	b[2] = byte(v >> 16)
}

func TestEmitLoopbackStream(t *testing.T)         { runLoopback(t, 1) }
func TestEmitLoopbackStreamParallel(t *testing.T) { runLoopback(t, 4) }

// runLoopback drives the full offline wire path with the given reconWorkers count.
// reconWorkers=1 is the serial path; >1 exercises the real worker() fork-join pool
// (including repartition on the client's DDC retune) end-to-end.
func runLoopback(t *testing.T, reconWorkers int) {
	fb := startFakeBoard(t, 500000)
	defer fb.close()

	em, err := NewEmitter("127.0.0.1", nil, "0.0.0.0", EmitterOpts{
		Gain: 1.0, Guard: guardDef, MTU: 3980, UDPPort: 55055, BinWidth: 24, Verbose: false,
		ReconWorkers: reconWorkers,
	})
	if err != nil {
		t.Skipf("cannot start emitter (ports busy?): %v", err)
	}
	go em.Serve()
	defer em.Stop()

	c, err := newP2Client("127.0.0.1")
	if err != nil {
		t.Fatalf("client: %v", err)
	}
	defer c.close()

	disc, ok := c.discover(2 * time.Second)
	if !ok {
		t.Fatalf("no discovery reply from emitter")
	}
	if disc.boardID != boardAngelia {
		t.Fatalf("discovery board_id=%d want %d", disc.boardID, boardAngelia)
	}

	fc := 700 * binW
	cfg := map[int]ddcConfig{0: {adc: 0, rate: 48000, bits: 24}}
	// configure + run, resend once (the radio may have just started), then let the
	// board stream + pipelines settle past the debounce + warm-up.
	c.configure(cfg, 1)
	c.setRun(true, []float64{fc})
	time.Sleep(150 * time.Millisecond)
	c.configure(cfg, 1)
	c.setRun(true, []float64{fc})
	time.Sleep(900 * time.Millisecond)

	iq := c.receive(1500*time.Millisecond, []int{0})
	c.setRun(false, []float64{fc})

	z := iq[0]
	// ~48 kHz for ~1.5 s of steady streaming -> tens of thousands of samples.  This
	// proves the whole wire path moves continuous reconstructed I/Q to the client;
	// DSP fidelity is covered separately by TestPushPhasesMatchesBatch.
	if len(z) < 10000 {
		t.Fatalf("ddc0: only %d samples (expected a continuous ~48 kHz stream)", len(z))
	}
	for i, v := range z {
		if math.IsNaN(real(v)) || math.IsNaN(imag(v)) || math.IsInf(real(v), 0) || math.IsInf(imag(v), 0) {
			t.Fatalf("sample %d not finite: %v", i, v)
		}
		if cmplx.Abs(v) > 2.0 {
			t.Fatalf("sample %d out of range: %v", i, v)
		}
	}
	// Steady state: constant board input -> the stream should neither decay nor blow
	// up.  Compare RMS of the first and second interior halves (skip warm-up edges).
	rms := func(s []complex128) float64 {
		var e float64
		for _, v := range s {
			e += real(v)*real(v) + imag(v)*imag(v)
		}
		return math.Sqrt(e / float64(len(s)))
	}
	lo, hi := len(z)/10, 9*len(z)/10
	mid := z[lo:hi]
	h1 := rms(mid[:len(mid)/2])
	h2 := rms(mid[len(mid)/2:])
	if h1 <= 0 || h2 <= 0 {
		t.Fatalf("reconstructed stream is all zeros (h1=%.3g h2=%.3g)", h1, h2)
	}
	if r := h2 / h1; r < 0.5 || r > 2.0 {
		t.Fatalf("stream not steady: RMS first-half=%.3g second-half=%.3g (ratio %.2f)", h1, h2, r)
	}
	t.Logf("loopback OK: %d samples, RMS %.3g, steady ratio %.2f", len(z), rms(mid), h2/h1)
}
