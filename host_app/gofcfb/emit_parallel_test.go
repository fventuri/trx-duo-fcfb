// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Byte-exactness + race gates for the emitter's multi-core reconstruction
// (recon_workers > 1).  The core guarantee is that spreading pipes across worker
// goroutines produces frame bytes IDENTICAL to the serial path, per pipe -- because
// each pipe is an independent deterministic chain whose block order is preserved.
package fcfb

import (
	"bytes"
	"fmt"
	"math/rand"
	"net"
	"sort"
	"testing"
)

// tpipeSpec describes one reconstruction pipe for the tests (params only; the pipe's
// mutable DSP state is built fresh per pipe set so serial and pooled start equal).
type tpipeSpec struct {
	radio, ddc int
	adc        int // 1=A, 2=B
	kaSub      []int
	idx        []int // column positions gathered (len == len(kaSub))
	kc         int
	tune       float64
	phases     int
	rate       int
}

// testPipeSpecs exercises both ADCs, several bin counts, phase counts and rates.
func testPipeSpecs() []tpipeSpec {
	return []tpipeSpec{
		{radio: 0, ddc: 0, adc: 1, kaSub: []int{698, 699, 700, 701, 702}, idx: []int{0, 1, 2, 3, 4}, kc: 700, tune: 0, phases: 5, rate: 48000},
		{radio: 0, ddc: 1, adc: 2, kaSub: []int{300, 301, 302}, idx: []int{0, 1, 2}, kc: 301, tune: 1234.5, phases: 5, rate: 48000},
		{radio: 0, ddc: 2, adc: 1, kaSub: []int{500, 501}, idx: []int{5, 6}, kc: 500, tune: -500, phases: 25, rate: 192000},
		{radio: 1, ddc: 0, adc: 2, kaSub: []int{100, 101, 102, 103}, idx: []int{2, 3, 4, 5}, kc: 101, tune: 250, phases: 5, rate: 96000},
	}
}

// buildPipeSet constructs a fresh pipe map from specs.  Radios carry a non-nil
// clientEP (processPipe skips a pipe whose client is gone).  Sockets are nil: the
// send seam is swapped in the tests, so no real socket is used.
func buildPipeSet(specs []tpipeSpec) map[pipeKey]*pipe {
	ep := &net.UDPAddr{IP: net.IPv4(127, 0, 0, 1), Port: 9999}
	radios := map[int]*Radio{}
	pipes := map[pipeKey]*pipe{}
	for _, s := range specs {
		r := radios[s.radio]
		if r == nil {
			r = &Radio{idx: s.radio}
			r.clientEP.Store(ep)
			radios[s.radio] = r
		}
		p := &pipe{
			radio: r, ddc: s.ddc, adc: s.adc,
			idx:    append([]int(nil), s.idx...),
			synth:  newStreamSynthP(s.kaSub, s.kc, s.tune, s.phases),
			resamp: newComplexResampler(float64(s.phases)*binRate, s.rate),
		}
		pipes[pipeKey{radio: s.radio, ddc: s.ddc}] = p
	}
	return pipes
}

// datagram is one board recv result: per-block bin columns for ADC A and B.
type datagram struct{ a, b [][]complex128 }

// makeDatagrams builds a deterministic sequence of datagrams with wa/wb bins wide.
func makeDatagrams(rng *rand.Rand, ndg, nblk, wa, wb int) []datagram {
	col := func(w int) []complex128 {
		c := make([]complex128, w)
		for i := range c {
			c[i] = complex(rng.NormFloat64(), rng.NormFloat64())
		}
		return c
	}
	dgs := make([]datagram, ndg)
	for d := range dgs {
		a := make([][]complex128, nblk)
		b := make([][]complex128, nblk)
		for k := 0; k < nblk; k++ {
			a[k] = col(wa)
			b[k] = col(wb)
		}
		dgs[d] = datagram{a: a, b: b}
	}
	return dgs
}

// capture wires the send seam into per-pipe byte buffers.  The map is fully
// pre-populated (read-only during the run), and each buffer is appended to by only
// its owning goroutine, so concurrent workers are race-free.
func capture(e *Emitter, pipes map[pipeKey]*pipe) (map[pipeKey]*[]byte, map[*pipe]pipeKey) {
	out := map[pipeKey]*[]byte{}
	rev := map[*pipe]pipeKey{}
	for k, p := range pipes {
		b := []byte{}
		out[k] = &b
		rev[p] = k
	}
	e.send = func(p *pipe, ep *net.UDPAddr, pkt []byte) {
		b := out[rev[p]]
		*b = append(*b, pkt...)
	}
	return out, rev
}

func newTestEmitter() *Emitter {
	return &Emitter{wireGain: 1.0 / float64(int64(1)<<23)}
}

func runSerial(e *Emitter, pipes map[pipeKey]*pipe, dgs []datagram) {
	scratch := []complex128{}
	for _, dg := range dgs {
		for _, p := range pipes {
			e.processPipe(p, dg.a, dg.b, &scratch)
		}
	}
}

func runPooled(e *Emitter, pipes map[pipeKey]*pipe, dgs []datagram, n int) {
	pool := newEmitPool(e, n)
	pool.repartition(pipes)
	for _, dg := range dgs {
		pool.dispatch(dg.a, dg.b)
	}
	pool.stop()
}

func snapshotBytes(m map[pipeKey]*[]byte) map[pipeKey][]byte {
	out := map[pipeKey][]byte{}
	for k, b := range m {
		out[k] = append([]byte(nil), (*b)...)
	}
	return out
}

func assertSameFrames(t *testing.T, serial, pooled map[pipeKey][]byte) {
	t.Helper()
	if len(serial) != len(pooled) {
		t.Fatalf("pipe count differs: serial %d pooled %d", len(serial), len(pooled))
	}
	keys := make([]pipeKey, 0, len(serial))
	for k := range serial {
		keys = append(keys, k)
	}
	sort.Slice(keys, func(i, j int) bool {
		if keys[i].radio != keys[j].radio {
			return keys[i].radio < keys[j].radio
		}
		return keys[i].ddc < keys[j].ddc
	})
	for _, k := range keys {
		s, p := serial[k], pooled[k]
		if len(s) == 0 {
			t.Fatalf("pipe r%dd%d: serial produced no frames (test too short?)", k.radio, k.ddc)
		}
		if !bytes.Equal(s, p) {
			t.Fatalf("pipe r%dd%d: pooled bytes differ from serial (serial %d B, pooled %d B)",
				k.radio, k.ddc, len(s), len(p))
		}
	}
	t.Logf("byte-exact across %d pipes (%s)", len(keys), func() string {
		var parts []string
		for _, k := range keys {
			parts = append(parts, fmt.Sprintf("r%dd%d=%dB", k.radio, k.ddc, len(serial[k])))
		}
		return fmt.Sprint(parts)
	}())
}

// TestEmitParallelByteExact is the core guarantee: for several worker counts, the
// pooled path emits byte-identical frames to the serial path, per pipe.
func TestEmitParallelByteExact(t *testing.T) {
	specs := testPipeSpecs()
	// wide enough columns to cover every pipe's idx (max idx here is 6 -> width 8).
	dgs := makeDatagrams(rand.New(rand.NewSource(42)), 40, 50, 8, 8)

	eS := newTestEmitter()
	serialPipes := buildPipeSet(specs)
	serCap, _ := capture(eS, serialPipes)
	runSerial(eS, serialPipes, dgs)
	serial := snapshotBytes(serCap)

	for _, n := range []int{2, 3, 4, 8} {
		eP := newTestEmitter()
		pooledPipes := buildPipeSet(specs)
		poolCap, _ := capture(eP, pooledPipes)
		runPooled(eP, pooledPipes, dgs, n)
		assertSameFrames(t, serial, snapshotBytes(poolCap))
	}
}

// TestEmitParallelRepartition proves a pipe that survives a mid-stream retune stays
// byte-identical across the repartition.  The pooled run adds/drops a DDC halfway
// (changing every survivor's round-robin worker), while the serial reference feeds the
// survivors the whole datagram sequence.  Only the survivors are compared.
func TestEmitParallelRepartition(t *testing.T) {
	survivors := testPipeSpecs()
	dgs := makeDatagrams(rand.New(rand.NewSource(7)), 40, 50, 8, 8)
	half := len(dgs) / 2

	// Serial reference: survivors only, fed every datagram.
	eS := newTestEmitter()
	serialPipes := buildPipeSet(survivors)
	serCap, _ := capture(eS, serialPipes)
	runSerial(eS, serialPipes, dgs)
	serial := snapshotBytes(serCap)

	// Pooled: start with survivors + an extra DDC; halfway, drop the extra and add a
	// different one (retune), repartitioning the SAME survivor pipe objects.
	eP := newTestEmitter()
	pipes := buildPipeSet(survivors)
	extra := buildPipeSet([]tpipeSpec{{radio: 2, ddc: 0, adc: 1, kaSub: []int{400, 401}, idx: []int{0, 1}, kc: 400, phases: 5, rate: 48000}})
	for k, p := range extra { // add the extra DDC to the initial set
		pipes[k] = p
	}
	poolCap, _ := capture(eP, pipes)

	pool := newEmitPool(eP, 4)
	pool.repartition(pipes)
	for i, dg := range dgs {
		if i == half {
			delete(pipes, pipeKey{radio: 2, ddc: 0}) // drop the extra
			added := buildPipeSet([]tpipeSpec{{radio: 3, ddc: 0, adc: 2, kaSub: []int{200, 201, 202}, idx: []int{0, 1, 2}, kc: 201, phases: 5, rate: 96000}})
			for k, p := range added {
				pipes[k] = p
			}
			// re-wire capture for the new full set, preserving survivor buffers.
			poolCap2 := map[pipeKey]*[]byte{}
			rev := map[*pipe]pipeKey{}
			for k, p := range pipes {
				if b, ok := poolCap[k]; ok {
					poolCap2[k] = b
				} else {
					nb := []byte{}
					poolCap2[k] = &nb
				}
				rev[p] = k
			}
			poolCap = poolCap2
			eP.send = func(p *pipe, ep *net.UDPAddr, pkt []byte) {
				*poolCap[rev[p]] = append(*poolCap[rev[p]], pkt...)
			}
			pool.repartition(pipes) // survivors move to different workers
		}
		pool.dispatch(dg.a, dg.b)
	}
	pool.stop()

	// Compare only the survivors.
	pooled := map[pipeKey][]byte{}
	for _, s := range survivors {
		k := pipeKey{radio: s.radio, ddc: s.ddc}
		pooled[k] = append([]byte(nil), (*poolCap[k])...)
	}
	assertSameFrames(t, serial, pooled)
}
