// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Live openHPSDR Protocol-2 radio server over the fcfb bin stream.  Go port of
// host_app/hpsdr_emit.py.  Presents the fcfb board (a wideband analysis
// channelizer) to any P2 client (piHPSDR, Thetis, linhpsdr, ...) as one or MORE
// HPSDR radios: it answers discovery, honours each client's per-DDC centre freq /
// sample rate / ADC, requests the UNION of the needed bins from the board (planner +
// BoardStream), reconstructs each DDC's complex baseband gaplessly (StreamSynth at a
// jitter-free ladder rate), resamples to the client's rate (complexResampler) and
// streams RX DDC I/Q packets back to each client, one UDP source port (1035+ddc) per
// DDC.  The board delivers bins in real time (40 kHz/block), so outgoing I/Q is paced
// by the board itself -- no explicit rate limiting.
//
// MULTI-RADIO (Program B): to break the per-client receiver cap (piHPSDR 2, ...),
// present N radios -- each on its own bind IP + MAC -- and drive them with N client
// instances.  The board serves ONE stream session at a time, so ALL radios share ONE
// BoardStream carrying the union of every enabled DDC's bins; each reconstructed DDC
// is routed back to its own radio's client.  A DDC retune only reprograms the board
// when the union bin set actually changes.
package fcfb

import (
	"errors"
	"fmt"
	"math"
	"net"
	"os"
	"sort"
	"strings"
	"sync"
	"sync/atomic"
	"time"
)

// Tunables (env-overridable to match the Python emitter's knobs).
var (
	defaultMAC      = [6]byte{0x02, 0x00, 0x00, 0xFC, 0xFB, 0x01} // locally-administered
	settleDur       = envDur("FCFB_EMIT_SETTLE", 250*time.Millisecond)
	maxSettleDur    = envDur("FCFB_EMIT_MAX_SETTLE", 1000*time.Millisecond)
	clientTimeout   = envDur("FCFB_CLIENT_TIMEOUT", 5*time.Second)
	discLogDebounce = envDur("FCFB_DISC_LOG_DEBOUNCE", 1*time.Second)
	recvTimeout     = 100 * time.Millisecond // worker config-responsiveness / stop poll
	defaultNDDC     = 8                      // advertised per radio
	emitP2Version   = 39
	emitFWVersion   = 40
)

func envDur(name string, def time.Duration) time.Duration {
	if v := os.Getenv(name); v != "" {
		if f, err := time.ParseDuration(v); err == nil {
			return f
		}
		// accept a bare float in seconds (matches the Python env vars)
		var s float64
		if _, err := fmt.Sscanf(v, "%f", &s); err == nil {
			return time.Duration(s * float64(time.Second))
		}
	}
	return def
}

var ctrlPorts = []int{generalFromHostPort, rxSpecFromHostPort, highPriFromHostPort,
	txSpecFromHostPort, audioFromHostPort, txIQFromHostPort}

// adcBits maps an HPSDR per-DDC ADC index (0=ADC0, 1=ADC1) to fcfb adc_bits (1=A,
// 2=B).  P2 carries ONE ADC per DDC (adc=2 is a SoapySDR-only index, never sent for
// a P2 radio); diversity is done by the client enabling two DDCs at the same freq,
// one per ADC, and combining them itself.  Any unexpected index falls back to ADC A.
func adcBits(hpsdrADC int) int {
	if hpsdrADC == 1 {
		return 2
	}
	return 1
}

func macFor(idx int) [6]byte {
	m := defaultMAC
	m[5] = byte((int(defaultMAC[5]) + idx) & 0xFF)
	return m
}

// ---------------------------------------------------------------- BoardStream

// BoardStream is the shared board session the emitter pulls dequantised bin-blocks
// from.  recv() returns one datagram's blocks (one bin-column per block) for ADC A
// and B; parseDatagram reuses its column buffers per record, so recv copies them.
type BoardStream struct {
	us   *net.UDPConn
	ts   net.Conn
	meta Meta
	buf  []byte
}

func openBoardStream(ip string, chans []Chan, udpPort, guard int) (*BoardStream, error) {
	us, ts, m, err := dialBoard(ip, chans, udpPort, -1, guard, rcvBuf, 10*time.Second)
	if err != nil {
		return nil, err
	}
	return &BoardStream{us: us, ts: ts, meta: m, buf: make([]byte, 1<<16)}, nil
}

// recv reads one datagram and returns its per-block columns (copied).  ok=false with
// nil err means "no data this poll" (read timeout, or a non-data/mismatched packet);
// the worker just loops again.  A non-timeout error is a real stream failure.
func (b *BoardStream) recv() (colsA, colsB [][]complex128, ok bool, err error) {
	_ = b.us.SetReadDeadline(time.Now().Add(recvTimeout))
	n, _, rerr := b.us.ReadFromUDP(b.buf)
	if rerr != nil {
		var ne net.Error
		if errors.As(rerr, &ne) && ne.Timeout() {
			return nil, nil, false, nil
		}
		return nil, nil, false, rerr
	}
	_, _, decode, dok := parseDatagram(b.buf[:n], b.meta)
	if !dok {
		return nil, nil, false, nil
	}
	decode(func(rec int, seq uint64, colA, colB []complex128) {
		colsA = append(colsA, append([]complex128(nil), colA...))
		colsB = append(colsB, append([]complex128(nil), colB...))
	})
	return colsA, colsB, true, nil
}

func (b *BoardStream) close() {
	if b.us != nil {
		b.us.Close()
	}
	if b.ts != nil {
		b.ts.Close()
	}
}

// ---------------------------------------------------------------- Radio

type ddcState struct {
	freq            float64
	rate, adc, bits int
}

// Radio is one P2 radio identity: its own bind IP + MAC + control/data sockets and
// the client-driven state (run bit, client endpoint, per-DDC config).
type Radio struct {
	idx     int
	bindIP  string
	mac     [6]byte
	boardID int
	nDDC    int

	// state below is guarded by Emitter.mu, except clientEP (atomic, read lock-free
	// by the worker's hot path).
	running  bool
	ddcs     map[int]*ddcState
	lastSeen time.Time
	clientEP atomic.Pointer[net.UDPAddr]

	ctrl   map[int]*net.UDPConn
	data   map[int]*net.UDPConn
	dataMu sync.Mutex
}

func (r *Radio) dataSocketFor(ddc int) (*net.UDPConn, error) {
	r.dataMu.Lock()
	defer r.dataMu.Unlock()
	if s := r.data[ddc]; s != nil {
		return s, nil
	}
	addr := &net.UDPAddr{IP: net.ParseIP(r.bindIP), Port: rxIQToHostPort0 + ddc}
	if r.bindIP == "" || r.bindIP == "0.0.0.0" {
		addr.IP = net.IPv4zero
	}
	s, err := net.ListenUDP("udp", addr)
	if err != nil {
		return nil, err
	}
	r.data[ddc] = s
	return s, nil
}

func (r *Radio) closeSockets() {
	for _, s := range r.ctrl {
		s.Close()
	}
	r.dataMu.Lock()
	for _, s := range r.data {
		s.Close()
	}
	r.dataMu.Unlock()
}

// ddcKey identifies the DDC config for change detection: sorted (ddc, rate, adc).
func ddcKey(ddcs map[int]*ddcState) string {
	keys := make([]int, 0, len(ddcs))
	for d := range ddcs {
		keys = append(keys, d)
	}
	sort.Ints(keys)
	var sb strings.Builder
	for _, d := range keys {
		c := ddcs[d]
		fmt.Fprintf(&sb, "%d:%d:%d;", d, c.rate, c.adc)
	}
	return sb.String()
}

// ---------------------------------------------------------------- Emitter

// pipe is one reconstruction chain: a channel's bins -> StreamSynth -> resampler ->
// RX I/Q on the radio's 1035+ddc socket.
type pipe struct {
	radio  *Radio
	ddc    int
	adc    int // 1=A, 2=B
	idx    []int
	synth  *StreamSynth
	resamp *complexResampler
	sock   *net.UDPConn
	buf    []complex128
	seq    uint32
	psig   psigKey
}

// psigKey identifies a pipe's ACTUAL content: the (fc, adc) it was built from (the
// fallback's, not the DDC's own, when budget-fed) + the client output rate.  Keying
// reuse off this rebuilds the pipe when a DDC flips admitted<->fallback.
type psigKey struct {
	fc   float64
	adc  int
	rate int
}

type pipeKey struct{ radio, ddc int }

// tag routes a planned channel back to its (radio, ddc, config).
type tag struct {
	radio *Radio
	ddc   int
	c     *ddcState
}

// Emitter is the multi-radio P2 server.  Default (one radio) is behaviourally the
// single-radio emitter.
type Emitter struct {
	boardIP      string
	udpPort      int
	wireGain     float64
	guard        int
	budget       int
	verbose      bool
	reconWorkers int               // parallel per-pipe reconstruction goroutines (1 = serial)
	monInterval  int               // board temperature-log poll interval (s); 0 = off
	dds          map[[2]int]uint32 // (radioIdx, ddc) -> dds_phase_inc (bench tone)

	// send routes a framed RX I/Q packet to the client.  Default writes to the pipe's
	// own UDP socket; tests swap it to capture per-pipe frames.  It is set once at
	// construction and only read afterwards, so it is safe to call from the pool workers.
	send func(p *pipe, ep *net.UDPAddr, pkt []byte)

	mu          sync.Mutex
	cfgVer      uint64
	radios      []*Radio
	lastDiscLog map[string]time.Time

	discSock *net.UDPConn
	stop     chan struct{}
	stopOnce sync.Once
	wg       sync.WaitGroup
}

// RadioSpec configures one radio identity.
type RadioSpec struct {
	BindIP  string
	MAC     *[6]byte
	BoardID int
	NDDC    int
}

// EmitterOpts holds the tunables shared across radios.
type EmitterOpts struct {
	Gain     float64
	Guard    int
	Budget   int // 0 => derive from MTU
	MTU      int
	BinWidth int
	UDPPort  int
	Verbose  bool
	DDS      map[[2]int]uint32
	// ReconWorkers > 1 spreads per-DDC reconstruction (synth+resample) across that many
	// goroutines; 0/1 keeps the byte-identical single-core path.
	ReconWorkers int
	// MonitorInterval > 0 logs the board's temperature every that many seconds (0 = off).
	MonitorInterval int
}

// NewEmitter builds a multi-radio emitter.  If radios is empty, one radio is created
// on bindIP.
func NewEmitter(boardIP string, radios []RadioSpec, bindIP string, opts EmitterOpts) (*Emitter, error) {
	if opts.Gain == 0 {
		opts.Gain = 1.0
	}
	if opts.BinWidth == 0 {
		opts.BinWidth = 24
	}
	if opts.MTU == 0 {
		opts.MTU = 1500
	}
	if opts.UDPPort == 0 {
		opts.UDPPort = 55055
	}
	guard := opts.Guard
	if guard < 0 {
		guard = 0
	}
	budget := opts.Budget
	if budget == 0 {
		budget = binBudgetMTU(opts.BinWidth, opts.MTU)
	}
	rw := opts.ReconWorkers
	if rw < 1 {
		rw = 1
	}
	e := &Emitter{
		boardIP:      boardIP,
		udpPort:      opts.UDPPort,
		wireGain:     opts.Gain / float64(int64(1)<<23),
		guard:        guard,
		budget:       budget,
		verbose:      opts.Verbose,
		reconWorkers: rw,
		monInterval:  opts.MonitorInterval,
		dds:          opts.DDS,
		send:         func(p *pipe, ep *net.UDPAddr, pkt []byte) { p.sock.WriteToUDP(pkt, ep) },
		lastDiscLog:  map[string]time.Time{},
		stop:         make(chan struct{}),
	}
	if len(radios) == 0 {
		radios = []RadioSpec{{BindIP: bindIP, BoardID: boardAngelia, NDDC: defaultNDDC}}
	}
	for i, spec := range radios {
		mac := macFor(i)
		if spec.MAC != nil {
			mac = *spec.MAC
		}
		nddc := spec.NDDC
		if nddc == 0 {
			nddc = defaultNDDC
		}
		bid := spec.BoardID
		if bid == 0 {
			bid = boardAngelia
		}
		r := &Radio{idx: i, bindIP: spec.BindIP, mac: mac, boardID: bid, nDDC: nddc,
			ddcs: map[int]*ddcState{}, ctrl: map[int]*net.UDPConn{}, data: map[int]*net.UDPConn{}}
		if err := e.openControl(r); err != nil {
			e.closeAll()
			return nil, err
		}
		e.radios = append(e.radios, r)
	}
	// A socket bound to a SPECIFIC IP does not receive broadcasts, but P2 clients
	// discover by broadcasting to 255.255.255.255.  So whenever the radios are on
	// specific IPs, add ONE shared 0.0.0.0:1024 listener that catches the broadcast
	// and answers once per radio (each reply FROM that radio's own IP).  A lone radio
	// on 0.0.0.0 already receives broadcasts, so it needs no extra listener.
	lone := len(e.radios) == 1 && (e.radios[0].bindIP == "0.0.0.0" || e.radios[0].bindIP == "")
	if !lone {
		s, err := net.ListenUDP("udp", &net.UDPAddr{IP: net.IPv4zero, Port: generalFromHostPort})
		if err != nil {
			e.closeAll()
			return nil, fmt.Errorf("shared discovery listener: %w", err)
		}
		e.discSock = s
	}
	return e, nil
}

func (e *Emitter) openControl(r *Radio) error {
	ip := net.ParseIP(r.bindIP)
	if r.bindIP == "" || r.bindIP == "0.0.0.0" {
		ip = net.IPv4zero
	}
	for _, port := range ctrlPorts {
		s, err := net.ListenUDP("udp", &net.UDPAddr{IP: ip, Port: port})
		if err != nil {
			return fmt.Errorf("radio%d bind %s:%d: %w", r.idx, r.bindIP, port, err)
		}
		r.ctrl[port] = s
	}
	return nil
}

func (e *Emitter) closeAll() {
	if e.discSock != nil {
		e.discSock.Close()
	}
	for _, r := range e.radios {
		r.closeSockets()
	}
}

func (e *Emitter) log(format string, a ...any) {
	if e.verbose {
		fmt.Fprintf(os.Stderr, "[hpsdr] "+format+"\n", a...)
	}
}

// ---------------------------------------------------------------- control plane

func (e *Emitter) handleCtrl(r *Radio, port int, pkt []byte, src *net.UDPAddr) {
	if port == generalFromHostPort && isDiscovery(pkt) {
		e.mu.Lock()
		running := r.running
		e.mu.Unlock()
		status := 2
		if running {
			status = 3
		}
		reply := buildDiscoveryReply(r.mac, r.boardID, status, emitP2Version, emitFWVersion, r.nDDC)
		r.ctrl[port].WriteToUDP(reply, src)
		e.log("radio%d discovery from %s -> reply (board_id=%d, n_ddc=%d)", r.idx, src, r.boardID, r.nDDC)
		return
	}
	now := time.Now()
	switch port {
	case highPriFromHostPort:
		hp := parseHighPriority(pkt)
		e.mu.Lock()
		r.lastSeen = now
		changed := false
		if cur := r.clientEP.Load(); cur == nil || cur.String() != src.String() {
			cp := *src
			r.clientEP.Store(&cp)
			changed = true
		}
		if hp.running != r.running {
			r.running = hp.running
			changed = true
			e.log("radio%d run=%v", r.idx, r.running)
		}
		for ddc, c := range r.ddcs {
			if ddc < len(hp.ddcFreqHz) {
				f := hp.ddcFreqHz[ddc]
				if math.Abs(c.freq-f) > 0.5 {
					c.freq = f
					changed = true
				}
			}
		}
		if changed {
			e.cfgVer++
		}
		e.mu.Unlock()
	case rxSpecFromHostPort:
		rs := parseReceiveSpecific(pkt)
		e.mu.Lock()
		r.lastSeen = now
		newDDCs := map[int]*ddcState{}
		for ddc, c := range rs.ddcs {
			prevFreq := 0.0
			if p := r.ddcs[ddc]; p != nil {
				prevFreq = p.freq
			}
			newDDCs[ddc] = &ddcState{freq: prevFreq, rate: c.rate, adc: c.adc, bits: c.bits}
		}
		if ddcKey(newDDCs) != ddcKey(r.ddcs) {
			r.ddcs = newDDCs
			e.cfgVer++
			var parts []string
			for _, d := range sortedDDCKeys(newDDCs) {
				parts = append(parts, fmt.Sprintf("%d:%dk adc%d", d, newDDCs[d].rate/1000, newDDCs[d].adc))
			}
			e.log("radio%d DDCs: %s", r.idx, strings.Join(parts, ", "))
		}
		e.mu.Unlock()
	default:
		// tx-specific / audio / tx-iq: RX-only server ignores content, but they are
		// also linhpsdr session heartbeats -> count them for liveness.
		e.mu.Lock()
		if r.running {
			r.lastSeen = now
		}
		e.mu.Unlock()
	}
}

func sortedDDCKeys(ddcs map[int]*ddcState) []int {
	keys := make([]int, 0, len(ddcs))
	for d := range ddcs {
		keys = append(keys, d)
	}
	sort.Ints(keys)
	return keys
}

func (e *Emitter) handleDiscoveryAll(pkt []byte, src *net.UDPAddr) {
	if !isDiscovery(pkt) {
		return
	}
	for _, r := range e.radios {
		e.mu.Lock()
		running := r.running
		e.mu.Unlock()
		status := 2
		if running {
			status = 3
		}
		reply := buildDiscoveryReply(r.mac, r.boardID, status, emitP2Version, emitFWVersion, r.nDDC)
		r.ctrl[generalFromHostPort].WriteToUDP(reply, src)
	}
	now := time.Now()
	if last, ok := e.lastDiscLog[src.IP.String()]; !ok || now.Sub(last) > discLogDebounce {
		e.lastDiscLog[src.IP.String()] = now
		e.log("broadcast discovery from %s -> %d radio replies", src.IP, len(e.radios))
	}
}

// checkLiveness tears down any running radio whose client has gone silent past the
// timeout, freeing its bins from the shared union.  Bumps cfgVer so the worker
// re-plans.
func (e *Emitter) checkLiveness() {
	now := time.Now()
	e.mu.Lock()
	defer e.mu.Unlock()
	for _, r := range e.radios {
		if r.running && !r.lastSeen.IsZero() && now.Sub(r.lastSeen) > clientTimeout {
			e.log("radio%d client %v silent >%.0fs -> teardown (freeing its bins)",
				r.idx, r.clientEP.Load(), clientTimeout.Seconds())
			r.running = false
			r.clientEP.Store(nil)
			r.ddcs = map[int]*ddcState{}
			r.lastSeen = time.Time{}
			e.cfgVer++
		}
	}
}

// radioSnap is a consistent copy of one radio's client-driven state.
type radioSnap struct {
	radio    *Radio
	running  bool
	clientEP *net.UDPAddr
	ddcs     map[int]*ddcState
}

func (e *Emitter) snapshot() (uint64, []radioSnap) {
	e.mu.Lock()
	defer e.mu.Unlock()
	snaps := make([]radioSnap, len(e.radios))
	for i, r := range e.radios {
		ddcs := make(map[int]*ddcState, len(r.ddcs))
		for d, c := range r.ddcs {
			cc := *c
			ddcs[d] = &cc
		}
		snaps[i] = radioSnap{radio: r, running: r.running, clientEP: r.clientEP.Load(), ddcs: ddcs}
	}
	return e.cfgVer, snaps
}

// ---------------------------------------------------------------- data plane

// freqValid reports a centre freq the board can grant a run for (k_lo >= 1).
func (e *Emitter) freqValid(fc float64) bool {
	return admitBin(fc)-e.guard >= 1
}

// plan collects every enabled (radio, ddc) into planner reqs + routing tags.  A DDC
// whose freq hasn't arrived yet (transiently 0) is fed from an already-streamed
// frequency (fallback) so an enabled DDC never goes dataless (which freezes
// linhpsdr); it retunes on the next rebuild.
func (e *Emitter) plan(snaps []radioSnap) ([]ChannelReq, []tag) {
	var fallback float64
	haveFallback := false
	for _, s := range snaps {
		if s.running && s.clientEP != nil && len(s.ddcs) > 0 {
			for _, d := range sortedDDCKeys(s.ddcs) {
				if e.freqValid(s.ddcs[d].freq) {
					fallback = s.ddcs[d].freq
					haveFallback = true
					break
				}
			}
		}
		if haveFallback {
			break
		}
	}
	var reqs []ChannelReq
	var tags []tag
	for _, s := range snaps {
		if !(s.running && s.clientEP != nil && len(s.ddcs) > 0) {
			continue
		}
		for _, d := range sortedDDCKeys(s.ddcs) {
			c := s.ddcs[d]
			fc := c.freq
			if !e.freqValid(fc) {
				if !haveFallback {
					continue // nothing streamed to reuse yet
				}
				fc = fallback
			}
			reqs = append(reqs, ChannelReq{FcHz: fc, BwHz: float64(c.rate), Adc: adcBits(c.adc),
				Name: fmt.Sprintf("r%dd%d", s.radio.idx, d)})
			tags = append(tags, tag{radio: s.radio, ddc: d, c: c})
		}
	}
	return e.admitBudget(reqs, tags)
}

// admitBudget keeps only as many DDCs as the board's bin budget allows, so we NEVER
// hand the board an over-budget union (TCP-rejected -> retried blindly takes down the
// shared session).  Greedy by priority order; an over-budget DDC is fed the fallback
// run (zero extra bins) rather than dropped, and retunes once budget frees.
func (e *Emitter) admitBudget(reqs []ChannelReq, tags []tag) ([]ChannelReq, []tag) {
	if len(reqs) <= 1 {
		return reqs, tags
	}
	var keptReqs []ChannelReq
	kept := map[int]bool{}
	for i, req := range reqs {
		p := planChannels(append(append([]ChannelReq(nil), keptReqs...), req), e.guard, e.budget)
		if p.CostA+p.CostB <= e.budget {
			kept[i] = true
			keptReqs = append(keptReqs, req)
		}
	}
	if len(kept) == len(reqs) {
		return reqs, tags // everything fits (common case)
	}
	var fb *ChannelReq
	if len(keptReqs) > 0 {
		fb = &keptReqs[0]
	}
	var outReqs []ChannelReq
	var outTags []tag
	for i := range reqs {
		if kept[i] {
			outReqs = append(outReqs, reqs[i])
			outTags = append(outTags, tags[i])
		} else if fb != nil {
			outReqs = append(outReqs, ChannelReq{FcHz: fb.FcHz, BwHz: fb.BwHz, Adc: fb.Adc,
				Name: reqs[i].Name + "!budget"})
			outTags = append(outTags, tags[i])
			e.log("%s: over bin budget (%d); fed fallback until budget frees", reqs[i].Name, e.budget)
		}
		// else: nothing streamed to borrow from -> drop
	}
	return outReqs, outTags
}

func psigOf(req ChannelReq, c *ddcState) psigKey {
	return psigKey{fc: req.FcHz, adc: req.Adc, rate: c.rate}
}

func (e *Emitter) makePipe(meta Meta, cp ChannelPlan, req ChannelReq, r *Radio, ddc int, c *ddcState, seq uint32) *pipe {
	ka := meta.Ka
	adc := 1
	if req.Adc&1 == 0 {
		ka = meta.Kb
		adc = 2
	}
	var idx, kaSub []int
	for i, k := range ka {
		if k >= cp.Klo && k <= cp.Khi {
			idx = append(idx, i)
			kaSub = append(kaSub, k)
		}
	}
	if len(idx) == 0 {
		e.log("radio%d ddc%d: bins %d..%d not granted; skipping", r.idx, ddc, cp.Klo, cp.Khi)
		return nil
	}
	sock, err := r.dataSocketFor(ddc)
	if err != nil {
		e.log("radio%d ddc%d: data socket: %v", r.idx, ddc, err)
		return nil
	}
	return &pipe{
		radio: r, ddc: ddc, adc: adc, idx: idx,
		synth:  newStreamSynthP(kaSub, cp.Kc, cp.TuneHz, cp.Phases),
		resamp: newComplexResampler(float64(cp.Phases)*binRate, c.rate),
		sock:   sock, seq: seq, psig: psigOf(req, c),
	}
}

// openBoard opens the shared BoardStream, retrying gently (a reopen just after a
// close can hit a transient board teardown race).  Returns nil on give-up.
func (e *Emitter) openBoard(reqs []ChannelReq, tags []tag) *BoardStream {
	var dds ddsInject
	if len(e.dds) > 0 {
		dds = ddsInject{}
		for i, t := range tags {
			if inc, ok := e.dds[[2]int{t.radio.idx, t.ddc}]; ok {
				dds[i] = struct {
					K     int
					PhInc uint32
				}{K: 0, PhInc: inc}
			}
		}
	}
	chans := toIngestChannels(reqs, dds)
	gaps := []time.Duration{0, 400 * time.Millisecond, 800 * time.Millisecond}
	var last error
	for attempt, gap := range gaps {
		if gap > 0 {
			time.Sleep(gap)
		}
		bs, err := openBoardStream(e.boardIP, chans, e.udpPort, e.guard)
		if err == nil {
			return bs
		}
		last = err
		e.log("board open attempt %d failed (%v)", attempt+1, err)
	}
	e.log("board open failed after %d tries: %v", len(gaps), last)
	return nil
}

// rebuildResult carries the (re)planned session.
type rebuildResult struct {
	stream *BoardStream
	sig    string
	pipes  map[pipeKey]*pipe
	retry  bool // transient board-open failure; caller retries shortly
	active bool // a stream is running
}

// rebuild (re)plans the union across all radios.  It reopens the BoardStream only
// when the union bin set changes; otherwise it keeps the stream and rebuilds only the
// pipes whose DDC config changed (so unchanged DDCs stay gapless).
func (e *Emitter) rebuild(snaps []radioSnap, stream *BoardStream, curSig string, pipes map[pipeKey]*pipe) rebuildResult {
	reqs, tags := e.plan(snaps)
	if len(reqs) == 0 {
		if stream != nil {
			stream.close()
		}
		return rebuildResult{}
	}
	plan := planChannels(reqs, e.guard, e.budget)
	sig := runSig(plan.RunsA) + "|" + runSig(plan.RunsB)
	rebuilt := stream == nil || sig != curSig
	if rebuilt {
		if stream != nil {
			stream.close()
			time.Sleep(150 * time.Millisecond) // let the one-session board tear down
		}
		stream = e.openBoard(reqs, tags)
		if stream == nil {
			return rebuildResult{retry: true}
		}
		curSig = sig
	}
	newPipes := map[pipeKey]*pipe{}
	for i := range tags {
		t, cp, req := tags[i], plan.Channels[i], reqs[i]
		key := pipeKey{radio: t.radio.idx, ddc: t.ddc}
		if !rebuilt {
			if old := pipes[key]; old != nil && old.psig == psigOf(req, t.c) {
				old.radio = t.radio // refresh routing reference
				newPipes[key] = old
				continue
			}
		}
		var seq uint32
		if old := pipes[key]; old != nil {
			seq = old.seq
		}
		if p := e.makePipe(stream.meta, cp, req, t.radio, t.ddc, t.c, seq); p != nil {
			newPipes[key] = p
		}
	}
	note := "bins unchanged"
	if rebuilt {
		note = "reprogrammed"
	}
	e.log("streaming: runs A=%v B=%v cost=%d; pipes %d (%s)",
		plan.RunsA, plan.RunsB, plan.CostA+plan.CostB, len(newPipes), note)
	return rebuildResult{stream: stream, sig: curSig, pipes: newPipes, active: true}
}

func runSig(runs []run) string {
	var sb strings.Builder
	for _, r := range runs {
		fmt.Fprintf(&sb, "%d:%d,", r.K0, r.W)
	}
	return sb.String()
}

// emitIQ frames iq into 238-sample RX I/Q packets (scaled by wireGain) and sends
// them to the client.  Partial frames stay buffered in the pipe for the next call.
func (e *Emitter) emitIQ(p *pipe, ep *net.UDPAddr, iq []complex128) {
	if len(iq) > 0 {
		p.buf = append(p.buf, iq...)
	}
	spf := samplesPerFrame
	frame := make([]complex128, spf)
	n := 0
	for len(p.buf)-n >= spf {
		for i := 0; i < spf; i++ {
			v := p.buf[n+i]
			frame[i] = complex(real(v)*e.wireGain, imag(v)*e.wireGain)
		}
		pkt := buildRxIQ(p.seq, frame, 24, 0)
		e.send(p, ep, pkt)
		p.seq++
		n += spf
	}
	p.buf = append(p.buf[:0], p.buf[n:]...) // compact remainder to the front
}

// processPipe runs one pipe's DSP for one datagram: gather its bins per block ->
// pushPhases -> resample -> frame+send.  scratch is the caller's reusable idx-gather
// buffer (one per goroutine, never shared).  It touches only this pipe's own state
// (synth/resamp/buf/seq) and its own socket, so distinct pipes can run concurrently.
// Ordering within a pipe is preserved (its blocks are pushed in datagram order), so
// the output is byte-identical regardless of which goroutine runs it.
func (e *Emitter) processPipe(p *pipe, colsA, colsB [][]complex128, scratch *[]complex128) {
	ep := p.radio.clientEP.Load()
	if ep == nil {
		return
	}
	cols := colsA
	if p.adc == 2 {
		cols = colsB
	}
	if len(cols) == 0 || len(cols[0]) == 0 {
		return
	}
	sub := *scratch
	if cap(sub) < len(p.idx) {
		sub = make([]complex128, len(p.idx))
		*scratch = sub
	}
	sub = sub[:len(p.idx)]
	var hr []complex128
	for _, col := range cols {
		for i, j := range p.idx {
			sub[i] = col[j]
		}
		p.synth.pushPhases(sub, &hr)
	}
	iq := p.resamp.process(hr)
	e.emitIQ(p, ep, iq)
}

func (e *Emitter) worker() {
	defer e.wg.Done()
	var stream *BoardStream
	var curSig string
	pipes := map[pipeKey]*pipe{}
	active := false
	var lastVer uint64
	var haveVer bool
	var rebuildAt, rebuildSince time.Time
	retryBackoff := time.Second

	var pool *emitPool
	if e.reconWorkers > 1 {
		pool = newEmitPool(e, e.reconWorkers)
		defer pool.stop()
	}
	serialScratch := []complex128{}
	for {
		select {
		case <-e.stop:
			if stream != nil {
				stream.close()
			}
			return
		default:
		}
		ver, snaps := e.snapshot()
		now := time.Now()
		if !haveVer || ver != lastVer {
			lastVer = ver
			haveVer = true
			rebuildAt = now.Add(settleDur)
			if rebuildSince.IsZero() {
				rebuildSince = now
			}
		}
		if !rebuildSince.IsZero() && now.Sub(rebuildSince) >= maxSettleDur && !rebuildAt.IsZero() {
			rebuildAt = now
		}
		if !rebuildAt.IsZero() && !now.Before(rebuildAt) {
			res := e.rebuild(snaps, stream, curSig, pipes)
			rebuildSince = time.Time{}
			if res.retry {
				stream, curSig, pipes, active = nil, "", map[pipeKey]*pipe{}, false
				retryBackoff *= 2
				if retryBackoff > 8*time.Second {
					retryBackoff = 8 * time.Second
				}
				rebuildAt = now.Add(retryBackoff)
				e.log("board open failed; next try in %.0fs", retryBackoff.Seconds())
			} else {
				stream, curSig, pipes, active = res.stream, res.sig, res.pipes, res.active
				retryBackoff = time.Second
				rebuildAt = time.Time{}
				// Repartition while the pool workers are parked on <-in (this runs
				// between fork-joins, so no worker is reading its pipe set); the next
				// dispatch establishes happens-before on the new assignment.
				if pool != nil {
					pool.repartition(pipes)
				}
			}
		}
		if !active || stream == nil {
			time.Sleep(10 * time.Millisecond)
			continue
		}
		colsA, colsB, ok, err := stream.recv()
		if err != nil {
			e.log("board stream error (%v); will re-plan", err)
			stream.close()
			stream, curSig, pipes, active = nil, "", map[pipeKey]*pipe{}, false
			continue
		}
		if !ok || len(colsA) == 0 {
			continue
		}
		if pool != nil {
			pool.dispatch(colsA, colsB) // fan out over cores, barrier on completion
		} else {
			for _, p := range pipes {
				e.processPipe(p, colsA, colsB, &serialScratch)
			}
		}
	}
}

// ---------------------------------------------------------------- run loop

// Serve runs the emitter until Stop() (or a fatal listen error).  It blocks.
func (e *Emitter) Serve() {
	ips := make([]string, len(e.radios))
	for i, r := range e.radios {
		ips[i] = r.bindIP
	}
	e.log("serving P2 (board %s); %d radio(s) on [%s] control 1024/1025/1027, RX I/Q from 1035+ddc",
		e.boardIP, len(e.radios), strings.Join(ips, ", "))

	// one reader goroutine per control socket + one for the shared discovery listener
	for _, r := range e.radios {
		for port, sock := range r.ctrl {
			e.wg.Add(1)
			go e.readCtrl(r, port, sock)
		}
	}
	if e.discSock != nil {
		e.wg.Add(1)
		go e.readDiscovery(e.discSock)
	}
	// liveness sweep
	e.wg.Add(1)
	go e.livenessLoop()
	// data worker
	e.wg.Add(1)
	go e.worker()
	// board temperature monitor (log-only)
	mon := startBoardMonitor(e.boardIP, time.Duration(e.monInterval)*time.Second)

	<-e.stop
	mon.Stop()
	e.closeAll()
	e.wg.Wait()
}

func (e *Emitter) readCtrl(r *Radio, port int, sock *net.UDPConn) {
	defer e.wg.Done()
	buf := make([]byte, 2048)
	for {
		select {
		case <-e.stop:
			return
		default:
		}
		_ = sock.SetReadDeadline(time.Now().Add(500 * time.Millisecond))
		n, src, err := sock.ReadFromUDP(buf)
		if err != nil {
			var ne net.Error
			if errors.As(err, &ne) && ne.Timeout() {
				continue
			}
			return // socket closed
		}
		pkt := append([]byte(nil), buf[:n]...)
		e.handleCtrl(r, port, pkt, src)
	}
}

func (e *Emitter) readDiscovery(sock *net.UDPConn) {
	defer e.wg.Done()
	buf := make([]byte, 2048)
	for {
		select {
		case <-e.stop:
			return
		default:
		}
		_ = sock.SetReadDeadline(time.Now().Add(500 * time.Millisecond))
		n, src, err := sock.ReadFromUDP(buf)
		if err != nil {
			var ne net.Error
			if errors.As(err, &ne) && ne.Timeout() {
				continue
			}
			return
		}
		e.handleDiscoveryAll(append([]byte(nil), buf[:n]...), src)
	}
}

func (e *Emitter) livenessLoop() {
	defer e.wg.Done()
	t := time.NewTicker(500 * time.Millisecond)
	defer t.Stop()
	for {
		select {
		case <-e.stop:
			return
		case <-t.C:
			e.checkLiveness()
		}
	}
}

// Stop signals the emitter to shut down.
func (e *Emitter) Stop() {
	e.stopOnce.Do(func() { close(e.stop) })
}
