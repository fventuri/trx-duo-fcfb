// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Minimal openHPSDR Protocol-2 client harness.  Drives the fcfb HPSDR-P2 emitter (or
// a real HPSDR radio) headlessly: discover -> configure DDCs -> set frequencies +
// run -> receive RX DDC I/Q, demultiplexed by UDP source port (1035+ddc) exactly as
// piHPSDR does.  Go port of host_app/hpsdr_client.py; used by the loopback test and
// available for headless HW validation.
package fcfb

import (
	"net"
	"time"
)

// P2Client is a headless P2 client bound to one ephemeral UDP socket (control source
// == data destination, the P2 convention).
type P2Client struct {
	radioIP string
	sock    *net.UDPConn
	seqRS   uint32
	seqHP   uint32
}

func newP2Client(radioIP string) (*P2Client, error) {
	s, err := net.ListenUDP("udp", &net.UDPAddr{IP: net.IPv4zero, Port: 0})
	if err != nil {
		return nil, err
	}
	_ = s.SetReadBuffer(16 << 20)
	return &P2Client{radioIP: radioIP, sock: s}, nil
}

func (c *P2Client) to(port int) *net.UDPAddr {
	return &net.UDPAddr{IP: net.ParseIP(c.radioIP), Port: port}
}

// discover sends a discovery request and returns the first available reply.
func (c *P2Client) discover(timeout time.Duration) (discoveryReply, bool) {
	req := make([]byte, 60)
	req[4] = 0x02
	c.sock.WriteToUDP(req, c.to(generalFromHostPort))
	deadline := time.Now().Add(timeout)
	buf := make([]byte, 2048)
	for time.Now().Before(deadline) {
		_ = c.sock.SetReadDeadline(time.Now().Add(500 * time.Millisecond))
		n, _, err := c.sock.ReadFromUDP(buf)
		if err != nil {
			continue
		}
		if d, ok := parseDiscoveryReply(buf[:n]); ok && (d.status == 2 || d.status == 3) {
			return d, true
		}
	}
	return discoveryReply{}, false
}

// configure sends the receiver-specific packet (which DDCs, rate, ADC).
func (c *P2Client) configure(ddcs map[int]ddcConfig, nAdc int) {
	c.seqRS++
	c.sock.WriteToUDP(buildReceiveSpecific(c.seqRS, ddcs, nAdc), c.to(rxSpecFromHostPort))
}

// setRun sends a high-priority packet (run bit + per-DDC centre frequencies).
func (c *P2Client) setRun(running bool, freqs []float64) {
	c.seqHP++
	c.sock.WriteToUDP(buildHighPriority(c.seqHP, running, freqs), c.to(highPriFromHostPort))
}

// receive collects RX I/Q for dur, returning {ddc: samples} in seq order, filling
// small seq gaps with zeros to keep timing intact (demux by UDP source port).
func (c *P2Client) receive(dur time.Duration, ddcIDs []int) map[int][]complex128 {
	parts := map[int][]complex128{}
	expect := map[int]int64{}
	for _, d := range ddcIDs {
		parts[d] = nil
		expect[d] = -1
	}
	deadline := time.Now().Add(dur)
	buf := make([]byte, 4096)
	for time.Now().Before(deadline) {
		_ = c.sock.SetReadDeadline(time.Now().Add(200 * time.Millisecond))
		n, src, err := c.sock.ReadFromUDP(buf)
		if err != nil {
			continue
		}
		ddc := src.Port - rxIQToHostPort0
		if _, ok := parts[ddc]; !ok || n < 16 {
			continue
		}
		r, ok := parseRxIQ(buf[:n])
		if !ok {
			continue
		}
		if expect[ddc] >= 0 {
			gap := (int64(r.seq) - expect[ddc]) & 0xFFFFFFFF
			if gap > 0 && gap < 100 {
				parts[ddc] = append(parts[ddc], make([]complex128, int(gap)*r.n)...)
			}
		}
		expect[ddc] = (int64(r.seq) + 1) & 0xFFFFFFFF
		parts[ddc] = append(parts[ddc], r.iq...)
	}
	return parts
}

func (c *P2Client) close() { c.sock.Close() }
