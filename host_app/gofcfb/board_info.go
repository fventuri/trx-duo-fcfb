// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Board-info (BINF) query + a lightweight health monitor for fcfbfarm / fcfbhpsdr.
//
// The board server answers a BINF query (a 36-byte fcfb_req-shaped frame with magic
// "BINF") with one fcfb_board_info reply carrying the live Zynq XADC die temperature
// and supply voltages, the sample rate, and the board/gateware identity.  BINF is
// served on a SEPARATE port (FCFB_TCP_INFO_PORT) by a dedicated thread, so it answers
// even while the single-session control port is busy streaming -- letting the client
// watch the board's temperature during a long run and warn before it overheats.
//
// An old server (no info port) refuses the connection; queryBoardInfo then reports
// ok=false and the monitor logs "unavailable" once and goes quiet -- the same safe
// degradation as the FPRM params query.
package fcfb

import (
	"encoding/binary"
	"flag"
	"fmt"
	"io"
	"math"
	"net"
	"os"
	"time"
)

const (
	infoPort         = 7374 // FCFB_TCP_INFO_PORT: board-info responder
	boardInfoRespLen = 156  // sizeof(struct fcfb_board_info)

	// Shared board temperature-monitor default (fcfbfarm + fcfbhpsdr): poll every
	// 30 s (0 = off).
	monitorIntervalDef = 30
)

var (
	binfMagic    = [4]byte{'B', 'I', 'N', 'F'}
	boardInfoTag = []byte("FCFBI\x00\x00\x00")
	// voltNames maps the reply's fixed volt[] order to the XADC rail names.
	voltNames = []string{"vccint", "vccaux", "vccbram", "vccpint", "vccpaux", "vccoddr", "vrefp", "vrefn"}
)

// BoardInfo is the decoded fcfb_board_info reply.  TempC and any Volt entry is NaN
// when the board could not read that sensor.
type BoardInfo struct {
	Version  int
	TempC    float64
	Volt     []float64 // parallel to VoltName
	VoltName []string
	FpgaID   int // Zynq PS IDCODE device field (2 = xc7z010); 0 if unknown
	Fs       float64
	Model    string
	Gateware string
	HwRev    string // u-boot hw_rev env (e.g. "STEM_125-14_LN_v1.1"); "" if unavailable
}

// fpgaZynqParts maps the Zynq-7000 PS IDCODE device field (SLCR 0x530 >>12 & 0x1f)
// to the part name.  2 = xc7z010 is verified on the TRX-duo; the rest are the
// commonly-cited mainline codes.  Unlisted ids print as "id N".
var fpgaZynqParts = map[int]string{
	2:    "xc7z010",
	7:    "xc7z020",
	0x0c: "xc7z030",
	0x11: "xc7z045",
	0x16: "xc7z100",
}

// FpgaName returns the part name for the id, or "id N" when unknown.
func (bi BoardInfo) FpgaName() string {
	if n, ok := fpgaZynqParts[bi.FpgaID]; ok {
		return n
	}
	return fmt.Sprintf("id %d", bi.FpgaID)
}

// queryBoardInfo opens a short TCP connection to the board's info port, sends the
// BINF query, and reads the fcfb_board_info reply.  ok=false with a nil error means
// the server is old / has no info port (board info unavailable); a non-nil error is a
// connect/IO failure.
func queryBoardInfo(ip string, timeout time.Duration) (BoardInfo, bool, error) {
	return queryBoardInfoAddr(fmt.Sprintf("%s:%d", ip, infoPort), timeout)
}

// queryBoardInfoAddr is queryBoardInfo against an explicit host:port (used by tests).
func queryBoardInfoAddr(addr string, timeout time.Duration) (BoardInfo, bool, error) {
	ts, err := net.DialTimeout("tcp", addr, timeout)
	if err != nil {
		return BoardInfo{}, false, nil // no info port (old server) -> unavailable
	}
	defer ts.Close()
	_ = ts.SetDeadline(time.Now().Add(timeout))
	// A 36-byte fcfb_req-shaped frame with BINF magic + our proto version; the
	// server's first read is sizeof(fcfb_req)=36 bytes, so send all 36.
	req := make([]byte, 36)
	copy(req[0:4], binfMagic[:])
	binary.LittleEndian.PutUint16(req[4:6], uint16(protoVer))
	if _, err := ts.Write(req); err != nil {
		return BoardInfo{}, false, err
	}
	resp := make([]byte, boardInfoRespLen)
	if _, err := io.ReadFull(ts, resp); err != nil {
		return BoardInfo{}, false, nil // server dropped it -> unavailable
	}
	if string(resp[0:8]) != string(boardInfoTag) {
		return BoardInfo{}, false, nil
	}
	f32 := func(o int) float64 {
		return float64(math.Float32frombits(binary.LittleEndian.Uint32(resp[o : o+4])))
	}
	nvolt := int(binary.LittleEndian.Uint16(resp[10:12]))
	if nvolt > len(voltNames) {
		nvolt = len(voltNames)
	}
	bi := BoardInfo{
		Version:  int(binary.LittleEndian.Uint16(resp[8:10])),
		TempC:    f32(12),
		FpgaID:   int(binary.LittleEndian.Uint32(resp[48:52])),
		Fs:       math.Float64frombits(binary.LittleEndian.Uint64(resp[52:60])),
		Model:    cstrAt(resp, 60, 32),
		Gateware: cstrAt(resp, 92, 32),
		HwRev:    cstrAt(resp, 124, 32),
	}
	for i := 0; i < nvolt; i++ {
		bi.Volt = append(bi.Volt, f32(16+4*i))
		bi.VoltName = append(bi.VoltName, voltNames[i])
	}
	return bi, true, nil
}

// cstrAt reads a NUL-terminated (or full-width) C string from b[o:o+n].
func cstrAt(b []byte, o, n int) string {
	s := b[o : o+n]
	if i := indexByte(s, 0); i >= 0 {
		s = s[:i]
	}
	return string(s)
}

// Summary renders the board info as a one-line status string.
func (bi BoardInfo) Summary() string {
	s := fmt.Sprintf("temp %s", tempStr(bi.TempC))
	for i, v := range bi.Volt {
		s += fmt.Sprintf(" %s=%.3f", bi.VoltName[i], v)
	}
	if bi.Model != "" {
		s += fmt.Sprintf(" [%s %s", bi.Model, bi.FpgaName())
		if bi.Gateware != "" {
			s += "/" + bi.Gateware
		}
		s += "]"
	}
	return s
}

func tempStr(c float64) string {
	if math.IsNaN(c) {
		return "n/a"
	}
	return fmt.Sprintf("%.1fC", c)
}

// LogBoardInfo runs a one-shot BINF query and writes a full human-readable report of
// every board value to w.  fcfbfarm and fcfbhpsdr both call it at startup (to w =
// stderr) as a diagnostic, and the -board-info flag calls it (to w = stdout).  It
// never fails hard: an unavailable board (old server / unreachable) is noted.
func LogBoardInfo(w io.Writer, ip string) {
	bi, ok, err := queryBoardInfo(ip, 3*time.Second)
	if err != nil {
		fmt.Fprintf(w, "board %s: board-info query error: %v\n", ip, err)
		return
	}
	if !ok {
		fmt.Fprintf(w, "board %s: board-info unavailable (server too old / no info port)\n", ip)
		return
	}
	fmt.Fprintf(w, "board %s\n", ip)
	fmt.Fprintf(w, "  model     %s\n", bi.Model)
	fmt.Fprintf(w, "  hw_rev    %s\n", bi.HwRev)
	fmt.Fprintf(w, "  fpga      %s (id %d)\n", bi.FpgaName(), bi.FpgaID)
	fmt.Fprintf(w, "  gateware  %s\n", bi.Gateware)
	fmt.Fprintf(w, "  fs        %.0f Hz\n", bi.Fs)
	fmt.Fprintf(w, "  temp      %s\n", tempStr(bi.TempC))
	for i, v := range bi.Volt {
		fmt.Fprintf(w, "  %-8s  %.3f V\n", bi.VoltName[i], v)
	}
}

// InfoMain is the entry point for the standalone fcfbinfo tool: a dependency-free
// client that sends one BINF query to the board and prints the full board report
// (model/hw_rev/fpga/gateware/fs/temp/voltages) to stdout, then exits.  It touches
// nothing else -- no streaming, no config -- so it is safe to run against a board
// that is actively streaming (BINF is served on its own port by its own thread).
//
//	fcfbinfo [-board IP]        # default 192.168.255.20
//	fcfbinfo IP                 # IP may also be given positionally
func InfoMain() {
	fs := flag.NewFlagSet("fcfbinfo", flag.ExitOnError)
	board := fs.String("board", "192.168.255.20", "fcfb board IP")
	_ = fs.Parse(os.Args[1:])
	ip := *board
	if fs.NArg() > 0 {
		ip = fs.Arg(0) // positional IP overrides the flag default
	}
	LogBoardInfo(os.Stdout, ip)
}

// boardMonitor periodically polls the board and logs its temperature.  It is
// log-only (no alerting) and never changes streaming.
type boardMonitor struct {
	stop chan struct{}
	done chan struct{}
}

// startBoardMonitor launches a background poller that logs the board's die
// temperature every interval.  interval <= 0 disables it (returns nil).  If the board
// has no info port it logs "unavailable" once and stops polling (no spam against an
// old server).
func startBoardMonitor(ip string, interval time.Duration) *boardMonitor {
	if interval <= 0 {
		return nil
	}
	m := &boardMonitor{stop: make(chan struct{}), done: make(chan struct{})}
	go func() {
		defer close(m.done)
		t := time.NewTicker(interval)
		defer t.Stop()
		if !m.poll(ip) { // first poll immediately; stop if unavailable
			return
		}
		for {
			select {
			case <-m.stop:
				return
			case <-t.C:
				if !m.poll(ip) {
					return
				}
			}
		}
	}()
	return m
}

// poll queries once and logs the temperature.  Returns false to stop the monitor
// (board has no info port -- unavailable); true to keep polling (including on a
// transient query error).
func (m *boardMonitor) poll(ip string) bool {
	bi, ok, err := queryBoardInfo(ip, 3*time.Second)
	if err != nil {
		fmt.Fprintf(os.Stderr, "[board] temperature query error: %v (will retry)\n", err)
		return true
	}
	if !ok {
		fmt.Fprintf(os.Stderr, "[board] temperature monitoring unavailable (server too old / no info port); disabling\n")
		return false
	}
	fmt.Fprintf(os.Stderr, "[board] temp %s\n", tempStr(bi.TempC))
	return true
}

// Stop halts the monitor and waits for its goroutine to exit.  Safe on a nil monitor.
func (m *boardMonitor) Stop() {
	if m == nil {
		return
	}
	close(m.stop)
	<-m.done
}
