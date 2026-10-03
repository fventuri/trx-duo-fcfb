// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// fcfbhpsdr command entry point: parse flags, build the emitter, serve until SIGINT.
// Go port of host_app/hpsdr_emit.py's main().
package fcfb

import (
	"flag"
	"fmt"
	"os"
	"os/signal"
	"syscall"
)

// stringList collects a repeatable flag (e.g. --radio).
type stringList []string

func (s *stringList) String() string { return fmt.Sprint([]string(*s)) }
func (s *stringList) Set(v string) error {
	*s = append(*s, v)
	return nil
}

// EmitMain is the fcfbhpsdr entry point.  With -config it reads a farm-style INI
// (see emitconfig.go); without one it builds a single radio (or -radio N) from flags.
func EmitMain() {
	fs := flag.NewFlagSet("fcfbhpsdr", flag.ExitOnError)
	cfgPath := fs.String("config", "", "config file (INI/TOML-style, see hpsdr.example.ini); overrides the flags below")
	board := fs.String("board", "192.168.255.20", "fcfb board IP")
	bind := fs.String("bind", "0.0.0.0", "single-radio bind interface (ignored if -radio given)")
	var radios stringList
	fs.Var(&radios, "radio", "bind IP for one radio; repeat for N radios (e.g. 127.0.0.2)")
	boardID := fs.Int("board-id", boardAngelia, "advertised HPSDR board id (3=Angelia dual-ADC, 1=Hermes single-ADC)")
	nDDC := fs.Int("n-ddc", defaultNDDC, "DDCs advertised per radio")
	gain := fs.Float64("gain", 1.0, "I/Q output gain")
	guard := fs.Int("guard", guardDef, "guard bins each side of every channel (non-default needs a v4-capable board server)")
	mtu := fs.Int("mtu", 1500, "link MTU the board server was started with (-u); sets the bin budget (1500->242, 3980->655 int24). MUST match the server")
	budget := fs.Int("budget", 0, "override the bin budget (union W_a+W_b cap) directly")
	udpPort := fs.Int("udp-port", 55055, "local UDP data port the board streams to")
	reconWorkers := fs.Int("recon-workers", 1, "parallel per-DDC reconstruction goroutines (1 = single-core; >1 spreads synth+resample across cores)")
	monInterval := fs.Int("monitor-interval", monitorIntervalDef, "board temperature-log poll interval in seconds (0 = off)")
	boardInfo := fs.Bool("board-info", false, "query the board's info (temp/voltages/model/hw_rev) once and exit")
	_ = fs.Parse(os.Args[1:])

	var boardIP string
	var opts EmitterOpts
	var specs []RadioSpec
	if *cfgPath != "" {
		cfg, err := parseHPSDRConfig(*cfgPath)
		if err != nil {
			fmt.Fprintln(os.Stderr, "config:", err)
			os.Exit(1)
		}
		if err := applyKernel(cfg.Kernel); err != nil {
			fmt.Fprintln(os.Stderr, "kernel:", err)
			os.Exit(1)
		}
		boardIP = cfg.BoardIP
		opts = EmitterOpts{Gain: cfg.Gain, Guard: cfg.Guard, Budget: cfg.Budget, MTU: cfg.MTU,
			UDPPort: cfg.UDPPort, BinWidth: cfg.BinWidth, ReconWorkers: cfg.ReconWorkers,
			MonitorInterval: cfg.MonitorInterval, Verbose: true}
		specs = cfg.Radios
	} else {
		if err := applyKernel(""); err != nil { // print the embedded-kernel line
			fmt.Fprintln(os.Stderr, "kernel:", err)
			os.Exit(1)
		}
		boardIP = *board
		opts = EmitterOpts{Gain: *gain, Guard: *guard, Budget: *budget, MTU: *mtu,
			UDPPort: *udpPort, ReconWorkers: *reconWorkers, MonitorInterval: *monInterval,
			Verbose: true, BinWidth: 24}
		for _, ip := range radios {
			specs = append(specs, RadioSpec{BindIP: ip, BoardID: *boardID, NDDC: *nDDC})
		}
		if len(specs) == 0 {
			specs = []RadioSpec{{BindIP: *bind, BoardID: *boardID, NDDC: *nDDC}}
		}
	}

	if *boardInfo {
		LogBoardInfo(os.Stdout, boardIP)
		return
	}
	// Startup diagnostic: dump all board info (temp/voltages/model/fpga/hw_rev) so a
	// support log captures the board's identity + health up front.  Never fatal.
	LogBoardInfo(os.Stderr, boardIP)

	// Same startup checks fcfbfarm runs, in the same order.  (1) The host interface
	// reaching the board must be at the configured MTU, or jumbo frames are silently
	// dropped (0 I/Q) -- fatal with the fix command.  (2) The synthesis kernel must
	// match the board's gateware analysis params (R,T); a real mismatch is fatal
	// (reconstruction would be garbage), while an unreachable board / an old server
	// that can't report params only warns, so the server still starts.
	if err := checkHostMTU(boardIP, opts.UDPPort, opts.MTU); err != nil {
		fmt.Fprintln(os.Stderr, "fcfbhpsdr:", err)
		os.Exit(1)
	}
	if err := verifyKernel(boardIP); err != nil {
		fmt.Fprintln(os.Stderr, "fcfbhpsdr:", err)
		os.Exit(1)
	}

	em, err := NewEmitter(boardIP, specs, *bind, opts)
	if err != nil {
		fmt.Fprintln(os.Stderr, "fcfbhpsdr:", err)
		os.Exit(1)
	}

	ch := make(chan os.Signal, 1)
	signal.Notify(ch, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-ch
		em.Stop()
	}()
	em.Serve()
}

// applyKernel loads an external synthesis kernel (.f64) when one is configured,
// logging its params like fcfbfarm does (empty path keeps the embedded K=6 default).
func applyKernel(path string) error {
	if path == "" {
		fmt.Fprintf(os.Stderr, "synthesis kernel: embedded %s (R=%d T=%d K=%d)\n",
			embedKernelName, kernelR, kernelT, kernelK)
		return nil
	}
	k, err := loadKernelFile(path)
	if err != nil {
		return err
	}
	if kernelNameParsed {
		fmt.Fprintf(os.Stderr, "synthesis kernel: %s (R=%d T=%d K=%d) -- verify it matches the board\n",
			path, kernelR, kernelT, k)
	} else {
		fmt.Fprintf(os.Stderr, "synthesis kernel: %s (K=%d, R/T unknown from name)\n", path, k)
	}
	return nil
}
