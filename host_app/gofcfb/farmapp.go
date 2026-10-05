// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// fcfbfarm: a standalone, dependency-free decoder farm for the fcfb board.
// Connects to the board (v3 TCP control + UDP data), reconstructs each configured
// channel from the shared bin stream, and runs jt9 (FT8/FT4) or wsprd (WSPR) on
// UTC-aligned windows.  Builds on Linux/Windows/macOS with only the Go standard
// library (the synthesis kernel is embedded).  Go port of host_app/farm.py.
//
//	fcfbfarm -config farm.ini
//	fcfbfarm -config farm.ini      # with replay=capture.bin -> offline decode
package fcfb

import (
	"flag"
	"fmt"
	"os"
	"os/signal"
	"runtime/pprof"
	"syscall"
)

func waitForSignal() {
	ch := make(chan os.Signal, 1)
	signal.Notify(ch, os.Interrupt, syscall.SIGTERM)
	<-ch
}

func FarmMain() {
	// Don't let a vanished stdout consumer kill us. Spot lines go to stdout (often
	// piped into tee/awk); if that reader dies -- e.g. when `timeout -s INT` signals
	// the whole pipeline group at the end of a timed run -- Go's default is to
	// terminate the process on SIGPIPE at fd 1, which would abort graceful shutdown
	// mid-flush (truncated final WAV, no "streaming done" summary). Ignoring SIGPIPE
	// turns those writes into harmless EPIPE errors instead, so the drain + stderr
	// summary still complete. (stderr is typically a file, so it stays writable.)
	signal.Ignore(syscall.SIGPIPE)

	cfgPath := flag.String("config", "farm.ini", "config file (INI/TOML-style)")
	cpuprofile := flag.String("cpuprofile", "", "write a CPU profile to this file (until SIGINT)")
	boardInfo := flag.Bool("board-info", false, "query the board's health (temp/voltages/model) once and exit")
	flag.Parse()

	if *cpuprofile != "" {
		f, ferr := os.Create(*cpuprofile)
		if ferr != nil {
			fmt.Fprintln(os.Stderr, "cpuprofile:", ferr)
			os.Exit(1)
		}
		defer f.Close()
		if ferr := pprof.StartCPUProfile(f); ferr != nil {
			fmt.Fprintln(os.Stderr, "cpuprofile:", ferr)
			os.Exit(1)
		}
		defer pprof.StopCPUProfile()
	}

	cfg, err := parseConfig(*cfgPath)
	if err != nil {
		fmt.Fprintln(os.Stderr, "config:", err)
		os.Exit(1)
	}
	if *boardInfo {
		LogBoardInfo(os.Stdout, cfg.BoardIP)
		return
	}
	if cfg.Kernel != "" {
		k, kerr := loadKernelFile(cfg.Kernel)
		if kerr != nil {
			fmt.Fprintln(os.Stderr, "kernel:", kerr)
			os.Exit(1)
		}
		if kernelNameParsed {
			fmt.Fprintf(os.Stderr, "synthesis kernel: %s (R=%d T=%d K=%d) -- verified against the board before streaming\n",
				cfg.Kernel, kernelR, kernelT, k)
		} else {
			fmt.Fprintf(os.Stderr, "synthesis kernel: %s (K=%d, R/T unknown from name)\n", cfg.Kernel, k)
		}
	} else {
		fmt.Fprintf(os.Stderr, "synthesis kernel: embedded %s (R=%d T=%d K=%d)\n",
			embedKernelName, kernelR, kernelT, kernelK)
	}
	if cfg.Replay != "" {
		err = runReplay(cfg)
	} else {
		err = runLive(cfg)
	}
	if err != nil {
		fmt.Fprintln(os.Stderr, "error:", err)
		os.Exit(1)
	}
}
