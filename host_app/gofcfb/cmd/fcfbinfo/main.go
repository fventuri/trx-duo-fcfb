// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// fcfbinfo: a tiny standalone tool that sends one BINF (board-info) query to the
// fcfb board and prints the reply -- die temperature, supply voltages, FPGA part,
// hardware revision, model, sample rate and gateware.  BINF is served on the board's
// separate info port by its own thread, so this is safe to run even while the board
// is streaming to fcfbfarm / fcfbhpsdr.  Builds with only the Go standard library.
//
//	fcfbinfo                 # query 192.168.255.20
//	fcfbinfo -board 10.0.0.5
//	fcfbinfo 10.0.0.5        # IP positionally
//
// The query/decode logic lives in package fcfb (shared with fcfbfarm/fcfbhpsdr);
// this is just the entry point.
package main

import "fcfb"

func main() { fcfb.InfoMain() }
