// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// fcfbhpsdr: a live openHPSDR Protocol-2 radio server over the fcfb bin stream.
// Presents the board (a wideband analysis channelizer) to any P2 client
// (piHPSDR, Thetis, linhpsdr) as one or more HPSDR radios, reconstructing each DDC's
// I/Q on the host.  Builds on Linux/Windows/macOS with only the Go standard library
// (the synthesis kernel is embedded).  Go port of host_app/hpsdr_emit.py.
//
//	fcfbhpsdr -board 192.168.255.20                    # one radio, all interfaces
//	fcfbhpsdr -board 192.168.255.20 -radio 127.0.0.2 -radio 127.0.0.3   # two radios
//
// The server logic lives in package fcfb (shared with fcfbfarm); this is the entry
// point.
package main

import "fcfb"

func main() { fcfb.EmitMain() }
