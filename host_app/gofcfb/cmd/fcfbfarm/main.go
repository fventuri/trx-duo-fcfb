// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// fcfbfarm: a standalone, dependency-free decoder farm for the fcfb board.
// Connects to the board (v3/v4 TCP control + UDP data), reconstructs each configured
// channel from the shared bin stream, and runs jt9 (FT8/FT4) or wsprd (WSPR) on
// UTC-aligned windows.  Builds on Linux/Windows/macOS with only the Go standard
// library (the synthesis kernel is embedded).  Go port of host_app/farm.py.
//
//	fcfbfarm -config farm.ini
//	fcfbfarm -config farm.ini      # with replay=capture.bin -> offline decode
//
// The farm logic lives in package fcfb (shared with fcfbhpsdr); this is just the
// entry point.
package main

import "fcfb"

func main() { fcfb.FarmMain() }
