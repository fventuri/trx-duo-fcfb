// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// wav-to-decoder: run jt9/wsprd (or any WSJT-X-style decoder) on a WAV fcfbfarm
// wrote, autodetecting real-audio vs complex-I/Q and single vs dual ADC from the
// header.  Replaces the Python dual-decoder.  See package fcfb (wavdecode.go).
//
//	wav-to-decoder jt9 --ft8 {wav}
//	wav-to-decoder wsprd -f {fmhz} {wav}
//	wav-to-decoder --iq jt9 --ft8 capture.wav     # force I/Q interpretation
//
// The logic lives in package fcfb (shared with the other commands); this is just the
// entry point.
package main

import "fcfb"

func main() { fcfb.WavToDecoderMain() }
