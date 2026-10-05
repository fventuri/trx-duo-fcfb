// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// common-mode-cancelling: run jt9/wsprd (or any WSJT-X-style decoder) on a dual-ADC
// diversity WAV fcfbfarm wrote, decoding THREE channels -- antenna A, antenna B, and
// the common-mode-cancelled channel C = B-β·A (--ref A, default) or A-β·B (--ref B).
// It emits all three tagged "#ANT A/B/CMC", so fcfbfarm merges them.  Like
// wav-to-decoder, but with the extra cancelled channel; needs dual-ADC I/Q (float32,
// 4 channels) and hard-fails on any other format.
//
//	common-mode-cancelling jt9 --ft8 {wav}
//	common-mode-cancelling --ref B jt9 --ft4 {wav}
//	common-mode-cancelling -v wsprd -f {fmhz} {wav}     # -v: print β to stderr
//
// The logic lives in package fcfb (shared with the other commands, reusing
// wav-to-decoder's WAV read / I-channel extraction / decode path); this is just the
// entry point.  See cmc.go.
package main

import "fcfb"

func main() { fcfb.CommonModeCancellingMain() }
