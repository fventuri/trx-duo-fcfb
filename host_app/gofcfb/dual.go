// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// The validated windowed-dual synthesis kernel g (length K*N = 24576, K=6),
// exported once from the Python synth.load_dual(K=6) and embedded so the binary
// is self-contained.  Raw little-endian complex128 (re,im f64 pairs).
package fcfb

import (
	_ "embed"
	"encoding/binary"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"regexp"
)

//go:embed dual_R3125_T4_K6.f64
var dualBytes []byte

// The embedded default kernel's declared analysis params, from its filename
// dual_R3125_T4_K6.f64 (the raw coefficients don't encode R/T -- the name is the
// source of truth). These start as the active kernel's params; loadKernelFile
// overrides them from a kernel= filename.
const embedKernelName = "dual_R3125_T4_K6.f64"

// gDual is the active synthesis kernel: the embedded validated dual (K=6, R=3125,
// T=4) by default, or an external file if [board] kernel= is set.
var gDual = parseDual(dualBytes)

// kernelR/kernelT/kernelK are the ACTIVE kernel's declared analysis params, used
// to verify the kernel matches the board (see verifyKernel). kernelNameParsed is
// false when a kernel= filename didn't match the convention, in which case
// kernelR/kernelT are 0 (unknown) and the board-match check is skipped.
var (
	kernelR, kernelT, kernelK = 3125, 4, 6
	kernelNameParsed          = true
)

// kernelNameRe matches the kernel filename convention dual_R<R>_T<T>_K<K>.f64.
var kernelNameRe = regexp.MustCompile(`^dual_R(\d+)_T(\d+)_K(\d+)\.f64$`)

// parseKernelName extracts (R,T,K) from a kernel basename. ok=false if it doesn't
// match the dual_R<R>_T<T>_K<K>.f64 convention.
func parseKernelName(base string) (r, t, k int, ok bool) {
	m := kernelNameRe.FindStringSubmatch(base)
	if m == nil {
		return 0, 0, 0, false
	}
	fmt.Sscanf(m[1], "%d", &r)
	fmt.Sscanf(m[2], "%d", &t)
	fmt.Sscanf(m[3], "%d", &k)
	return r, t, k, true
}

func parseDual(b []byte) []complex128 {
	n := len(b) / 16
	g := make([]complex128, n)
	for i := 0; i < n; i++ {
		re := math.Float64frombits(binary.LittleEndian.Uint64(b[i*16 : i*16+8]))
		im := math.Float64frombits(binary.LittleEndian.Uint64(b[i*16+8 : i*16+16]))
		g[i] = complex(re, im)
	}
	return g
}

// loadKernelFile replaces the embedded kernel with one read from disk (raw
// little-endian complex128 pairs, as exported by the Python synth.load_dual).
// It must be a whole number of N-sample frames (length K*N) and fit for the SAME
// analysis hop R=3125 / T=4 the board uses -- a mismatched kernel silently
// produces garbage.  Returns the K it inferred.
func loadKernelFile(path string) (int, error) {
	b, err := os.ReadFile(path)
	if err != nil {
		return 0, err
	}
	if len(b) == 0 || len(b)%16 != 0 {
		return 0, fmt.Errorf("kernel %s: %d bytes is not a whole number of complex128", path, len(b))
	}
	g := parseDual(b)
	if len(g)%nFFT != 0 {
		return 0, fmt.Errorf("kernel %s: %d samples is not a whole number of N=%d frames", path, len(g), nFFT)
	}
	gDual = g
	kFromLen := len(g) / nFFT
	// Declared analysis params come from the filename (the coefficients don't
	// encode R/T). A non-conforming name -> R/T unknown; the board-match check is
	// then skipped rather than failing on a naming choice.
	base := filepath.Base(path)
	if r, t, k, ok := parseKernelName(base); ok {
		kernelR, kernelT, kernelK, kernelNameParsed = r, t, k, true
		if k != kFromLen {
			fmt.Fprintf(os.Stderr, "kernel warning: %s filename says K=%d but its length implies K=%d (mislabelled?)\n",
				base, k, kFromLen)
		}
	} else {
		kernelR, kernelT, kernelNameParsed = 0, 0, false
		kernelK = kFromLen
		fmt.Fprintf(os.Stderr, "kernel warning: %s is not in dual_R<R>_T<T>_K<K>.f64 form; cannot determine R/T -- board-match check skipped\n",
			base)
	}
	return kFromLen, nil
}
