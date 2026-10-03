#!/usr/bin/env python3
# Golden-reference extractor for the M3 Stage-1 RTL block tests.
#
# Re-exposes the M1-validated numpy Stage-1 (../../sim/chanfcfb_model.py) as
# per-block reference arrays the Amaranth block tests compare against:
#
#   analysis(x, T)      -> packed complex FFT bins  Y[k, m]        (pre-split)
#   ab_split(Y)         -> A[k, m], B[k, m]          (the A/B split target)
#   run + quantise      -> int16 wire bins           (bin_select + quantise target)
#
# This is a THIN wrapper: no new DSP is defined here. Any behaviour the RTL must
# match is defined in chanfcfb_model.py / emit_stream.py, not re-implemented.
#
# CLEAN-ROOM: numpy only; no ka9q / npapi source. maia-hdl is a separate MIT
# dependency used by the RTL, not by this reference.

import os
import sys

import numpy as np

# Import the M1 model + the byte-exact emitter from ../../sim.
_SIM = os.path.join(os.path.dirname(__file__), "..", "..", "sim")
sys.path.insert(0, os.path.abspath(_SIM))
import chanfcfb_model as M            # noqa: E402  (M1 golden model)

N, FS, BINW = M.N, M.FS, M.BINW


def pack_scene(freqs_a, amps_a, freqs_b=None, amps_b=None):
    """Two real ADC scenes -> packed complex input z = a + j*b (M1 convention).

    ADC0 (A) is the real part, ADC1 (B) the imaginary part, of ONE complex FFT
    (PLAN §3). Pass freqs_b=None for an ADC0-only scene (matches emit_stream.py).
    """
    a = M.tones(freqs_a, amps_a)
    if freqs_b is None:
        return a + 0j
    b = M.tones(freqs_b, amps_b)
    return a + 1j * b


def stage1_bins(z, T=4):
    """Full Stage-1 up to the A/B split. Returns (Y, A, B), each (N, NBLK)."""
    Y = M.analysis(z, T)
    A, B = M.ab_split(Y)
    return Y, A, B


def quantise_run(S_run):
    """Match emit_stream.py: scale a run (W, NBLK) to int16 I/Q + bin_scale."""
    bin_scale = np.abs(S_run).max() / 32000.0
    Q = np.round(S_run / bin_scale)
    Qi = np.clip(Q.real, -32768, 32767).astype(np.int16)
    Qq = np.clip(Q.imag, -32768, 32767).astype(np.int16)
    return Qi, Qq, bin_scale


def default_scene():
    """The emit_stream.py demo scene: one ADC0 tone near bin KB, T=4, run KB-3..+3."""
    T = 4
    KB = M.KB
    K0, W = KB - 3, 7
    f_tone = (KB + 0.15) * BINW
    z = pack_scene([f_tone], [1.0])
    Y, A, B = stage1_bins(z, T)
    S_run = A[K0:K0 + W, :]
    Qi, Qq, bin_scale = quantise_run(S_run)
    return dict(T=T, KB=KB, K0=K0, W=W, f_tone=f_tone,
                Y=Y, A=A, B=B, S_run=S_run, Qi=Qi, Qq=Qq, bin_scale=bin_scale)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "stage1_ref.npz"
    d = default_scene()
    np.savez_compressed(
        out,
        T=d["T"], N=N, FS=FS, KB=d["KB"], K0=d["K0"], W=d["W"],
        f_tone=d["f_tone"], bin_scale=d["bin_scale"],
        Y=d["Y"], A=d["A"], B=d["B"], S_run=d["S_run"],
        Qi=d["Qi"], Qq=d["Qq"],
    )
    print(f"wrote {out}")
    print(f"  T={d['T']}  N={N}  KB={d['KB']}  run k0={d['K0']} W={d['W']}  "
          f"nblocks={d['Y'].shape[1]}")
    print(f"  |Y| peak = {np.abs(d['Y']).max():.3e}   "
          f"|A| peak = {np.abs(d['A']).max():.3e}   "
          f"|B| peak = {np.abs(d['B']).max():.3e}")
    print(f"  bin_scale = {d['bin_scale']:.6e}   "
          f"int16 I range = [{d['Qi'].min()}, {d['Qi'].max()}]")


if __name__ == "__main__":
    main()
