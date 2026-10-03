# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Regression for the wideband HW-verify residual metric (sim/verify_dds_hw_wideband).

A live-retune wideband capture is phase-referenced to whatever sample the fold
restarted on, which need not match the from-scratch golden's origin. That benign
sub-block offset multiplies bin k by exp(-j2*pi*k*s/N) -- a per-bin phase LINEAR
in k -- which a single global best-fit gain cannot remove, so the OLD metric
false-FAILed a correct board once the kept bins spanned a wide k-range (the
-14 dB seen at W=51). ``resid_db_slope`` also removes the linear-in-k phase slope
(the sample-delay ambiguity), so a correct board scores its fixed-point floor at
any W while a GENUINE per-bin error (not a clean k-ramp) is still caught.

The metric unit tests are pure numpy (fast). One end-to-end test drives the full
fixed-point golden through ``verify`` at wide W to guard the wiring.
"""
import os
import struct
import sys

import numpy as np
import pytest

# The verify harness lives in sim/ (not on the fpga test path by default).
_SIM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "sim")
if _SIM not in sys.path:
    sys.path.insert(0, _SIM)

import verify_dds_hw_wideband as VW          # noqa: E402

N = 1 << VW.ORDER_LOG2


def _make_ref(W, M, kc, seed=0xB0A):
    """A golden-like (W, M) block: a strong tone bin + a modest bit-exact-ish
    skirt, with the per-bin, per-block phase rotation the real fold has."""
    rng = np.random.default_rng(seed)
    ks = np.arange(W)
    ref = (rng.standard_normal((W, M)) + 1j * rng.standard_normal((W, M))) * 5.0
    ref[kc] += 4000.0 * np.exp(1j * 2 * np.pi * 0.06 * np.arange(M))   # tone
    return ref, ks


def test_slope_fit_absorbs_benign_fold_offset():
    """A pure linear-in-k phase (sub-block sample offset) must fit to ~0 residual
    at wide k-span; the plain global-gain fit cannot."""
    W, M, kc = 51, 200, 25
    ref, ks = _make_ref(W, M, kc)
    dk = (ks - kc).astype(float)[:, None]
    for s in (0.0, 30.0, 100.0, 300.0):
        x = ref * np.exp(-1j * 2 * np.pi * (ks[:, None]) * s / N)
        db, theta, g = VW.resid_db_slope(x, ref, dk)
        assert db < -80.0, f"benign s={s} left residual {db:.1f} dB"
        # the fitted slope recovers the injected sample offset
        s_fit = -theta * N / (2 * np.pi)
        # phase is defined modulo the bin-0 reference; compare the slope, not kc
        assert abs(((s_fit - s + N / 2) % N) - N / 2) < 1.0

    # the OLD metric (global gain only) is wrecked by the same benign offset:
    # it false-FAILs the -60 dB bar, while the slope fit above passed at < -80.
    x = ref * np.exp(-1j * 2 * np.pi * (ks[:, None]) * 100.0 / N)
    db_plain, _ = VW.resid_db(x.reshape(-1), ref.reshape(-1))
    assert db_plain > VW.PASS_DB, \
        f"plain gain should false-FAIL the benign offset, got {db_plain:.1f} dB"


def test_slope_fit_does_not_mask_genuine_spur():
    """A real per-bin error (energy in a bin the golden lacks) is not a clean
    k-ramp, so the slope fit cannot hide it: residual tracks the spur level."""
    W, M, kc = 51, 200, 25
    ref, ks = _make_ref(W, M, kc)
    dk = (ks - kc).astype(float)[:, None]
    carrier = np.abs(ref[kc]).mean()
    for dbc in (-20.0, -40.0):
        x = ref.copy()
        x[kc - 10] += carrier * 10 ** (dbc / 20.0)      # inject an image
        db, _, _ = VW.resid_db_slope(x, ref, dk)
        assert abs(db - dbc) < 3.0, f"spur {dbc} dBc scored {db:.1f} dB"


# ------------------------------------------------------------------ end to end
def _write_stream(path, S, k0, W, mask, nb, bin_scale, fs=125e6):
    with open(path, "wb") as f:
        f.write(b"FCFBv1\0\0")
        f.write(struct.pack("<II", 1, N))
        f.write(struct.pack("<d", fs))
        f.write(struct.pack("<IIII", k0, W, mask, nb))
        f.write(struct.pack("<d", bin_scale))
        for m in range(nb):
            f.write(struct.pack("<Q", m))
            iq = np.empty(2 * W, np.int16)
            iq[0::2] = np.round(S[:, m].real).astype(np.int16)
            iq[1::2] = np.round(S[:, m].imag).astype(np.int16)
            f.write(iq.tobytes())


@pytest.mark.parametrize("s", [0, 100])
def test_verify_wideband_end_to_end(tmp_path, s):
    """Full fixed-point golden through ``verify`` at wide W: a clean capture and
    one with a benign sub-block fold offset both PASS; a genuine spur FAILs."""
    W, center, k = 51, 1500, 1500
    k0, mask, nb = center - W // 2, 1, 160
    Sg = VW.golden_run_bins(k, 8192, k0, W, mask, 6, 3125, nb + 8)[:, :nb]
    ks = np.arange(k0, k0 + W)

    def run(S, tag):
        p = str(tmp_path / f"{tag}.bin")
        Sq = np.round(S.real).astype(np.int16) + 1j * np.round(S.imag).astype(np.int16)
        _write_stream(p, Sq, k0, W, mask, nb, float(1 << 6))
        return VW.verify(p, k=k, amp=8192, hop=3125, shift=6, warmup=0, verbose=False)

    S = Sg * np.exp(-1j * 2 * np.pi * ks[:, None] * s / N)
    rc, info = run(S, f"benign_s{s}")
    assert rc == 0 and info["residual_db"] < VW.PASS_DB, \
        f"benign s={s}: {info['residual_db']:.1f} dB"
    assert info["peak"] == k

    # genuine -20 dBc spur in a bin the golden lacks -> must FAIL
    carrier = np.abs(Sg[W // 2]).mean()
    Sbad = Sg.copy().astype(np.complex128)
    Sbad[W // 2 - 10] += carrier * 10 ** (-20.0 / 20.0)
    rc, info = run(Sbad, "spur")
    assert rc == 1 and info["residual_db"] > -30.0, \
        f"genuine spur not caught: {info['residual_db']:.1f} dB"
