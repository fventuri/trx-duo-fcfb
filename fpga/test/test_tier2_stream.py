# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Tier-2: the RTL-faithful fixed-point Stage-1 (T=4) reproduces the golden wire.

Runs the composed fixed-point chain (WOLA fold w/ quantized coeffs -> maia FFT
fixed-point model -> exact A/B split -> 2^shift int16 quantise) at the real
N=4096 on the emit_stream.py scene, then checks the shipped bins vs sim/stream.bin
and the reconstructed channel vs sim/ref_channel.f64. Each stage's model is
separately proven bit-exact to its Amaranth RTL (test_wola_prefilter,
test_ab_split, test_quantise, test_stage1), so this validates the composed
numerics the RTL emits, without a (prohibitively slow) full N=4096 cycle sim.
"""
import os

import pytest

from models import tier2_stream as T2


def test_tier2_reproduces_golden():
    r = T2.run(nblk_use=256)

    # WOLA output must fit the 17-bit FFT input rule with headroom.
    assert r["wola_peak"] <= 65535, f"WOLA out peak {r['wola_peak']} > 17-bit"
    assert r["sat"] == 0, f"int16 quantise saturated on {r['sat']} bins"

    # Clean single-tone T=4 reconstruction: far above M1 T=1 (~45 dB).
    assert r["sinad_db"] > 80.0, f"SINAD only {r['sinad_db']:.1f} dB"

    # Shipped bins match the golden float wire (int16 + coeff-quant floor).
    if r["bin_resid_db"] is not None:
        assert r["bin_resid_db"] < -60.0, \
            f"shipped-bin residual only {r['bin_resid_db']:.1f} dB vs stream.bin"

    # Reconstructed channel matches the host's golden reference.
    if r["ch_resid_db"] is not None:
        assert r["ch_resid_db"] < -70.0, \
            f"channel residual only {r['ch_resid_db']:.1f} dB vs ref_channel.f64"
