# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Byte-exact round-trip tests for the v3 (masked multi-run) stream reader in
``sim/fcfb_stream_read.py``: header parse, run-list -> per-ADC bin lists, and the
W_a A-run + W_b B-run record split, for both int16 and int24 widths."""
import os
import struct
import sys

import numpy as np
import pytest

_SIM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "sim")
if _SIM not in sys.path:
    sys.path.insert(0, _SIM)

from fcfb_stream_read import (parse_header, runs_to_bins, record_size_ab,  # noqa: E402
                              unpack_ab, read_stream_v3)

N, FS = 4096, 125e6


def _bin_le(val, bb):
    """int -> bb little-endian signed bytes (two's complement)."""
    return int(val).to_bytes(bb, "little", signed=True)


def _build_v3(runs, blocks_a, blocks_b, bin_width, shift=0):
    """Assemble a v3 stream: header + run-list + records. blocks_a[m] is the list
    of W_a complex A-bins for block m (ascending abs-k), blocks_b likewise."""
    bb = bin_width // 8
    ka, kb = runs_to_bins(runs)
    W_a, W_b = len(ka), len(kb)
    nb = len(blocks_a)
    adc_mask = (1 if W_a else 0) | (2 if W_b else 0)
    hdr = (b"FCFBv1\0\0" + struct.pack("<II", 3, N) + struct.pack("<d", FS)
           + struct.pack("<IIII", adc_mask, W_a, W_b, nb)
           + struct.pack("<d", float(1 << shift)) + struct.pack("<II", bin_width,
                                                                len(runs)))
    for (k0, W, ab) in runs:
        hdr += struct.pack("<III", k0, W, ab)
    rec = record_size_ab(W_a, W_b, bin_width)
    body = b""
    for m in range(nb):
        r = struct.pack("<Q", m + 7)                 # seq
        for c in list(blocks_a[m]) + list(blocks_b[m]):
            r += _bin_le(int(c.real), bb) + _bin_le(int(c.imag), bb)
        r += b"\0" * (rec - len(r))                   # pad to record boundary
        assert len(r) == rec
        body += r
    return hdr + body, ka, kb, W_a, W_b, rec


@pytest.mark.parametrize("bin_width", [16, 24])
def test_v3_roundtrip(bin_width, tmp_path):
    # Two A-runs (non-adjacent) + one B-run; W_a != W_b, one bin (2050) shared A&B.
    runs = [(300, 3, 1), (2050, 2, 3), (1000, 4, 2)]
    ka, kb = runs_to_bins(runs)
    assert ka == [300, 301, 302, 2050, 2051]         # A: run0 + the shared bin
    assert kb == [1000, 1001, 1002, 1003, 2050, 2051]  # B: run2 + the shared bin
    W_a, W_b = len(ka), len(kb)

    rng = np.random.default_rng(4)
    lim = 1 << (bin_width - 2)
    blocks_a = [rng.integers(-lim, lim, W_a) + 1j * rng.integers(-lim, lim, W_a)
                for _ in range(5)]
    blocks_b = [rng.integers(-lim, lim, W_b) + 1j * rng.integers(-lim, lim, W_b)
                for _ in range(5)]

    data, ka2, kb2, Wa2, Wb2, rec = _build_v3(runs, blocks_a, blocks_b, bin_width)
    assert (ka2, kb2, Wa2, Wb2) == (ka, kb, W_a, W_b)

    h = parse_header(data)
    assert h["version"] == 3 and h["W_a"] == W_a and h["W_b"] == W_b
    assert h["runs"] == runs and h["bin_width"] == bin_width

    body = data[h["hdr_len"]:]
    S_a, S_b = unpack_ab(body, W_a, W_b, bin_width)
    assert S_a.shape == (W_a, 5) and S_b.shape == (W_b, 5)
    for m in range(5):
        assert np.array_equal(S_a[:, m], np.asarray(blocks_a[m]))
        assert np.array_equal(S_b[:, m], np.asarray(blocks_b[m]))

    p = tmp_path / "v3.bin"
    p.write_bytes(data)
    hw = read_stream_v3(str(p))
    assert hw["ka"] == ka and hw["kb"] == kb
    assert np.array_equal(hw["S_a"], np.stack(blocks_a, axis=1))
    assert np.array_equal(hw["S_b"], np.stack(blocks_b, axis=1))
    assert list(hw["seq"]) == [m + 7 for m in range(5)]


def test_v3_single_adc_odd_pad():
    # Single-ADC odd W_a int24 -> a 2-byte pad to the 32-bit boundary.
    runs = [(500, 3, 1)]
    assert record_size_ab(3, 0, 24) == 8 + ((3 * 6 + 3) & ~3)   # 8 + 20 = 28
    blocks_a = [np.array([1 + 2j, -3 + 4j, 5 - 6j])]
    data, ka, kb, W_a, W_b, rec = _build_v3(runs, blocks_a, [[]], 24)
    assert (W_a, W_b, rec) == (3, 0, 28) and kb == []
    h = parse_header(data)
    S_a, S_b = unpack_ab(data[h["hdr_len"]:], W_a, W_b, 24)
    assert np.array_equal(S_a[:, 0], blocks_a[0]) and S_b.shape == (0, 1)
