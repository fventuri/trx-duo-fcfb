# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Top-level smoke test: the fcfb Stage1Top IP core elaborates and converts to
Verilog on the fcfb platform, and its SVD generates.

The top is mostly structural / Instance-based (the FIFO18E1 in the CDC and the
AXI3 master in the DMA), so it is not exercised by the Amaranth simulator here;
this verifies the whole thing wires up and produces Verilog (the real check
happens in Vivado), while its datapath blocks are simulated individually
elsewhere (the 33-test Stage-1 suite + test_registers).
"""
import amaranth.back.verilog

from maia_fcfb.stage1_top import Stage1Top
from maia_fcfb.fcfb_platform import FcfbPlatform


def _small_top():
    # Keep the FFT small (N=64, still a valid T=4 WOLA) so conversion is fast.
    return Stage1Top(order_log2=6)


def test_top_converts_to_verilog():
    top = _small_top()
    verilog = amaranth.back.verilog.convert(
        top, platform=FcfbPlatform(), ports=top.ports())
    assert 'module' in verilog
    # The write-gated CDC must instantiate the Xilinx FIFO primitive.
    assert 'FIFO18E1' in verilog


def test_top_svd():
    svd = _small_top().svd()
    assert b'product_id' in svd
    assert b'mask_load' in svd
    assert b'fcfb Stage-1' in svd
    # DDS input-scene injection register (step 6e).
    assert b'dds' in svd and b'phase_inc' in svd
    # Soft datapath reset (dp_reset) control bit.
    assert b'dp_reset' in svd
    # Read-only params bank (item 1: kernel-match handshake).
    assert b'param_magic' in svd and b'r_hop' in svd and b'param_ver' in svd


def test_two_tone_top_converts():
    """A two-DDS (n_dds=2) build converts and maps the dds2 register (for the
    two-tone / two-window input scene); wideband too."""
    for kw in (dict(order_log2=6), dict(order_log2=6, hop=48, T=4)):
        top = Stage1Top(n_dds=2, bin_width=24, **kw)
        verilog = amaranth.back.verilog.convert(
            top, platform=FcfbPlatform(), ports=top.ports())
        assert 'module' in verilog
        svd = top.svd()
        assert b'dds2' in svd and b'mask_load' in svd


def test_wideband_top_converts_with_dp_reset():
    """The wideband two-rate top (with the dp_reset fft-domain reset path) also
    elaborates + converts, and the dp_reset net reaches the Verilog."""
    top = Stage1Top(order_log2=6, hop=48)     # small N=64, hop<N -> two-rate
    verilog = amaranth.back.verilog.convert(
        top, platform=FcfbPlatform(), ports=top.ports())
    assert 'module' in verilog
    assert 'dp_reset' in verilog


def test_production_top_drops_dds_selftest():
    """A production (n_dds=0) build converts, DROPS the DDS tone-injection
    self-test (no cos/sin ROM in the datapath), but KEEPS the `dds` register in
    the map so the AXI register map + the server binary are unchanged."""
    for kw in (dict(order_log2=6), dict(order_log2=6, hop=48)):   # critical + WB
        top = Stage1Top(n_dds=0, **kw)
        verilog = amaranth.back.verilog.convert(
            top, platform=FcfbPlatform(), ports=top.ports())
        assert 'module' in verilog
        assert 'cos_rom' not in verilog and 'sin_rom' not in verilog
        # the register map (SVD) still exposes the dds register
        assert b'dds' in top.svd() and b'phase_inc' in top.svd()
