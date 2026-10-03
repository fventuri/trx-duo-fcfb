# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# Mirrors the trx-duo TrxduoPlatform (© Daniel Estevez / MaiaSDR port, MIT).
# CLEAN-ROOM: no ka9q-radio / npapi source is used anywhere in fcfb.
"""Amaranth platform for the fcfb Stage-1 IP core (TRX-duo / Red Pitaya 125-16).

Intentionally minimal, like upstream ``PlutoPlatform`` / ``TrxduoPlatform``:
fcfb Stage-1 is built as a packaged IP core dropped into a Vivado block design,
so pin constraints and clock generation live in the block design (`.tcl`/`.xdc`),
not here.  The part is the Zynq ``xc7z010clg400-1``.

Clocking (block-design responsibility; the single-125 plan, confirmed 2026-08-15):
fcfb packs BOTH ADCs as ``z = adc0 + j*adc1`` at the full 125 Msps complex, so the
FFT datapath runs one sample per sample-clock and ``cmult3x`` (3x = 375 MHz) is
infeasible.  The FFT therefore uses ``cmult3x=False`` / ``window=None`` (the WOLA
supplies the analysis window), and the whole datapath is a SINGLE 125 MHz domain
-- there are no ``clk2x`` / ``clk3x`` domains.

    MMCM from the LTC2208 DCO (125 MHz):
      sampling (1x) = 125 MHz   (ADC IOB capture)
      sync     (1x) = 125 MHz   (WOLA + FFT + A/B split + backend + DMA)
    s_axi_lite = PS FCLK0 (~100 MHz)

The top level only *declares* these domains and exposes their clock pins as IP
ports (see ``stage1_top.py``); the block design wires the MMCM outputs to them.
"""

from amaranth.vendor import XilinxPlatform


class FcfbPlatform(XilinxPlatform):
    device = "xc7z010"
    package = "clg400"
    speed = "1"
    resources = []
    connectors = []
