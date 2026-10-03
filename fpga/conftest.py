# SPDX-License-Identifier: MIT
"""Test path wiring for the fcfb fpga/ tree.

Puts this directory (so ``maia_fcfb`` / ``models`` imports work), the read-only
upstream maia-hdl (for ``maia_hdl.*``), and the maia-sdr-trx-duo hdl tree (for
``maia_trxduo.*``, when a block reuses one) on sys.path. Locations can be
overridden with the MAIA_HDL / MAIA_TRXDUO environment variables.
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_MAIA_HDL = os.environ.get(
    "MAIA_HDL", os.path.expanduser("~/maia-hdl"))
_MAIA_TRXDUO = os.environ.get(
    "MAIA_TRXDUO", os.path.expanduser("~/maia-sdr-trx-duo/hdl"))

for p in (_HERE, _MAIA_HDL, _MAIA_TRXDUO):
    if p not in sys.path:
        sys.path.insert(0, p)
