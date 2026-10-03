#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
#
# Adapted from the trx-duo MaiaSDR Vivado flow (© Daniel Estevez / MaiaSDR port,
# MIT).  Build the fcfb Stage-1 Vivado project: generate the IP-core Verilog from
# Amaranth, then run Vivado to package the IP, build the block design, and produce
# a bitstream.
#
# Usage:  ./build.sh [all|package|bd|bit]     (default: all)

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FPGA_DIR="$(dirname "$HERE")"
MODE="${1:-all}"

VIVADO="${VIVADO:-/opt/Xilinx/2026.1/Vivado/bin/vivado}"
VENV="${VENV:-$HOME/maia-sdr-trx-duo/hdl/.venv}"
MAIA_HDL="${MAIA_HDL:-$HOME/maia-hdl}"
MAIA_TRXDUO="${MAIA_TRXDUO:-$HOME/maia-sdr-trx-duo/hdl}"

# 1. Generate the IP-core Verilog + SVD from Amaranth.  fcfb reuses the trx-duo
#    AdcCapture / IqCDC (maia_trxduo) and maia-hdl, so both are on PYTHONPATH.
mkdir -p "$HERE/build"
(
    # shellcheck disable=SC1091
    source "$VENV/bin/activate"
    cd "$FPGA_DIR"
    # Wideband (Phase-3b): set FCFB_HOP=3125 to generate the two-rate build (fast
    # fft domain + CDC FIFOs).  Unset -> the critical single-125 MHz shipping IP.
    HOP_ARG=""
    if [ -n "${FCFB_HOP:-}" ]; then HOP_ARG="--hop ${FCFB_HOP}"; fi
    # Set FCFB_NO_DDS=1 for a PRODUCTION build (drops the DDS tone-injection
    # self-test); unset -> a verification build that includes it.
    DDS_ARG=""
    if [ -n "${FCFB_NO_DDS:-}" ]; then DDS_ARG="--no-dds"; fi
    # Set FCFB_N_DDS=2 for a two-tone / two-window verification build (two DDS
    # generators summed); unset -> 1 (single tone).  Ignored if FCFB_NO_DDS set.
    NDDS_ARG=""
    if [ -n "${FCFB_N_DDS:-}" ]; then NDDS_ARG="--n-dds ${FCFB_N_DDS}"; fi
    # Set FCFB_BIN_WIDTH=24 for the Option-B int24 build (streams the full 23-bit
    # internal bin -> ~-102 dBc reconstructed-channel ceiling); unset/16 -> the
    # shipping int16 wire format.  Internal FFT datapath is unchanged either way.
    BINW_ARG=""
    if [ -n "${FCFB_BIN_WIDTH:-}" ]; then BINW_ARG="--bin-width ${FCFB_BIN_WIDTH}"; fi
    # Set FCFB_WMAX to shrink the StreamFormat buffer depth (default 512).  A
    # smaller wmax shortens the int24 bit-packer ld_idx loop (aw=ceil(log2(wmax)))
    # and eases the marginal 166.67 MHz fft clock -- use 128 for narrow-W
    # verification builds (e.g. n_dds=2 two-tone). Keep the server FCFB_WMAX guard
    # in sync so admitted W never exceeds the built ceiling.
    WMAX_ARG=""
    if [ -n "${FCFB_WMAX:-}" ]; then WMAX_ARG="--wmax ${FCFB_WMAX}"; fi
    # Provenance stamp baked into the read-only params bank (build_id_lo/hi). Use
    # FCFB_BUILD_ID if set, else auto-derive the current git short hash so every
    # build self-identifies (the host params query reports it).
    BUILDID="${FCFB_BUILD_ID:-}"
    if [ -z "$BUILDID" ]; then
        BUILDID="0x$(git -C "$FPGA_DIR" rev-parse --short=8 HEAD 2>/dev/null || echo 0)"
    fi
    PYTHONPATH="$FPGA_DIR:$MAIA_HDL:$MAIA_TRXDUO" \
        python -m maia_fcfb.stage1_top \
        "$HERE/build/stage1_top.v" --svd "$HERE/build/stage1_top.svd" \
        $HOP_ARG $DDS_ARG $NDDS_ARG $BINW_ARG $WMAX_ARG --build-id "$BUILDID"
)
echo "generated $HERE/build/stage1_top.v${FCFB_HOP:+ (wideband hop=$FCFB_HOP)}${FCFB_NO_DDS:+ (no DDS)}${FCFB_N_DDS:+ (n_dds=$FCFB_N_DDS)}${FCFB_BIN_WIDTH:+ (int${FCFB_BIN_WIDTH})}${FCFB_WMAX:+ (wmax=$FCFB_WMAX)}"

# 2. Run Vivado (from this dir so relative cfg/tmp paths resolve).
cd "$HERE"
"$VIVADO" -nolog -nojournal -mode batch -source build.tcl -tclargs "$MODE"

# 3. Byte-swap the raw .bit.bin (32-bit words) for THIS board's fpga-region.
#    Vivado's write_bitstream -bin_file emits big-endian config words; the
#    mainline Zynq fpga-manager here expects them 32-bit byte-swapped (the
#    known-good bitstream is swapped) -- the un-swapped bin fails to program
#    with "write init error" (HW-verified 2026-08-16). No-op for modes that
#    don't emit the bin (package/bd).
BIN="$HERE/build/fcfb_stage1.bit.bin"
if [ -f "$BIN" ]; then
    python3 - "$BIN" <<'PY'
import sys
p = sys.argv[1]
d = open(p, 'rb').read()
assert len(d) % 4 == 0, f'{p}: length {len(d)} not a multiple of 4'
open(p, 'wb').write(b''.join(d[i:i+4][::-1] for i in range(0, len(d), 4)))
print(f'byte-swapped (32-bit) {p} for the board fpga-region')
PY
fi
