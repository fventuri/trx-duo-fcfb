#!/bin/sh
# fcfb post-build: build the "fcfb_stage1" FPGA project artifacts into the target,
# alongside the base board's post-build.sh (which handles uEnv.txt + led_blinker).
#
# Runs as a second BR2_ROOTFS_POST_BUILD_SCRIPT (see configs/fcfb.fragment).
# Buildroot passes the target dir in $1 and exports HOST_DIR/TARGET_DIR/etc.
set -e

BOARD_DIR="$(dirname "$0")"

# Program the PL from Linux via a device-tree overlay applied to the fpga-region
# (mainline Zynq FPGA manager). Load on the board with `start-project fcfb_stage1`.
# Two artifacts in the "project" layout (see start-project):
#
#   /lib/firmware/fcfb_stage1.bit.bin              - the .bit converted to the
#       byte-swapped .bin the mainline zynq-fpga driver requires (found via the
#       overlay's firmware-name).
#   .../projects/fcfb_stage1/fcfb_stage1.dtbo      - the overlay naming that bitstream.
#
# The PL DMA egress ring at 0x10000000..0x1a000000 is fenced off from Linux by a
# no-map reserved-memory node in the BASE device tree, added by the fcfb kernel
# patch board/patches/linux/<ver>/0002-fcfb-dt.patch (fcfb_server maps the ring
# via /dev/mem). The overlay itself only names the bitstream (see
# board/fcfb_stage1-overlay.dts).
mkdir -p "${TARGET_DIR}/lib/firmware"
mkdir -p "${TARGET_DIR}/usr/share/trx-duo/projects/fcfb_stage1"
python3 "${BOARD_DIR}/bit2bin.py" "${BOARD_DIR}/fcfb_stage1.bit" \
	"${TARGET_DIR}/lib/firmware/fcfb_stage1.bit.bin"
"${HOST_DIR}/bin/dtc" -@ -I dts -O dtb \
	-o "${TARGET_DIR}/usr/share/trx-duo/projects/fcfb_stage1/fcfb_stage1.dtbo" \
	"${BOARD_DIR}/fcfb_stage1-overlay.dts"
