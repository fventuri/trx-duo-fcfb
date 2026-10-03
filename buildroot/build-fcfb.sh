#!/bin/sh
# build-fcfb.sh - build the TRX-duo fcfb Stage-1 channelizer firmware image.
#
# fcfb is layered on the stock TRX-duo board support as a *second* Buildroot
# BR2_EXTERNAL tree (br2-external-fcfb/, next to this script), so the base
# trx-duo-buildroot tree stays pristine on its main branch. This wrapper:
#   1. stacks the two externals (base : fcfb),
#   2. loads the base trx-duo_defconfig (inherits the kernel version, rootfs
#      size, etc.),
#   3. merges the fcfb defconfig fragment on top (overlay, post-build script,
#      DT patch dir), and
#   4. runs the requested make target (default: a full build).
#
# Prerequisites (clone next to an unpacked Buildroot 2026.08):
#   git clone https://github.com/fventuri/trx-duo-buildroot.git   # the base external
#
# Overridable via the environment:
#   BR         Buildroot source/output dir   (default below)
#   BASE_EXT   trx-duo-buildroot checkout (base external, main branch)
#   BR2_DL_DIR Buildroot download cache
#
# Usage:  ./build-fcfb.sh [make-target ...]      (e.g. `./build-fcfb.sh`,
#         `./build-fcfb.sh linux-dirclean`, `./build-fcfb.sh menuconfig`)
set -eu

BR="${BR:-$HOME/buildroot-2026.08}"
BASE_EXT="${BASE_EXT:-$HOME/trx-duo-buildroot}"
BR2_DL_DIR="${BR2_DL_DIR:-$HOME/buildroot-dl}"
FCFB_EXT="$(cd "$(dirname "$0")/br2-external-fcfb" && pwd)"

export BR2_DL_DIR
EXT="${BASE_EXT}:${FCFB_EXT}"
FRAG="${FCFB_EXT}/configs/fcfb.fragment"

echo ">> Buildroot : $BR"
echo ">> externals : $EXT"

cd "$BR"

# (Re)configure only when needed: reload the base defconfig, then merge the fcfb
# fragment. Appending the fragment and running olddefconfig lets the fragment's
# values win over the base's (kconfig takes the last assignment), which is how
# the list-valued options (overlay / post-build / patch-dir) end up carrying both
# the base and the fcfb paths.
make BR2_EXTERNAL="$EXT" trx-duo_defconfig
cat "$FRAG" >> .config
make BR2_EXTERNAL="$EXT" olddefconfig

# Build (default target is a full image build).
make BR2_EXTERNAL="$EXT" "$@"
