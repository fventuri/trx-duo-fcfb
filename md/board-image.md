# The board image

The board runs a small Buildroot Linux that programs the FPGA with the fcfb Stage-1
bitstream and starts the board-side server at boot. It is built as a **second
`BR2_EXTERNAL` tree stacked on top of the base
[trx-duo-buildroot](https://github.com/fventuri/trx-duo-buildroot)**, so the base board
support stays pristine and fcfb adds only its own overlay, bitstream, and a DT patch.

## Prebuilt image

The [latest release](https://github.com/fventuri/trx-duo-fcfb/releases/latest) ships a
ready-to-write `sdcard.img` (verify against `SHA256SUMS.txt`). Write it to a micro-SD
card:

```sh
sudo dd if=sdcard.img of=/dev/sdX bs=4M conv=fsync status=progress
```

Boot the board, give it a network address, and it is ready for a host application.

## Building it

Clone both repositories next to an unpacked Buildroot 2026.08, then run the wrapper:

```sh
git clone https://github.com/fventuri/trx-duo-buildroot.git
git clone https://github.com/fventuri/trx-duo-fcfb.git

cd trx-duo-fcfb/buildroot
./build-fcfb.sh            # stacks base:fcfb, builds output/images/sdcard.img
```

The wrapper loads the base `trx-duo_defconfig`, merges the fcfb fragment on top
(overlay, post-build script, DT patch), and builds the image. Override `BR`,
`BASE_EXT`, and `BR2_DL_DIR` via the environment to match your layout.

## What fcfb adds to the base

- the fcfb Stage-1 bitstream (`fcfb_stage1.bit`) and its device-tree overlay;
- a rootfs overlay that autostarts the FPGA gateware and the `fcfb_server` at boot,
  sets a jumbo MTU on `eth0`, and loads the `u-dma-buf` driver for the zero-copy ring;
- a small device-tree patch reserving the PL DMA ring in DDR.

The board-side server itself (`fcfb_server`) is the C program in
[`host/`](https://github.com/fventuri/trx-duo-fcfb/tree/main/host); the shipped image
carries a prebuilt ARM build of it.
