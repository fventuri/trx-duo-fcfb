#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""
Convert a Xilinx .bit bitstream to the byte-swapped .bin the mainline Zynq
FPGA manager (drivers/fpga/zynq-fpga.c) expects.

The mainline driver rejects the .bit outright ("Bitstream must be a byte
swapped .bin file") and scans for the sync word 66 55 99 AA, i.e. the .bit's
big-endian AA 99 55 66 with every 32-bit word byte-swapped. So we:
  1. strip the .bit ASCII header (walk its tagged fields to the 'e' field), and
  2. byte-swap every 32-bit word of the raw config data.

Unlike Xilinx bootgen this needs no Vivado SDK. Run at build time from
post-build.sh:  bit2bin.py led_blinker.bit /lib/firmware/led_blinker.bit.bin
"""
import sys
import struct

src, dst = sys.argv[1], sys.argv[2]
data = open(src, 'rb').read()

# .bit header: initial length-prefixed magic field, then a 2-byte 0x0001, then
# tagged fields a/b/c/d (2-byte length each) and finally 'e' + 4-byte length +
# raw bitstream. Walk it rather than guessing offsets.
i = 0
l = struct.unpack('>H', data[i:i + 2])[0]; i += 2 + l   # magic field
l = struct.unpack('>H', data[i:i + 2])[0]; i += 2       # 0x0001 marker
raw = None
while i < len(data):
    tag = data[i:i + 1]; i += 1
    if tag == b'e':
        n = struct.unpack('>I', data[i:i + 4])[0]; i += 4
        raw = data[i:i + n]
        break
    n = struct.unpack('>H', data[i:i + 2])[0]; i += 2
    i += n
assert raw is not None, "no 'e' field in .bit header"
assert len(raw) % 4 == 0, "raw data not 32-bit aligned: %d" % len(raw)

out = bytearray(len(raw))
for k in range(0, len(raw), 4):
    out[k], out[k + 1], out[k + 2], out[k + 3] = raw[k + 3], raw[k + 2], raw[k + 1], raw[k]
open(dst, 'wb').write(out)

# Sanity: the driver's sync word must be present.
if out[:64].find(b'\x66\x55\x99\xaa') < 0:
    sys.exit("ERROR: sync word 66 55 99 AA not found in output; not a valid .bin")
print("bit2bin: wrote %s (%d bytes), sync word OK" % (dst, len(out)))
