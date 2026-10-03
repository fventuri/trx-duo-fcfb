#!/usr/bin/env python3
"""Emit OOC Verilog for ABSplit (share_write_port BRAM-dedup spike).

Usage: emit_ab_ooc.py <out.v> [share] [extra_pipe]
  share      : 1 => share_write_port (single-copy), 0 => replicated (default 0)
  extra_pipe : 1 => extra_pipe (wideband), 0 => off (default 1)

Real params: width_out=23, order_log2=12 (N=4096).
"""
import sys
import amaranth.back.verilog
from maia_fcfb.ab_split import ABSplit


def main():
    out = sys.argv[1]
    share = bool(int(sys.argv[2])) if len(sys.argv) > 2 else False
    extra = bool(int(sys.argv[3])) if len(sys.argv) > 3 else True
    dut = ABSplit(23, 12, extra_pipe=extra, share_write_port=share)
    ports = [dut.clken, dut.re_in, dut.im_in, dut.input_last,
             dut.a_re, dut.a_im, dut.b_re, dut.b_im,
             dut.out_valid, dut.out_last]
    with open(out, "w") as f:
        f.write(amaranth.back.verilog.convert(dut, name="ab", ports=ports))
    print(f"emitted {out}  share={share} extra_pipe={extra}")


if __name__ == "__main__":
    main()
