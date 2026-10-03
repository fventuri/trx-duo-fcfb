#!/usr/bin/env python3
"""Emit OOC Verilog for WolaPrefilter (Phase-3 NB=5 BRAM spike).

Usage: emit_wola_ooc.py <out.v> [hop_R] [pages]
  hop_R : block hop (default N=4096 critical; pass 3125 for the wideband fold)
  pages : NB page-bank override (default T+1 critical / T+2 hop=R)

Real params: order_log2=12 (N=4096), T=4.
"""
import sys
import amaranth.back.verilog
from amaranth.hdl import Fragment
from maia_fcfb.wola_prefilter import WolaPrefilter


def main():
    out = sys.argv[1]
    hop = int(sys.argv[2]) if len(sys.argv) > 2 else None
    pages = int(sys.argv[3]) if len(sys.argv) > 3 else None
    dut = WolaPrefilter(order_log2=12, T=4, sample_width=16, coeff_width=18,
                        out_width=17, hop=hop, pages=pages)
    ports = [dut.clken, dut.in_valid, dut.re_in, dut.im_in,
             dut.re_out, dut.im_out, dut.out_valid, dut.out_first, dut.out_last]
    with open(out, "w") as f:
        f.write(amaranth.back.verilog.convert(dut, name="wola", ports=ports))
    print(f"emitted {out}  hop={dut.hop} NB={dut.NB} "
          f"(critical={dut.critical})")


if __name__ == "__main__":
    main()
