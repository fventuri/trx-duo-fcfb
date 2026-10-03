#!/usr/bin/env python3
# M3 tier-2: the RTL-faithful fixed-point Stage-1 (T=4) reproduces the golden
# wire stream. Runs the COMPOSED fixed-point chain at the real N=4096 on the
# emit_stream.py scene, using the block models that are each proven bit-exact to
# their Amaranth RTL (test_wola_prefilter / test_ab_split / test_quantise /
# test_stage1), then feeds the shipped int16 bins through the SAME Stage-2 the
# host uses and compares the reconstructed channel to sim/ref_channel.f64.
#
# vs fixed_point_sim.py (which used IDEAL coeffs + max/32000 quantise), this adds
# the two RTL realities: QUANTIZED WOLA coefficients (cw bits) and the 2^shift
# int16 quantise. It answers the design's tier-2: "dequantised bins vs the golden,
# allow int16 LSB diffs, expect >> the M1 16-bit floor."
#
#   . ~/maia-sdr-trx-duo/hdl/.venv/bin/activate
#   PYTHONPATH=$MAIA_HDL python fpga/models/tier2_stream.py

import os
import struct
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_MAIA_HDL = os.environ.get(
    "MAIA_HDL", os.path.expanduser("~/maia-hdl"))
for p in (_MAIA_HDL, os.path.join(_HERE, "..")):
    if p not in sys.path:
        sys.path.insert(0, os.path.abspath(p))
_SIM = os.path.join(_HERE, "..", "..", "sim")
sys.path.insert(0, os.path.abspath(_SIM))

import chanfcfb_model as M                        # noqa: E402
import emit_stream as E                           # noqa: E402
from maia_hdl.fft import FFT                       # noqa: E402
from maia_hdl.util import bit_invert               # noqa: E402
from maia_fcfb.wola_prefilter import WolaPrefilter  # noqa: E402

N, ORDER = M.N, 12
KB, K0, W = E.KB, E.K0, E.W                        # 700, 697, 7
T = E.T                                            # 4
CW = 18                                            # WOLA coeff width
ADC_AMP = 32767                                    # scale the amp-1.0 tone to FS


def q_shift(x, shift):
    """RTL int16 quantise: round-half-up >> shift, saturate."""
    x = np.rint(x).astype(np.int64)
    y = (x + (1 << (shift - 1))) >> shift if shift >= 1 else x
    return np.clip(y, -32768, 32767).astype(np.int64)


def read_stream_bins(path):
    """Parse sim/stream.bin -> (bin_scale, S[W, nblk] dequantised complex)."""
    with open(path, "rb") as f:
        h = f.read(48)
        assert h[:8] == b"FCFBv1\0\0"
        _, n, fs, k0, w, mask, nblocks, bscale = struct.unpack("<IIdIIIId", h[8:])
        S = np.empty((w, nblocks), complex)
        for m in range(nblocks):
            (seq,) = struct.unpack("<Q", f.read(8))
            iq = np.frombuffer(f.read(4 * w), np.int16).astype(np.float64)
            S[:, m] = iq[0::2] + 1j * iq[1::2]
    return bscale, S * bscale


def _resid_db(x, ref):
    """Best-fit-complex-gain-aligned residual power (dB) of x vs ref."""
    gain = np.vdot(ref, x) / np.vdot(ref, ref)
    resid = x - gain * ref
    return 10 * np.log10(np.mean(np.abs(resid) ** 2) /
                         max(np.mean(np.abs(gain * ref) ** 2), 1e-30))


def run(nblk_use=320, coeff_width=CW):
    """Composed RTL-faithful fixed-point Stage-1 (T=4) on the emit scene.

    Returns metrics dict: sinad_db, bin_resid_db (vs stream.bin), ch_resid_db
    (vs ref_channel.f64), shift, sat, wola_peak, blocks.
    """
    # scene: emit_stream's ADC0-only tone, scaled to 16-bit full scale
    a = M.tones([E.F_TONE], [1.0])
    z = np.round(ADC_AMP * a).astype(np.int64) + 0j
    z = z[: (nblk_use + T) * N]

    # WOLA fold (quantized peak-normalized coeffs)
    wola = WolaPrefilter(ORDER, T=T, sample_width=16, coeff_width=coeff_width,
                         out_width=17)
    bufs = wola.model(z)
    nblk = bufs.shape[0]
    wola_peak = float(max(np.abs(bufs.real).max(), np.abs(bufs.imag).max()))

    # fixed-point FFT (batched) + exact A/B split
    fft = FFT(17, ORDER, "R22", width_twiddle=16,
              truncates=[[0, 1]] * (ORDER // 2), window=None)
    ro, io = fft.model(bufs.real.astype(np.int64).ravel(),
                       bufs.imag.astype(np.int64).ravel())
    Yrev = (np.asarray(ro, np.int64) + 1j * np.asarray(io, np.int64)
            ).reshape(nblk, N).T
    inv = np.array([bit_invert(k, ORDER, 1) for k in range(N)])
    Y = Yrev[inv, :]
    mir = (N - np.arange(N)) % N
    Cr, Ci = Y.real.astype(np.int64), Y.imag.astype(np.int64)
    A = ((Cr + Cr[mir]) >> 1) + 1j * ((Ci - Ci[mir]) >> 1)
    S_run = A[K0:K0 + W, :]

    # int16 quantise (2^shift), pick shift so max maps to ~+/-32000
    mx = float(np.abs(S_run).max())
    shift = max(0, int(np.ceil(np.log2(mx / 32000.0))))
    Qi = q_shift(S_run.real, shift)
    Qq = q_shift(S_run.imag, shift)
    sat = int(np.sum((np.abs(Qi) >= 32767) | (np.abs(Qq) >= 32767)))
    S_deq = (Qi.astype(np.float64) + 1j * Qq.astype(np.float64)) * (1 << shift)

    # Stage-2 reconstruct + SINAD (emit parks the tone at +1000 Hz)
    ch = E.stage2(S_deq, K0, E.F_CENTER)
    fs_ch = W * M.BINW / E.DECIM
    f_off = E.F_TONE - E.F_CENTER
    c = ch[ch.size // 8: -ch.size // 8]
    e = np.exp(2j * np.pi * f_off / fs_ch * np.arange(c.size))
    amp = np.vdot(e, c) / c.size
    sinad = float(10 * np.log10(abs(amp) ** 2 /
                  max(np.mean(np.abs(c - amp * e) ** 2), 1e-30)))

    out = dict(sinad_db=sinad, shift=shift, sat=sat, wola_peak=wola_peak,
               blocks=nblk, bin_resid_db=None, ch_resid_db=None)

    stream_path = os.path.join(_SIM, "stream.bin")
    if os.path.exists(stream_path):
        _, S_gold = read_stream_bins(stream_path)
        k = min(S_gold.shape[1], S_deq.shape[1])
        out["bin_resid_db"] = float(
            _resid_db(S_deq[:, :k].ravel(), S_gold[:, :k].ravel()))

    ref_path = os.path.join(_SIM, "ref_channel.f64")
    if os.path.exists(ref_path):
        ref = np.fromfile(ref_path, np.complex128)
        k = min(ref.size, ch.size)
        s = slice(k // 8, -k // 8)
        out["ch_resid_db"] = float(_resid_db(ch[:k][s], ref[:k][s]))
    return out


def main():
    nblk_use = int(os.environ.get("NBLK", "320"))
    print("=" * 76)
    print(" M3 tier-2  --  RTL-faithful fixed-point Stage-1 (T=4) vs golden wire")
    print("=" * 76)
    print(f" N={N} order={ORDER} T={T} run k0={K0} W={W} coeff_width={CW}")
    r = run(nblk_use)
    print(f" WOLA out peak = {r['wola_peak']:.0f} (17-bit lim 65535)   "
          f"blocks={r['blocks']}")
    print(f" shift={r['shift']}  int16 sat count={r['sat']}")
    print(f"\n reconstructed wanted-tone SINAD (T=4, fixed point) = "
          f"{r['sinad_db']:6.1f} dB")
    if r["bin_resid_db"] is not None:
        print(f" shipped bins vs stream.bin (gain-aligned): residual "
              f"{r['bin_resid_db']:6.1f} dB")
    if r["ch_resid_db"] is not None:
        print(f" channel vs ref_channel.f64 (gain-aligned): residual "
              f"{r['ch_resid_db']:6.1f} dB")
    print("\n" + "-" * 76)
    print(" Expect: SINAD >> M1 T=1 (~45 dB); residuals well below 0 dB (int16 +")
    print(" coeff-quant, above the M1 -294 dB float floor but far under the")
    print(" shipped-bin dynamic range).")
    print("=" * 76)


if __name__ == "__main__":
    main()
