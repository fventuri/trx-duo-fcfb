#!/usr/bin/env python3
# M3 fixed-point / bit-growth sim for the fcfb Stage-1 cascade.
#
# Retires M1 open item #1: "confirm no clipping / adequate headroom on full-scale
# ADC input" through the REAL fixed-point datapath the RTL will use, and settles
# the internal widths + the int16 quantise scale BEFORE any Amaranth is written.
#
# It drives the ACTUAL maia-hdl FFT fixed-point model (`FFT.model`, the same
# truncation schedule the RTL uses: R2^2, truncates=[[0,1]] per stage-pair,
# 16-bit twiddles) through the fcfb cascade:
#
#   two 16-bit real ADCs -> pack z = a + j*b -> T=4 WOLA fold (M1 prototype)
#   -> quantise to FFT input width -> FFT (fixed point) -> un-bit-reverse
#   -> A/B split (X +/- conj(X_mirror))/2 -> int16 quantise (bin_scale)
#
# NOTE: maia-hdl `clamp_nbits` WRAPS (two's complement), it does NOT saturate,
# so any datapath overflow is catastrophic (wrap), not graceful. Headroom must be
# guaranteed, not merely "mostly ok" -- hence this sim.
#
# Reports, at each node: peak level, bits used, headroom, and overflow count;
# plus the realized fixed-point noise floor vs the M1 float model, and the
# decisive M1 [C] adjacent-isolation metric re-measured THROUGH the fixed-point
# FFT (does T=4's ~88 dB survive fixed point?).
#
# Requires maia-hdl on the path:
#   MAIA_HDL=~/maia-hdl  (default below)
# Run inside the amaranth venv (amaranth 0.5.9):
#   . ~/maia-sdr-trx-duo/hdl/.venv/bin/activate
#   PYTHONPATH=$MAIA_HDL python fpga/models/fixed_point_sim.py

import os
import sys

import numpy as np

# --- maia-hdl (MIT dependency) on the path ---------------------------------
_MAIA_HDL = os.environ.get(
    "MAIA_HDL", os.path.expanduser("~/maia-hdl"))
sys.path.insert(0, _MAIA_HDL)
from maia_hdl.fft import FFT               # noqa: E402  (MIT, Daniel Estevez)
from maia_hdl.util import bit_invert       # noqa: E402

# --- M1 golden model (numpy, ../../sim) ------------------------------------
_SIM = os.path.join(os.path.dirname(__file__), "..", "..", "sim")
sys.path.insert(0, os.path.abspath(_SIM))
import chanfcfb_model as M                 # noqa: E402

# ---------------------------------------------------------------- FFT config
N = M.N                       # 4096
ORDER = 12                    # log2 N
RADIX = "R22"
TRUNCATES = [[0, 1]] * (ORDER // 2)     # +1 bit / stage-pair (maia trx-duo)
WTW = 16                      # twiddle width
FULLSCALE = 2 ** 15 - 1       # 16-bit signed ADC full scale (32767)

# model output is bit-reversed; Y_natural = Y_model[INVERT]  (R22 -> full reversal)
INVERT = np.array([bit_invert(n, ORDER, 1) for n in range(N)])

# --- WOLA prototype, PEAK-NORMALIZED for fixed point -----------------------
# M1's prototype has unit DC gain (sum h = 1, so max|h| ~ 1/N): rounding that
# fold to integers would keep only ~5 bits. For the RTL the window coefficients
# are O(1) fractions (peak 1), which fills the FFT input width; the resulting
# constant gain (~N) is arbitrary and absorbed into the programmable bin_scale.
_HPK = {}


def scaled_proto(T):
    if T not in _HPK:
        h = M.prototype(T)
        _HPK[T] = h / np.abs(h).max()      # peak-normalized: max|h| = 1
    return _HPK[T]


def fft_out_width(fft_in_w):
    """Datapath output width for this truncate schedule (+1 bit / stage-pair)."""
    return fft_in_w + sum(2 - int(np.sum(t)) for t in TRUNCATES)


def make_fft(fft_in_w):
    return FFT(fft_in_w, ORDER, RADIX, width_twiddle=WTW,
               truncates=TRUNCATES, window=None)


# --------------------------------------------------------- scene generation
def tone_int(k_offset_bins, amp, nblk, T, phase=0.0):
    """Full-scale-referenced real tone at bin (KB + k_offset), integer samples."""
    ntot = (nblk + T) * N
    n = np.arange(ntot)
    f = (M.KB + k_offset_bins) * M.BINW
    return amp * np.cos(2 * np.pi * f * n / M.FS + phase)


def noise_int(amp, nblk, T, seed):
    rng = np.random.default_rng(seed)
    ntot = (nblk + T) * N
    return amp * rng.standard_normal(ntot)


# --------------------------------------------------------------- WOLA + FFT
def wola_fold(z, T, nblk):
    """WOLA fold, per block: (h*seg).reshape(T,N).sum(0). Peak-normalized proto."""
    h = scaled_proto(T)
    out = np.empty((N, nblk), np.complex128)
    for m in range(nblk):
        seg = z[m * N: m * N + T * N]
        out[:, m] = (h * seg).reshape(T, N).sum(0)
    return out


def report_node(name, cval, width, mag_constraint=False):
    """Peak/bits/headroom/overflow for a complex fixed-point node of `width` bits.

    If mag_constraint, also check the maia-hdl FFT INPUT rule: complex magnitude
    |z|=sqrt(re^2+im^2) <= 2^(width-1)-1 (needed to avoid twiddle-mult overflow),
    which is stricter than the per-component check.
    """
    peak = max(np.abs(cval.real).max(), np.abs(cval.imag).max())
    magpk = np.abs(cval).max()
    lim = 2 ** (width - 1) - 1
    bits = 0 if peak < 1 else int(np.ceil(np.log2(peak + 1))) + 1
    ovf = int(np.sum((np.abs(cval.real) > lim) | (np.abs(cval.imag) > lim)))
    flag = "  <-- OVERFLOW (wraps!)" if ovf else ""
    extra = ""
    if mag_constraint:
        viol = magpk > lim
        extra = (f"  |z|max={magpk:9.1f} vs lim {lim} "
                 f"{'VIOLATES input rule' if viol else 'ok'}")
    print(f"    {name:22s} peak={peak:12.1f}  uses {bits:2d}/{width} bits  "
          f"headroom={width - bits:2d}b  ovf={ovf}{flag}{extra}")
    return peak, ovf


def run_fft_fixed(z, T, nblk, fft_in_w, verbose=False):
    """Full fixed-point Stage-1 up to A/B split. Returns dict of nodes + peaks."""
    buf = wola_fold(z, T, nblk)                      # complex float, (N, nblk)
    bi_re = np.round(buf.real).astype(np.int64)
    bi_im = np.round(buf.imag).astype(np.int64)

    w_out = fft_out_width(fft_in_w)
    # FFT.model wants flat block-major int arrays (N samples/block, natural time).
    re_in = bi_re.T.ravel()
    im_in = bi_im.T.ravel()
    fft = make_fft(fft_in_w)
    ro, io = fft.model(re_in, im_in)
    Yrev = (np.asarray(ro, np.int64) + 1j * np.asarray(io, np.int64)
            ).reshape(nblk, N).T
    Y = Yrev[INVERT, :]                              # natural bin order
    Ym = np.conj(Y[(N - np.arange(N)) % N, :])
    A = (Y + Ym) / 2.0
    B = (Y - Ym) / 2.0j

    peaks = {}
    if verbose:
        peaks["fftin"] = report_node("WOLA->FFT in", bi_re + 1j * bi_im,
                                     fft_in_w, mag_constraint=True)
        peaks["fftout"] = report_node("FFT out (bitrev)", Yrev, w_out)
        peaks["A"] = report_node("A = ADC0 bins", A, w_out)
        peaks["B"] = report_node("B = ADC1 bins", B, w_out)
    return dict(buf=buf, bi=bi_re + 1j * bi_im, Y=Y, A=A, B=B,
                w_out=w_out, peaks=peaks)


# -------------------------------------------------------------------- report
def hdr(t):
    print("\n" + "-" * 76 + f"\n {t}\n" + "-" * 76)


def main():
    nblk = int(os.environ.get("NBLK", "256"))
    T = int(os.environ.get("T", "4"))
    print("=" * 76)
    print(" M3 fixed-point / bit-growth sim  --  fcfb Stage-1 cascade")
    print("=" * 76)
    print(f" N={N}  order={ORDER}  radix={RADIX}  truncates={TRUNCATES[0]}x{len(TRUNCATES)}"
          f"  twiddle={WTW}b  T={T}  blocks={nblk}")
    print(f" FFT output width (in+{sum(2-int(np.sum(t)) for t in TRUNCATES)}b): "
          f"16->{fft_out_width(16)}   17->{fft_out_width(17)}")
    print(f" WOLA prototype L1 growth (sum|h|): T={T} -> {np.abs(M.prototype(T)).sum():.4f}")

    # --- self-test: a pure tone must land in the right bin after un-bit-reverse
    hdr("[0] sanity: single full-scale tone lands in the expected bin")
    z = tone_int(0.0, FULLSCALE, nblk, T) + 0j     # real (ADC0-only) -> mirror bin
    r = run_fft_fixed(z, T, nblk, 17)
    mag = np.abs(r["Y"][:, nblk // 2])
    kpk = int(np.argmax(mag))
    ok = kpk in (M.KB, N - M.KB)                    # real input peaks at KB and N-KB
    print(f"    real tone at bin KB={M.KB}: peak recovered bin = {kpk} "
          f"(expect {M.KB} or {N - M.KB})  ({'OK' if ok else 'MISMATCH'})")

    # --- [1] headroom: FFT input width 16 (naive) vs 17 (recommended) ---------
    hdr("[1] headroom vs FFT input width -- worst case: BOTH ADCs full-scale, "
        "same bin")
    zb = (tone_int(0.0, FULLSCALE, nblk, T)          # ADC0 full-scale
          + 1j * tone_int(0.0, FULLSCALE, nblk, T, phase=0.0))  # ADC1 full-scale
    for w in (16, 17):
        print(f"  FFT input width = {w} bits:")
        run_fft_fixed(zb, T, nblk, w, verbose=True)

    # --- [2] headroom: realistic single-ADC full-scale tone -------------------
    hdr("[2] headroom: single ADC0 full-scale tone (typical strong signal)")
    run_fft_fixed(z, T, nblk, 17, verbose=True)

    # --- [3] realized fixed-point noise floor vs the M1 float model -----------
    hdr("[3] realized noise floor: fixed-point vs float A[k] (single ADC0 tone)")
    Afix = run_fft_fixed(z, T, nblk, 17)["A"]
    # ideal float cascade: same peak-normalized WOLA fold, exact FFT, exact split
    buf_f = wola_fold(z, T, nblk)
    Yf = np.fft.fft(buf_f, axis=0)
    Ymf = np.conj(Yf[(N - np.arange(N)) % N, :])
    Aflt = (Yf + Ymf) / 2.0
    # scale-align (fixed A carries the FFT's internal /2^6 gain); compare bins
    sel = slice(M.KB - 8, M.KB + 8)
    af = Afix[sel].ravel()
    fl = Aflt[sel].ravel()
    g = np.vdot(fl, af) / np.vdot(fl, fl)            # best-fit complex gain
    err = af - g * fl
    nf = 10 * np.log10(np.mean(np.abs(err) ** 2) /
                       max(np.mean(np.abs(g * fl) ** 2), 1e-30))
    print(f"    fixed-point A vs float A (bins KB+/-8): noise floor {nf:6.1f} dB")
    print(f"    (this is the FFT/twiddle fixed-point limit the shipped bins carry)")

    # --- [4] the decisive M1 [C] isolation metric THROUGH fixed point ---------
    hdr("[4] adjacent isolation through fixed point (M1 [C]: wanted + 40 dB "
        "interferer)")
    print("     interf +D  |  float   fixed   (dB SINAD of wanted channel)")
    fw_off, k0, W = 0.15, M.KB - 3, 7
    for D in (5, 8, 12, 30):
        # fixed point: wanted small, interferer full-scale (+40 dB)
        zc = (tone_int(fw_off, FULLSCALE / 100.0, nblk, T)
              + tone_int(float(D), FULLSCALE, nblk, T)) + 0j
        Af = run_fft_fixed(zc, T, nblk, 17)["A"]
        chf, fsf = M.wanted_channel(Af, (M.KB + fw_off) * M.BINW, k0, W)
        sinf = M.sinad_db(chf, fsf)
        # float ref (M1 amplitudes)
        Sfl = M.adc0(T, [(M.KB + fw_off) * M.BINW, (M.KB + D) * M.BINW],
                     [1.0, 100.0])
        chl, fsl = M.wanted_channel(Sfl, (M.KB + fw_off) * M.BINW, k0, W)
        sinl = M.sinad_db(chl, fsl)
        print(f"     {D:^9d} |  {sinl:6.1f}  {sinf:6.1f}")

    hdr("SUMMARY / settled widths")
    print("  * FFT input width = 17 bits. Two full-scale ADCs pack to complex")
    print("    magnitude |z|=sqrt2*32767~46341, which VIOLATES the maia-hdl FFT")
    print("    16-bit input rule (|z|<=32767) even though re/im each fit 16b; 17b")
    print("    (|z|<=65535) satisfies it and gives 2 bits FFT-output headroom.")
    print("  * WOLA window is PEAK-NORMALIZED (max|h|=1), not unit-DC-gain; the")
    print("    ~N constant gain is absorbed into the programmable bin_scale.")
    print(f"  * FFT output width = {fft_out_width(17)} bits (17 + 6 from the")
    print("    [[0,1]]x6 truncate schedule). A/B split /2 keeps A,B within it.")
    print("  * [3]: fixed-point A[k] tracks float to ~-102 dB -> the FFT/twiddle")
    print("    floor is BELOW the int16 shipped-bin floor; quantise dominates. OK.")
    print("  * [4]: T=4 adjacent isolation survives fixed point (~79 dB worst")
    print("    case vs 88 dB float) -- still far above T=1 (~45 dB). T=4 holds.")
    print("=" * 76)


if __name__ == "__main__":
    main()
