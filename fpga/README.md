# fpga — fcfb Stage-1 (FPGA analysis FFT + A/B split + bin-select egress)

The FPGA Stage-1 of the split FCFB channelizer. It runs on the TRX-duo Zynq-7010 and
produces the **byte-exact [`../host/fcfb_stream.h`](../host/fcfb_stream.h) stream** the
host side consumes.

## What this is

A packed **dual-ADC** complex 4096-point **T=4 WOLA** analysis FFT → **A/B split** on
the kept bins → **bin-select** run (wanted ± guard) → int16/int24 quantise → **stream
format** with `seq` tagging. Built in **[Amaranth](https://amaranth-lang.org/)**,
extending the tested MaiaSDR TRX-duo port (its FFT core, ADC capture, clocking, DMA,
AXI registers, and Vivado flow).

See [`ATTRIBUTION.md`](ATTRIBUTION.md): maia-hdl / maia-sdr-trx-duo are MIT
dependencies reused with attribution; ka9q-radio and npapi remain
behavioural-reference-only (no source used anywhere in fcfb).

## Layout

```
maia_fcfb/   Amaranth RTL blocks (adc_pack, wola_prefilter, ab_split,
             bin_select, quantise, stream_format, top, registers)
models/      Python golden / bit-accurate models + reference extractors
test/        Amaranth-sim block tests vs the numpy golden model
vivado/      block design + build.sh (adapted from the TRX-duo port)
```

## Dependencies

The RTL and its tests import two external MaiaSDR projects (kept on the `PYTHONPATH`,
not vendored):

- **maia-hdl** — <https://github.com/maia-sdr/maia-hdl> (the FFT core, `Window`,
  `Twiddle`, and supporting utilities).
- the **TRX-duo MaiaSDR port** — `AdcCapture`, the MMCM/clocking + CDC, the AXI-Lite
  register-file pattern, the DMA-to-DDR block, and the Vivado build flow.

## Dev environment

Point the environment at your checkouts of the two dependencies (the tests default to
`~/maia-hdl` and `~/maia-sdr-trx-duo` if these are unset — override as needed):

```bash
export MAIA_HDL=/path/to/maia-hdl
export MAIA_TRXDUO=/path/to/maia-sdr-trx-duo/hdl
# an Amaranth venv (amaranth 0.5.9); the TRX-duo port ships one you can reuse:
# . /path/to/maia-sdr-trx-duo/hdl/.venv/bin/activate

PYTHONPATH="$PWD:$MAIA_HDL:$MAIA_TRXDUO" python -m pytest -q     # block tests
```

## Building the bitstream

```bash
cd vivado
./build.sh            # Vivado 2026.1 flow (adapted from the TRX-duo port)
```

The prebuilt production bitstream is shipped in the board image
([`../buildroot/br2-external-fcfb/board/fcfb_stage1.bit`](../buildroot)).

## Verification

Tiered acceptance:

1. per-block **Amaranth-sim** block tests against the numpy golden model (`pytest`,
   above);
2. a full-Stage-1 simulation whose emitted stream the host side reconstructs;
3. on-board HW verification — the Go host
   ([`../host_app/gofcfb`](../host_app/gofcfb)) reconstructs the model channel from the
   board's live stream within the fixed-point floor.
