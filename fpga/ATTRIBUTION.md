# Attribution — fcfb/fpga (M3 Stage-1)

## Reused dependencies (MIT, with attribution)

- **maia-hdl** — MaiaSDR HDL by Daniel Estévez (EA4GPZ), MIT
  (<https://github.com/maia-sdr/maia-hdl>). Used: the `FFT` core (R2²SDF pipelined
  FFT, `Window`, `Twiddle`) and supporting utilities. Kept on the PYTHONPATH as a
  dependency; not vendored into fcfb. MIT headers preserved in any file that
  derives from it.
- **maia-sdr-trx-duo** — TRX-duo port of MaiaSDR, MIT. Used: `AdcCapture`, the MMCM/clocking and
  CDC scheme, the AXI-Lite register-file pattern, the DMA-to-DDR block, and the
  Vivado 2026.1 `build.sh` flow — adapted for fcfb's datapath. Board files
  (`red_pitaya.xml` PS7 preset, ADC pin/clock XDC) originate from the
  TRX_DUO_125-16 board package, © Pavel Demin, MIT.

Where an fcfb Amaranth module is derived from one of the above, it carries the
original MIT header + a note describing the change.

## Behavioural-reference-only (NO source used, anywhere in fcfb)

- **ka9q-radio** (Phil Karn) — architecture/parameter reference only.
- **npapi RE material** (Primoz Beltram) — performance proof-point only (that
  ~119 MB/s zero-copy + jumbo frames is reachable on this SoC).

These two are never copied or adapted. fcfb's transport and DSP are written fresh.
