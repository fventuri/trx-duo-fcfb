# Attribution

fcfb is released under the [MIT license](https://github.com/fventuri/trx-duo-fcfb/blob/main/LICENSE).
The canonical attribution lives in
[`fpga/ATTRIBUTION.md`](https://github.com/fventuri/trx-duo-fcfb/blob/main/fpga/ATTRIBUTION.md);
this page summarises it.

## Reused dependencies (MIT, with attribution)

- **[maia-hdl](https://github.com/maia-sdr/maia-hdl)** — the MaiaSDR HDL by Daniel
  Estévez (EA4GPZ), MIT. fcfb uses its FFT core (an R2²SDF pipelined FFT, with
  `Window` and `Twiddle`) and supporting utilities, kept on the `PYTHONPATH` as a
  dependency rather than vendored.
- **maia-sdr-trx-duo** — the TRX-duo port of MaiaSDR, MIT. fcfb adapts its ADC capture,
  clocking/CDC, AXI-Lite register-file pattern, DMA-to-DDR block, and Vivado build
  flow. Its board files (the PS7 preset and ADC pin/clock constraints) originate from
  the TRX_DUO_125-16 board package, © Pavel Demin, MIT.

Where an fcfb Amaranth module derives from one of the above, it carries the original
MIT header plus a note describing the change.

## Behavioural references only (no source used)

- **[ka9q-radio](https://github.com/ka9q/ka9q-radio)** (Phil Karn) — an
  architecture and parameter reference only.
- **npapi reverse-engineering material** (Primoz Beltram) — a performance proof-point
  only (that ~119 MB/s zero-copy with jumbo frames is reachable on this SoC).

Neither is copied or adapted anywhere in fcfb; its transport and DSP are written fresh.

## References

- Mark Borgerding, *Turning Overlap-Save into a Multiband Mixing, Downsampling Filter
  Bank* —
  [PDF](https://www.iro.umontreal.ca/~mignotte/IFT3205/Documents/TipsAndTricks/MultibandFilterbank.pdf).
  The fast-convolution / overlap-save filter-bank technique at the heart of fcfb.
- Daniel Estévez, *Maia SDR* (SDRA 2023) —
  [PDF](https://destevez.net/wp-content/uploads/2023/06/destevez_maiasdr_sdra2023.pdf).
  The MaiaSDR project whose HDL fcfb's FPGA Stage-1 builds on.
- Primož Beltram, *npapi_test* —
  [github.com/pbeltram/npapi_test](https://github.com/pbeltram/npapi_test). The
  zero-copy UDP + jumbo-frame work fcfb's transport uses as a performance reference.
