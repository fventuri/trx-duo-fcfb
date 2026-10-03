# fcfb — split fast-convolution filter bank for the TRX-duo

A distributed fast-convolution filter bank (FCFB) in the spirit of
[ka9q-radio](https://github.com/ka9q/ka9q-radio), split across the **TRX-duo** FPGA
(a Xilinx Zynq-7010 SDR board) and a host PC so that 1 GbE is not the bottleneck:

- **FPGA (Stage 1):** a 4096-point analysis FFT at the full 125 Msps ADC rate. Both
  ADCs are packed into one complex FFT. Only the **client-selected** bins (+ guard)
  are shipped over the network.
- **Host (Stage 2):** reconstructs fine channels from the selected subbands —
  arbitrary bandwidth, Kaiser filters, demodulation, coherent diversity combine. The
  production host side is a zero-dependency **Go** implementation
  (`host_app/gofcfb`).
- **Transport:** UDP + jumbo frames over a clean-room zero-copy DMA→NIC path
  (targeting the ~119 MB/s proven on this SoC). The client requests
  **{frequency range, ADC(s)}**; the board does admission control against the link
  budget.

## The host applications

The Go host side (`host_app/gofcfb`) builds three self-contained binaries — no
dependencies beyond the Go standard library, and the synthesis kernel is embedded, so
each is a single static binary that runs on Linux, Windows and macOS:

- **`fcfbfarm`** — a decoder farm: reconstructs many channels from the one board
  stream and runs `jt9` (FT8/FT4) or `wsprd` (WSPR) on UTC-aligned windows.
- **`fcfbhpsdr`** — a live **openHPSDR Protocol-2** radio server: presents the board
  to any P2 client (piHPSDR, Thetis, linhpsdr) as one or more HPSDR radios.
- **`fcfbinfo`** — a one-shot board-info client (temperature, voltages, FPGA part,
  gateware).

Prebuilt binaries for Linux, Windows and macOS, plus the board SD-card image, are on
the [releases page](https://github.com/fventuri/trx-duo-fcfb/releases/latest)
(verify against `SHA256SUMS.txt`).

## Layout

```
fpga/            Stage-1 FPGA design (Amaranth HDL): 4096 analysis FFT + bin-select egress
host/            board-side (ARM) TCP-control + UDP-data server (C) + wire protocol
host_app/gofcfb/ the Go host applications (fcfbfarm / fcfbhpsdr / fcfbinfo)
buildroot/       stacked BR2_EXTERNAL that builds the board SD-card image
```

The FPGA design depends on [maia-hdl](https://github.com/maia-sdr/maia-hdl) (the
MaiaSDR HDL by Daniel Estévez, EA4GPZ) and the TRX-duo MaiaSDR port; see
[`fpga/README.md`](fpga/README.md) for how to point the build at them.

## Target hardware

TRX-duo, **xc7z010clg400-1** (Zynq-7010). The SD-card image is built as a second
`BR2_EXTERNAL` stacked on the base
[trx-duo-buildroot](https://github.com/fventuri/trx-duo-buildroot) tree — see
[`buildroot/`](buildroot/).

## Documentation

Full documentation — architecture, signal flow, the board image, the host apps, and
the wire protocol — is published at <https://fventuri.github.io/trx-duo-fcfb/>.

## References

- Mark Borgerding, *Turning Overlap-Save into a Multiband Mixing, Downsampling Filter
  Bank* —
  [PDF](https://www.iro.umontreal.ca/~mignotte/IFT3205/Documents/TipsAndTricks/MultibandFilterbank.pdf).
  The fast-convolution / overlap-save filter-bank technique at the heart of fcfb.
- Phil Karn, KA9Q, *ka9q-radio* —
  [github.com/ka9q/ka9q-radio](https://github.com/ka9q/ka9q-radio).
  The fast-convolution multichannel SDR fcfb is modelled on — a behavioural and
  performance reference only.
- Daniel Estévez, *Maia SDR* (SDRA 2023) —
  [PDF](https://destevez.net/wp-content/uploads/2023/06/destevez_maiasdr_sdra2023.pdf).
  The MaiaSDR project whose HDL fcfb's FPGA Stage-1 builds on.
- Primož Beltram, *npapi_test* —
  [github.com/pbeltram/npapi_test](https://github.com/pbeltram/npapi_test). The
  zero-copy UDP + jumbo-frame work fcfb's transport uses as a performance reference.

## License

[MIT](LICENSE) © Franco Venturi. See [`fpga/ATTRIBUTION.md`](fpga/ATTRIBUTION.md) for
third-party components and references.
