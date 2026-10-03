# Architecture

fcfb splits a channelizer across two machines joined by ordinary Ethernet.

## The split

- **FPGA (Stage 1), on the board.** A single packed **dual-ADC** complex 4096-point
  analysis FFT runs at the full 125 Msps ADC rate (a T=4 WOLA prefilter feeds the FFT).
  Both ADCs are packed into one complex transform. The board keeps only the
  **client-selected** bins (plus a guard band each side), quantises them, tags each
  record with a sequence number, and streams them out.
- **Host (Stage 2), on a PC.** From that selected-bin stream the host reconstructs
  fine channels — arbitrary centre frequency and bandwidth, Kaiser filters,
  demodulation, and coherent dual-ADC diversity combining. Because every channel is
  carved from the *same* shared bin stream, the number of channels is limited by the
  spectral span requested (the link budget), not by the number of receivers.
- **Transport.** UDP with jumbo frames over a zero-copy DMA→NIC path, targeting the
  ~119 MB/s shown to be reachable on this SoC. A TCP control connection carries the
  client's request (**{frequency range, ADC(s)}**) and stays open for live retunes; the
  board does admission control against the link budget.

## Why one wideband FFT

A 1 GbE link cannot carry 2×125 Msps of raw ADC data. Doing the expensive wideband
analysis **once** on the board and shipping only the selected bins turns the link cost
into a function of *how much spectrum you asked for*, not the full Nyquist band. The
host then runs the cheap per-receiver synthesis, as many times as it wants, on one
copy of the data.
