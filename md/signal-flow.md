# Signal flow

![Signal flow](/img/signal-flow.png)

## On the board (FPGA Stage 1)

1. **Dual ADC capture** at 125 Msps — both ADCs (A and B) are captured and packed into
   one complex stream.
2. **WOLA prefilter + 4096-point FFT** — a T=4 weighted-overlap-add prefilter feeds a
   4096-point analysis FFT, producing a bin every ~30.5 kHz across the band.
3. **A/B split** — the packed complex spectrum is unpacked back into the two per-ADC
   spectra on the kept bins.
4. **Bin-select** — only the bins covering the client's requested band (plus a guard
   bin each side) are kept.
5. **Quantise + stream format** — the kept bins are quantised (int16 or int24) and
   framed with a per-record sequence number.

## Over the wire

6. **TCP control** — the client sends an `fcfb_req` (`{f_lo, f_hi, adc_mask, …}`); the
   board runs admission against the link budget, programs the Stage-1 registers, and
   replies with the stream header. The connection stays open for live retunes.
7. **UDP data (jumbo frames)** — the selected-bin records stream straight out of the
   DDR ring the PL DMA fills, over a zero-copy path.

## On the host (Stage 2)

8. **Ingest + demux** — the arriving records are demultiplexed into per-bin columns,
   anchored to a wall-clock using the record sequence numbers (UDP loss is handled by
   zero-filling small gaps and re-anchoring on large ones).
9. **Windowed-dual synthesis** — each channel is reconstructed from the bins it needs
   with a fixed windowed-dual FIR (the embedded kernel), NCO-tuned to baseband, then
   resampled to the output rate.
10. **Output** — depending on the application:
    - **`fcfbfarm`** cuts UTC-aligned WAVs and runs `jt9` (FT8/FT4) or `wsprd` (WSPR),
      emitting decoded spots;
    - **`fcfbhpsdr`** streams RX I/Q to an openHPSDR Protocol-2 client (piHPSDR,
      Thetis, linhpsdr), one DDC per receiver;
    - **dual-ADC diversity** (`adc=3`) reconstructs the same channel from both antennas
      and combines them coherently (maximal-ratio, or null-steering to cancel an
      interferer).
