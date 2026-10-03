# The host applications

The host (PC) side is a **zero-dependency Go** program
([`host_app/gofcfb`](https://github.com/fventuri/trx-duo-fcfb/tree/main/host_app/gofcfb)).
It builds three self-contained static binaries — the synthesis kernel is embedded, so
nothing else is needed at runtime — that run on Linux, Windows and macOS.

Prebuilt binaries are on the
[releases page](https://github.com/fventuri/trx-duo-fcfb/releases/latest). To build
from source:

```sh
cd host_app/gofcfb
go build ./cmd/fcfbfarm
go build ./cmd/fcfbhpsdr
go build ./cmd/fcfbinfo
```

## fcfbinfo — board health

A one-shot query of the board's telemetry (die temperature, supply voltages, FPGA
part, hardware revision, model, sample rate, loaded gateware), served on a separate
port so it is safe to run even while the board is streaming:

```sh
fcfbinfo 192.168.255.20
```

## fcfbhpsdr — a live HPSDR radio

Presents the board to any openHPSDR **Protocol-2** client (piHPSDR, Thetis, linhpsdr)
as one or more HPSDR radios. The client chooses each DDC's centre frequency, sample
rate and ADC at runtime; the emitter asks the board for the union of bins all enabled
DDCs need, reconstructs each DDC's baseband on the host, and streams RX I/Q.

```sh
fcfbhpsdr -board 192.168.255.20                 # one radio, all interfaces
fcfbhpsdr -config hpsdr.ini                      # multi-radio, from config
```

Multiple radio identities can share the one board session, each routed to its own
client — breaking the per-client receiver cap.

## fcfbfarm — a decoder farm

Reconstructs many channels from the one shared board stream and runs weak-signal
decoders (`jt9` for FT8/FT4, `wsprd` for WSPR) on UTC-aligned windows. The channel
count is decoupled from the wire cost — the ceiling is the bin budget (spectral span),
not the number of decoders.

```sh
fcfbfarm -config farm.ini
```

Each `[channel]` references a `[decoder]` by name; `[decoder]` gives the external
command (`jt9`/`wsprd`, installed separately and on `PATH`) and its window timing. See
the example `farm.example.ini` shipped with the binaries.

## Dual-ADC diversity

The board is a dual-ADC channelizer. Request a channel on `adc=3` and the same bins
stream from both antennas; the host reconstructs the channel once per antenna and
combines them coherently — **maximal-ratio** for the best SNR, or **null-steering** to
cancel a strong co-channel interferer.
