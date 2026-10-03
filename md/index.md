# TRX-duo fcfb

A distributed **fast-convolution filter bank** (FCFB) for the **TRX-duo**, a Xilinx
Zynq-7010 SDR board. In the spirit of [ka9q-radio](https://github.com/ka9q/ka9q-radio),
the work is split between the board's FPGA and a host PC so that a 1 GbE link is never
the bottleneck: the FPGA does one wideband analysis FFT and ships only the bins a client
actually asked for, and the host reconstructs as many fine channels as it likes from
that one shared stream.

![System architecture](/img/architecture.png)

1. [Architecture](/architecture/) — the board/host split and why it is clean-room
1. [Signal flow](/signal-flow/) — from the ADCs to decoded spots and HPSDR I/Q
1. [The board image](/board-image/) — the SD-card image, stacked on trx-duo-buildroot
1. [The host applications](/host-apps/) — `fcfbfarm`, `fcfbhpsdr`, `fcfbinfo`
1. [Wire protocol](/protocol/) — the TCP control + UDP data path
1. [Attribution](/attribution/) — dependencies and behavioural references

## Quick start

Download the host binaries for your platform and the board SD-card image from the
[latest release](https://github.com/fventuri/trx-duo-fcfb/releases/latest) (verify
against `SHA256SUMS.txt`), write the image to a micro-SD card, boot the board, then
point a host application at it:

```sh
# a one-shot board health check
./fcfbinfo 192.168.255.20

# present the board to a piHPSDR / Thetis / linhpsdr client as an HPSDR radio
./fcfbhpsdr -board 192.168.255.20

# or run a decoder farm (FT8/FT4/WSPR) over many channels at once
./fcfbfarm -config farm.ini
```

## Source and license

The repository is on GitHub at
[fventuri/trx-duo-fcfb](https://github.com/fventuri/trx-duo-fcfb), released under the
[MIT license](https://github.com/fventuri/trx-duo-fcfb/blob/main/LICENSE).
