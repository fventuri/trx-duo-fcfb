# Wire protocol

The board and host speak a small two-part protocol: a **TCP control** channel that
negotiates what to stream, and a **UDP data** channel that carries the selected bins.
The authoritative reference is
[`host/PROTOCOL_V3.md`](https://github.com/fventuri/trx-duo-fcfb/blob/main/host/PROTOCOL_V3.md)
and the headers it documents
([`fcfb_net.h`](https://github.com/fventuri/trx-duo-fcfb/blob/main/host/fcfb_net.h),
[`fcfb_stream.h`](https://github.com/fventuri/trx-duo-fcfb/blob/main/host/fcfb_stream.h));
this page is an overview.

## TCP control

A client connects to the board's control port (default 7373) and sends an `fcfb_req`
channel selection — the requested frequency range, the ADC mask, and framing options.
The board:

1. runs **admission**: maps the band to a contiguous run of bins `{k0, W}` plus one
   guard bin each side, and checks it against the link budget;
2. programs the FPGA Stage-1 registers;
3. replies with the stream header (an `fcfb_stream.h` header: `magic, version, N, fs,
   k0, W, adc_mask, …`);
4. begins UDP streaming.

The control connection **stays open**. Sending another `fcfb_req` performs a **live
retune** — a generation counter (`run_gen`) bumps so the host can drop stale in-flight
datagrams — and closing the connection stops the stream (it is also the liveness
watchdog). The **v4** request path is guard-aware. A separate board-info port (7374)
answers telemetry (`BINF`) queries on its own thread, so it responds even while a
stream holds the control port.

## UDP data

The selected-bin records are streamed over UDP, with **jumbo frames** when the link
allows, straight out of the DDR ring the PL DMA fills (a zero-copy path). Each record
carries an absolute-block **sequence number** (`seq`) followed by the `W` complex bins
for each selected ADC. Samples are quantised to **int16** or **int24** on the wire;
`adc_mask` bit 0 is ADC0/A and bit 1 is ADC1/B.

The host uses `seq` as the absolute block index to stay UTC-locked: it anchors the
wall-clock once, zero-fills small forward gaps, and re-anchors on a large jump.

## Link budget and guard bins

The number of bins a request may keep is bounded by the UDP MTU and the on-wire sample
width — e.g. at MTU 3980 the int24 budget is ~655 bins. Each kept run is padded by a
guard band so that the host's windowed-dual reconstruction is clean at the edges; the
board's admission predicate and the host's planner agree on the guard so predicted and
granted runs match.
