# fcfb board-side server (ARM)

The board side of the split FCFB channelizer: the C program that runs on the TRX-duo
(Zynq-7010 ARM/Linux), programs the FPGA Stage-1 registers, and UDP-streams the
**selected-bin stream** straight out of the DDR ring the PL DMA fills.

**Clean-room.** No ka9q-radio source and no npapi reverse-engineering code appear
here; both are behaviour/performance references only (see
[`../fpga/ATTRIBUTION.md`](../fpga/ATTRIBUTION.md)). All framing, packing, and the
transport path are original.

The host (PC) side that *consumes* this stream — reconstruction, decoding, the
HPSDR-P2 emitter — is the Go application in
[`../host_app/gofcfb`](../host_app/gofcfb).

## Files

- `fcfb_server.c` — the live board server (TCP control + UDP data). See below.
- `fcfb_capture.c` — the file-capture driver the server grew from
  (DMA→DDR→file), kept for bring-up and diagnostics.
- `fcfb_net.h` — the board-server wire protocol (TCP control + UDP data) and the
  budget/guard admission predicates.
- `fcfb_stream.h` — the byte-exact FPGA→host stream contract + a reader.
- `fcfb_zcudp_abi.h` — the ABI for the in-kernel zero-copy UDP TX path.
- `test_budget.c` — a host-side unit check of the admission predicates in `fcfb_net.h`.
- `PROTOCOL_V3.md` — the wire protocol reference.

## The live board server (TCP control + UDP data)

A client connects over **TCP** and sends an `fcfb_req` channel selection
(`{f_lo, f_hi, adc_mask, dds_k, shift, udp_port}`); the server runs admission
(band → contiguous bin run `{k0, W}` + 1 guard bin each side), programs the fcfb
Stage-1 AXI-Lite registers, replies with the 48-byte accept (an `fcfb_stream.h`
header), and **UDP-streams** the selected-bin records straight out of the DDR ring the
PL DMA fills. The TCP connection stays open for **live retunes** (send another
`fcfb_req`; `run_gen` bumps so the host drops stale in-flight datagrams) and is the
liveness watchdog — closing it stops egress.

```
# cross-build and deploy to the board, then run AFTER `start-project fcfb_stage1`:
make CROSS_CC=/path/to/buildroot/output/host/bin/arm-buildroot-linux-gnueabihf-gcc fcfb_server
./fcfb_server [tcp_port]                     # default 7373
```

A board-info (`BINF`) query is answered on a separate port (7374) by its own thread,
so it responds even while a stream holds the control port. Point the Go host apps
([`../host_app/gofcfb`](../host_app/gofcfb)) at the board to drive it.

**Continuous DMA re-arm.** `DmaStreamWrite` is one-shot linear (fills the ring once,
then stops); the server re-arms it at the ring-end wrap to run forever. Three hardware
realities the server handles (documented inline): `dma_next_address` leads committed
non-coherent DDR by ≤2 bursts (forward with a `DRAIN_LAG` trailing margin); the egress
FIFO leaves a sub-record lead-in at each (re)arm (lock the record boundary at runtime
via the free-running `+1` seq); and the first `dma_next` read after any other register
access aliases (double-read).

## The SD-image build

The shipped SD-card image expects the server built with the image defaults (int24
on-wire to match the wideband bitstream, and jumbo MTU 3980 to match the overlay's
`eth0` default):

```
make CROSS_CC=/path/to/.../arm-buildroot-linux-gnueabihf-gcc fcfb_server_sd
# copy the result over buildroot/br2-external-fcfb/board/overlay/root/fcfb_server
```

`-u` / `-b` still override at runtime.

## Stream format

See the header comment in `fcfb_stream.h`, and `PROTOCOL_V3.md` for the full wire
protocol. Little-endian; a 48-byte header
(`magic, version, N, fs, k0, W, adc_mask, nblocks, bin_scale`) followed by records of
`{u64 seq, W·(I, Q) per selected ADC}`. `adc_mask` bit0 = ADC0/A, bit1 = ADC1/B.

## Tests

```
make test        # builds and runs test_budget (admission predicates)
```
