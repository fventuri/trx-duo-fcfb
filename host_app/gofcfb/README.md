# gofcfb — standalone Go host apps for the fcfb board

`package fcfb`: a dependency-free Go implementation of the fcfb host side (board =
wideband analysis channelizer, host = per-receiver synthesis). One shared package —
the byte-exact board transport (v3/v4 TCP control + UDP data), the windowed-dual
`StreamSynth`, the resampler and the planner — builds into two binaries under
`cmd/`:

* **`fcfbfarm`** — the decoder farm (port of `host_app/farm.py`): reconstructs each
  configured channel from the shared bin stream and runs `jt9` (FT8/FT4) or `wsprd`
  (WSPR) on UTC-aligned windows, many receivers over the board's one stream session.
* **`fcfbhpsdr`** — a live openHPSDR **Protocol-2** radio server (port of
  `host_app/hpsdr_emit.py`): presents the board to any P2 client (piHPSDR, Thetis,
  linhpsdr) as one or more HPSDR radios, reconstructing each DDC's I/Q on the host.
* **`fcfbinfo`** — a tiny one-shot `BINF` client: sends one board-info query and
  prints the board's temperature, voltages, FPGA part, hw revision, model, sample
  rate and gateware, then exits. Served on the board's own info port, so it is safe
  to run while the board is streaming to `fcfbfarm` / `fcfbhpsdr`.

Both speak the board's **v4** request path (guard-aware) and share one synthesis
kernel and DSP, so the farm and the emitter stay bit-for-bit consistent.

**Zero dependencies.** Builds with only the Go standard library; the synthesis
kernel (`dual_R3125_T4_K6.f64`, the validated windowed-dual `g`) is embedded with
`//go:embed`, so each build is one self-contained static binary. Runs on Linux,
Windows and macOS (amd64 and arm64).

Decoders (`jt9` / `wsprd` from WSJT-X, or any tool you like) are invoked as
external programs — install them and put them on `PATH`, or give a full path in
a `[decoder]` `cmd =`.

## Build

```
cd host_app/gofcfb
go build -o fcfbfarm   ./cmd/fcfbfarm
go build -o fcfbhpsdr  ./cmd/fcfbhpsdr
go build -o fcfbinfo   ./cmd/fcfbinfo

# cross-compile (either binary):
GOOS=windows GOARCH=amd64 go build -o fcfbfarm.exe ./cmd/fcfbfarm
GOOS=darwin  GOARCH=arm64 go build -o fcfbhpsdr    ./cmd/fcfbhpsdr
```

## Run the decoder farm

```
./fcfbfarm -config farm.ini        # see farm.example.ini
```

Config is INI/TOML-style: a `[board]` section, one `[decoder]` per external
decoder (its `cmd`, output `parser` = `jt9`|`wspr`, and window `period`/`capture`),
and one `[channel]` per receiver that references a decoder by name (see
`farm.example.ini`). `cmd` takes `{wav} {fmhz} {fhz} {utc}` placeholders. For jumbo
frames, set the host NIC MTU to match the server's `-u` first
(`sudo ip link set <if> mtu 3980`).

Offline decode of a captured v3 stream (for testing, no board): set
`replay = capture.bin` in `[board]` — it decodes every channel from that one
window and exits. Verified bit-for-message-identical to `python3 farm.py --replay`.

## Run the HPSDR-P2 radio server

```
./fcfbhpsdr -config hpsdr.ini      # farm-style config (see hpsdr.example.ini)

# or, config-less quick start (flags):
./fcfbhpsdr -board 192.168.255.20                       # one radio, all interfaces
./fcfbhpsdr -board 192.168.255.20 -radio 127.0.0.2 -radio 127.0.0.3   # two radios
```

Config is the same INI/TOML style as `fcfbfarm`: a `[board]` section (`ip`,
`udp_port`, `guard`, `mtu`, `budget`, `gain`, `kernel`, `recon_workers`) and one
`[radio]` per radio identity (`bind`, `mac`, `board_id` = `angelia`/`hermes`/… or a
number, `n_ddc`) — omit `[radio]` for a single radio on `0.0.0.0`. The DDCs /
frequencies / sample rates are **not** in the config: the P2 client chooses them at
runtime. See `hpsdr.example.ini`. `-config` overrides the flags.

**Multi-core reconstruction (`recon_workers`, default 1).** The emitter reconstructs
every enabled DDC on a single worker goroutine. For a single client (≤8 DDCs) that is
the right choice: HW-measured, 8 DDCs @ 1536 kHz (both ADCs) sustains on the **serial**
path at ~1 core with zero drops, because the DSP for 8 DDCs is only ~1 core.
`recon_workers = N` in `[board]` (or `-recon-workers N`) spreads each datagram's
per-DDC synth+resample across N goroutines/cores; it pays off only when you scale
**past one client** — many DDCs across several `[radio]`s (16/24/32…), where the serial
DSP would exceed one core's real-time budget. Below that point `> 1` is a net cost:
each worker adds ~0.8 cores of per-datagram fork-join overhead (recon_workers=2 ran the
same 8-DDC load at ~1.9 cores vs serial's ~1.0). The dispatcher (board `recv` + retune
planning) stays single-goroutine; only the DSP fans out, and the output is
**byte-identical** to the serial path (each pipe is an independent deterministic chain,
reordered across pipes but never within one — the same invariant that makes the farm's
`recon_workers` bit-exact).

At startup `fcfbhpsdr` runs the same two board checks as `fcfbfarm` (fatal on
failure): (1) the host interface reaching the board must be at the configured `mtu`,
or the board's jumbo frames are silently dropped (0 I/Q) — the error names the exact
`sudo ip link set … mtu …` to run; (2) the synthesis kernel must match the board's
gateware analysis params (`R`/`T`) — a real mismatch is fatal (reconstruction would
be garbage), while an unreachable board or an old server that can't report params
only warns, so the server still starts. So set `mtu` to the server's `-u` (and the
host NIC to match).

Then point a P2 client (piHPSDR/Thetis/linhpsdr) at the host. It answers discovery,
honours each client's per-DDC centre frequency / sample rate / ADC, asks the board
for the **union** of the bins all enabled DDCs need (planner + budget admission),
reconstructs each DDC's baseband gaplessly at a jitter-free ladder rate
(`phasesForBW` → a divisor of R), resamples to the client rate, and streams RX I/Q —
one UDP source port (`1035+ddc`) per DDC. The board paces the output in real time
(40 kHz/block), so there is no explicit rate limiting. `-mtu` must match the server's
`-u` (sets the bin budget: 1500→242, 3980→655 for int24). Multi-radio breaks the
per-client receiver cap: N radios share the one board session, each routed to its own
client.

## Board info & temperature monitoring

The board server answers a **`BINF`** query with the board's live telemetry: Zynq XADC
die temperature and supply voltages, the FPGA part (`xc7z010`, from the PS IDCODE), the
hardware revision (`hw_rev`, e.g. `STEM_125-14_LN_v1.1`, from the u-boot env), the
board model, sample rate, and loaded gateware name. `BINF` is served on a **separate
port (7374)** by its own thread, so it answers even while a stream holds the single
control port (7373).

* **Startup dump.** Both `fcfbfarm` and `fcfbhpsdr` query `BINF` once at startup and
  log every value to stderr — a support log then captures the board's identity + health
  up front.
* **One-shot.** `fcfbfarm -board-info` / `fcfbhpsdr -board <ip> -board-info` prints the
  same full report to stdout and exits. The standalone **`fcfbinfo`** does only this:
  `fcfbinfo [-board IP]` (or `fcfbinfo IP`), no config, nothing else touched.
* **Temperature log.** Both binaries then poll every `monitor_interval` seconds
  (default 30, `0` = off) and log just the die temperature, so you can watch a long run
  (set it in `[board]`, or `-monitor-interval` on the emitter). It is log-only — it
  never alerts or changes streaming.

A board server too old to answer `BINF` (no info port) degrades safely: the dump/one-shot
prints "unavailable" and the temperature log notes it once, then goes quiet.

## How it reconstructs (streaming, low-memory)

The farm streams: a single ingest goroutine fans each arriving bin-block out to
per-channel `StreamChannel`s that synthesise + resample incrementally and cut
UTC-aligned WAVs. There is no rolling bin buffer and no whole-window copy — only
each channel's one active-window 12 kHz audio buffer lives at a time (peak RSS in
the low hundreds of MB across all bands, vs ~19 GB for a buffer-everything design).

* **Per-block synthesis** — an 8-column bin ring feeds a scatter FIR (`StreamSynth`);
  one baseband sample per block, NCO'd with the absolute block index (phase
  continuous). Bit-exact with the whole-window reference (`synthChannel`).
* **Gapless resampler** — `StreamResampler` emits a 40 kHz→rate output only once
  every input it needs has arrived; input history is bounded to ~L/up samples.
* **UDP-loss handling** — the board's per-record `seq` is the absolute block index
  (`t0 = now − seq/binRate`, anchored once). Small forward gaps are zero-filled to
  stay UTC-locked; a big jump re-anchors. The read socket is sized to 128 MB.
* **Backpressure** — decoders run on a bounded worker pool fed by a queue; live
  ingest never blocks (a window is dropped only on true queue overflow).

The whole-window path (`synthChannel` / `channelBaseband` / `toWavRate`) is kept as
the reference the streaming path is checked byte-for-byte against in the tests.

## Synthesis kernel

Reconstruction uses a fixed dual filter `g` (`dual_R3125_T4_K6.f64`), embedded in
the binary via `//go:embed` so it needs no data file. It's the validated
windowed dual exported from Python's `synth.load_dual(K=6)`: raw little-endian
`complex128` pairs, length `K*N = 6*4096 = 24576`. `K=6` reaches the ~−108 dBc
reconstruction floor.

You can override it with `kernel = path.f64` in `[board]` — the number of taps
(`D = ⌈len/R⌉`) is derived from the file, so different `K` just works. Reasons to:

* trade SFDR for CPU — a smaller `K` (e.g. `K=4`, ~−79 dBc) is lighter; a larger
  `K` is cleaner (diminishing returns past the int24 floor);
* serve a board with a different bitstream, or test a refit dual, without
  rebuilding the binary.

**The kernel must be fit for the board's analysis hop `R=3125` and prototype
`T=4`** (baked into the gateware) — a mismatched kernel silently produces garbage.
Only `K` (the length) is a safe host-side knob for the stock board. The stock `K=6`
kernel is embedded, so you need nothing extra for the shipped board. An alternative
`K` is a raw little-endian `complex128` file of the windowed dual `g` (length `K·N`)
generated by the FCFB Python synthesis tooling — an advanced option for a refit dual
or a different bitstream.

## Scope / notes

* The farm uses narrowband synthesis only (`phases = 1`): FT8/FT4/WSPR. The
  multi-phase `phases > 1` ladder (used by `fcfbhpsdr` for wider DDC rates) is in the
  shared `StreamSynth.pushPhases`; the farm's `push` (phases=1) is the unchanged
  byte-exact fast path.
* The resampler (40 kHz → 12 kHz) is a Blackman-Harris windowed-sinc polyphase
  filter — not bit-identical to scipy's `resample_poly`, but decodes match (a
  reconstructed tone lands within ~1 Hz; live and replay decode the same
  messages/SNR as the Python farm).
* Single stream session, like the board itself — run one farm at a time.
