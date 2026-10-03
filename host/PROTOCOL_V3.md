# fcfb wire protocol v3 — masked multi-channel

Reference for the fcfb board-server wire protocol, **request version 3** (masked
multi-channel) and its **stream/accept version 3** response. Authoritative source
is `host/fcfb_net.h` (structs) + `host/fcfb_server.c` (server) + `host/fcfb_udp_recv.c`
(client) + `sim/fcfb_stream_read.py` (reader). This doc describes the v3 path; the
v1 (single contiguous run) and v2 (two-window) paths are the special cases it
generalises.

## Overview

- **Control** rides one **TCP** connection (default port **7373**, `FCFB_TCP_PORT`).
  Client sends a selection request; server replies with a variable-length accept
  header. The connection stays open as a liveness watchdog — closing it stops egress.
- **Data** rides **UDP** to the client's `udp_port`: `N` per-block records per
  datagram behind a small descriptor.
- Both peers are **little-endian** (ARM LE board + x86 LE host); all structs go on
  the wire **as-is, no byte-swapping**. All multi-byte fields are little-endian.
- v3 selects the multi-channel path whenever `fcfb_req.version == 3`
  (`FCFB_REQ_V3`). The **UDP datagram wire stays `FCFB_NET_VER = 1`** across all
  request versions — only the *record layout* (per-ADC bin counts, bin width) changes.
- Channel-list cap: **`FCFB_MAX_CHAN = 64`** (admission bounds it further).

```
   client (host)                          server (board)
        |------------- TCP connect :7373 ------------>|
        |--- fcfb_req(v3) + nextra + nextra*chan ---->|   (request)
        |<-- fcfb_accept_v3 + nruns*fcfb_run ---------|   (accept, or close on reject)
        |<== UDP: fcfb_udp_hdr + n*record ============|   (data stream, run_gen tagged)
        |               ... until TCP close ...        |
```

---

## 1. Request (client → server, TCP)

A v3 request is **three back-to-back pieces** on the TCP stream:

### 1a. Base `fcfb_req` (36 bytes, packed) — this IS channel 0

| off | type   | field           | meaning |
|----:|--------|-----------------|---------|
|  0  | char[4]| `magic`         | `"FREQ"` (`FCFB_REQ_MAGIC`) |
|  4  | u16    | `version`       | **3** (`FCFB_REQ_V3`) |
|  6  | u16    | `adc_mask`      | channel 0 ADC select: bit0=A, bit1=B (0 → A) |
|  8  | f64    | `f_lo`          | channel 0 low edge (Hz) |
| 16  | f64    | `f_hi`          | channel 0 high edge (Hz) |
| 24  | u16    | `udp_port`      | client's UDP receive port (data destination) — **global** |
| 26  | u16    | `dds_k`         | channel 0 tone: 0 = live ADC; else on-bin tone at bin k (DDS gen 0) |
| 28  | i32    | `shift`         | quantiser shift; **< 0 = server default** — **global** |
| 32  | u32    | `dds_phase_inc` | 0 = use `dds_k`; else raw 24-bit DDS phase_inc (off-grid) |

Total `sizeof(struct fcfb_req)` = **36** bytes (packed, no padding). The base
`fcfb_req` is byte-identical across v1/v2/v3, so the version field alone selects the path.

`udp_port` and `shift` are taken from the base request and apply to the **whole run**.
Channel 0's band/ADC/tone come from `f_lo/f_hi/adc_mask/dds_k/dds_phase_inc`.

### 1b. `u16 nextra`

Number of **additional** channels that follow (channels `1 .. nextra`). Total channel
count `nchan = nextra + 1` must be `≤ FCFB_MAX_CHAN`.

### 1c. `nextra × fcfb_chan` (24 bytes each) — channels 1..nextra

| off | type | field           | meaning |
|----:|------|-----------------|---------|
|  0  | f64  | `f_lo`          | low edge (Hz) |
|  8  | f64  | `f_hi`          | high edge (Hz) |
| 16  | u16  | `adc_bits`      | ADC select: bit0=A, bit1=B (0 → A) |
| 18  | u16  | `dds_k`         | 0 = no tone; else on-bin tone at bin k on DDS **generator i** |
| 20  | u32  | `dds_phase_inc` | 0 = use `dds_k`; else raw 24-bit phase_inc (off-grid) |

Per-channel tones map to DDS generator `i` (channel index). A generator exists only
on an `n_dds > i` **verification** bitstream; a tone on a channel with no generator is
dropped (production/no-DDS builds ignore all tones — live ADC input).

**Live retune:** the multi-channel list is read **once**, on the first request of a
connection. The base-request poll used for v1/v2 live retune is single-window; a v3
capture is one run for the life of the connection (reconnect to change the channel set).

---

## 2. Admission (server side, no wire bytes)

For each channel, `fcfb_admit` maps `[f_lo, f_hi]` → a contiguous bin run:

```
k_lo = round(f_lo / BINW),  k_hi = round(f_hi / BINW)     BINW = 125e6/4096 ≈ 30517.578 Hz
k0   = k_lo − G                                            G = FCFB_GUARD_BINS = 3
W    = (k_hi − k_lo) + 2·G + 1        (occupied bins + G guard bins each side)
```

Runs are OR'd into the two per-ADC **keep-bitmaps** (a union — channels sharing bins
cost the bin once). Totals: `W_a = popcount(mask_a)`, `W_b = popcount(mask_b)`; a bin
selected on both ADCs counts in **both**.

The request is **rejected** (server closes the TCP connection, no accept sent) if:
- any channel fails to fit `[1, N)` or `W > FCFB_WMAX` (compile-time wire max), or
- `W_a > wmax` **or** `W_b > wmax` — the PL bank buffer depth (`-w`, per ADC), or
- `W_a + W_b > fcfb_bin_budget(bin_width)` — the loss-free transport ceiling:
  the largest total bin count whose record still fits **one 1500-byte MTU**
  unfragmented (**int24 = 242, int16 = 363** total bins; both ADCs share it).
  Above it the datagram IP-fragments and loss climbs sharply (HW-measured).

A rejected request appears to the client as a short/absent accept (“admission rejected?”).

---

## 3. Accept / response (server → client, TCP)

A **variable-length v3 header** = fixed part + the granted run-list.

### 3a. `fcfb_accept_v3` (56 bytes, `FCFB_HDR_V3_FIXED`)

| off | type   | field       | meaning |
|----:|--------|-------------|---------|
|  0  | char[8]| `magic`     | `"FCFBv1\0\0"` |
|  8  | u32    | `version`   | **3** (`FCFB_STREAM_V3`) |
| 12  | u32    | `N`         | 4096 |
| 16  | f64    | `fs`        | 125e6 |
| 24  | u32    | `adc_mask`  | union of the run `adc_bits` |
| 28  | u32    | `W_a`       | total A bins (`popcount(mask_a)`) |
| 32  | u32    | `W_b`       | total B bins (`popcount(mask_b)`) |
| 36  | u32    | `nblocks`   | **0 = unbounded stream** |
| 40  | f64    | `bin_scale` | `complex_bin = (I + jQ)·bin_scale`; = `2^shift` |
| 48  | u32    | `bin_width` | on-wire I/Q component bits (**16 or 24**) |
| 52  | u32    | `nruns`     | number of `fcfb_run` entries that follow (= `nchan`) |

### 3b. `nruns × fcfb_run` (12 bytes each) — the granted run-list

| off | type | field      | meaning |
|----:|------|------------|---------|
|  0  | u32  | `k0`       | first absolute bin of the run |
|  4  | u32  | `W`        | bins in the run |
|  8  | u32  | `adc_bits` | bit0=A, bit1=B |

Run `i` is channel `i`'s **granted** run (in channel order — runs are *not* merged
or sorted). The reader expands them into the per-ADC bin lists (§5). Header size:
`fcfb_hdr_size_v3(nruns) = 56 + nruns·12`.

The accept header is **byte-identical to the file header** written at the top of a
`stream.bin`, so a receiver can persist it directly.

---

## 4. Data stream (server → client, UDP)

Datagrams to `udp_port`, each = descriptor + packed records.

### 4a. `fcfb_udp_hdr` (12 bytes)

| off | type   | field         | meaning |
|----:|--------|---------------|---------|
|  0  | char[4]| `magic`       | `"FUDP"` (`FCFB_UDP_MAGIC`) |
|  4  | u16    | `version`     | `FCFB_NET_VER` = **1** (UDP wire is version-stable) |
|  6  | u16    | `run_gen`     | bumped on every (re)program; client **drops stale run_gen** |
|  8  | u16    | `record_size` | bytes per record (`fcfb_record_size_ab`) |
| 10  | u16    | `n_records`   | records packed in this datagram |

### 4b. Record (`fcfb_record_size_ab(W_a, W_b, bin_width)` bytes)

```
+-----------+----------------------------+----------------------------+--------+
| u64 seq   | W_a × {I,Q}  (ADC A bins)  | W_b × {I,Q}  (ADC B bins)  | pad    |
+-----------+----------------------------+----------------------------+--------+
   8 B        ascending absolute-k          ascending absolute-k        to 4B
```

- `seq` — per-block u64 sequence number; a jump ⇒ dropped blocks (loss detection).
- Each `I`/`Q` component is `bin_width/8` bytes (**2** for int16, **3** for int24),
  little-endian **signed**.
- A-run bins come first (ascending absolute k), then B-run bins (ascending absolute k).
- The whole payload is zero-padded up to a **32-bit-word boundary** (a pad occurs only
  for a single-ADC run of odd `W` at `bin_width=24`).
- `record_size = 8 + roundup4((W_a + W_b)·(bin_width/8)·2)`.

The client detects loss via the `seq` jumps and **drops any datagram whose `run_gen`
≠ the accepted run's gen** (stale pre-retune traffic still in flight).

---

## 5. Demux (client reconstruction)

`runs_to_bins(runs)` expands the run-list into the two sorted absolute-k lists:

```
ka = sorted union of { k : run covers k and run.adc_bits & 1 }   # ADC A
kb = sorted union of { k : run covers k and run.adc_bits & 2 }   # ADC B
```

`len(ka) == W_a`, `len(kb) == W_b`. Within each record, the `W_a` A-components map to
`ka` in order and the `W_b` B-components map to `kb` in order. The complex value of a
bin is `(I + jQ) · bin_scale`. Downstream Stage-2 reconstruction (host windowed dual)
takes it from there.

---

## 6. Worked example

Request: two channels, both on ADC A, no tones (production), `udp_port=54000`,
default shift.
- ch0: `f_lo, f_hi` around bins 700..709 → admit `k0=697, W=16` (10 occupied + 3×2 guard)
- ch1: `f_lo, f_hi` around bins 900..900 (point) → admit `k0=897, W=7`

Wire out (TCP): `fcfb_req{version=3, adc_mask=1, f_lo,f_hi=ch0, udp_port=54000,
shift<0}` · `nextra=1` · `fcfb_chan{f_lo,f_hi=ch1, adc_bits=1}`.

Server admits: `mask_a` = bins {697..712} ∪ {897..903}; `W_a = 16 + 7 = 23`, `W_b = 0`.
Budget check `23 ≤ 242` ✓, `23 ≤ wmax` ✓.

Accept (TCP): `fcfb_accept_v3{version=3, W_a=23, W_b=0, adc_mask=1, bin_width=24,
nblocks=0, nruns=2}` · `fcfb_run{697,16,1}` · `fcfb_run{897,7,1}`.

Data (UDP): records of `8 + roundup4(23·3·2) = 8 + 138 = 146` bytes: `seq` then 23
ascending-k A bins (I,Q int24). `ka = [697,…,712, 897,…,903]`.

---

## 7. Relationship to v1 / v2

| | request | accept | record |
|---|---|---|---|
| **v1** | `fcfb_req` (32 B), version 1 | `fcfb_accept` 48 B, int16 | seq + per-ADC `W`×{I,Q} |
| **v2** | `fcfb_req` + `fcfb_req_ext` (two-window) | `fcfb_accept` 52 B (+`bin_width`), int24 | one A-run = W0+W1 bins |
| **v3** | `fcfb_req` + `u16 nextra` + `nextra×fcfb_chan` | `fcfb_accept_v3` 56 B + run-list | seq + `W_a` A-bins + `W_b` B-bins |

v1/v2 are the single-contiguous-run / symmetric-`W` special cases; v3 generalises to
N channels, per-ADC masks, and `W_a ≠ W_b`.
