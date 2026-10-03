// fcfb_net.h -- fcfb M4 board-server wire protocol (TCP control + UDP data).
//
// CLEAN-ROOM: written from scratch for the fcfb project. No ka9q-radio / npapi
// source; those are behavioural/performance references only.
//
// Two peers share this header:
//   - the board server  (host/fcfb_server.c, runs on the TRX-duo ARM/Linux)
//   - a client/receiver (host/fcfb_udp_recv.c, runs on the dev host)
// Both ends are little-endian (ARM LE + x86 LE), so structs go on the wire
// as-is with no byte swapping. All multi-byte fields are little-endian.
//
// Control (TCP): client -> fcfb_req selection request; server -> 48-B stream
// header (fcfb_stream.h layout) as accept. The TCP connection stays open for
// live retunes (send another fcfb_req) and acts as the liveness watchdog:
// closing it stops egress. UDP data carries the per-block records verbatim
// (byte-identical to the 6e / fcfb_stream.h file body), N records per datagram
// behind a small fcfb_udp_hdr descriptor; the host detects loss via the
// per-record u64 seq and drops datagrams whose run_gen != the accepted one.

#ifndef FCFB_NET_H
#define FCFB_NET_H

#include <stdint.h>

#define FCFB_TCP_PORT   7373        // default control port
#define FCFB_TCP_INFO_PORT 7374     // board-info (BINF) port -- answered by a
                                    // separate thread so it works DURING a stream
                                    // (the single-session control port is busy then)
#define FCFB_REQ_MAGIC  "FREQ"      // client -> server selection request
#define FCFB_UDP_MAGIC  "FUDP"      // server -> client data datagram
#define FCFB_NET_VER    1           // req / udp-datagram base protocol version

// -------- params query (client validates its kernel before streaming) --------
// A client asks the server for the board's fixed analysis parameters (R, T, N,
// ...) WITHOUT starting a stream, then checks that its synthesis kernel g was fit
// for the same (R,T). The query is a 36-byte fcfb_req-shaped frame whose magic is
// FCFB_PRM_MAGIC ("FPRM") instead of "FREQ" (so the server's existing 36-byte
// first-read handles it unchanged); its `version` carries the client's protocol
// version and all other fields are ignored. The reply is one fcfb_params struct,
// after which the server closes the connection (connect-query-close; the client
// reconnects to stream). An OLD server reads the 36 bytes, sees a non-"FREQ"
// magic, and drops the connection -> the client gets a short/closed read and
// treats params as UNAVAILABLE (warn, no check), so the query is safe against
// pre-params servers.
#define FCFB_PRM_MAGIC     "FPRM"          // client -> server params query
#define FCFB_PARAMS_MAGIC  "FCFBP\0\0\0"   // server -> client params reply (8 B)
#define FCFB_PROTO_VER     4               // server's request-protocol version (v1..v4)

// -------- board-info query (client monitors board health, e.g. temperature) --------
// A client asks the server for the board's LIVE telemetry (Zynq XADC die temperature
// and supply voltages, sample rate, board/gateware identity) so fcfbfarm/fcfbhpsdr
// can watch it during a long run and guard against overheating. Like FPRM it is a
// 36-byte fcfb_req-shaped frame whose magic is FCFB_BINF_MAGIC ("BINF"); the reply is
// one fcfb_board_info, then the connection closes (connect-query-close). Because the
// single control port (FCFB_TCP_PORT) is blocked inside a stream session, BINF is
// served by a SEPARATE thread on FCFB_TCP_INFO_PORT, so it answers even mid-stream
// (XADC sysfs reads are independent of the AXI/DMA datapath). An OLD server has no
// info port -> the client's connect fails and it treats board info as UNAVAILABLE.
#define FCFB_BINF_MAGIC       "BINF"          // client -> server board-info query
#define FCFB_BOARDINFO_MAGIC  "FCFBI\0\0\0"   // server -> client board-info reply (8 B)
#define FCFB_BOARDINFO_VER    1
#define FCFB_BOARDINFO_NVOLT  8               // XADC supply-voltage rails reported
// fcfb_params.param_ver: 0 = analysis params UNKNOWN (old gateware/server, or an
// interim name-table miss) -> the client cannot verify and must warn, not fail.
// >=1 = params valid; 1 = the interim server-side source (constants / gateware
// name table). A future param-register bitstream may report a higher value.
#define FCFB_PARAM_VER_UNKNOWN    0
#define FCFB_PARAM_VER_NAMETABLE  1
#define FCFB_PARAM_VER_PLREGS     2   // read from the PL param register bank

// Request version carried in fcfb_req.version. v1 = the base 32-B request only.
// v2 = the base request FOLLOWED BY an fcfb_req_ext (below) describing a SECOND
// window + its own DDS tone -- the two-tone / two-window HW verification through
// the real UDP path. The base 32-B fcfb_req is byte-identical across v1/v2, so a
// v1 client/server pair is unaffected; a v2-aware server reads the extension only
// when it sees version >= 2. The UDP datagram wire stays FCFB_NET_VER (=1).
#define FCFB_REQ_V1     1
#define FCFB_REQ_V2     2
// v3 = masked MULTI-CHANNEL request: the base fcfb_req is channel 0, followed by
// u16 nextra and nextra * fcfb_chan (channels 1..nextra). Each channel names a
// band + per-channel ADC selection + an optional DDS tone (verification). The
// server admits every channel to a bin run, ORs them into the per-ADC keep-bitmap,
// and returns the granted run-list in a v3 accept header.
#define FCFB_REQ_V3     3
// v4 = v3 framing plus a caller-chosen guard: the base fcfb_req and u16 nextra are
// followed by a u16 guard (bins each side, applied to EVERY channel), then the
// nextra * fcfb_chan. Lets the host override FCFB_GUARD_BINS per request. A v3
// request (no guard field) keeps the default guard, so old clients are unaffected.
#define FCFB_REQ_V4     4
#define FCFB_MAX_CHAN   64          // v3 channel-list cap (admission also bounds it)

// Stream/accept header version. v1 = 48-B header, int16 bins (shipping); v2 =
// 52-B header (appends u32 bin_width), int24 bins (Option-B wideband). The server
// picks the version from its build's bin width; both are readable, version-gated.
// v3 = masked MULTI-RUN (int16 or int24, per-ADC): a variable-length header whose
// selection is a run-list, and a record = W_a A-bins then W_b B-bins (W_a != W_b
// allowed). v1/v2 stay the single-contiguous-run/symmetric-W special case.
#define FCFB_STREAM_V1  1
#define FCFB_STREAM_V2  2
#define FCFB_STREAM_V3  3
#define FCFB_HDR_V1_SIZE 48u
#define FCFB_HDR_V2_SIZE 52u
#define FCFB_HDR_V3_FIXED 56u       // fixed part; + nruns * sizeof(fcfb_run)

// analysis-side constants (mirror fcfb_stream.h / the RTL). BINW = fs/N.
#define FCFB_FS      125000000.0
#define FCFB_N       4096
#define FCFB_BINW    (FCFB_FS / FCFB_N)     // ~30517.578 Hz
// Analysis filterbank hop R (5^5) and prototype-frame count T. These are fixed
// gateware properties (fpga/maia_fcfb) that the HOST synthesis kernel g must be
// fit for -- a kernel built for a different (R,T) reconstructs garbage silently.
// Exposed to clients via the params query (fcfb_params) so they can verify their
// kernel matches the board before streaming. (Interim source: the server fills
// these from here; a future param-register bitstream reports them from the PL.)
#define FCFB_R       3125
#define FCFB_T       4
// Max run width W the PL can hold: StreamFormat's per-block record buffer is
// wmax bins deep (fpga/maia_fcfb/stream_format.py). Admitting a wider run would
// let the server program a W the hardware silently truncates (bins at in-run
// index >= wmax are dropped on capture and read back as 0). MUST equal the
// synthesized Stage1Top wmax. ~512 bins => ~15.6 MHz band at 30.5 kHz/bin.
// Compile-overridable so a wmax=1024 bitstream gets a matching server without
// editing this shipping default: build the server with -DFCFB_WMAX=1024.
#ifndef FCFB_WMAX
#define FCFB_WMAX    512
#endif

// -------- TCP control: client selection request (36 bytes, packed) --------
// f_lo/f_hi are the requested passband edges in Hz; the server maps them to a
// contiguous bin run (see fcfb_admit). shift < 0 asks the server to use its
// default quantiser shift; dds_k != 0 injects an exactly-on-bin-k test tone in
// the PL (loopback, no external gear) instead of the live ADC input.
//
// dds_phase_inc (24-bit) is the OFF-GRID escape hatch: when nonzero it OVERRIDES
// dds_k and programs the DDS phase accumulator directly, placing the tone at
// f = dds_phase_inc * FS / 2^24 (resolution ~7.45 Hz). The interpolated NCO
// keeps this clean (~ -92 dBc) so an off-grid sweep can expose the wideband
// bank's inter-bin images (~ -79 dBc) -- see verify_sfdr_hw_wideband.py. The
// on-bin word is dds_k<<12, so dds_phase_inc = (k<<12) == dds_k=k exactly.
struct fcfb_req {
    char     magic[4];      // "FREQ"
    uint16_t version;       // FCFB_NET_VER
    uint16_t adc_mask;      // bit0 = ADC0 (A), bit1 = ADC1 (B)
    double   f_lo;          // Hz, requested low edge
    double   f_hi;          // Hz, requested high edge
    uint16_t udp_port;      // client's UDP receive port (dest for the data)
    uint16_t dds_k;         // 0 = ADC input; else PL DDS tone at bin k
    int32_t  shift;         // <0 = server default; else fixed quantiser shift
    uint32_t dds_phase_inc; // 0 = use dds_k; else raw 24-bit DDS phase_inc
} __attribute__((packed));

// -------- TCP control: optional v2 request extension (two-window) --------
// Present on the wire ONLY when fcfb_req.version >= FCFB_REQ_V2, immediately after
// the base fcfb_req. Adds a SECOND window [f_lo2,f_hi2] with its own DDS tone
// (dds2_k / dds2_phase_inc, same on-bin vs off-grid semantics as the base tone).
// Both windows are selected on the SAME ADC set (adc_mask); the block record then
// carries window-0's bins then window-1's bins as one ascending-bin A-run (the
// server reports k0 = window-0 base and W = W0 + W1 in the accept). Used to prove
// the multi-window datapath end-to-end over UDP (the n_dds=2 verification build).
struct fcfb_req_ext {
    double   f_lo2;         // Hz, window-1 low edge
    double   f_hi2;         // Hz, window-1 high edge
    uint16_t dds2_k;        // 0 = no 2nd tone; else on-bin DDS2 tone at bin k
    uint16_t _pad;          // reserved (keep 8-byte alignment of the next field)
    uint32_t dds2_phase_inc;// 0 = use dds2_k; else raw 24-bit DDS2 phase_inc
} __attribute__((packed));

// -------- TCP control: v3 multi-channel request --------
// One additional channel (channels 1..nextra follow the base fcfb_req + u16
// nextra on the wire). Each channel has its own band, ADC selection, and optional
// DDS tone. The server maps channel i's tone (if any) to DDS generator i, which
// exists only on an n_dds > i verification bitstream (a tone on a channel with no
// generator is dropped). Off-grid dds_phase_inc overrides the on-bin dds_k.
struct fcfb_chan {
    double   f_lo;          // Hz, low edge
    double   f_hi;          // Hz, high edge
    uint16_t adc_bits;      // bit0 = ADC0 (A), bit1 = ADC1 (B); 0 => A
    uint16_t dds_k;         // 0 = no tone; else on-bin DDS tone at bin k
    uint32_t dds_phase_inc; // 0 = use dds_k; else raw 24-bit DDS phase_inc
} __attribute__((packed));  // 24 bytes

// -------- Selection run entry (v3 accept run-list) --------
// One granted contiguous bin run k0..k0+W-1 kept on the ADC(s) in adc_bits. The
// reader expands the run-list into the sorted per-ADC absolute-k bin lists that
// index the record's W_a A-bins and W_b B-bins.
struct fcfb_run {
    uint32_t k0;            // first absolute bin
    uint32_t W;             // bins in the run
    uint32_t adc_bits;      // bit0 = A, bit1 = B
} __attribute__((packed));  // 12 bytes

// -------- TCP control: v3 accept = multi-run stream header (fixed part) --------
// Followed on the wire (and in the stream.bin) by nruns * fcfb_run. W_a / W_b are
// the total kept-bin counts per ADC (popcount of each keep-bitmap); the record is
// u64 seq + W_a*{I,Q}(A, ascending k) + W_b*{I,Q}(B, ascending k), zero-padded to
// a 32-bit-word boundary. Bins shared by both ADCs (adc_bits==3) count in both.
struct fcfb_accept_v3 {
    char     magic[8];      // "FCFBv1\0\0"
    uint32_t version;       // 3
    uint32_t N;             // 4096
    double   fs;            // 125e6
    uint32_t adc_mask;      // union of the run adc_bits
    uint32_t W_a;           // total A bins (popcount mask_a)
    uint32_t W_b;           // total B bins (popcount mask_b)
    uint32_t nblocks;       // 0 = streaming
    double   bin_scale;     // complex_bin = (iN + j iN) * bin_scale
    uint32_t bin_width;     // on-wire I/Q component bits (16 or 24)
    uint32_t nruns;         // fcfb_run entries that follow
} __attribute__((packed));  // FCFB_HDR_V3_FIXED (56) bytes

// -------- TCP control: server accept = fcfb_stream.h header --------
// Byte-identical to the file header in fcfb_stream.h so a receiver can write it
// straight to a stream.bin. nblocks is left 0 to mean "unbounded stream". The
// trailing bin_width field is present ONLY for version >= 2 (int24); a version-1
// accept is the first 48 bytes only (send/read fcfb_hdr_size(version) bytes).
struct fcfb_accept {
    char     magic[8];      // "FCFBv1\0\0"
    uint32_t version;       // 1 (int16) or 2 (int24)
    uint32_t N;             // 4096
    double   fs;            // 125e6
    uint32_t k0;            // first absolute bin of the granted run
    uint32_t W;             // bins in the run
    uint32_t adc_mask;      // granted ADC mask
    uint32_t nblocks;       // 0 = streaming
    double   bin_scale;     // complex_bin = (iN + j iN) * bin_scale
    uint32_t bin_width;     // v2 only: on-wire I/Q component bits (16 or 24)
} __attribute__((packed));

// -------- params query reply (100 bytes, packed) --------
// The board's fixed analysis parameters + build/provenance, returned for an FPRM
// query. R/T are what a host synthesis kernel MUST match. `proto_ver` is the
// server's request-protocol version; `param_ver` says whether R/T/N are trusted
// (see FCFB_PARAM_VER_*). fs/N/bin_width/wmax/guard_default mirror the accept
// path. n_dds/features/build_id are 0/empty on the interim (name-table) source
// and get filled once a param-register bitstream exists. gateware_name is the
// /tmp/current-gateware contents (NUL-terminated, truncated to 31 chars).
struct fcfb_params {
    char     magic[8];          // "FCFBP\0\0\0"
    uint16_t proto_ver;         // server request-protocol version (FCFB_PROTO_VER)
    uint16_t param_ver;         // 0 = params unknown; >=1 = R/T/N valid
    uint32_t R;                 // analysis hop (5^5 = 3125)
    uint32_t T;                 // analysis prototype frames (4)
    uint32_t N;                 // FFT size (4096)
    uint32_t wmax;              // PL per-run bin buffer depth
    uint32_t bin_width;         // on-wire I/Q component bits (16 or 24)
    uint32_t n_dds;             // DDS generators on this build (0 = not reported)
    double   fs;                // sample rate (125e6)
    uint32_t guard_default;     // default guard bins each side (FCFB_GUARD_BINS)
    uint32_t features;          // reserved bitmask (0 on the interim source)
    char     build_id[16];      // gateware build id / provenance (empty on interim)
    char     gateware_name[32]; // /tmp/current-gateware contents
} __attribute__((packed));      // 100 bytes

// -------- board-info query reply (120 bytes, packed) --------
// LIVE board telemetry for an FCFB_BINF_MAGIC query. temp_c is the Zynq XADC die
// temperature (deg C); volt[] are the on-chip supply rails (V) in the fixed XADC
// order (same channels the stock TRX-duo server reads):
//   0 vccint  1 vccaux  2 vccbram  3 vccpint  4 vccpaux  5 vccoddr  6 vrefp  7 vrefn
// nvolt is how many entries of volt[] are valid (FCFB_BOARDINFO_NVOLT). Any value the
// server could not read is NaN (the client checks). fs mirrors the accept path; model
// is /proc/device-tree/model ("TRX-duo SDR"); gateware is the loaded PL name.
struct fcfb_board_info {
    char     magic[8];          // "FCFBI\0\0\0"
    uint16_t version;           // FCFB_BOARDINFO_VER
    uint16_t nvolt;             // valid entries in volt[] (FCFB_BOARDINFO_NVOLT)
    float    temp_c;            // Zynq XADC die temperature (deg C); NaN if unavailable
    float    volt[FCFB_BOARDINFO_NVOLT]; // supply rails (V), fixed order above; NaN if n/a
    uint32_t fpgaid;            // Zynq PS IDCODE device field (SLCR 0x530 >>12 & 0x1f);
                                // 2 = xc7z010 (the TRX-duo part). 0 if unreadable.
    double   fs;                // sample rate (125e6)
    char     model[32];         // /proc/device-tree/model, NUL-terminated
    char     gateware[32];      // loaded PL gateware name (/tmp/current-gateware)
    char     hwrev[32];         // u-boot `hw_rev` env (e.g. "STEM_125-14_LN_v1.1"),
                                // read from the EEPROM env block; "" if unavailable
} __attribute__((packed));      // 156 bytes

// On-wire header size for a v1/v2 stream (v1 omits bin_width). v3 is variable
// (run-list) -- use fcfb_hdr_size_v3.
static inline uint32_t fcfb_hdr_size(uint32_t version) {
    return version >= FCFB_STREAM_V2 ? FCFB_HDR_V2_SIZE : FCFB_HDR_V1_SIZE;
}
// v3 header size: fixed part + the run-list.
static inline uint32_t fcfb_hdr_size_v3(uint32_t nruns) {
    return FCFB_HDR_V3_FIXED + nruns * (uint32_t)sizeof(struct fcfb_run);
}

// -------- UDP data: per-datagram descriptor (12 bytes) --------
// Followed by n_records * record_size raw record bytes. Each record is
// { u64 seq; per selected ADC: W*{I, Q} } exactly as fcfb_stream.h, where each
// component is bin_width/8 little-endian signed bytes (2 for int16, 3 for int24)
// and the record is zero-padded up to a 32-bit-word boundary. bin_width is
// advertised in the accept header; record_size disambiguates it on the wire.
struct fcfb_udp_hdr {
    char     magic[4];      // "FUDP"
    uint16_t version;       // FCFB_NET_VER
    uint16_t run_gen;       // bumped on every (re)program; stale => drop
    uint16_t record_size;   // bytes per record (fcfb_record_size(mask,W,bin_w))
    uint16_t n_records;     // records packed in this datagram
} __attribute__((packed));

static inline uint32_t fcfb_popcount2(uint32_t m) {
    return (m & 1) + ((m >> 1) & 1);
}
static inline uint32_t fcfb_bin_bytes(uint32_t bin_width) {
    return (bin_width + 7u) / 8u;               // 2 (int16) or 3 (int24)
}
// Record = u64 seq + per selected ADC W*{I,Q} (bin_bytes each), the whole
// payload zero-padded up to a 32-bit-word boundary (matches the PL bit-packer;
// a pad occurs only for a single-ADC run of odd W at bin_width=24).
static inline uint32_t fcfb_record_size(uint32_t adc_mask, uint32_t W,
                                        uint32_t bin_width) {
    uint32_t payload = fcfb_popcount2(adc_mask) * W * (fcfb_bin_bytes(bin_width) * 2u);
    return 8u + ((payload + 3u) & ~3u);
}
// v3 record size from the per-ADC bin counts (W_a A-bins + W_b B-bins). Generalises
// fcfb_record_size (which assumed each present ADC carries the SAME W): here the two
// runs differ freely. u64 seq + (W_a+W_b)*{I,Q}, zero-padded to a 32-bit word.
static inline uint32_t fcfb_record_size_ab(uint32_t W_a, uint32_t W_b,
                                           uint32_t bin_width) {
    uint32_t payload = (W_a + W_b) * (fcfb_bin_bytes(bin_width) * 2u);
    return 8u + ((payload + 3u) & ~3u);
}

// Guard bins padded on EACH side of a channel's occupied band before it is kept
// and reconstructed (host Stage-2 windowed dual). MEASURED
// (measured, Part A): the intrinsic (float) edge-truncation floor of the hop=R K=6 bank vs
// guard G, CONSTANT across occupied width w_sig in {1,5,13} -- G=0: -12 dBc,
// G=1: -90 dBc, G=2: -103 dBc (int24 ~-102 fixed-point ceiling), G=3: -109 dBc
// (full K=6 sim floor). G=2 is the tight minimum; G=3 keeps the guard a full ~7
// dB below the fixed-point ceiling so it is NEVER the limiter for int16 or int24.
// The fixed-point point-channel sweep confirms G=1 is insufficient (-64 dBc at
// W=3). The planner (M-B) pads each cluster EDGE by this G, then unions.
#define FCFB_GUARD_BINS  3

// -------- bin-budget (transport admission) --------
// The bottleneck is NOT the GbE line rate but the UNFRAGMENTED DATAGRAM: a record
// is datagram-atomic, and the stock macb driver at the 1500-byte MTU (no jumbo
// frames) IP-fragments any datagram over one MTU. HW-MEASURED on board .20
// (host/measure_bin_budget.py, wmax=512 int24 build, 2026-08-24): W_total <= 242
// (one record per MTU, 40k packets/s) is reliably loss-free over 200k-block runs;
// the moment a record fragments (W >= 243, ~80k packets/s) loss appears and it is
// DETERMINISTICALLY ~99% lost by W~448 (~107 MB/s). The naive GbE line-rate budget
// (~490 int24) is NOT reachable -- fragmentation doubles the packet rate and kills
// it first. So the admission budget = the largest W_total whose record still fits
// one MTU:
//   record   = 8 (u64 seq) + W_total * 2 * bin_bytes   (padded to 4 bytes)
//   datagram = fcfb_udp_hdr (12) + record
//   IP total = 20 (IP) + 8 (UDP) + datagram  <=  MTU (1500)
//   => W_total <= floor( (1500 - 20 - 8 - 12 - 8) / (2 * bin_bytes) ) = 1452/(2*bb)
// int24 -> 242, int16 -> 363 total bins (both ADCs SHARE it; symmetric dual =
// 121/181 per ADC). NB: a SEPARATE, W-independent stochastic host-receiver stall
// (userspace UDP at 40k pkt/s: if the socket buffer ever falls behind it never
// recovers -> a whole capture drops ~95%) can still hit any W in the 1-rec/dgram
// regime; that is a receiver-robustness issue (recvmmsg / RT priority / bigger
// SO_RCVBUF), not something admission can prevent. FCFB_STREAM_HZ is the per-block
// egress rate (per-bin rate FS/R, R=3125 -> exactly 40000 blocks/s).
//
// RAISING THE CEILING (Zynq/macb jumbo frames, Pavel Demin, RedPitaya forum t=25577):
// the Cadence macb/GEM driver
// ships with jumbo DISABLED (verified on board .20: `ip link set eth0 mtu 2340` ->
// SIOCSIFMTU: Invalid argument). Enabling it needs a kernel patch to
// cadence/macb_main.c zynq_config (add MACB_CAPS_JUMBO + jumbo_max_len=10240) + a
// rebuild -- shipped as board/patches/linux/7.1.7/0003-macb-jumbo.patch in the fcfb
// SD image. HW-TESTED on that image (board .20, 2026-08-24), UDP egress, both eth0
// MTUs raised (macb needs `down; sleep 1; up` -- an immediate `up` fails to get
// carrier):
//   * MTU 2340 (Pavel's sweet spot) WORKS: W=256 and W=320 -- both catastrophic at
//     MTU 1500 (fragmentation) -- run clean (~0.01% wrap transient). Reliable
//     ceiling ~320 bins; W>=350 (~84 MB/s) goes heavy-loss. So jumbo-2340 lifts the
//     budget ~242 -> ~320 (+32%). Above ~320 the limit is the board's sustained UDP
//     throughput (~80 MB/s at 40k pkt/s, board TX + host recvmmsg), NOT fragmentation
//     (that wall is now 382) NOR line rate -- so the effective 2340 budget ~= 320
//     (rate-bound), BELOW the floor((2340-48)/6)=382 MTU cap.
//   * MTU 3980: the 2026-08-24 "UNUSABLE, DO NOT use" verdict was a DE-FRAME-ERA
//     ARTIFACT and is OVERTURNED (Step 5, 2026-08-31). After the de-frame RTL fix +
//     sendmmsg + affinity-default + auto-M, 3980 runs CLEAN: loss-free ~79 MB/s
//     (W<=327, rpd>=2), raw ~107 MB/s / 854 Mb/s dual-ADC (rpd=1, lossy), sweep 4/4.
//     The old "GEM missing-alternate-packets erratum" symptom was the server chase
//     de-framing, not the NIC. The jumbo image now defaults eth0 to MTU 3980.
// The wall is now packets/s (~28-31k), not MTU/fragmentation: production is a fixed
// 40k rec/s, so loss-free needs rpd>=2 (2*rec+8 <= mtu-28 -> W<=327 @3980, W<=190
// @2340). Break past the pps wall (io_uring/AF_XDP) for loss-free line rate.
#define FCFB_STREAM_HZ       40000u
#ifndef FCFB_MTU                          // -DFCFB_MTU=<m>u overrides (SD build = 3980u)
#define FCFB_MTU             1500u        // stock macb; jumbo image defaults eth0 to 3980
#endif
#define FCFB_BIN_BUDGET_24   242u         // int24: floor(1452/6)  (382@2340 cap; ~320 usable)
#define FCFB_BIN_BUDGET_16   363u         // int16: floor(1452/4)  (573@2340 cap)
// Budget for an arbitrary link MTU (jumbo-frame test path): largest W_total whose
// record still fits one unfragmented datagram of `mtu` bytes.
static inline uint32_t fcfb_bin_budget_mtu(uint32_t bin_width, uint32_t mtu) {
    uint32_t bb = fcfb_bin_bytes(bin_width);                 // 2 (int16) or 3 (int24)
    uint32_t overhead = 20u + 8u + (uint32_t)sizeof(struct fcfb_udp_hdr) + 8u;
    if (mtu <= overhead) return 0;
    return (mtu - overhead) / (2u * bb);          // IP(20)+UDP(8)+udp_hdr+seq(8)
}
static inline uint32_t fcfb_bin_budget(uint32_t bin_width) {
    return fcfb_bin_budget_mtu(bin_width, FCFB_MTU);
}

// Largest guard we accept (a sane clamp; fcfb_admit still rejects any guard that
// makes the run exceed FCFB_WMAX or fall outside [1,N)).
#define FCFB_GUARD_MAX  64

// Admission: map [f_lo,f_hi] Hz to a contiguous bin run {k0,W} padding `guard` bins
// each side (PLAN §4; FCFB_GUARD_BINS is the default -- the host may override it per
// request in a v4 request). Returns 0 on success, -1 if it can't fit in [1,N).
static inline int fcfb_admit(double f_lo, double f_hi, long guard,
                             uint32_t *k0, uint32_t *W) {
    if (f_hi < f_lo) { double t = f_lo; f_lo = f_hi; f_hi = t; }
    if (guard < 0) guard = 0;
    if (guard > FCFB_GUARD_MAX) guard = FCFB_GUARD_MAX;
    long k_lo = (long)(f_lo / FCFB_BINW + 0.5);
    long k_hi = (long)(f_hi / FCFB_BINW + 0.5);
    long g    = guard;
    long kk0  = k_lo - g;                 // guard bins below
    long ww   = (k_hi - k_lo) + 2 * g + 1;// occupied + guard bins above/below
    if (ww < 1) ww = 1;
    if (ww > FCFB_WMAX) return -1;        // wider than the PL can buffer
    if (kk0 < 1) return -1;
    if (kk0 + ww - 1 >= FCFB_N) return -1;
    *k0 = (uint32_t)kk0;
    *W  = (uint32_t)ww;
    return 0;
}

#endif // FCFB_NET_H
