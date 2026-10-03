// fcfb_server.c -- fcfb M4 board server (TCP control + UDP data) for Stage-1.
//
// Runs on the TRX-duo (Zynq 7010) ARM/Linux. Replaces the 6e
// "DMA -> DDR -> file -> scp" capture path (host/fcfb_capture.c) with a live
// server: a client connects over TCP, sends a channel selection (fcfb_req),
// the server runs admission, programs the fcfb Stage-1 AXI-Lite registers over
// /dev/mem, replies the 48-B accept header, and then UDP-streams the
// selected-bin records to the client's endpoint straight out of the reserved
// DDR ring the PL DMA fills. The TCP connection stays open for live retunes and
// is the liveness watchdog: closing it stops egress.
//
// Build (cross): arm-buildroot-linux-gnueabihf-gcc -O2 -static
//                    -o fcfb_server fcfb_server.c
// Usage: ./fcfb_server [tcp_port] [-M dgram_target] [-b 16|24] [-w wmax] [-u mtu] [-E sendmmsg|zcudp] [-A [cpu]] [-D]
//   -E selects the egress backend: zcudp (DEFAULT; in-kernel zero-copy UDP TX via the
//   macb_zcudp path -- no socket/copy/get_user_pages, HW-verified loss-free at 40k
//   dgram/s / ~117 MB/s, breaks the pps wall) or sendmmsg (the old default;
//   MSG_ZEROCOPY/copy socket path, capped ~28-31k pps). The wire format is identical
//   so fcfb_udp_recv is unchanged and the two are A/B-selectable. zcudp needs the
//   macb_zcudp kernel; if it is absent the server AUTO-FALLS-BACK to sendmmsg.
//   Affinity is ON BY DEFAULT: the stream thread is pinned to CPU1 and the eth0 IRQ
//   steered to a different core, killing the bistable ~21k<->31k pps send state (the
//   sender and the eth0 TX softirq must not share a core). -A <cpu> overrides the
//   stream CPU; --no-affinity disables. See the Step-2 affinity handoff.
//   -M auto: on a jumbo link (-u above 1500) with no explicit -M, the datagram fill
//   target is raised to the MTU payload (mtu-28) so mid-W packs 2+ records/datagram.
//   -D = print per-second egress diagnostics to stderr (datagrams/s, MB/s,
//   sendmsg mean/max latency, ENOBUFS, zero-copy completions reaped, resyncs) to
//   pin the send-throughput ceiling. Diagnostic-only; two clock_gettime/datagram.
//   -b MUST match the loaded bitstream's on-wire bin width (16 = shipping int16,
//   24 = Option-B int24). Default 16.  (default tcp_port 7373)
//   -w MUST match the loaded bitstream's per-ADC kept-bin ceiling (StreamFormat
//   wmax): 512 for the n_dds=0/1 builds, 128 for the narrow n_dds=2 build.
//   Default 512. A wider request than -w is rejected (the PL would truncate it).
//   -u = link MTU for the transport bin budget (default 1500). Raise to a jumbo
//   value (e.g. 3980) ONLY when the board runs the MACB_CAPS_JUMBO kernel and eth0
//   + the host NIC MTU are set to match; it must equal the real eth0 MTU or
//   admitted runs fragment. A request whose W_a+W_b exceeds the budget is rejected.
//
// SAFETY (learned on HW):
//  - The PL must be loaded (start-project fcfb_stage1, FIRST after boot) BEFORE
//    running this: touching 0x4000_0000 with no PL loaded hangs the AXI bus.
//  - NEVER assert/pulse sdr_reset (control bit0). Clear it ONCE (write 0x08=0)
//    after PL load, never re-assert. Reading/writing 0x20+ while sdr_reset=1
//    hangs the bus.
//  - AXI-HP0 is not cache-coherent; /dev/mem is opened O_SYNC (uncached).
//  - The DMA (DmaStreamWrite, circular=True per PLAN 3c) fills RING_BASE..RING_END
//    continuously, wrapping at the end without ever stopping. This server chases
//    next_address modulo the ring; there is no re-arm and no per-wrap re-lock.

#define _GNU_SOURCE                          // sendmmsg / struct mmsghdr (Step 1 batch)
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>                    // NAN (board-info telemetry sentinel)
#include <pthread.h>                 // BINF board-info responder thread
#include <time.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/errqueue.h>          // sock_extended_err, SO_EE_ORIGIN_ZEROCOPY (-D diag)
#include <sched.h>                   // sched_setaffinity / cpu_set_t (-A affinity)
#include <sys/ioctl.h>              // ioctl (zcudp SIOCDEVPRIVATE + SIOCGIF*)
#include <net/if.h>                 // struct ifreq / IFNAMSIZ (zcudp / eth0 addrs)

#include "fcfb_net.h"
#include "fcfb_zcudp_abi.h"          // -E zcudp: in-kernel zero-copy UDP TX backend

// ---- register / ring map ----
#define REG_BASE   0x40000000u
// Gateware guard. start-project records the name of the gateware it programmed into
// the PL in this (tmpfs) file; fcfb_server refuses to touch the AXI bus unless the
// name identifies the fcfb gateware. Reading a PL register (R_PRODUCT below) with no
// (or the wrong) bitstream loaded wedges the AXI bus and hard-hangs the board (needs
// a power cycle). A fresh boot has no PL and, tmpfs, no record -- so the server
// safely refuses until start-project has loaded the fcfb gateware. The file itself
// is gateware-agnostic (start-project writes whatever project it loaded, e.g.
// led_blinker); the fcfb-specific policy (accept names starting with "fcfb") lives
// here. Override with -F if you are certain the fcfb PL is loaded.
#define FCFB_CURRENT_GATEWARE "/tmp/current-gateware"
#define FCFB_GATEWARE_PREFIX  "fcfb"       // accepted current-gateware name prefix
// Legacy fixed reserved-region ring (pre-udmabuf) + the DMA ring size baked into
// the bitstream (dma_end - dma_start). With Option B the actual base is runtime
// (u-dma-buf phys_addr, programmed into the PL dma_base register); these are the
// fallback for an old image with no /dev/udmabuf0.
#define RING_BASE_DEFAULT  0x10000000u
#define RING_SIZE_DEFAULT  0x0a000000u              // 160 MiB
// Runtime ring base/size: set from u-dma-buf sysfs (Option B) or the legacy
// defaults (fallback). The DMA writes to [g_ring_base, g_ring_base+g_ring_size);
// the PL dma_next_address read-back is absolute (base + internal), so the chase
// math (next - RING_BASE) is unchanged. Macros keep the rest of the file intact.
static uint32_t g_ring_base = RING_BASE_DEFAULT;
static uint32_t g_ring_size = RING_SIZE_DEFAULT;
#define RING_BASE  g_ring_base
#define RING_SIZE  g_ring_size
#define RING_END   (g_ring_base + g_ring_size)

enum { R_PRODUCT=0x00, R_CONTROL=0x08, R_RUN=0x20, R_QUANT=0x24,
       R_DMA_CTL=0x28, R_DMA_NEXT=0x2C, R_DDS=0x30, R_MASK=0x34, R_DDS2=0x38,
       R_DDS3=0x3C };
// Runtime PL DMA write-base (Option B). Shares fcfb bank slot 0b110 (byte 0x38)
// with R_DDS2 -- present as dma_base only on the n_dds<2 builds (production +
// single-tone verify), which are exactly the ones that stream over this path.
#define R_DMA_BASE   0x38u
// Exact PL ring-lap counter (fcfb dma_laps). Shares fcfb bank slot 0b111 (byte
// 0x3C) with R_DDS3 -- present only on the n_dds<2 builds (production + single-
// tone verify), i.e. exactly the streaming builds. The PL counts every wrap of
// the circular DMA write pointer (it sees next_address on every clock edge), so
// pairing it with R_DMA_NEXT gives an exact, alias-proof committed-byte position
// -- replacing the old software wrap heuristic that de-framed the egress.
#define R_DMA_LAPS   0x3Cu
// Per-channel DDS generators (0x30/0x38/0x3C); generator i exists only on an
// n_dds > i build. R_DDS_REG[i] gives channel i's tone register.
static const unsigned R_DDS_REG[3] = { R_DDS, R_DDS2, R_DDS3 };
// Read-only params bank (byte 0x40..; one 32-bit register per value) that a
// param-register bitstream carries -- the authoritative source for the params
// query. R_PARAM_MAGIC reads back FCFB_PARAM_REG_MAGIC ('FCFP') ONLY on such a
// build; an older bitstream aliases 0x40 to 0x00 and reads PRODUCT_ID instead, so
// the server checks the magic and falls back to the interim name-table params.
enum { R_PARAM_MAGIC=0x40, R_PARAM_VER=0x44, R_PARAM_R=0x48, R_PARAM_T=0x4C,
       R_PARAM_N=0x50, R_PARAM_WMAX=0x54, R_PARAM_BINW=0x58, R_PARAM_NDDS=0x5C,
       R_PARAM_FEATURES=0x60, R_PARAM_BUILD_LO=0x64, R_PARAM_BUILD_HI=0x68 };
#define FCFB_PARAM_REG_MAGIC 0x50464346u        // 'FCFP' little-endian (matches RTL)
#define N_ORDER      12                         // log2(N), N=4096
#define DDS_ENABLE   (1u << 24)
#define DMA_START    0x1
#define DMA_STOP     0x2
// run register (R_RUN): bit0 = enable, bit1 = mask_busy (R, clear-sweep running).
#define RUN_ENABLE   0x1
#define MASK_BUSY    0x2
// mask_load register (R_MASK): addr[0:12], keep_a[12], keep_b[13], load[14]
// (Wpulse: store {keep_b,keep_a} at addr), clear[15] (Wpulse: sweep-zero all N).
#define MASK_LOAD    (1u << (N_ORDER + 2))       // bit 14
#define MASK_CLEAR   (1u << (N_ORDER + 3))       // bit 15
// control (R_CONTROL) bits: bit0 = sdr_reset (clear ONCE after PL load, NEVER
// pulse -- wedges the AXI bus), bit1 = dp_reset (soft datapath reset; safe to
// pulse -- realigns the two-rate fold on retune, leaves RegisterCDC/regs/DMA).
#define DP_RESET     0x2
#define PRODUCT_ID   0x62666366u
#define DEF_SHIFT    6                          // int16 default (DDS amp 8192)
// int24 default: shift 0 streams the full 23-bit internal bin (23 <= 24, so it
// NEVER clips for any in-datapath value) -- that is the whole point of int24
// (exposes the ~-102 dBc ceiling). A larger shift is still selectable for AGC.
#define DEF_SHIFT24  0
#define DGRAM_TARGET 1400u                      // default target payload/datagram
                                                //   (fits 1500 MTU); -M raises it
                                                //   for a jumbo link (see below).
#define DGRAM_MAX    65500u                     // hard cap (< max UDP payload)
// Jumbo target. HW-TESTED VERDICT (2026-08-16): DO NOT raise -M above 1400 on the
// stock kernel. The TRX-duo runs the mainline `macb` driver, which REJECTS
// MTU > 1500 (SIOCSIFMTU: Invalid argument) -- true jumbo needs npapi's custom
// `macbenet` driver. With the board MTU pinned at 1500, a -M 3900 datagram is
// IP-FRAGMENTED into 3x1500 frames; the host kernel reassembles it (so host recv
// syscalls do drop ~3x, but recvmmsg already batched those), while a single
// dropped fragment loses ALL 3 records -- measured WORSE (28k vs 14k gaps at
// W=300). So -M jumbo is a net loss here. The option and DGRAM_JUMBO remain only
// for the day the macbenet jumbo driver is installed; then the GEM's ~4 KiB
// TX-frame buffer caps the frame < ~3994 B (npapi runs daemon MTU 3980 for
// ~120 MB/s), so the payload must stay <= ~3950 => target ~3900, NOT 8900. A
// jumbo frame = 42 B Eth+IP+UDP header + our UDP payload.
#define DGRAM_JUMBO  3900u                      // -M for a REAL jumbo link (macbenet)
// Runtime-selectable datagram fill target. Default DGRAM_TARGET (1400) is the
// tested-best value on the stock driver; the receiver reads whatever size
// arrives, so this is a server-local knob.
static uint32_t g_dgram_target = DGRAM_TARGET;
// On-wire I/Q component width, set with -b to MATCH the loaded bitstream: 16 for
// the shipping/critical int16 build, 24 for the Option-B int24 build. It changes
// the record byte layout + the accept header version (1 vs 2), so a mismatch vs
// the PL corrupts every record -- the operator must set it to the build's width.
// The COMPILE-TIME default is 16; the SD-image binary that ships alongside the
// int24 bitstream is built with -DFCFB_DEFAULT_BIN_WIDTH=24 so it is correct
// without -b (which still overrides at runtime).
#ifndef FCFB_DEFAULT_BIN_WIDTH
#define FCFB_DEFAULT_BIN_WIDTH 16
#endif
static uint32_t g_bin_width = FCFB_DEFAULT_BIN_WIDTH;
// Per-ADC kept-bin ceiling (StreamFormat bank depth), set with -w to MATCH the
// loaded bitstream's synthesized Stage1Top wmax: 512 for the n_dds=0/1 builds,
// 128 for the narrow n_dds=2 verification build. Admitting a wider run than the
// PL can buffer would let the hardware SILENTLY truncate bins at in-run index
// >= wmax (read back as 0), so the server rejects W_a/W_b > g_wmax. The absolute
// compile ceiling is FCFB_WMAX (the wire/egress max); -w only lowers it. Like -b,
// this is a runtime knob the operator sets per loaded bitstream (default 512).
static uint32_t g_wmax = FCFB_WMAX;
// Link MTU used for the transport (unfragmented-datagram) bin budget. Default is
// FCFB_MTU (1500, stock macb). Set with -u to a jumbo MTU (e.g. 3980) when the
// board runs the MACB_CAPS_JUMBO kernel AND eth0/host MTU are raised to match, to
// admit larger runs for the jumbo-frame budget test. MUST match the actual eth0
// MTU: admitting past it re-introduces fragmentation and its loss.
static uint32_t g_mtu = FCFB_MTU;
// Loaded gateware name (from FCFB_CURRENT_GATEWARE), reported in the params query.
// Empty when the -F override skips the gateware-name check.
static char g_gateware_name[32] = {0};
// UDP socket send-buffer size for -S (SO_SNDBUFFORCE). Default 0 = leave the kernel
// default (wmem_default). HW NOTE (2026-08-30): a LARGE sndbuf REGRESSES throughput
// (16 MB eff -> 44 MB/s vs ~65 at ~0.5 MB) -- bufferbloat -- and the 44<->65 MB/s
// split is bistable run-to-run regardless of -S (looks core/IRQ-placement driven,
// not buffer driven). So -S is an experiment knob, NOT a default win. See handoff.
static uint32_t g_sndbuf = 0;                    // 0 = don't setsockopt; -S sets it
// CPU/IRQ affinity (-A): kill the bistable pps state. The eth0 TX-completion
// softirq and the single stream thread contend for one core when the scheduler
// floats the thread onto the eth0 IRQ's CPU -> the sender falls into the ~21k-pps
// "bad" mode (bufferbloat-independent, bistable run-to-run; see the Step-1 handoff).
// Pinning the stream thread to a core the eth0 IRQ does NOT run on locks in the
// ~31k-pps "good" mode every run. DEFAULT ON (HW-verified 6/6 good vs 3/4 without);
// -A <cpu> overrides the stream CPU, --no-affinity disables.
static int g_affinity = 1;                        // pinning on by default
static int g_srv_cpu  = 1;                         // stream thread -> this CPU
static int g_irq_cpu  = 0;                         // eth0 IRQ      -> this CPU (differs)
// The PL advances dma_next_address when it hands a burst to the AXI-HP0 write
// channel -- BEFORE that burst has committed to (non-coherent) DDR. Reading the
// ring right up to next_address therefore races the in-flight writes and yields
// stale bytes. Trail the write pointer by this margin (>> the max outstanding
// write data, ~448 B) so everything we forward has definitely landed. With the
// circular DMA this margin is uniform across the wrap too -- the write pointer
// advances continuously, so the same trailing rule holds at the ring end.
#define DRAIN_LAG    16384u                     // bytes to trail next_address
// Datagrams forwarded per poll before re-reading the DMA pointer and re-checking
// the frontier (overwrite) guard. Small enough that the DMA cannot lap the ring
// during one chunk (256 * ~2 KB = ~0.5 MB << the >=40 MB overwrite margin), big
// enough that the per-chunk pointer re-read is negligible overhead.
#define FWD_BUDGET   256u

static volatile uint32_t *regs;
static volatile uint8_t  *ring;

static inline void     wr(unsigned off, uint32_t v){ regs[off/4] = v; }
static inline uint32_t rd(unsigned off){ return regs[off/4]; }

// Program the per-ADC keep-bitmap for a single contiguous window k0..k0+W-1,
// gated by adc_mask (bit0=A, bit1=B).  Clear-sweep the whole memory first, then
// write one bin per AXI store.  Must run while the datapath is disabled (the
// selection is quasi-static).  Each store to R_MASK is a blocking AXI-Lite write
// (RegisterCDC round-trip), which spaces the load/clear Wpulses far wider than
// the sync->fft PulseSynchronizer latency, so no pulse is dropped.
// Load one contiguous window k0..k0+W-1 (keep bits already shifted into ka/kb)
// WITHOUT clearing -- so several windows can be loaded into one cleared bitmap.
static void mask_load_bins(uint32_t k0, uint32_t W, uint32_t ka, uint32_t kb){
    for (uint32_t j = 0; j < W; j++)
        wr(R_MASK, ((k0 + j) & 0xFFF) | ka | kb | MASK_LOAD);
}

static void mask_program(uint32_t k0, uint32_t W, uint32_t mask){
    uint32_t ka = (mask & 1) ? (1u << N_ORDER) : 0;          // keep_a bit
    uint32_t kb = (mask & 2) ? (1u << (N_ORDER + 1)) : 0;    // keep_b bit
    wr(R_MASK, MASK_CLEAR);                                  // sweep-zero all N
    usleep(3000);                                            // >> 4096-cyc sweep
    mask_load_bins(k0, W, ka, kb);
}

// Two-window keep-bitmap: clear once, then load window 0 (k0a..k0a+Wa-1) and
// window 1 (k0b..k0b+Wb-1) on the same ADC set.  The two windows must not overlap
// and must be given in ascending bin order (window 0 below window 1) so the block
// record's A-run is ascending: W0 window-0 bins then W1 window-1 bins.
static void mask_program2(uint32_t k0a, uint32_t Wa, uint32_t k0b, uint32_t Wb,
                          uint32_t mask){
    uint32_t ka = (mask & 1) ? (1u << N_ORDER) : 0;
    uint32_t kb = (mask & 2) ? (1u << (N_ORDER + 1)) : 0;
    wr(R_MASK, MASK_CLEAR);
    usleep(3000);
    mask_load_bins(k0a, Wa, ka, kb);
    mask_load_bins(k0b, Wb, ka, kb);
}

// v3 multi-channel keep-bitmap: OR every run's bins (with its per-run adc_bits)
// into a keep[] map, then clear once and load each SET bin ONCE with its combined
// keep_a/keep_b -- so a bin shared by two runs on different ADCs keeps BOTH (a
// per-bin load would otherwise overwrite). Loading ascending matches nothing in
// particular (the RTL emits by scanning bins ascending, A before B), but it keeps
// W_a/W_b == the popcount the reader recomputes from the run-list. keep[] is the
// caller's N-byte scratch; returns the per-ADC totals in *W_a / *W_b.
static void mask_program_runs(const struct fcfb_run *runs, uint32_t nruns,
                              uint8_t *keep, uint32_t *W_a, uint32_t *W_b){
    memset(keep, 0, FCFB_N);
    for (uint32_t r = 0; r < nruns; r++) {
        uint32_t ab = runs[r].adc_bits ? (runs[r].adc_bits & 0x3) : 0x1;
        for (uint32_t j = 0; j < runs[r].W; j++)
            keep[(runs[r].k0 + j) & 0xFFF] |= ab;
    }
    wr(R_MASK, MASK_CLEAR);
    usleep(3000);
    uint32_t wa = 0, wb = 0;
    for (uint32_t bin = 0; bin < FCFB_N; bin++) {
        if (!keep[bin]) continue;
        uint32_t ka = (keep[bin] & 1) ? (1u << N_ORDER) : 0;
        uint32_t kb = (keep[bin] & 2) ? (1u << (N_ORDER + 1)) : 0;
        wr(R_MASK, (bin & 0xFFF) | ka | kb | MASK_LOAD);
        wa += (keep[bin] & 1) ? 1 : 0;
        wb += (keep[bin] & 2) ? 1 : 0;
    }
    *W_a = wa; *W_b = wb;
}

// Read dma_next_address, de-aliased. The RegisterCDC returns the register
// fetched by the PREVIOUS read request, so the first read after touching any
// OTHER register (e.g. the DMA_START re-arm write) aliases to a stale value
// (HW rule #4). Two reads on the same address always return the
// true dma_next: in steady state both are correct; after a switch the second
// is. Cheap enough to use for every poll.
static inline uint32_t read_dma_next(void){
    (void)rd(R_DMA_NEXT);
    return rd(R_DMA_NEXT);
}

// Use the PL's exact ring-lap counter (dma_laps @ R_DMA_LAPS) instead of the
// legacy software wrap heuristic. Default on; the n_dds<2 streaming builds all
// carry the register. -L forces the legacy heuristic for the rare n_dds>=2
// two-tone bench builds, where slot 0b111 is dds3 (dma_laps absent -> reads 0).
static int g_pl_laps = 1;

// Read any fcfb register de-aliased. The RegisterCDC read pipeline returns the
// value fetched by the PREVIOUS read request (HW rule #4), so the
// first read after touching a DIFFERENT register aliases; a second read on the
// same address always returns the true value. read_dma_next is the R_DMA_NEXT
// specialisation of this; use this for R_DMA_LAPS in the same poll.
static inline uint32_t read_reg2(unsigned off){
    (void)rd(off);
    return rd(off);
}

// DDS register value for a tone: a raw off-grid phase_inc overrides an on-bin k;
// neither leaves the generator disabled (word 0 = no tone / live ADC).
static inline uint32_t dds_word(uint32_t dds_k, uint32_t dds_phase_inc){
    if (dds_phase_inc) return (dds_phase_inc & 0xFFFFFFu) | DDS_ENABLE;
    if (dds_k)         return ((dds_k & 0xFFFu) << (24 - N_ORDER)) | DDS_ENABLE;
    return 0;
}

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int s){ (void)s; g_stop = 1; }

// Disable the datapath and stop the DMA (safe teardown between clients).
// ORDER MATTERS with the circular DMA (3c): it never stops on its own, so it
// always has outstanding AXI write bursts in flight. Stop the DMA FIRST, while
// the datapath is still feeding the egress FIFO, so those bursts complete their
// beats (wlast) and the DMA quiesces cleanly. Disabling the datapath first would
// starve an in-flight burst mid-transfer and wedge AXI-HP0 with no recovery short
// of a reboot (observed: the 2nd client after a circular run stalled, no data).
static void datapath_off(void){
    wr(R_DMA_CTL, DMA_STOP);
    usleep(1000);                                // let the <=2 bursts drain
    wr(R_RUN, 0);                                // enable=0
    usleep(1000);
}

// ------------------------------------------------------------------ streaming
// One accepted session: registers already programmed + enabled + DMA armed.
// Drains the ring to UDP until the client disconnects/retunes or a signal.
// Returns: 0 = client closed cleanly, 1 = new request pending on cfd (retune),
//          -1 = signal/fatal. On retune the pending fcfb_req is left in *req.
struct session {
    uint32_t k0, W, adc_mask, rec, shift, run_gen;
    uint32_t W_a, W_b;                          // v3 per-ADC bin counts (else 0)
    uint32_t recs_per_dgram;                    // records packed per datagram
    int      udp_fd;
    struct sockaddr_in dst;
};

// Read exactly n bytes from a blocking fd; 0 on success, -1 on EOF/error.
static int read_full(int fd, void *buf, size_t n){
    uint8_t *p = buf; size_t got = 0;
    while (got < n) {
        ssize_t r = read(fd, p + got, n - got);
        if (r == 0) return -1;                  // EOF
        if (r < 0) { if (errno == EINTR) continue; return -1; }
        got += (size_t)r;
    }
    return 0;
}

// Answer a params query (FPRM): fill the board's fixed analysis parameters and
// write one fcfb_params reply, then the caller closes the connection. R/T/N are
// the interim server-side source (constants + gateware-name table) -- all current
// fcfb gateware is R=3125/T=4/N=4096, so param_ver is NAMETABLE whenever an fcfb
// bitstream is loaded; a future param-register bitstream will report these from
// the PL. Fields the server does not track on this build (n_dds, features,
// build_id) are left 0/empty. Called on the still-blocking control fd.
static void send_params(int cfd){
    struct fcfb_params p; memset(&p, 0, sizeof p);
    memcpy(p.magic, FCFB_PARAMS_MAGIC, 8);
    p.proto_ver     = FCFB_PROTO_VER;
    p.fs            = FCFB_FS;
    p.guard_default = FCFB_GUARD_BINS;
    snprintf(p.gateware_name, sizeof p.gateware_name, "%s", g_gateware_name);
    // Prefer the authoritative PL param bank when the bitstream carries it (magic
    // matches); otherwise fall back to the interim source (server constants for
    // R/T/N -- all current fcfb gateware is R3125/T4/N4096 -- and the live g_wmax/
    // g_bin_width). The magic check makes reading 0x40 safe on an old bitstream
    // (there it aliases 0x00 and reads PRODUCT_ID, so we fall back).
    if (rd(R_PARAM_MAGIC) == FCFB_PARAM_REG_MAGIC) {
        p.param_ver = FCFB_PARAM_VER_PLREGS;
        p.R         = rd(R_PARAM_R);
        p.T         = rd(R_PARAM_T);
        p.N         = rd(R_PARAM_N);
        p.wmax      = rd(R_PARAM_WMAX);
        p.bin_width = rd(R_PARAM_BINW);
        p.n_dds     = rd(R_PARAM_NDDS);
        p.features  = rd(R_PARAM_FEATURES);
        uint32_t lo = rd(R_PARAM_BUILD_LO), hi = rd(R_PARAM_BUILD_HI);
        // Render the stamp into build_id[16] (a fixed field, NUL-terminated only
        // when it is shorter than 16). A 32-bit git short hash is 8 hex chars; a
        // full 64-bit stamp is 16 and fills the field exactly (no NUL -- the
        // clients split on NUL, so all 16 are read).
        char tmp[24];
        int n = 0;
        if (hi)        n = snprintf(tmp, sizeof tmp, "%x%08x", hi, lo);
        else if (lo)   n = snprintf(tmp, sizeof tmp, "%08x", lo);
        if (n > (int)sizeof p.build_id) n = (int)sizeof p.build_id;
        if (n > 0) memcpy(p.build_id, tmp, (size_t)n);
        // else: build_id unset -> leave empty (already zeroed)
    } else {
        p.param_ver = FCFB_PARAM_VER_NAMETABLE;   // R/T/N known (fcfb family)
        p.R         = FCFB_R;
        p.T         = FCFB_T;
        p.N         = FCFB_N;
        p.wmax      = g_wmax;
        p.bin_width = g_bin_width;
        p.n_dds     = 0;                           // not tracked (interim)
        p.features  = 0;
    }
    if (write(cfd, &p, sizeof p) != (ssize_t)sizeof p)
        fprintf(stderr, "params query: short write\n");
    else
        fprintf(stderr, "params query answered (R=%u T=%u N=%u wmax=%u bin_width=%u)\n",
                p.R, p.T, p.N, p.wmax, p.bin_width);
}

// ---- board-info (BINF): live board telemetry from the Zynq XADC (IIO sysfs) ----
// Answered by a SEPARATE thread on FCFB_TCP_INFO_PORT (info_thread below) so a client
// can poll the board's temperature/voltages EVEN WHILE a stream session holds the
// single control port. The XADC IIO sysfs is independent of the AXI/DMA datapath, so
// these reads are safe concurrently with streaming (they touch neither /dev/mem nor
// the ring). Same source and formulae as the stock TRX-duo server (server.c).
#define FCFB_XADC_DIR "/sys/bus/iio/devices/iio:device0/"

// read_sysfs_float reads one number from a sysfs file. 0 on success, -1 on error.
static int read_sysfs_float(const char *path, float *out){
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char buf[64];
    int ok = (fgets(buf, sizeof buf, f) != NULL);
    fclose(f);
    if (!ok) return -1;
    *out = strtof(buf, NULL);
    return 0;
}

// xadc_temp_c: die temperature = (offset + raw) * scale / 1000, NaN if unreadable.
static float xadc_temp_c(void){
    float off, raw, scl;
    if (read_sysfs_float(FCFB_XADC_DIR "in_temp0_offset", &off) ||
        read_sysfs_float(FCFB_XADC_DIR "in_temp0_raw",    &raw) ||
        read_sysfs_float(FCFB_XADC_DIR "in_temp0_scale",  &scl))
        return NAN;
    return (off + raw) * scl / 1000.0f;
}

// xadc_volt: supply rail `chan` (e.g. "in_voltage0_vccint") = raw * scale / 1000 V.
static float xadc_volt(const char *chan){
    char path[256]; float raw, scl;
    snprintf(path, sizeof path, FCFB_XADC_DIR "%s_raw", chan);
    if (read_sysfs_float(path, &raw)) return NAN;
    snprintf(path, sizeof path, FCFB_XADC_DIR "%s_scale", chan);
    if (read_sysfs_float(path, &scl)) return NAN;
    return raw * scl / 1000.0f;
}

// fpga_id returns the Zynq PS IDCODE device field (SLCR 0x530 >>12 & 0x1f), e.g.
// 2 = xc7z010 (the TRX-duo part). Read once from /dev/mem and cached (the id is
// fixed for the life of the board); 0 if it cannot be read. Reads a PS register, not
// the fcfb AXI datapath, so it is always safe (independent of PL state / streaming).
static uint32_t fpga_id(void){
    static uint32_t cached = 0;
    static int done = 0;
    if (done) return cached;
    done = 1;
    int fd = open("/dev/mem", O_RDONLY);
    if (fd < 0) return 0;
    volatile uint32_t *slcr = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ,
                                   MAP_SHARED, fd, 0xF8000000);
    if (slcr != MAP_FAILED) {
        cached = (slcr[0x530 / 4] >> 12) & 0x1f;   // PSS_IDCODE device field
        munmap((void*)slcr, sysconf(_SC_PAGESIZE));
    }
    close(fd);
    return cached;
}

// read_hw_rev copies the u-boot `hw_rev` env var into out (e.g. "STEM_125-14_LN_v1.1").
// fw_printenv is not on the fcfb image, so read the env directly from where u-boot keeps
// it -- the EEPROM env block (see the reference fw_env.config: /sys/.../0-0050/eeprom,
// offset 0x1800, size 0x400): a 4-byte CRC32 then NUL-separated key=value entries
// (an empty entry ends the list). out is left "" if the var / EEPROM is unavailable.
#define FCFB_EEPROM     "/sys/bus/i2c/devices/0-0050/eeprom"
#define FCFB_UENV_OFF   0x1800
#define FCFB_UENV_SIZE  0x400
static void read_hw_rev(char *out, size_t n){
    if (n == 0) return;
    out[0] = '\0';
    int fd = open(FCFB_EEPROM, O_RDONLY);
    if (fd < 0) return;
    static char env[FCFB_UENV_SIZE];
    ssize_t r = pread(fd, env, sizeof env, FCFB_UENV_OFF);
    close(fd);
    if (r <= 4) return;
    const char *p = env + 4, *end = env + r;   // skip CRC32
    while (p < end && *p) {                     // NUL-separated key=value; empty ends it
        size_t len = strnlen(p, (size_t)(end - p));
        if (strncmp(p, "hw_rev=", 7) == 0) {
            snprintf(out, n, "%s", p + 7);
            return;
        }
        p += len + 1;
    }
}

// read_dt_model copies /proc/device-tree/model (NUL/space-trimmed) into out.
static void read_dt_model(char *out, size_t n){
    if (n == 0) return;
    out[0] = '\0';
    FILE *f = fopen("/proc/device-tree/model", "r");
    if (!f) return;
    size_t r = fread(out, 1, n - 1, f);
    fclose(f);
    out[r] = '\0';
    size_t len = strlen(out);
    while (len > 0 && (out[len-1] == ' ' || out[len-1] == '\n' || out[len-1] == '\r'))
        out[--len] = '\0';
}

// fill_board_info populates bi from the live XADC + board/gateware identity.
static void fill_board_info(struct fcfb_board_info *bi){
    static const char *const volt_chan[FCFB_BOARDINFO_NVOLT] = {
        "in_voltage0_vccint",  "in_voltage1_vccaux",  "in_voltage2_vccbram",
        "in_voltage3_vccpint", "in_voltage4_vccpaux", "in_voltage5_vccoddr",
        "in_voltage6_vrefp",   "in_voltage7_vrefn",
    };
    memset(bi, 0, sizeof *bi);
    memcpy(bi->magic, FCFB_BOARDINFO_MAGIC, 8);
    bi->version = FCFB_BOARDINFO_VER;
    bi->nvolt   = FCFB_BOARDINFO_NVOLT;
    bi->temp_c  = xadc_temp_c();
    for (int i = 0; i < FCFB_BOARDINFO_NVOLT; i++)
        bi->volt[i] = xadc_volt(volt_chan[i]);
    bi->fpgaid = fpga_id();
    bi->fs = FCFB_FS;
    read_dt_model(bi->model, sizeof bi->model);
    snprintf(bi->gateware, sizeof bi->gateware, "%s", g_gateware_name);
    read_hw_rev(bi->hwrev, sizeof bi->hwrev);
}

// info_thread: a standalone TCP responder on FCFB_TCP_INFO_PORT. Loops
// accept -> read a 36-byte fcfb_req-shaped frame -> if its magic is BINF, reply one
// fcfb_board_info and close. Runs for the life of the server, independent of the
// stream session, so board health is pollable at any time (including mid-capture).
static void *info_thread(void *arg){
    (void)arg;
    int lfd = socket(AF_INET, SOCK_STREAM, 0);
    if (lfd < 0) { perror("info socket"); return NULL; }
    int one = 1;
    setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a; memset(&a, 0, sizeof a);
    a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port = htons(FCFB_TCP_INFO_PORT);
    if (bind(lfd, (struct sockaddr*)&a, sizeof a) != 0) {
        perror("info bind"); close(lfd); return NULL;
    }
    if (listen(lfd, 4) != 0) { perror("info listen"); close(lfd); return NULL; }
    fprintf(stderr, "board-info responder on TCP :%d\n", FCFB_TCP_INFO_PORT);
    while (!g_stop) {
        int cfd = accept(lfd, NULL, NULL);
        if (cfd < 0) { if (errno == EINTR) continue; break; }
        struct fcfb_req req;
        if (read_full(cfd, &req, sizeof req) == 0 &&
            memcmp(req.magic, FCFB_BINF_MAGIC, 4) == 0) {
            struct fcfb_board_info bi;
            fill_board_info(&bi);
            if (write(cfd, &bi, sizeof bi) != (ssize_t)sizeof bi)
                fprintf(stderr, "board-info query: short write\n");
        }
        close(cfd);
    }
    close(lfd);
    return NULL;
}

// ---- zero-copy egress (Option B, -Z) ----
// When enabled, send_datagram scatter-gathers the payload STRAIGHT FROM the
// udmabuf ring via sendmsg(MSG_ZEROCOPY): the NIC DMAs it and the CPU never
// touches the payload (the copy-heavy path -- an uncached-DDR memcpy + the skb
// copy -- is what caps the board at ~80 MB/s). ON by default on the udmabuf
// ring; auto-disabled on the /dev/mem fallback (VM_PFNMAP can't be pinned) or if
// SO_ZEROCOPY is unavailable. -C forces the copy path (A/B / legacy). NOTE:
// copy egress from the udmabuf ring is CACHE-UNSAFE (the mmap is cached; the PL
// writes DDR non-coherently) -> stale reads corrupt retunes; it is throughput-
// valid but data-invalid, so -C on udmabuf is for budget A/B only.
static int      g_zerocopy = 1;
static uint64_t g_zc_sends = 0;                // MSG_ZEROCOPY datagrams issued

// ---- egress backend (-E) ----
// ZCUDP (default): hand pre-formed datagram descriptors to the in-kernel zcudp path
// (macb_zcudp), which DMAs them straight out of the udmabuf ring with no socket, no
// copy, no get_user_pages -- HW-verified loss-free at 40k dgram/s / ~117 MB/s, which
// breaks the ~28-31k pps sendmmsg wall (25-30% loss at high W). SENDMMSG: the
// sendmmsg(MSG_ZEROCOPY)/copy socket path above (the old default). The wire format is
// byte-identical, so fcfb_udp_recv is unchanged and the two are A/B-selectable with
// -E. zcudp needs the macb_zcudp kernel; if its STATS ioctl fails at startup the
// server AUTO-FALLS-BACK to sendmmsg, so a plain run still works on a stock kernel.
// g_zerocopy is irrelevant in ZCUDP mode (batch_add/batch_flush are not called).
enum egress_mode { EGRESS_SENDMMSG = 0, EGRESS_ZCUDP = 1 };
static int g_egress    = EGRESS_ZCUDP;          // default; auto-falls back to sendmmsg
static int g_zcudp_fd  = -1;                    // AF_INET dgram fd for zcudp ioctls
static int g_zcudp_armed = 0;                   // ARM issued (disarm on teardown)

// -D per-second egress diagnostics (Step 0 of the throughput-ceiling handoff).
// All updates happen on the single stream_session thread, so no locking is
// needed. Totals are cumulative except the two latency accumulators, which the
// once/second printer (diag_tick) resets so mean/max are per-interval.
static int      g_diag         = 0;            // -D: enable the per-second report
static uint64_t g_diag_dgrams  = 0;            // datagrams handed to send (cumulative)
static uint64_t g_diag_sendcalls = 0;          // send SYSCALLS issued (sendmmsg/sendto)
static uint64_t g_diag_bytes   = 0;            // UDP-payload bytes sent (hdr+records)
static uint64_t g_diag_enobufs = 0;            // ENOBUFS hits (zerocopy budget full)
static uint64_t g_diag_zc_reaped = 0;          // zero-copy completions reaped (parsed)
static uint64_t g_diag_lat_sum_ns = 0;         // sum of send-call latency this interval
static uint64_t g_diag_lat_max_ns = 0;         // max send-call latency this interval
static uint64_t g_diag_resyncs = 0;            // frontier resyncs (mirrors the local)
// Header pool: a header iov must stay valid until its datagram's zero-copy
// completion. The kernel copies segments this small synchronously, but rotate a
// pool far larger than the in-flight window (40k dgram/s x microseconds in
// flight << NHDR) as cheap insurance against kernel-version threshold changes.
#define NHDR 1024
static uint8_t  g_hdrpool[NHDR][sizeof(struct fcfb_udp_hdr)];
static uint32_t g_hdr_idx = 0;

// Drain the socket error queue of MSG_ZEROCOPY completions (non-blocking). We do
// NOT gate buffer reuse on them (the 160 MiB ring is not overwritten for seconds
// vs microseconds in flight), but the queue MUST be drained or the per-socket
// zerocopy budget fills and sendmsg starts returning ENOBUFS.
static void drain_zc_completions(int fd){
    char ctrl[256];
    for (;;) {
        struct msghdr m = {0};
        m.msg_control = ctrl; m.msg_controllen = sizeof ctrl;
        if (recvmsg(fd, &m, MSG_ERRQUEUE | MSG_DONTWAIT) < 0) break;
        if (!g_diag) continue;
        // Each completion cmsg reports a contiguous range [ee_info, ee_data] of
        // finished zero-copy send IDs; sum the range widths for the true count.
        for (struct cmsghdr *cm = CMSG_FIRSTHDR(&m); cm; cm = CMSG_NXTHDR(&m, cm)) {
            if (cm->cmsg_level != SOL_IP || cm->cmsg_type != IP_RECVERR) continue;
            struct sock_extended_err *see =
                (struct sock_extended_err *)CMSG_DATA(cm);
            if (see->ee_origin == SO_EE_ORIGIN_ZEROCOPY)
                g_diag_zc_reaped += (uint64_t)(see->ee_data - see->ee_info) + 1;
        }
    }
}

// Batched egress (Step 1): accumulate up to VLEN datagrams, then issue ONE
// sendmmsg(MSG_ZEROCOPY). Step 0 measured the ceiling as a per-datagram send cost
// (~35 us x ~27k dgram/s = ~95% of the thread in send), so amortising the syscall
// ~VLEN:1 is the primary lever. Each datagram is still a whole fcfb_udp_hdr + nrec
// records; the wire format and the client are UNCHANGED. Zero-copy only -- the copy
// path (legacy / -C A-B) stays per-datagram sendto (batch_add sends it inline).
#define SENDMMSG_VLEN 64
struct dgram_batch {
    struct mmsghdr msgs[SENDMMSG_VLEN];
    struct iovec   iov[SENDMMSG_VLEN][3];   // hdr + 1..2 ring segments (wrap split)
    uint32_t       len[SENDMMSG_VLEN];      // UDP-payload bytes (for -D accounting)
    int            n;
};
// Copy-path (non-zerocopy) batch buffers: one contiguous hdr+payload per entry, so
// the copy path can also use sendmmsg. Static (one session at a time, single
// thread), sized for any -M up to DGRAM_MAX. ~4 MiB BSS -- fine on the 256 MB board.
static uint8_t g_copypool[SENDMMSG_VLEN][sizeof(struct fcfb_udp_hdr) + DGRAM_MAX];

// Issue the accumulated batch. sendmmsg sends a prefix and returns how many; on a
// short send or ENOBUFS (per-socket zerocopy budget full) we drain completions and
// retry the unsent remainder, so no datagram is silently dropped. A non-ENOBUFS
// error drops the rest of THIS batch (a clean seq gap to the client, never a
// de-frame). Resets the batch. Drains the error queue once per flush to keep the
// zerocopy budget from filling (was: every ~1024 single sends).
static void batch_flush(struct dgram_batch *b, struct session *s){
    int off = 0;
    while (off < b->n) {
        struct timespec ta;
        if (g_diag) clock_gettime(CLOCK_MONOTONIC, &ta);
        int k = sendmmsg(s->udp_fd, b->msgs + off,
                         (unsigned)(b->n - off), g_zerocopy ? MSG_ZEROCOPY : 0);
        if (g_diag) {
            struct timespec tb; clock_gettime(CLOCK_MONOTONIC, &tb);
            uint64_t ns = (uint64_t)(tb.tv_sec - ta.tv_sec) * 1000000000ull
                        + (uint64_t)(tb.tv_nsec - ta.tv_nsec);
            g_diag_lat_sum_ns += ns;
            if (ns > g_diag_lat_max_ns) g_diag_lat_max_ns = ns;
            g_diag_sendcalls++;
        }
        if (k < 0) {
            if (errno == ENOBUFS) {              // budget full: drain, retry same off
                if (g_diag) g_diag_enobufs++;
                drain_zc_completions(s->udp_fd);
                continue;
            }
            static int nlog = 0;                 // other error: log the first few
            if (nlog < 5) { nlog++;
                fprintf(stderr, "fcfb_server: sendmmsg failed: %s (n=%d off=%d)\n",
                        strerror(errno), b->n, off); }
            break;                               // drop the rest of this batch
        }
        if (g_diag) {
            g_diag_dgrams += (uint64_t)k;
            for (int i = 0; i < k; i++) g_diag_bytes += b->len[off + i];
        }
        off += k;
        if (off < b->n) {                        // short send: budget hit mid-batch
            if (g_diag) g_diag_enobufs++;
            drain_zc_completions(s->udp_fd);
        }
    }
    b->n = 0;
    drain_zc_completions(s->udp_fd);             // keep the ERRQUEUE / optmem clear
}

// Queue one datagram (fcfb_udp_hdr + nrec records from the ring at byte offset
// `ringpos`, split at RING_END on wrap) into the batch; auto-flush when full.
static void batch_add(struct dgram_batch *b, struct session *s,
                      uint32_t ringpos, uint32_t nrec){
    struct fcfb_udp_hdr h;
    memcpy(h.magic, FCFB_UDP_MAGIC, 4);
    h.version     = FCFB_NET_VER;
    h.run_gen     = (uint16_t)s->run_gen;
    h.record_size = (uint16_t)s->rec;
    h.n_records   = (uint16_t)nrec;
    uint32_t total = nrec * s->rec;
    uint32_t first = RING_SIZE - ringpos;        // bytes until the ring end

    if (!g_zerocopy) {                           // legacy copy path: hdr+payload into
        int e = b->n;                            // a pooled buffer, one iov, batched
        uint8_t *pkt = g_copypool[e];
        memcpy(pkt, &h, sizeof h);
        if (first >= total) {
            memcpy(pkt + sizeof h, (const void*)(ring + ringpos), total);
        } else {                                 // straddles RING_END: two parts
            memcpy(pkt + sizeof h, (const void*)(ring + ringpos), first);
            memcpy(pkt + sizeof h + first, (const void*)ring, total - first);
        }
        b->iov[e][0].iov_base = pkt;
        b->iov[e][0].iov_len  = sizeof h + total;
        struct msghdr *mc = &b->msgs[e].msg_hdr;
        memset(mc, 0, sizeof *mc);
        mc->msg_name = &s->dst; mc->msg_namelen = sizeof s->dst;
        mc->msg_iov  = b->iov[e]; mc->msg_iovlen = 1;
        b->msgs[e].msg_len = 0;
        b->len[e] = sizeof h + total;
        if (++b->n == SENDMMSG_VLEN) batch_flush(b, s);
        return;
    }

    // Zero-copy: header from the rotating pool (small -> kernel-copied synchronously,
    // so it need only survive the sendmmsg call), payload scatter-gathered from the
    // udmabuf ring (one or two iovecs on wrap). Each batch entry owns its iov[3].
    int e = b->n;
    uint8_t *hp = g_hdrpool[g_hdr_idx++ & (NHDR - 1)];
    memcpy(hp, &h, sizeof h);
    struct iovec *iov = b->iov[e];
    int niov = 0;
    iov[niov].iov_base = hp;                          iov[niov++].iov_len = sizeof h;
    if (first >= total) {
        iov[niov].iov_base = (void*)(ring + ringpos); iov[niov++].iov_len = total;
    } else {
        iov[niov].iov_base = (void*)(ring + ringpos); iov[niov++].iov_len = first;
        iov[niov].iov_base = (void*)ring;             iov[niov++].iov_len = total - first;
    }
    struct msghdr *m = &b->msgs[e].msg_hdr;
    memset(m, 0, sizeof *m);
    m->msg_name = &s->dst; m->msg_namelen = sizeof s->dst;
    m->msg_iov  = iov;     m->msg_iovlen  = (size_t)niov;
    b->msgs[e].msg_len = 0;
    b->len[e] = sizeof h + total;
    g_zc_sends++;
    if (++b->n == SENDMMSG_VLEN) batch_flush(b, s);
}

// ============================ zcudp egress (-E zcudp) ========================
// The in-kernel zero-copy UDP TX path. batch_add_zcudp builds a flat descriptor
// (inline fcfb_udp_hdr + 1-2 ring fragments given as {phys,len}) mirroring
// batch_add; batch_flush_zcudp hands a batch to macb via ZCUDP_IOC_TX_BATCH. The
// kernel prepends the templated Eth/IPv4/UDP header and DMAs it out of the ring.

// Issue a zcudp SIOCDEVPRIVATE ioctl on eth0 with `payload` in ifr_data.
static int zcudp_ioctl(int cmd, void *payload){
    struct ifreq ifr; memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, "eth0", IFNAMSIZ - 1);
    ifr.ifr_data = (void *)payload;
    return ioctl(g_zcudp_fd, cmd, &ifr);
}

// zcudp descriptor batch. One TX_BATCH ioctl drains up to ZCUDP_VLEN datagrams;
// the kernel flow-controls internally (blocks on GEM TX completions), so this is
// the zcudp analogue of one sendmmsg. Static (single thread, one session at a time).
#define ZCUDP_VLEN 64
struct zcudp_batch {
    struct zcudp_dgram_desc desc[ZCUDP_VLEN];
    uint32_t len[ZCUDP_VLEN];              // UDP-payload bytes (for -D accounting)
    int      n;
};

// Issue the accumulated batch via ZCUDP_IOC_TX_BATCH. The driver processes descs
// in order and writes #PLACED back into count. Our descriptors are always valid
// (frag inside the armed ring, len < mtu), so the driver never drops one -> placed
// == consumed, and a short count means only that a signal interrupted the
// synchronous drain: resubmit the remainder (same semantics as a short sendmmsg).
// A hard ioctl error, or zero progress (window-full at a signal), drops the rest of
// THIS batch as a clean seq gap to the client, never a de-frame. Resets the batch.
static void batch_flush_zcudp(struct zcudp_batch *b, struct session *s){
    (void)s;
    int off = 0;
    while (off < b->n) {
        struct zcudp_tx_batch tb;
        tb.count = (uint32_t)(b->n - off);
        tb.descs = (uint32_t)(uintptr_t)(b->desc + off);
        struct timespec ta;
        if (g_diag) clock_gettime(CLOCK_MONOTONIC, &ta);
        int rc = zcudp_ioctl(ZCUDP_IOC_TX_BATCH, &tb);
        if (g_diag) {
            struct timespec tb2; clock_gettime(CLOCK_MONOTONIC, &tb2);
            uint64_t ns = (uint64_t)(tb2.tv_sec - ta.tv_sec) * 1000000000ull
                        + (uint64_t)(tb2.tv_nsec - ta.tv_nsec);
            g_diag_lat_sum_ns += ns;
            if (ns > g_diag_lat_max_ns) g_diag_lat_max_ns = ns;
            g_diag_sendcalls++;
        }
        if (rc < 0) {
            static int nlog = 0;
            if (nlog < 5) { nlog++;
                fprintf(stderr, "fcfb_server: TX_BATCH ioctl failed: %s (n=%d off=%d)\n",
                        strerror(errno), b->n, off); }
            break;                             // drop the rest of this batch
        }
        uint32_t acc = tb.count;               // #accepted (written back)
        if (g_diag) {
            g_diag_dgrams += acc;
            for (uint32_t i = 0; i < acc; i++) g_diag_bytes += b->len[off + i];
        }
        if (acc == 0) break;                   // no progress: avoid a busy spin
        off += (int)acc;
    }
    b->n = 0;
}

// Queue one datagram (fcfb_udp_hdr + nrec records from the ring at byte offset
// `ringpos`, split at RING_END on wrap) as a zcudp descriptor; auto-flush when full.
// frag_phys = the ring PHYSICAL address of the fragment (g_ring_base + offset); the
// wrapped 2nd fragment starts at the ring base. Exact analogue of batch_add's iovec.
static void batch_add_zcudp(struct zcudp_batch *b, struct session *s,
                            uint32_t ringpos, uint32_t nrec){
    struct fcfb_udp_hdr h;
    memcpy(h.magic, FCFB_UDP_MAGIC, 4);
    h.version     = FCFB_NET_VER;
    h.run_gen     = (uint16_t)s->run_gen;
    h.record_size = (uint16_t)s->rec;
    h.n_records   = (uint16_t)nrec;
    uint32_t total = nrec * s->rec;
    uint32_t first = RING_SIZE - ringpos;        // bytes until the ring end

    int e = b->n;
    struct zcudp_dgram_desc *d = &b->desc[e];
    d->hdr_len = (uint32_t)sizeof h;
    memcpy(d->hdr, &h, sizeof h);
    if (first >= total) {
        d->nfrag = 1;
        d->frag_phys[0] = RING_BASE + ringpos; d->frag_len[0] = total;
        d->frag_phys[1] = 0;                   d->frag_len[1] = 0;
    } else {                                     // straddles RING_END: two fragments
        d->nfrag = 2;
        d->frag_phys[0] = RING_BASE + ringpos; d->frag_len[0] = first;
        d->frag_phys[1] = RING_BASE;           d->frag_len[1] = total - first;
    }
    b->len[e] = sizeof h + total;
    if (++b->n == ZCUDP_VLEN) batch_flush_zcudp(b, s);
}

// Read the eth0 hardware (src) MAC into mac[6]. 0 on success, -1 on failure.
static int get_eth0_mac(uint8_t mac[6]){
    struct ifreq ifr; memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, "eth0", IFNAMSIZ - 1);
    if (ioctl(g_zcudp_fd, SIOCGIFHWADDR, &ifr) < 0) return -1;
    memcpy(mac, ifr.ifr_hwaddr.sa_data, 6);
    return 0;
}

// Read eth0's IPv4 address (network order) into *ip. 0 on success, -1 on failure.
static int get_eth0_ip(uint32_t *ip){
    struct ifreq ifr; memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, "eth0", IFNAMSIZ - 1);
    ifr.ifr_addr.sa_family = AF_INET;
    if (ioctl(g_zcudp_fd, SIOCGIFADDR, &ifr) < 0) return -1;
    *ip = ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr;
    return 0;
}

// Look up `want_ip` (network order) in /proc/net/arp; copy its MAC into mac[6].
// Returns 0 if a COMPLETE (flags & 0x2) entry with a parseable MAC is found.
static int arp_lookup(uint32_t want_ip, uint8_t mac[6]){
    FILE *f = fopen("/proc/net/arp", "r");
    if (!f) return -1;
    char line[256];
    (void)!fgets(line, sizeof line, f);          // skip the column header
    int rc = -1;
    while (fgets(line, sizeof line, f)) {
        char ips[64], hws[8], flags[16], hw[64], mask[64], dev[32];
        if (sscanf(line, "%63s %7s %15s %63s %63s %31s",
                   ips, hws, flags, hw, mask, dev) != 6) continue;
        struct in_addr a;
        if (inet_aton(ips, &a) == 0 || a.s_addr != want_ip) continue;
        if ((int)strtol(flags, NULL, 0) & 0x2) {  // ATF_COM: MAC resolved
            unsigned m[6];
            if (sscanf(hw, "%x:%x:%x:%x:%x:%x",
                       &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6) {
                for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
                rc = 0;
            }
        }
        break;
    }
    fclose(f);
    return rc;
}

// Resolve the client's MAC. The TCP control connection is already established from
// the same IP, so the neighbor cache is normally already populated; send a warm-up
// datagram to the UDP endpoint as a fallback (a well-behaved client drops it: wrong
// run_gen / empty payload) and retry the ARP read a few times.
static int resolve_client_mac(struct session *s, uint8_t mac[6]){
    for (int try = 0; try < 20; try++) {
        if (arp_lookup(s->dst.sin_addr.s_addr, mac) == 0) return 0;
        struct fcfb_udp_hdr warm; memset(&warm, 0, sizeof warm);
        memcpy(warm.magic, FCFB_UDP_MAGIC, 4); warm.version = FCFB_NET_VER;
        sendto(s->udp_fd, &warm, sizeof warm, 0,
               (struct sockaddr *)&s->dst, sizeof s->dst);
        usleep(50000);
    }
    return -1;
}

// (Re)template + arm the zcudp path for this session: resolve the client MAC, read
// the board's src MAC/IP, set the header template (client MAC + this session's UDP
// port), the MTU, and arm the ring phys window. Re-run per (re)program (the UDP port
// can change on retune); ARM is idempotent. Returns 0 on success, -1 on failure.
static int zcudp_arm_session(struct session *s){
    uint8_t dst_mac[6], src_mac[6];
    uint32_t src_ip_net;
    if (resolve_client_mac(s, dst_mac) != 0) {
        fprintf(stderr, "fcfb_server: zcudp -- could not resolve client MAC "
                "for %s (no ARP entry)\n", inet_ntoa(s->dst.sin_addr));
        return -1;
    }
    if (get_eth0_mac(src_mac) != 0 || get_eth0_ip(&src_ip_net) != 0) {
        fprintf(stderr, "fcfb_server: zcudp -- SIOCGIFHWADDR/SIOCGIFADDR(eth0) "
                "failed: %s\n", strerror(errno));
        return -1;
    }
    // src port: the UDP socket's local port (assigned after resolve_client_mac's
    // warm-up send). The client does not filter on it; 0 is fine if unbound.
    uint16_t src_port = 0;
    struct sockaddr_in la; socklen_t ll = sizeof la;
    if (getsockname(s->udp_fd, (struct sockaddr *)&la, &ll) == 0)
        src_port = ntohs(la.sin_port);

    struct zcudp_tmpl t; memset(&t, 0, sizeof t);
    memcpy(t.dst_mac, dst_mac, 6);
    memcpy(t.src_mac, src_mac, 6);
    t.src_ip   = ntohl(src_ip_net);              // template is HOST order
    t.dst_ip   = ntohl(s->dst.sin_addr.s_addr);
    t.src_port = src_port;
    t.dst_port = ntohs(s->dst.sin_port);
    if (zcudp_ioctl(ZCUDP_IOC_SET_TMPL, &t) < 0) {
        fprintf(stderr, "fcfb_server: zcudp SET_TMPL failed: %s\n", strerror(errno));
        return -1;
    }
    uint32_t mtu = g_mtu;
    if (zcudp_ioctl(ZCUDP_IOC_SET_MTU, &mtu) < 0) {
        fprintf(stderr, "fcfb_server: zcudp SET_MTU failed: %s\n", strerror(errno));
        return -1;
    }
    struct zcudp_arm a = { .arm = 1, .ring_phys = RING_BASE, .ring_size = RING_SIZE };
    if (zcudp_ioctl(ZCUDP_IOC_ARM, &a) < 0) {
        fprintf(stderr, "fcfb_server: zcudp ARM failed: %s\n", strerror(errno));
        return -1;
    }
    g_zcudp_armed = 1;
    fprintf(stderr, "fcfb_server: zcudp armed -- dst_mac %02x:%02x:%02x:%02x:%02x:%02x "
            "udp %s:%u src_port %u mtu %u ring 0x%08x+%uMiB\n",
            dst_mac[0], dst_mac[1], dst_mac[2], dst_mac[3], dst_mac[4], dst_mac[5],
            inet_ntoa(s->dst.sin_addr), ntohs(s->dst.sin_port), src_port, mtu,
            RING_BASE, RING_SIZE >> 20);
    return 0;
}

// Disarm the zcudp path (session end). Idempotent; safe if never armed.
static void zcudp_disarm(void){
    if (!g_zcudp_armed) return;
    struct zcudp_arm a = { .arm = 0, .ring_phys = 0, .ring_size = 0 };
    zcudp_ioctl(ZCUDP_IOC_ARM, &a);
    g_zcudp_armed = 0;
}

// Read the kernel zcudp counters (for -D). 0 on success.
static int zcudp_read_stats(struct zcudp_stats *st){
    memset(st, 0, sizeof *st);
    return zcudp_ioctl(ZCUDP_IOC_STATS, st) < 0 ? -1 : 0;
}

// -D: once/second, print the egress rates and the stall indicators that tell us
// which suspect (syscall latency vs. ENOBUFS/completion stalls) dominates the
// send ceiling. Called every loop iteration; prints only when >=1 s has elapsed.
// Reports interval DELTAS. lat mean/max are per-interval (reset here); dgram_gap
// = dgrams issued minus zero-copy completions reaped this interval (a growing gap
// means completions are lagging sends -> the zerocopy budget is the throttle).
static void diag_tick(const struct session *s){
    if (!g_diag) return;
    static struct timespec t_prev;
    static int      have_prev = 0;
    static uint64_t d0, c0, b0, e0, r0, z0;
    struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
    if (!have_prev) {
        t_prev = now; have_prev = 1;
        d0 = g_diag_dgrams; c0 = g_diag_sendcalls; b0 = g_diag_bytes;
        e0 = g_diag_enobufs; r0 = g_diag_resyncs; z0 = g_diag_zc_reaped;
        g_diag_lat_sum_ns = 0; g_diag_lat_max_ns = 0;
        return;
    }
    double dt = (now.tv_sec - t_prev.tv_sec)
              + (now.tv_nsec - t_prev.tv_nsec) / 1e9;
    if (dt < 1.0) return;
    uint64_t dd = g_diag_dgrams - d0, dc = g_diag_sendcalls - c0;
    uint64_t db = g_diag_bytes - b0;
    uint64_t de = g_diag_enobufs - e0, dr = g_diag_resyncs - r0;
    uint64_t dz = g_diag_zc_reaped - z0;
    double mbps  = db / dt / 1e6;                 // payload MB/s (steady-state)
    double dgs   = dd / dt;                        // datagrams/s
    double calls = dc / dt;                        // send SYSCALLS/s (the ceiling knob)
    double per_call = dc ? (double)dd / dc : 0.0;  // datagrams per sendmmsg (batch fill)
    // Latency is now PER SEND-CALL (a sendmmsg covers up to VLEN datagrams).
    double lat_mean_us = dc ? (g_diag_lat_sum_ns / (double)dc) / 1e3 : 0.0;
    double lat_max_us  = g_diag_lat_max_ns / 1e3;
    fprintf(stderr,
            "fcfb_server: [diag] %.1f MB/s (%.0f Mb/s) %.0f dgram/s | "
            "%.0f syscall/s (%.1f dgram/call) mean %.2f us max %.1f us | "
            "ENOBUFS %llu zc_reaped %llu (sent-reaped %+lld) | resync %llu | "
            "W=%u rec=%uB rpd=%u\n",
            mbps, mbps * 8.0, dgs, calls, per_call, lat_mean_us, lat_max_us,
            (unsigned long long)de, (unsigned long long)dz,
            (long long)dd - (long long)dz, (unsigned long long)dr,
            s->W, s->rec, s->recs_per_dgram);
    if (g_egress == EGRESS_ZCUDP) {
        // Kernel-side truth: TX_BATCH accounting is userspace-visible above, but
        // underruns (drain waited on GEM space) and dropped (rejected descs) come
        // only from the driver. inflight is the current GEM ring occupancy.
        static uint32_t u0, p0; static int have_k = 0;
        struct zcudp_stats st;
        if (zcudp_read_stats(&st) == 0) {
            uint32_t du = st.underruns - u0, dp = st.dropped - p0;
            if (!have_k) { du = 0; dp = 0; have_k = 1; }
            fprintf(stderr, "fcfb_server: [diag] zcudp: kern tx_dgrams %u "
                    "underruns +%u dropped +%u inflight %u\n",
                    st.tx_dgrams, du, dp, st.inflight);
            u0 = st.underruns; p0 = st.dropped;
        }
    }
    t_prev = now;
    d0 = g_diag_dgrams; c0 = g_diag_sendcalls; b0 = g_diag_bytes;
    e0 = g_diag_enobufs; r0 = g_diag_resyncs; z0 = g_diag_zc_reaped;
    g_diag_lat_sum_ns = 0; g_diag_lat_max_ns = 0;
}

// Poll cfd (non-blocking) for a retune request or EOF without stalling egress.
// Returns 1 if a full fcfb_req was read into *req, 0 if nothing/partial, -1 EOF.
static int poll_control(int cfd, struct fcfb_req *req, uint8_t *rbuf, size_t *rlen){
    for (;;) {
        ssize_t r = read(cfd, rbuf + *rlen, sizeof(struct fcfb_req) - *rlen);
        if (r == 0) return -1;                  // client closed
        if (r < 0) { if (errno == EINTR) continue; return 0; } // EAGAIN: no data
        *rlen += (size_t)r;
        if (*rlen == sizeof(struct fcfb_req)) {
            memcpy(req, rbuf, sizeof *req);
            *rlen = 0;
            return 1;
        }
    }
}

// Find the record-boundary byte offset (from RING_BASE) at which the DMA's
// record stream is cleanly framed, using the free-running per-record seq (which
// increments by exactly 1). Record 0 does not always start at RING_BASE: the
// egress FIFO can hold sub-record residue from a prior run, and a wrap re-arm
// can leave a corrupted/gappy region at the ring start (records lost while the
// FIFO briefly starved). So this scans every 32-bit-aligned candidate offset
// AND slides forward through the landed window, returning the START of the
// first run of ALIGN_NEED consecutive-seq records -- i.e. it skips any leading
// corruption. `landed` = bytes past RING_BASE known committed. NO_ALIGN if no
// clean run is found yet (caller retries as more data lands).
#define ALIGN_NEED 8u                            // consecutive +1 seqs to lock
#define ALIGN_SCAN 262144u                       // cap the scan span (bytes)
#define NO_ALIGN   0xFFFFFFFFu

// Find the record-boundary byte offset (from RING_BASE) at which the record
// stream is cleanly framed, using the free-running per-record seq (increments by
// exactly 1). Record 0 does not start at RING_BASE: the egress FIFO holds
// sub-record residue at datapath enable. With the circular DMA this runs exactly
// ONCE per session -- after the lock the framing holds forever (no re-arm ever
// re-introduces residue), so records stay contiguous straddling the ring end.
// Scans every 32-bit-aligned candidate and slides forward, returning the START
// of the first run of ALIGN_NEED consecutive-seq records. NO_ALIGN if no clean
// run is found yet (caller retries as more data lands).
static uint32_t find_alignment(uint32_t rec, uint32_t landed){
    if (landed < rec * (ALIGN_NEED + 1)) return NO_ALIGN;  // need more data
    uint32_t span = landed < ALIGN_SCAN ? landed : ALIGN_SCAN;
    for (uint32_t off = 0; off < rec; off += 4) {
        uint64_t prev; memcpy(&prev, (const void*)(ring + off), 8);
        uint32_t run = 1, start = off;
        for (uint32_t p = off + rec; p + 8 <= span; p += rec) {
            uint64_t s; memcpy(&s, (const void*)(ring + p), 8);
            if (s == prev + 1) { if (++run >= ALIGN_NEED) return start; }
            else               { run = 1; start = p; }
            prev = s;
        }
    }
    return NO_ALIGN;                             // no clean run (yet)
}

static int stream_session(struct session *s, int cfd, struct fcfb_req *req){
    uint8_t  rbuf[sizeof(struct fcfb_req)];
    size_t   rlen = 0;
    // CIRCULAR DMA (3c): the DMA never stops -- it writes the record byte-stream
    // continuously around the ring, so next_address just advances and wraps. We
    // track everything as an ABSOLUTE byte offset into that infinite stream:
    // stream byte P lives at ring[P % RING_SIZE]. Align to a record boundary ONCE
    // (records are contiguous forever after, straddling the ring end), then chase
    // `produced - DRAIN_LAG`, forwarding whole records. There is no re-arm, no
    // per-wrap re-lock, and DRAIN_LAG trailing is uniform across the wrap.
    uint64_t read_abs  = 0;                      // absolute byte offset to forward
    uint64_t laps      = 0;                      // ring wraps seen (absolute)
    uint32_t last_off  = RING_BASE;              // previous next_address (ring off)
    uint32_t pl_laps_prev = 0;                   // previous PL dma_laps reading
    int      laps_init = 0;                       // pl_laps_prev seeded yet?
    uint64_t produced  = 0;                       // last ACCEPTED committed count
    uint64_t align_off = 0;                       // record-boundary phase (abs)
    uint64_t resyncs   = 0;                       // frontier resyncs (dropped backlog)
    int      aligned   = 0;
    struct dgram_batch batch; batch.n = 0;        // Step 1: sendmmsg egress batch
    struct zcudp_batch zbatch; zbatch.n = 0;      // -E zcudp: in-kernel egress batch
    // Keep read_abs within a ring of the write frontier. The DMA overwrites a
    // ring position once `produced` laps back to it (produced - read_abs >=
    // RING_SIZE), so if the consumer ever falls that far behind, forwarding
    // reads being-overwritten bytes -> a de-frame. Resync to the live frontier
    // well before that (keep a >=1/4-ring overwrite margin), dropping the stale
    // backlog as a clean seq gap instead. This also discards the one-time
    // startup backlog: the DMA free-runs while the record-boundary lock scans,
    // so by the time we align, `produced` is already ~a ring ahead of offset 0.
    const uint64_t RESYNC_LAG = RING_SIZE - RING_SIZE / 4;

    for (;;) {
        if (g_stop) return -1;
        diag_tick(s);                            // -D: per-second egress report

        // 1. Service control channel (retune / disconnect) without blocking.
        int c = poll_control(cfd, req, rbuf, &rlen);
        if (c < 0) return 0;                     // clean disconnect
        if (c > 0) return 1;                     // retune pending in *req

        // 2. Read the DMA write pointer and the ring-lap count.
        uint32_t next;
        if (g_pl_laps) {
            // The PL counts every ring wrap on its own clock (it sees
            // next_address on EVERY edge), so the lap count is exact -- there is
            // no software wrap heuristic to alias near the ring end and
            // misregister the stream by a lap (the de-frame). Read {laps, next,
            // laps} coherently; a differing re-read means the poll straddled a
            // wrap between the two register reads (astronomically rare) -- retry.
            uint32_t l0 = read_reg2(R_DMA_LAPS);
            next        = read_dma_next();
            uint32_t l1 = read_reg2(R_DMA_LAPS);
            if (l0 != l1) { usleep(50); continue; }        // wrap straddle; retry
            if (next < RING_BASE || next > RING_END) { usleep(200); continue; }
            if (!laps_init) { pl_laps_prev = l0; laps_init = 1; }
            laps += (uint32_t)(l0 - pl_laps_prev);         // delta: origin-agnostic
            pl_laps_prev = l0;
        } else {
            // Legacy software heuristic (n_dds>=2 bench builds lack dma_laps):
            // a large backward step of next_address is treated as a wrap.
            next = read_dma_next();
            if (next < RING_BASE || next > RING_END) { usleep(200); continue; }
            if (next < last_off && (uint32_t)(last_off - next) > RING_SIZE / 2)
                laps++;
        }
        last_off = next;

        // 3. Absolute committed-byte count. With the PL lap count EXACT, `p` is
        //    the true committed position, so keep it strictly monotone and NEVER
        //    reject a large forward step: at these rates a single poll can
        //    legitimately advance a large fraction of the ring (a poll that
        //    scheduled out, or the gap while the record-boundary lock scans), so
        //    a magnitude cap would wedge the chase. A rare backward glitch in the
        //    next_address read is simply ignored (the exact laps mean a real wrap
        //    never looks backward); the frontier guard (4b) bounds how far behind
        //    read_abs may fall, so a stale-high read cannot cause a de-frame.
        uint64_t p = laps * (uint64_t)RING_SIZE + (next - RING_BASE);
        if (p > produced) produced = p;                    // monotone; ignore backward
        uint64_t safe = produced > DRAIN_LAG ? produced - DRAIN_LAG : 0;

        // 4. One-time record-boundary lock (drops the sub-record FIFO lead-in).
        //    Scan only the first lap's landed bytes; after this the framing holds
        //    forever (no re-arm ever re-introduces residue).
        if (!aligned) {
            if (safe < (uint64_t)s->rec * (ALIGN_NEED + 1)) { usleep(200); continue; }
            uint32_t landed = safe > RING_SIZE ? RING_SIZE : (uint32_t)safe;
            uint32_t off = find_alignment(s->rec, landed);
            if (off == NO_ALIGN) { usleep(200); continue; }
            align_off = off;                     // record-boundary phase (lap-0 offset)
            aligned = 1;
            // Start at the LIVE frontier, not offset `off`: the DMA has run far
            // ahead of the ring start while we were locking, so forwarding from
            // `off` would chase ~a ring of being-overwritten data (the de-frame).
            read_abs = align_off + ((safe - align_off) / s->rec) * s->rec;
        }

        // 4b. Overwrite guard: if the consumer fell too far behind (sustained
        //     overload, or a scheduling stall), skip forward to the frontier so
        //     we never forward records the PL is overwriting. Drops appear to the
        //     client as a seq gap, never as corruption.
        if (produced - read_abs > RESYNC_LAG) {
            if (resyncs < 3 || (resyncs % 1000) == 0)
                fprintf(stderr, "fcfb_server: frontier resync #%llu (behind %llu B) "
                        "-- consumer overloaded; dropping stale backlog\n",
                        (unsigned long long)resyncs,
                        (unsigned long long)(produced - read_abs));
            read_abs = align_off + ((safe - align_off) / s->rec) * s->rec;
            resyncs++;
            g_diag_resyncs++;                    // -D: mirror for the per-second report
        }

        // 5. Forward whole records in [read_abs, safe) as UDP datagrams, mapping
        //    each to its ring position (batch_add splits at the ring end). Datagrams
        //    accumulate into `batch` and go out ~VLEN at a time via one sendmmsg
        //    (Step 1). CAP the drain per poll: forwarding a large backlog in one
        //    shot lets the DMA lap the ring UNDER us mid-drain (the frontier check
        //    at 4b only runs between polls), forwarding being-overwritten data.
        //    Bound it to FWD_BUDGET datagrams and re-poll -- the 4b guard then
        //    re-checks the live frontier every ~chunk bytes, long before overwrite.
        int did_work = 0;
        uint32_t budget = FWD_BUDGET;            // datagrams before re-polling
        while (budget-- && safe > read_abs && safe - read_abs >= s->rec) {
            uint64_t avail = (safe - read_abs) / s->rec;
            uint32_t nrec  = avail > s->recs_per_dgram ? s->recs_per_dgram : (uint32_t)avail;
            if (g_egress == EGRESS_ZCUDP)
                batch_add_zcudp(&zbatch, s, (uint32_t)(read_abs % RING_SIZE), nrec);
            else
                batch_add(&batch, s, (uint32_t)(read_abs % RING_SIZE), nrec);
            read_abs += (uint64_t)nrec * s->rec;
            did_work = 1;
        }
        // flush the partial batch before re-poll (whichever backend is active)
        if (g_egress == EGRESS_ZCUDP) { if (zbatch.n) batch_flush_zcudp(&zbatch, s); }
        else                         { if (batch.n)  batch_flush(&batch, s); }

        if (!did_work) usleep(200);              // idle: yield, keep latency low
    }
}

// --------------------------------------------------------------- admit+program
// Program the FPGA for a request and (re)arm/enable the datapath. Fills the
// session + accept header. Returns 0 on success, -1 if admission fails.
static int program(struct fcfb_req *req, const struct fcfb_req_ext *ext,
                   struct session *s, uint32_t peer_ip, struct fcfb_accept *acc,
                   long guard){
    uint32_t k0, W;
    if (fcfb_admit(req->f_lo, req->f_hi, guard, &k0, &W) != 0) return -1;
    uint32_t mask = req->adc_mask & 0x3;
    if (mask == 0) mask = 0x1;                   // default ADC0
    // Two-window (v2) verification: a second window on the SAME ADC, appended to
    // the record as an ascending A-run (W0 then W1). Admit it and force a single
    // ADC (record symmetry: record_size assumes each present ADC carries W bins,
    // so both windows must sit on one ADC and W = W0 + W1, W_other = 0).
    uint32_t k0b = 0, Wb = 0, two_win = 0;
    if (ext && (ext->f_lo2 != 0.0 || ext->f_hi2 != 0.0)) {
        if (fcfb_admit(ext->f_lo2, ext->f_hi2, guard, &k0b, &Wb) != 0) return -1;
        if (k0b <= k0) return -1;                // require ascending, non-overlap
        if (k0 + W - 1 >= k0b) return -1;
        mask = 0x1;                              // both windows on ADC A
        two_win = 1;
    }
    uint32_t def_shift = (g_bin_width == 24) ? DEF_SHIFT24 : DEF_SHIFT;
    uint32_t shift = (req->shift < 0) ? def_shift : (uint32_t)req->shift;
    if (shift > 31) shift = 31;

    // teardown any prior run before reprogramming (safe; no sdr_reset touch)
    datapath_off();

    uint32_t Wtot = two_win ? (W + Wb) : W;      // total A-run bins in the record
    if (Wtot > g_wmax) {                          // wider than the PL bank buffers
        fprintf(stderr, "reject: W=%u > wmax=%u (PL would truncate); set -w to "
                "the bitstream's wmax\n", Wtot, g_wmax);
        return -1;
    }
    // GbE transport budget: total STREAMED bins across both ADCs (this path keeps
    // the same Wtot on every present ADC) must fit the loss-free wire ceiling.
    uint32_t nbins = fcfb_popcount2(mask) * Wtot;
    uint32_t budget = fcfb_bin_budget_mtu(g_bin_width, g_mtu);
    if (nbins > budget) {
        fprintf(stderr, "reject: %u bins > GbE budget %u (bin_width=%u); the wire "
                "cannot sustain this loss-free\n", nbins, budget, g_bin_width);
        return -1;
    }
    s->k0 = k0; s->W = Wtot; s->adc_mask = mask; s->shift = shift;
    s->rec = fcfb_record_size(mask, Wtot, g_bin_width);
    // records per datagram: fill ~DGRAM_TARGET but never exceed DGRAM_MAX; at
    // least one record even if a single record already exceeds the target.
    uint32_t rpd = (g_dgram_target - sizeof(struct fcfb_udp_hdr)) / s->rec;
    uint32_t cap = (DGRAM_MAX     - sizeof(struct fcfb_udp_hdr)) / s->rec;
    if (rpd < 1) rpd = 1;
    if (rpd > cap) rpd = cap;
    s->recs_per_dgram = rpd ? rpd : 1;
    s->run_gen++;

    // program: keep datapath disabled, load the keep-bitmap for this window(s),
    // set quantiser shift + DDS (tone or ADC input).
    wr(R_RUN, 0);                                // enable=0 while (re)programming
    if (two_win)
        mask_program2(k0, W, k0b, Wb, mask);     // clear + load both windows on A
    else
        mask_program(k0, W, mask);               // clear + load k0..k0+W-1
    wr(R_QUANT, shift);
    // DDS: raw off-grid phase_inc overrides on-bin dds_k; either enables the tone
    // (DDS_ENABLE); neither leaves the live ADC selected.  In two-window (v2) mode
    // dds2 (0x38) injects window-1's tone; it exists only on the n_dds>=2 bitstream
    // (a write is a harmless no-op on n_dds=1).
    wr(R_DDS, dds_word(req->dds_k, req->dds_phase_inc));
    if (two_win)
        wr(R_DDS2, dds_word(ext->dds2_k, ext->dds2_phase_inc));

    // Realign the free-running two-rate WOLA fold (+ InCDC, DDS phase, egress)
    // to the fresh-boot condition on every (re)program by pulsing the soft
    // datapath reset (control bit1), with the run still disabled and the just-
    // written run/quant/dds programming preserved.  Unlike sdr_reset this does
    // NOT reset the RegisterCDC / fcfb_registers / DMA, so it cannot wedge the
    // AXI bus or disturb the AXI-HP0 DMA burst state.  Without it, only the
    // first capture after start-project is valid: the free-running fold keeps a
    // stale block phase across a retune (frequency-scaling error + static spur).
    wr(R_CONTROL, DP_RESET);                     // assert (sdr_reset stays 0)
    usleep(1000);
    wr(R_CONTROL, 0);                            // release
    usleep(1000);

    // arm DMA at ring base, then enable the datapath
    wr(R_DMA_CTL, DMA_START);
    wr(R_RUN,     RUN_ENABLE);

    // UDP destination = TCP peer IP + requested port
    memset(&s->dst, 0, sizeof s->dst);
    s->dst.sin_family = AF_INET;
    s->dst.sin_addr.s_addr = peer_ip;
    s->dst.sin_port = htons(req->udp_port);

    // accept header (byte-identical to fcfb_stream.h; nblocks=0 => streaming).
    // int24 (bin_width=24) advertises stream version 2 + the bin_width field;
    // int16 stays version 1 (the trailing field is not sent).
    memcpy(acc->magic, "FCFBv1\0\0", 8);
    acc->version = (g_bin_width == 24) ? FCFB_STREAM_V2 : FCFB_STREAM_V1;
    acc->N = FCFB_N; acc->fs = FCFB_FS;
    // k0 = window-0 base; W = total A-run bins (W0+W1 in two-window mode). The
    // record carries window-0's W0 bins then window-1's W1 bins in that order.
    acc->k0 = k0; acc->W = Wtot; acc->adc_mask = mask; acc->nblocks = 0;
    acc->bin_scale = (double)(1u << shift);
    acc->bin_width = g_bin_width;

    if (two_win)
        fprintf(stderr, "program: two-window k0=%u W0=%u | k0b=%u W1=%u (Wtot=%u) "
                        "mask=0x%x shift=%u binw=%u rec=%uB dds_k=%u dds2_k=%u "
                        "udp=%s:%u gen=%u\n", k0, W, k0b, Wb, Wtot, mask, shift,
                g_bin_width, s->rec, req->dds_k, ext->dds2_k,
                inet_ntoa(*(struct in_addr*)&peer_ip), req->udp_port, s->run_gen);
    else
        fprintf(stderr, "program: k0=%u W=%u mask=0x%x shift=%u binw=%u rec=%uB "
                        "dds_k=%u dds_inc=%u udp=%s:%u gen=%u\n", k0, W, mask, shift,
                g_bin_width, s->rec, req->dds_k, req->dds_phase_inc,
                inet_ntoa(*(struct in_addr*)&peer_ip), req->udp_port, s->run_gen);
    return 0;
}

// v3 multi-channel program: admit each channel to a bin run, OR them into the
// per-ADC keep-bitmap, inject channel i's tone on DDS generator i (i<3), and
// build the variable-length v3 accept (fixed header + run-list) into `hdrbuf`
// (>= FCFB_HDR_V3_FIXED + FCFB_MAX_CHAN*sizeof(fcfb_run)); *hdr_len gets its size.
// Returns 0 on success, -1 if the channel count or any admission fails.
static int program_v3(const struct fcfb_chan *chans, uint32_t nchan, int req_shift,
                      uint16_t udp_port, struct session *s, uint32_t peer_ip,
                      uint8_t *hdrbuf, uint32_t *hdr_len, long guard){
    static uint8_t keep[FCFB_N];
    struct fcfb_run runs[FCFB_MAX_CHAN];
    if (nchan == 0 || nchan > FCFB_MAX_CHAN) return -1;

    uint32_t adc_union = 0;
    for (uint32_t i = 0; i < nchan; i++) {
        uint32_t k0, W;
        if (fcfb_admit(chans[i].f_lo, chans[i].f_hi, guard, &k0, &W) != 0) return -1;
        uint32_t ab = chans[i].adc_bits ? (chans[i].adc_bits & 0x3) : 0x1;
        runs[i].k0 = k0; runs[i].W = W; runs[i].adc_bits = ab;
        adc_union |= ab;
    }
    uint32_t def_shift = (g_bin_width == 24) ? DEF_SHIFT24 : DEF_SHIFT;
    uint32_t shift = (req_shift < 0) ? def_shift : (uint32_t)req_shift;
    if (shift > 31) shift = 31;

    datapath_off();
    s->run_gen++;

    // program: keep datapath disabled, load the union keep-bitmap, set shift + the
    // per-channel DDS tones (generator i for channel i; unused generators cleared).
    wr(R_RUN, 0);
    uint32_t W_a, W_b;
    mask_program_runs(runs, nchan, keep, &W_a, &W_b);
    if (W_a > g_wmax || W_b > g_wmax) {           // wider than the PL bank buffers
        fprintf(stderr, "reject: W_a=%u W_b=%u > wmax=%u (PL would truncate); set "
                "-w to the bitstream's wmax\n", W_a, W_b, g_wmax);
        return -1;                                // datapath left disabled
    }
    // GbE transport budget: total streamed bins W_a+W_b (the union popcount, what
    // record_size_ab sizes) must fit the loss-free wire ceiling.
    uint32_t budget = fcfb_bin_budget_mtu(g_bin_width, g_mtu);
    if (W_a + W_b > budget) {
        fprintf(stderr, "reject: %u bins (W_a=%u+W_b=%u) > GbE budget %u "
                "(bin_width=%u); the wire cannot sustain this loss-free\n",
                W_a + W_b, W_a, W_b, budget, g_bin_width);
        return -1;                                // datapath left disabled
    }
    wr(R_QUANT, shift);
    // Per-channel DDS tones on generator i.  CRITICAL: on the production/streaming
    // build (g_pl_laps) slots 0x38/0x3C are R_DMA_BASE/R_DMA_LAPS, NOT DDS2/DDS3 --
    // writing them here clobbers the PL DMA write-base and makes the DMA scribble
    // over physical memory, wedging the board (the network dies entirely).  Only
    // gen 0 (R_DDS @ 0x30) is a real DDS on that build; the n_dds>=2 bench build
    // (-L, g_pl_laps==0) has all three and no dma_laps.  Mirror program()'s proven
    // rule (it writes 0x38 only in two-window/-L-era mode).  DDS is a bench feature;
    // real multichannel captures request no tone, so dropping ch>=1 tones here on a
    // production build is fine.
    uint32_t n_gen = g_pl_laps ? 1u : 3u;
    for (uint32_t i = 0; i < n_gen; i++)
        wr(R_DDS_REG[i], (i < nchan)
           ? dds_word(chans[i].dds_k, chans[i].dds_phase_inc) : 0u);

    wr(R_CONTROL, DP_RESET); usleep(1000);       // realign the two-rate fold
    wr(R_CONTROL, 0);        usleep(1000);
    wr(R_DMA_CTL, DMA_START);
    wr(R_RUN,     RUN_ENABLE);

    s->k0 = runs[0].k0; s->W = W_a + W_b; s->W_a = W_a; s->W_b = W_b;
    s->adc_mask = adc_union; s->shift = shift;
    s->rec = fcfb_record_size_ab(W_a, W_b, g_bin_width);
    uint32_t rpd = (g_dgram_target - sizeof(struct fcfb_udp_hdr)) / s->rec;
    uint32_t cap = (DGRAM_MAX     - sizeof(struct fcfb_udp_hdr)) / s->rec;
    if (rpd < 1) rpd = 1;
    if (rpd > cap) rpd = cap;
    s->recs_per_dgram = rpd ? rpd : 1;

    memset(&s->dst, 0, sizeof s->dst);
    s->dst.sin_family = AF_INET;
    s->dst.sin_addr.s_addr = peer_ip;
    s->dst.sin_port = htons(udp_port);

    // Build the v3 accept: fixed header + the run-list.
    struct fcfb_accept_v3 h; memset(&h, 0, sizeof h);
    memcpy(h.magic, "FCFBv1\0\0", 8);
    h.version = FCFB_STREAM_V3; h.N = FCFB_N; h.fs = FCFB_FS;
    h.adc_mask = adc_union; h.W_a = W_a; h.W_b = W_b; h.nblocks = 0;
    h.bin_scale = (double)(1u << shift); h.bin_width = g_bin_width; h.nruns = nchan;
    memcpy(hdrbuf, &h, sizeof h);
    memcpy(hdrbuf + sizeof h, runs, nchan * sizeof runs[0]);
    *hdr_len = fcfb_hdr_size_v3(nchan);

    fprintf(stderr, "program v3: nchan=%u W_a=%u W_b=%u adc=0x%x shift=%u binw=%u "
                    "rec=%uB udp=%s:%u gen=%u\n", nchan, W_a, W_b, adc_union, shift,
            g_bin_width, s->rec, inet_ntoa(*(struct in_addr*)&peer_ip), udp_port,
            s->run_gen);
    return 0;
}

// Read one hex/dec value from a u-dma-buf sysfs attribute. Returns 0 on success.
static int udmabuf_sysfs(const char *name, const char *attr, unsigned long long *out){
    char path[128];
    snprintf(path, sizeof path, "/sys/class/u-dma-buf/%s/%s", name, attr);
    FILE *fp = fopen(path, "r");
    if (!fp) return -1;
    int n = fscanf(fp, "%lli", (long long*)out);   // phys_addr is 0x-prefixed hex
    fclose(fp);
    return (n == 1) ? 0 : -1;
}

// Map the DMA egress ring. Prefer the page-backed u-dma-buf (Option B): read its
// physical base+size from sysfs and mmap /dev/udmabuf0, so the ring lives in real
// struct pages the NIC can zero-copy (MSG_ZEROCOPY) from and the caller can point
// the PL DMA base at. Fall back to the legacy fixed /dev/mem reserved region when
// u-dma-buf is absent (an old image), preserving the pre-Option-B behavior.
// Sets *base/*size (physical) and *udmabuf (1 if the udmabuf path was used).
static volatile uint8_t *map_ring(int memfd, uint32_t *base, uint32_t *size,
                                  int *udmabuf){
    unsigned long long phys = 0, sz = 0;
    // The u-dma-buf module is an out-of-tree .ko that is not guaranteed to be
    // coldplug-autoloaded; load it ourselves (idempotent) so /dev/udmabuf0 and
    // its sysfs appear. Harmless on an old image with no such module.
    if (access("/dev/udmabuf0", F_OK) != 0) {
        // quirk_mmap_mode=4 (PAGE) is REQUIRED for MSG_ZEROCOPY (page-backed mmap
        // -> get_user_pages); also set by the DT option + modprobe.d, this is the
        // last-resort load path if the module was not already up.
        if (system("modprobe u-dma-buf quirk_mmap_mode=4 2>/dev/null") == 0)
            usleep(200000);                        // let the DT probe settle
    }
    if (udmabuf_sysfs("udmabuf0", "phys_addr", &phys) == 0 &&
        udmabuf_sysfs("udmabuf0", "size", &sz) == 0 && phys && sz) {
        int ufd = open("/dev/udmabuf0", O_RDWR);   // pages must be pinnable for TX
        if (ufd >= 0) {
            void *p = mmap(0, (size_t)sz, PROT_READ, MAP_SHARED, ufd, 0);
            if (p != MAP_FAILED) {
                *base = (uint32_t)phys; *size = (uint32_t)sz; *udmabuf = 1;
                fprintf(stderr, "fcfb_server: ring = /dev/udmabuf0 phys=0x%08x "
                        "size=%u MiB (zero-copy path)\n", *base, *size >> 20);
                return p;
            }
            close(ufd);
        }
        fprintf(stderr, "fcfb_server: /dev/udmabuf0 present but unmappable; "
                "falling back to /dev/mem\n");
    }
    void *p = mmap(0, RING_SIZE_DEFAULT, PROT_READ, MAP_SHARED, memfd,
                   RING_BASE_DEFAULT);
    *base = RING_BASE_DEFAULT; *size = RING_SIZE_DEFAULT; *udmabuf = 0;
    fprintf(stderr, "fcfb_server: ring = /dev/mem @0x%08x (legacy path, no "
            "udmabuf)\n", *base);
    return (p == MAP_FAILED) ? NULL : p;
}

// Find eth0's IRQ number by scanning /proc/interrupts for the line whose last
// whitespace-delimited token is "eth0" (its first token is "<irq>:"). Returns
// the IRQ number, or -1 if not found.
static int find_eth0_irq(void){
    FILE *f = fopen("/proc/interrupts", "r");
    if (!f) return -1;
    char line[512]; int irq = -1;
    while (fgets(line, sizeof line, f)) {
        char *nl = strchr(line, '\n'); if (nl) *nl = 0;
        char *save, *first = strtok_r(line, " \t", &save);
        if (!first) continue;
        char *last = first, *p;
        while ((p = strtok_r(NULL, " \t", &save))) last = p;
        if (!strcmp(last, "eth0")) { irq = atoi(first); break; }  // atoi("37:")==37
    }
    fclose(f);
    return irq;
}

// Force every core's cpufreq governor to "performance". On this Zynq-7010 the
// only two operating points are 333/667 MHz, and the default "ondemand"
// governor downclocks the cores to 333 MHz under the IRQ/softirq-heavy egress
// load (the completion softirq runs on CPU0, whose measured utilisation looks
// low even while it is busy). That drops GEM egress to ~35.6k dgram/s
// (~888 Mb/s), ~11% below the stream's fixed 40k blocks/s (~977 Mb/s) block
// rate; the 160 MiB udmabuf ring masks the deficit for ~12 s, then overflows
// into sustained ~9% loss (clean drop, back=0 big=0 -- NOT a de-frame). Pinning
// "performance" (both cores at 667 MHz) makes egress match the block rate and
// the stream runs loss-free at line rate. Both 7.1.7 and 7.2.3 default to
// ondemand, but they do NOT behave the same: under this exact load 7.1.7 keeps
// the cores at 667 MHz (HW-measured) while 7.2.3 drops them to 333 MHz -- a real
// kernel regression. The ondemand governor code is unchanged; its input
// get_cpu_idle_time() (tick-sched/cputime) changed on 7.2.3 so the IRQ/softirq-
// busy completion core now reads as under-loaded (< the 80% up_threshold) and
// gets downclocked. Forcing "performance" sidesteps the governor entirely.
static void set_performance_governor(void){
    int n = 0, ok = 0;
    for (int cpu = 0; cpu < 64; cpu++) {          // cpus are contiguous from 0
        char path[80];
        snprintf(path, sizeof path,
                 "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_governor", cpu);
        FILE *f = fopen(path, "w");
        if (!f) break;                            // past the last cpu (or no cpufreq)
        n++;
        if (fprintf(f, "performance\n") > 0) ok++;
        fclose(f);
    }
    if (n == 0)
        fprintf(stderr, "fcfb_server: WARN no cpufreq scaling_governor "
                "(governor left as-is; expect underruns near line rate)\n");
    else
        fprintf(stderr, "fcfb_server: CPU governor -> performance (%d/%d cores)\n",
                ok, n);
}

// -A: pin the (single) stream thread to g_srv_cpu and steer the eth0 IRQ to
// g_irq_cpu, so the TX-completion softirq and the sender never share a core;
// and force the "performance" governor so neither core is downclocked mid-stream.
static void apply_affinity(void){
    if (!g_affinity) return;
    set_performance_governor();
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(g_srv_cpu, &set);
    if (sched_setaffinity(0, sizeof set, &set) < 0)
        fprintf(stderr, "fcfb_server: WARN sched_setaffinity(CPU%d): %s\n",
                g_srv_cpu, strerror(errno));
    else
        fprintf(stderr, "fcfb_server: stream thread pinned to CPU%d\n", g_srv_cpu);
    int irq = find_eth0_irq();
    if (irq < 0) {
        fprintf(stderr, "fcfb_server: WARN eth0 IRQ not found in /proc/interrupts "
                "(leaving IRQ affinity untouched)\n");
        return;
    }
    char path[64]; snprintf(path, sizeof path, "/proc/irq/%d/smp_affinity", irq);
    FILE *f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "fcfb_server: WARN open %s: %s\n", path, strerror(errno));
        return;
    }
    fprintf(f, "%x\n", 1u << g_irq_cpu);
    fclose(f);
    fprintf(stderr, "fcfb_server: eth0 IRQ %d -> CPU%d (smp_affinity mask 0x%x)\n",
            irq, g_irq_cpu, 1u << g_irq_cpu);
}

static void print_usage(const char *argv0){
    printf(
"Usage: %s [tcp_port] [options]\n"
"  fcfb Stage-1 board server: TCP control + UDP bin-data streaming.\n"
"\n"
"  The fcfb PL MUST be loaded first ('start-project fcfb_stage1'); running\n"
"  before that wedges the AXI bus and hard-hangs the board. The server reads\n"
"  %s (written by start-project with the loaded gateware's\n"
"  name) and refuses to start unless it names the fcfb gateware, unless -F is\n"
"  given.\n"
"\n"
"Options:\n"
"  -h, --help        show this help and exit (touches nothing)\n"
"  -F                force: skip the loaded-gateware check\n"
"  -M <bytes>        UDP datagram fill target (default auto from -u)\n"
"  -b <16|24>        on-wire bin width; MUST match the loaded bitstream\n"
"  -w <wmax>         per-ADC kept-bin ceiling; MUST match the bitstream\n"
"  -u <mtu>          link MTU for the bin budget (default 1500)\n"
"  -E <sendmmsg|zcudp>  egress backend (default zcudp, falls back to sendmmsg)\n"
"  -Z / -C           force zero-copy / copy egress\n"
"  -A [cpu]          pin stream thread + steer eth0 IRQ (default on)\n"
"  --no-affinity     disable the default CPU pinning\n"
"  -S <bytes>        UDP SO_SNDBUF size\n"
"  -L                legacy SW ring-wrap heuristic (n_dds>=2 builds)\n"
"  -D                per-second egress diagnostics to stderr\n"
"  tcp_port          positional TCP control port (default %d)\n",
        argv0, FCFB_CURRENT_GATEWARE, FCFB_TCP_PORT);
}

int main(int argc, char **argv){
    // Args: [tcp_port] [-M dgram_target_bytes]. Port stays positional for
    // backward compat (`./fcfb_server 7373`). LEAVE -M AT DEFAULT 1400 on the
    // stock kernel: raising it only helps with npapi's macbenet jumbo driver;
    // otherwise the datagram just IP-fragments and loss gets worse (see above).
    int port = FCFB_TCP_PORT;
    int m_explicit = 0;                          // -M given? (else auto from -u MTU)
    int force = 0;                               // -F: skip the gateware guard-file check
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            print_usage(argv[0]);                // help ONLY: no /dev/mem, no AXI, no PL
            return 0;
        } else if (!strcmp(argv[i], "-F")) {
            force = 1;                            // caller certifies the PL is loaded
        } else if (!strcmp(argv[i], "-M") && i + 1 < argc) {
            long v = atol(argv[++i]);
            if (v < (long)sizeof(struct fcfb_udp_hdr) + 8 || v > (long)DGRAM_MAX) {
                fprintf(stderr, "-M %ld out of range [%zu, %u]\n", v,
                        sizeof(struct fcfb_udp_hdr) + 8, DGRAM_MAX);
                return 2;
            }
            g_dgram_target = (uint32_t)v;
            m_explicit = 1;
        } else if (!strcmp(argv[i], "-b") && i + 1 < argc) {
            long v = atol(argv[++i]);            // on-wire bin width; MATCH the PL
            if (v != 16 && v != 24) {
                fprintf(stderr, "-b %ld invalid (must be 16 or 24)\n", v);
                return 2;
            }
            g_bin_width = (uint32_t)v;
        } else if (!strcmp(argv[i], "-w") && i + 1 < argc) {
            long v = atol(argv[++i]);            // per-ADC wmax; MATCH the PL
            if (v < 1 || v > (long)FCFB_WMAX) {
                fprintf(stderr, "-w %ld out of range [1, %u]\n", v, FCFB_WMAX);
                return 2;
            }
            g_wmax = (uint32_t)v;
        } else if (!strcmp(argv[i], "-u") && i + 1 < argc) {
            long v = atol(argv[++i]);            // link MTU for the bin budget
            if (v < 576 || v > 16320) {          // min IPv4 MTU .. GEM RX_BUFFER_MAX
                fprintf(stderr, "-u %ld out of range [576, 16320]\n", v);
                return 2;
            }
            g_mtu = (uint32_t)v;                 // MUST match the actual eth0 MTU
        } else if (!strcmp(argv[i], "-Z")) {
            g_zerocopy = 1;                      // force zero-copy (default anyway)
        } else if (!strcmp(argv[i], "-C")) {
            g_zerocopy = 0;                      // force copy egress (A/B / legacy)
        } else if (!strcmp(argv[i], "-E") && i + 1 < argc) {
            const char *m = argv[++i];           // egress backend
            if (!strcmp(m, "zcudp"))        g_egress = EGRESS_ZCUDP;
            else if (!strcmp(m, "sendmmsg")) g_egress = EGRESS_SENDMMSG;
            else { fprintf(stderr, "-E %s invalid (sendmmsg|zcudp)\n", m); return 2; }
        } else if (!strcmp(argv[i], "-L")) {
            g_pl_laps = 0;                       // legacy SW wrap heuristic
        } else if (!strcmp(argv[i], "--no-affinity")) {
            g_affinity = 0;                      // opt out of the default pinning
        } else if (!strcmp(argv[i], "-A")) {
            g_affinity = 1;                      // pin stream thread + steer eth0 IRQ
            if (i + 1 < argc && argv[i+1][0] >= '0' && argv[i+1][0] <= '9') {
                int c = atoi(argv[++i]);         // stream CPU; IRQ goes to a diff core
                g_srv_cpu = c;
                g_irq_cpu = (c == 0) ? 1 : 0;
            }
        } else if (!strcmp(argv[i], "-D")) {
            g_diag = 1;                          // per-second egress diagnostics
        } else if (!strcmp(argv[i], "-S") && i + 1 < argc) {
            long v = atol(argv[++i]);            // UDP SO_SNDBUF bytes
            if (v < 65536 || v > (long)256*1024*1024) {
                fprintf(stderr, "-S %ld out of range [65536, 268435456]\n", v);
                return 2;
            }
            g_sndbuf = (uint32_t)v;
        } else {
            port = atoi(argv[i]);                // positional tcp port
        }
    }
    // Step 3: on a jumbo link (MTU above standard 1500-B Ethernet) with no explicit
    // -M, fill each datagram to the full MTU payload (mtu - 28 B IP+UDP hdr) so mid-W
    // runs pack MULTIPLE records per datagram. The send ceiling is packets/s (~31k
    // in the good mode), so halving datagrams/s where 2 records fit (W<=~190 @ 2340,
    // W<=327 @ 3980) makes that range loss-free (40k blk/s -> 20k dgram/s < 31k). On a
    // stock 1500 link we leave the tested-best 1400 (raising it there only IP-frags).
    // NB: the threshold is the literal 1500, NOT FCFB_MTU (which the SD build sets to
    // 3980) -- else a 3980-default binary at MTU 3980 would skip packing.
    if (!m_explicit && g_mtu > 1500u) {
        uint32_t t = g_mtu - 28u;                // max UDP payload that fits one frame
        if (t > DGRAM_MAX) t = DGRAM_MAX;
        g_dgram_target = t;
        fprintf(stderr, "fcfb_server: -M auto = %u B (MTU %u payload; pack "
                "records/datagram to cut pps)\n", g_dgram_target, g_mtu);
    }
    fprintf(stderr, "fcfb_server: bin_width=%u (stream v%u), wmax=%u, mtu=%u "
            "(budget %u bins); -b/-w MUST match the loaded bitstream, -u the eth0 "
            "MTU\n", g_bin_width, g_bin_width == 24 ? 2 : 1, g_wmax, g_mtu,
            fcfb_bin_budget_mtu(g_bin_width, g_mtu));
    // 42 B of Eth+IP+UDP header precede the UDP payload on the wire; warn if the
    // frame would exceed the board GEM's ~4 KiB TX buffer and fragment.
    if (g_dgram_target + 42u > 3994u)
        fprintf(stderr, "fcfb_server: WARNING -M %u -> ~%u-B frame > board GEM "
                "~4 KiB ceiling; expect IP fragmentation (use -M %u)\n",
                g_dgram_target, g_dgram_target + 42u, DGRAM_JUMBO);
    fprintf(stderr, "fcfb_server: dgram_target=%u B%s\n", g_dgram_target,
            g_dgram_target > DGRAM_TARGET ? " (jumbo)" : "");

    // SIGINT/SIGTERM must INTERRUPT the blocking accept() so the g_stop loop can
    // exit. signal() installs handlers with SA_RESTART on Linux -> accept() would
    // auto-restart and never see g_stop (the process then only dies to SIGKILL);
    // sigaction with sa_flags=0 makes accept() return EINTR (handled in the loop).
    struct sigaction sact = {0};
    sact.sa_handler = on_sig;                     // sa_flags = 0 -> NO SA_RESTART
    sigaction(SIGINT,  &sact, NULL);
    sigaction(SIGTERM, &sact, NULL);
    signal(SIGPIPE, SIG_IGN);

    // Refuse to touch the AXI bus unless start-project recorded that the fcfb
    // gateware is loaded. The first PL register read (R_PRODUCT, below) with no (or
    // the wrong) bitstream loaded wedges the AXI bus and hard-hangs the board (power
    // cycle to recover). start-project writes the loaded gateware's name to
    // FCFB_CURRENT_GATEWARE; a fresh boot has neither the PL nor (tmpfs) the record.
    // -F overrides for the rare case the fcfb PL is known good but the record is
    // missing (e.g. an older start-project that predates it).
    if (!force) {
        char gw[128] = {0};
        FILE *gf = fopen(FCFB_CURRENT_GATEWARE, "r");
        if (!gf) {
            fprintf(stderr,
                "fcfb_server: no gateware loaded -- %s absent.\n"
                "  Run 'start-project fcfb_stage1' first: a fresh boot has no PL and\n"
                "  reading a PL register would wedge the AXI bus and hang the board.\n"
                "  (Use -F to override if you are certain the fcfb PL is loaded.)\n",
                FCFB_CURRENT_GATEWARE);
            return 1;
        }
        if (!fgets(gw, sizeof gw, gf)) gw[0] = '\0';
        fclose(gf);
        gw[strcspn(gw, "\r\n")] = '\0';          // strip trailing newline
        if (strncmp(gw, FCFB_GATEWARE_PREFIX, strlen(FCFB_GATEWARE_PREFIX)) != 0) {
            fprintf(stderr,
                "fcfb_server: loaded gateware '%s' is not the fcfb gateware (expected\n"
                "  a name starting with '%s'). Run 'start-project fcfb_stage1' first;\n"
                "  talking to non-fcfb registers would wedge the AXI bus.\n"
                "  (Use -F to override if you are certain the fcfb PL is loaded.)\n",
                gw, FCFB_GATEWARE_PREFIX);
            return 1;
        }
        fprintf(stderr, "fcfb_server: gateware '%s' loaded (per %s)\n",
                gw, FCFB_CURRENT_GATEWARE);
        snprintf(g_gateware_name, sizeof g_gateware_name, "%s", gw);
    }

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    regs = mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, REG_BASE);
    if (regs == MAP_FAILED) { perror("mmap regs"); return 1; }
    // Ring: page-backed u-dma-buf (Option B / zero-copy) or the legacy reserved
    // region. Sets g_ring_base/g_ring_size (physical) used by the chase math.
    int ring_udmabuf = 0;
    ring = map_ring(fd, &g_ring_base, &g_ring_size, &ring_udmabuf);
    if (!ring) { perror("mmap ring"); return 1; }

    if (rd(R_PRODUCT) != PRODUCT_ID) {
        fprintf(stderr, "product_id=0x%08x (expected 0x%08x) -- is the PL loaded "
                "(start-project fcfb_stage1)?\n", rd(R_PRODUCT), PRODUCT_ID);
        return 1;
    }
    // Clear sdr_reset ONCE (brings the sync domain out of reset). Never re-assert.
    wr(R_CONTROL, 0); usleep(1000);
    // Program the runtime PL DMA base = the ring's physical address, BEFORE any
    // dma_start. Held in the sync-domain RegisterCDC (dp_reset does not clear it).
    // On a bitstream too old to have dma_base, slot 0b110 reads back as R_DDS2 and
    // this write is a harmless DDS phase_inc=base (DDS disabled) -- but such a
    // bitstream also bakes the fixed 0x10000000 base, so pair it with the /dev/mem
    // fallback (g_ring_base == RING_BASE_DEFAULT), which this write then matches.
    wr(R_DMA_BASE, g_ring_base);
    fprintf(stderr, "fcfb_server: PL dma_base <- 0x%08x%s\n", g_ring_base,
            ring_udmabuf ? " (udmabuf)" : " (legacy)");
    fprintf(stderr, "fcfb_server: ring-wrap tracking = %s\n", g_pl_laps
            ? "PL dma_laps (exact)" : "legacy SW heuristic (-L; n_dds>=2 builds)");

    int lfd = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1; setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in sa = {0};
    sa.sin_family = AF_INET; sa.sin_addr.s_addr = INADDR_ANY;
    sa.sin_port = htons((uint16_t)port);
    if (bind(lfd, (struct sockaddr*)&sa, sizeof sa) < 0) { perror("bind"); return 1; }
    if (listen(lfd, 1) < 0) { perror("listen"); return 1; }
    fprintf(stderr, "fcfb_server: listening on TCP :%d (UDP data to client)\n", port);

    // Board-info (BINF) responder on its own thread + port, so a client can poll the
    // board's temperature/voltages even while this single-session control port is busy
    // streaming. The thread only reads XADC sysfs + its own socket (never the AXI/DMA
    // datapath), so it is safe alongside streaming. Non-fatal if it fails to start.
    pthread_t info_tid;
    if (pthread_create(&info_tid, NULL, info_thread, NULL) != 0)
        fprintf(stderr, "warning: board-info responder thread failed to start\n");
    else
        pthread_detach(info_tid);

    int udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) { perror("socket udp"); return 1; }
    // Enlarge the socket send buffer. The default wmem (~176 KB) holds only ~tens of
    // ~2 KB datagrams, so on the COPY path sendmmsg blocks after ~1 datagram (each is
    // copied into sndbuf) and never batches; a big sndbuf lets a full VLEN batch queue
    // before the NIC drains it. We are root -> SO_SNDBUFFORCE bypasses wmem_max.
    // Opt-in (-S) only: a big sndbuf was measured to REGRESS throughput here.
    if (g_sndbuf) {
        int snd = (int)g_sndbuf;
        if (setsockopt(udp_fd, SOL_SOCKET, SO_SNDBUFFORCE, &snd, sizeof snd) < 0)
            setsockopt(udp_fd, SOL_SOCKET, SO_SNDBUF, &snd, sizeof snd);
        int got = 0; socklen_t gl = sizeof got;
        getsockopt(udp_fd, SOL_SOCKET, SO_SNDBUF, &got, &gl);
        fprintf(stderr, "fcfb_server: SO_SNDBUF requested %d -> %d B (kernel doubles "
                "for bookkeeping)\n", snd, got);
    }
    if (g_egress == EGRESS_ZCUDP) {
        // In-kernel zcudp path: the datagrams never touch a socket, so SO_ZEROCOPY
        // is irrelevant; reuse udp_fd as the ioctl channel to macb (any AF_INET
        // dgram fd works) + for the ARP warm-up send. The ring phys window is armed
        // per session in zcudp_arm_session.
        g_zcudp_fd = udp_fd;
        struct zcudp_stats st;                   // probe: is the zcudp kernel present?
        if (zcudp_read_stats(&st) < 0) {
            // No macb_zcudp kernel -> gracefully use the sendmmsg path instead so a
            // default run still works on a stock kernel. -E zcudp lands here too.
            fprintf(stderr, "fcfb_server: zcudp unavailable (%s: macb_zcudp kernel "
                    "not present) -- falling back to sendmmsg egress\n",
                    strerror(errno));
            g_egress = EGRESS_SENDMMSG;
            g_zcudp_fd = -1;
        } else {
            if (!ring_udmabuf)
                fprintf(stderr, "fcfb_server: WARNING zcudp on the /dev/mem fallback "
                        "ring (0x%08x) -- verify the GEM can DMA it; udmabuf is the "
                        "tested path\n", g_ring_base);
            fprintf(stderr, "fcfb_server: egress = zcudp (in-kernel zero-copy UDP TX)\n");
        }
    }
    if (g_egress == EGRESS_SENDMMSG) {
    if (g_zerocopy && !ring_udmabuf) {
        // /dev/mem fallback is VM_PFNMAP -> get_user_pages can't pin it; copy
        // egress is correct there (the O_SYNC mapping is uncached), so use it.
        g_zerocopy = 0;
    } else if (g_zerocopy) {
        int one = 1;
        if (setsockopt(udp_fd, SOL_SOCKET, SO_ZEROCOPY, &one, sizeof one) < 0) {
            fprintf(stderr, "fcfb_server: SO_ZEROCOPY unavailable (%s) -- falling "
                    "back to copy egress (CACHE-UNSAFE on udmabuf!)\n",
                    strerror(errno));
            g_zerocopy = 0;
        }
    }
    if (!g_zerocopy && ring_udmabuf)
        fprintf(stderr, "fcfb_server: WARNING copy egress from the CACHED udmabuf "
                "ring is data-unsafe (stale reads corrupt retunes) -- throughput "
                "A/B only; the PL needs u-dma-buf quirk_mmap_mode=4 for zero-copy\n");
    fprintf(stderr, "fcfb_server: egress = %s\n",
            g_zerocopy ? "zero-copy (MSG_ZEROCOPY from udmabuf)" : "copy (sendto)");
    }
    if (g_diag)
        fprintf(stderr, "fcfb_server: -D per-second egress diagnostics ON\n");

    apply_affinity();                            // -A: pin stream thread, steer IRQ

    struct session s; memset(&s, 0, sizeof s);
    s.udp_fd = udp_fd; s.run_gen = 0;

    while (!g_stop) {
        struct sockaddr_in cli; socklen_t cl = sizeof cli;
        int cfd = accept(lfd, (struct sockaddr*)&cli, &cl);
        if (cfd < 0) { if (errno == EINTR) continue; perror("accept"); break; }
        fprintf(stderr, "client %s connected\n", inet_ntoa(cli.sin_addr));

        // First request must arrive on a blocking read; then go non-blocking.
        // A v2 request (version >= FCFB_REQ_V2) is the base fcfb_req followed by
        // an fcfb_req_ext (two-window). The extension is read ONCE, on the first
        // request; live retunes (poll_control) are base-only, so a two-window
        // capture is a single run that closes the connection when done.
        struct fcfb_req req;
        struct fcfb_req_ext ext; memset(&ext, 0, sizeof ext);
        const struct fcfb_req_ext *extp = NULL;
        struct fcfb_chan chans[FCFB_MAX_CHAN];
        uint32_t nchan = 0; int is_v3 = 0;
        long guard = FCFB_GUARD_BINS;             // default; v4 requests may override
        if (read_full(cfd, &req, sizeof req) != 0) {
            fprintf(stderr, "bad/short request; dropping client\n");
            close(cfd); continue;
        }
        // Params query (FPRM): reply with the board's analysis params and close.
        // A standalone connect-query-close so a client can validate its synthesis
        // kernel before it requests a stream.
        if (memcmp(req.magic, FCFB_PRM_MAGIC, 4) == 0) {
            send_params(cfd);
            close(cfd);
            fprintf(stderr, "client disconnected (params query)\n");
            continue;
        }
        if (memcmp(req.magic, FCFB_REQ_MAGIC, 4) != 0) {
            fprintf(stderr, "bad/short request; dropping client\n");
            close(cfd); continue;
        }
        if (req.version == FCFB_REQ_V2) {         // two-window extension
            if (read_full(cfd, &ext, sizeof ext) != 0) {
                fprintf(stderr, "short v2 request extension; dropping client\n");
                close(cfd); continue;
            }
            extp = &ext;
        } else if (req.version == FCFB_REQ_V3 ||   // multi-channel: nextra + channels
                   req.version == FCFB_REQ_V4) {   // v4 also carries a u16 guard
            uint16_t nextra;
            if (read_full(cfd, &nextra, sizeof nextra) != 0 ||
                (uint32_t)nextra + 1u > FCFB_MAX_CHAN) {
                fprintf(stderr, "bad v3 channel count; dropping client\n");
                close(cfd); continue;
            }
            if (req.version == FCFB_REQ_V4) {      // caller-chosen guard bins
                uint16_t gv;
                if (read_full(cfd, &gv, sizeof gv) != 0) {
                    fprintf(stderr, "short v4 guard field; dropping client\n");
                    close(cfd); continue;
                }
                guard = (long)gv;                  // fcfb_admit clamps to [0,MAX]
            }
            // channel 0 = the base request; channels 1..nextra follow.
            chans[0].f_lo = req.f_lo; chans[0].f_hi = req.f_hi;
            chans[0].adc_bits = req.adc_mask ? req.adc_mask : 0x1;
            chans[0].dds_k = req.dds_k; chans[0].dds_phase_inc = req.dds_phase_inc;
            if (nextra && read_full(cfd, &chans[1],
                                    (size_t)nextra * sizeof(struct fcfb_chan)) != 0) {
                fprintf(stderr, "short v3 channel list; dropping client\n");
                close(cfd); continue;
            }
            nchan = (uint32_t)nextra + 1; is_v3 = 1;
            fprintf(stderr, "request v%u: %u channel(s), guard=%ld bins\n",
                    req.version, nchan, guard);
        }
        fcntl(cfd, F_SETFL, O_NONBLOCK);

        static uint8_t hdrbuf[FCFB_HDR_V3_FIXED
                              + FCFB_MAX_CHAN * sizeof(struct fcfb_run)];
        for (;;) {                               // request -> stream -> retune loop
            struct fcfb_accept acc;
            uint32_t hlen; const void *hp;
            if (is_v3) {                          // multi-channel (variable header)
                if (program_v3(chans, nchan, req.shift, req.udp_port, &s,
                               cli.sin_addr.s_addr, hdrbuf, &hlen, guard) != 0) {
                    fprintf(stderr, "v3 admission failed; closing\n");
                    break;
                }
                hp = hdrbuf;
            } else {
                if (program(&req, extp, &s, cli.sin_addr.s_addr, &acc, guard) != 0) {
                    fprintf(stderr, "admission failed for [%.0f,%.0f] Hz; closing\n",
                            req.f_lo, req.f_hi);
                    break;                        // reject = drop connection
                }
                hlen = fcfb_hdr_size(acc.version); hp = &acc;
            }
            // (accept header on the blocking-then-nonblocking cfd: write is small).
            fcntl(cfd, F_SETFL, 0);
            if (write(cfd, hp, hlen) != (ssize_t)hlen) break;
            fcntl(cfd, F_SETFL, O_NONBLOCK);

            // zcudp: (re)template + arm now the client is up and s->dst is final
            // (the UDP port can change on retune, so re-arm every (re)program).
            if (g_egress == EGRESS_ZCUDP && zcudp_arm_session(&s) != 0) {
                fprintf(stderr, "zcudp session arm failed; closing\n");
                break;
            }

            int r = stream_session(&s, cfd, &req);
            if (r == 1 && !is_v3) { extp = NULL; continue; } // v1/v2 live retune
            break;                                // v3, disconnect, or signal
        }

        datapath_off();
        zcudp_disarm();                          // -E zcudp: disarm between clients
        close(cfd);
        fprintf(stderr, "client disconnected; datapath idle\n");
    }

    datapath_off();
    fprintf(stderr, "fcfb_server: exit\n");
    return 0;
}
