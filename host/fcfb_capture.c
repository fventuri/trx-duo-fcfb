// fcfb_capture.c -- bring-up-grade PS capture driver for fcfb Stage-1 (M3 6e).
//
// Runs on the TRX-duo (Zynq 7010) ARM/Linux. Programs the fcfb Stage-1 AXI-Lite
// registers over /dev/mem, streams selected-bin records to the reserved DDR ring
// via the PL AXI-HP0 DMA, then writes an fcfb_stream.h file (48-B header + raw
// records) for the clean-room host Stage-2 decoder.
//
// Build (cross): arm-buildroot-linux-gnueabihf-gcc -O2 -static -o fcfb_capture fcfb_capture.c
// Usage: ./fcfb_capture [k0 W adc_mask shift nblocks outfile dds_k bin_width]
//   defaults: k0=694 W=7 adc_mask=1(ADC0) shift=6 nblocks=8192 out=stream.bin
//             dds_k=0(ADC) bin_width=16.  Set bin_width=24 for an int24 build (a
//             v2 52-B header + 6-byte bins; MUST match the loaded bitstream).
//
// SAFETY NOTES (learned on HW):
//  - The DMA ring 0x1000_0000..0x1A00_0000 must be reserved (boot with mem=256M),
//    or the PL DMA corrupts kernel RAM.
//  - The fcfb register bank (0x20+) is in the PL 'sync' domain, held in reset by
//    sdr_reset. NEVER touch 0x20+ while sdr_reset=1 -> the RegisterCDC read/write
//    never acks -> AXI bus hang -> CPU wedge (power cycle). This tool only writes
//    the control bank (0x08) while reset is asserted.
//  - AXI-HP0 is not cache-coherent; /dev/mem is opened O_SYNC (uncached) so ring
//    reads see the DMA's writes.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#define REG_BASE   0x40000000u
#define RING_BASE  0x10000000u
#define RING_END   0x1A000000u
#define RING_SIZE  (RING_END - RING_BASE)          // 160 MiB

enum { R_CONTROL=0x08, R_RUN=0x20, R_QUANT=0x24, R_DMA_CTL=0x28, R_DMA_NEXT=0x2C,
       R_DDS=0x30 };
#define N_ORDER    12                          // log2(N), N=4096
#define DDS_ENABLE (1u << 24)                  // 'enable' bit of the dds register

static volatile uint32_t *regs;
static inline void     wr(unsigned off, uint32_t v){ regs[off/4] = v; }
static inline uint32_t rd(unsigned off){ return regs[off/4]; }

static uint32_t run_word(uint32_t k0, uint32_t W, uint32_t mask, uint32_t en){
    return (k0 & 0xFFF) | ((W & 0x1FFF) << 12) | ((mask & 0x3) << 25) | ((en & 1) << 27);
}

int main(int argc, char** argv){
    uint32_t k0       = argc>1 ? (uint32_t)strtoul(argv[1],0,0) : 694;
    uint32_t W        = argc>2 ? (uint32_t)strtoul(argv[2],0,0) : 7;
    uint32_t adc_mask = argc>3 ? (uint32_t)strtoul(argv[3],0,0) : 1;
    uint32_t shift    = argc>4 ? (uint32_t)strtoul(argv[4],0,0) : 6;
    uint32_t nblk_tgt = argc>5 ? (uint32_t)strtoul(argv[5],0,0) : 8192;
    const char* out   = argc>6 ? argv[6] : "stream.bin";
    // Optional 8th arg: DDS tone bin k (0 = DDS off, use the real ADC). When
    // non-zero, inject an exactly-on-bin-k complex tone (phase_inc = k<<12).
    uint32_t dds_k    = argc>7 ? (uint32_t)strtoul(argv[7],0,0) : 0;
    // Optional 9th arg: on-wire bin width (16 or 24); MUST match the bitstream.
    uint32_t binw     = argc>8 ? (uint32_t)strtoul(argv[8],0,0) : 16;
    if (binw != 16 && binw != 24) { fprintf(stderr,"bad bin_width (16|24)\n"); return 2; }

    uint32_t pc = __builtin_popcount(adc_mask & 0x3);
    uint32_t bin_bytes = binw / 8;               // 2 (int16) or 3 (int24)
    uint32_t payload = pc * W * bin_bytes * 2u;
    uint32_t rec = 8 + ((payload + 3u) & ~3u);   // u64 seq + padded {I,Q} run
    if (rec == 0 || pc == 0) { fprintf(stderr,"bad adc_mask\n"); return 2; }

    int fd = open("/dev/mem", O_RDWR|O_SYNC);
    if (fd < 0){ perror("open /dev/mem"); return 1; }
    regs = (volatile uint32_t*)mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, REG_BASE);
    volatile uint8_t* ring = (volatile uint8_t*)mmap(0, RING_SIZE, PROT_READ, MAP_SHARED, fd, RING_BASE);
    if (regs == MAP_FAILED || ring == MAP_FAILED){ perror("mmap"); return 1; }

    if (rd(0x00) != 0x62666366u)
        fprintf(stderr, "warning: product_id=0x%08x (expected 0x62666366)\n", rd(0x00));

    // 1. Clear sdr_reset (brings the sync domain / fcfb register bank out of
    //    reset so 0x20+ is safe to access). NOTE: do NOT assert-then-release
    //    sdr_reset here -- pulsing it desyncs the datapath restart and the DMA
    //    stalls. For a clean egress FIFO / record-0-at-base, run this tool right
    //    after a fresh `start-project fcfb_stage1` (full PL reconfig).
    wr(R_CONTROL, 0); usleep(1000);

    // 2. Program the run (datapath disabled) + quantiser shift + DDS.
    wr(R_RUN,   run_word(k0, W, adc_mask, 0));
    wr(R_QUANT, shift);
    if (dds_k) wr(R_DDS, (dds_k << (24 - N_ORDER)) | DDS_ENABLE);  // tone at bin k
    else       wr(R_DDS, 0);                                       // ADC input

    // 3. Arm the DMA (next_address <- ring base), then 4. enable the datapath.
    wr(R_DMA_CTL, 0x1);
    wr(R_RUN,     run_word(k0, W, adc_mask, 1));

    // 5. Capture: wait for nblk_tgt records or until the ring is nearly full.
    //    Guarded: a wall-clock cap and a no-progress breakout so the poll can
    //    never spin forever (reads are same-address dma_next => bus-safe).
    uint32_t want = RING_BASE + nblk_tgt * rec;
    if (want > RING_END - 64) want = RING_END - 64;
    uint32_t next = rd(R_DMA_NEXT), last = next, stalled = 0;
    int ms_cap = 8000, ms = 0;
    for(; ms < ms_cap; ms++){
        next = rd(R_DMA_NEXT);
        if (next >= want) break;
        if (next == last) { if (++stalled > 500) { fprintf(stderr,"stall at 0x%08x\n", next); break; } }
        else { stalled = 0; last = next; }
        usleep(1000);
    }
    if (ms >= ms_cap) fprintf(stderr, "capture timed out at 0x%08x\n", next);

    // 6. Stop: disable datapath, pulse dma_stop, re-read the final pointer.
    wr(R_RUN,     run_word(k0, W, adc_mask, 0));
    wr(R_DMA_CTL, 0x2);
    usleep(1000);
    next = rd(R_DMA_NEXT);
    uint32_t nbytes  = next - RING_BASE;
    uint32_t nblocks = nbytes / rec;

    // 7. Write stream.bin: fcfb_stream.h header (48-B v1 int16 / 52-B v2 int24)
    //    + nblocks raw records.
    FILE* f = fopen(out, "wb");
    if (!f){ perror("fopen out"); return 1; }
    struct __attribute__((packed)) {
        char m[8]; uint32_t ver, N; double fs;
        uint32_t k0, W, mask, nblocks; double bin_scale; uint32_t bin_width;
    } h = { {'F','C','F','B','v','1',0,0}, binw==24?2u:1u, 4096, 125e6,
            k0, W, adc_mask, nblocks, (double)(1u << shift), binw };
    fwrite(&h, binw==24 ? 52u : 48u, 1, f);      // v1 omits bin_width
    fwrite((const void*)ring, rec, nblocks, f);
    fclose(f);

    fprintf(stderr, "wrote %s: nblocks=%u rec=%uB shift=%u binw=%u k0=%u W=%u mask=0x%x dds_k=%u bin_scale=%.0f\n",
            out, nblocks, rec, shift, binw, k0, W, adc_mask, dds_k, (double)(1u<<shift));
    return 0;
}
