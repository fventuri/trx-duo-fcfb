// fcfb_zcudp_abi.h -- userspace mirror of the in-kernel zcudp ABI.
//
// This is a hand-kept copy of the ABI half of the kernel header
//   buildroot/br2-external-fcfb/board/patches/linux/7.1.7/0004-macb-zero-copy-udp-tx.patch
//   (drivers/net/ethernet/cadence/macb_zcudp.h)
// It MUST match that file's ioctl numbers and struct layouts byte-for-byte.
// Target is little-endian 32-bit ARM (the TRX-duo), so __u32/__u16/__u8 map to the
// stdint types below and __packed maps to __attribute__((packed)).
//
// "zcudp" = the kernel's zero-copy UDP TX path compiled into macb. fcfb_server sets
// a header template once, arms the udmabuf ring phys window, then hands the kernel
// batches of {inline fcfb header + 1-2 ring fragments} descriptors; the kernel DMAs
// them out of the PL ring with no copy and no get_user_pages.
#ifndef FCFB_ZCUDP_ABI_H
#define FCFB_ZCUDP_ABI_H

#include <stdint.h>

// ---- ioctl numbers (SIOCDEVPRIVATE range) ----
#define ZCUDP_IOC_SET_TMPL      0x89f1  // set Eth/IP/UDP header template
#define ZCUDP_IOC_SET_MTU       0x89f3  // set MTU (IP length budget)
#define ZCUDP_IOC_TX_BATCH      0x89f4  // transmit a batch of datagrams
#define ZCUDP_IOC_ARM           0x89f6  // arm/disarm + set ring phys bounds
#define ZCUDP_IOC_STATS         0x89f7  // read TX counters (for -D diag)

// ---- limits (must match macb_zcudp.h) ----
#define ZCUDP_MAX_FRAGS         2       // ring wrap needs at most 2 fragments
#define ZCUDP_MAX_INLINE_HDR    32      // fcfb_udp_hdr is 12 B; 32 is headroom

// 0x89f1: header template. IPs/ports in HOST byte order; the kernel byte-swaps.
struct zcudp_tmpl {
    uint8_t  dst_mac[6];
    uint8_t  src_mac[6];
    uint32_t src_ip;
    uint32_t dst_ip;
    uint16_t src_port;
    uint16_t dst_port;
} __attribute__((packed));

// 0x89f6: arm(1)/disarm(0) and declare the ring phys window for bounds-check.
struct zcudp_arm {
    uint32_t arm;
    uint32_t ring_phys;
    uint32_t ring_size;
} __attribute__((packed));

// one datagram descriptor (flat; batch-copied from userspace)
struct zcudp_dgram_desc {
    uint32_t hdr_len;                    // inline header bytes that follow
    uint32_t nfrag;                      // 1 or 2
    uint32_t frag_phys[ZCUDP_MAX_FRAGS]; // ring phys of each fragment
    uint32_t frag_len[ZCUDP_MAX_FRAGS];  // length of each fragment
    uint8_t  hdr[ZCUDP_MAX_INLINE_HDR];  // fcfb_udp_hdr bytes to prepend
} __attribute__((packed));

// 0x89f4: TX batch header; driver writes #accepted back into count.
struct zcudp_tx_batch {
    uint32_t count;                      // #descriptors; <- #accepted on return
    uint32_t descs;                      // userspace pointer to zcudp_dgram_desc[]
} __attribute__((packed));

// 0x89f7: TX counters.
struct zcudp_stats {
    uint32_t tx_dgrams;                  // datagrams handed to the GEM
    uint32_t tx_bytes;                   // payload+header bytes emitted (wraps)
    uint32_t underruns;                  // times the drain waited on ring space
    uint32_t dropped;                    // descriptors rejected (bad frag/bounds)
    uint32_t inflight;                   // current queue[0].zcudp_tx_inflight
    uint32_t rsvd[7];
} __attribute__((packed));

#endif // FCFB_ZCUDP_ABI_H
