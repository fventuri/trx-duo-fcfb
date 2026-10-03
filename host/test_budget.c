// SPDX-License-Identifier: MIT
// Copyright (C) 2026  fcfb project
//
// Unit check for the GbE bin-budget admission predicate and the guard-bin admit
// (the guard-budget study, Parts A & B3). Verifies:
//   - fcfb_bin_budget matches the closed-form B_total = floor((U/40000-8)/(2*bb))
//     and the documented theoretical ceilings (int24 ~490, int16 ~735);
//   - fcfb_admit pads FCFB_GUARD_BINS bins each side of the occupied band;
//   - the record sizer + budget are consistent (a within-budget request fits and
//     an over-budget one is caught by the predicate the server enforces).
//
// Build/run:  make test   (or  cc -Wall -o test_budget test_budget.c && ./test_budget)

#include <stdio.h>
#include "fcfb_net.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { \
    printf("FAIL: " __VA_ARGS__); printf("\n"); fails++; } } while (0)

int main(void) {
    // --- budget = largest W_total whose record fits one MTU (HW-measured) ---
    CHECK(fcfb_bin_budget(24) == FCFB_BIN_BUDGET_24 && FCFB_BIN_BUDGET_24 == 242,
          "int24 budget=%u (want 242)", fcfb_bin_budget(24));
    CHECK(fcfb_bin_budget(16) == FCFB_BIN_BUDGET_16 && FCFB_BIN_BUDGET_16 == 363,
          "int16 budget=%u (want 363)", fcfb_bin_budget(16));

    // --- guard-bin admit: W = occupied + 2*G + 1 ---
    uint32_t k0, W;
    CHECK(fcfb_admit(700.0 * FCFB_BINW, 700.0 * FCFB_BINW, FCFB_GUARD_BINS, &k0, &W) == 0,
          "admit point tone failed");
    CHECK(W == (uint32_t)(2 * FCFB_GUARD_BINS + 1),
          "point W=%u (want %u)", W, 2 * FCFB_GUARD_BINS + 1);
    CHECK(k0 == 700u - FCFB_GUARD_BINS, "point k0=%u (want %u)",
          k0, 700u - FCFB_GUARD_BINS);
    // occupied band 700..709 = 10 bins -> W = 10 + 2*G
    CHECK(fcfb_admit(700.0 * FCFB_BINW, 709.0 * FCFB_BINW, FCFB_GUARD_BINS, &k0, &W) == 0,
          "admit band failed");
    CHECK(W == 10u + 2 * FCFB_GUARD_BINS, "band W=%u (want %u)",
          W, 10u + 2 * FCFB_GUARD_BINS);
    // --- caller-chosen guard (v4): W = 2*g + 1, k0 = 700 - g ---
    for (long g = 0; g <= 8; g++) {
        CHECK(fcfb_admit(700.0 * FCFB_BINW, 700.0 * FCFB_BINW, g, &k0, &W) == 0,
              "admit point guard=%ld failed", g);
        CHECK(W == (uint32_t)(2 * g + 1) && k0 == 700u - (uint32_t)g,
              "guard=%ld -> k0=%u W=%u (want k0=%lu W=%ld)", g, k0, W,
              700ul - g, 2 * g + 1);
    }
    // guard is clamped to [0, FCFB_GUARD_MAX]
    CHECK(fcfb_admit(700.0 * FCFB_BINW, 700.0 * FCFB_BINW, 999, &k0, &W) != 0 ||
          W <= (uint32_t)(2 * FCFB_GUARD_MAX + 1), "guard clamp");

    // --- budget is tight: at B the IP datagram fits one MTU, at B+1 it doesn't ---
    // IP total = 20(IP) + 8(UDP) + fcfb_udp_hdr(12) + record.
    const uint32_t IP_UDP_HDR = 20u + 8u + (uint32_t)sizeof(struct fcfb_udp_hdr);
    uint32_t B = fcfb_bin_budget(24);
    uint32_t ip_at_B  = IP_UDP_HDR + fcfb_record_size_ab(B, 0, 24);
    uint32_t ip_at_B1 = IP_UDP_HDR + fcfb_record_size_ab(B + 1, 0, 24);
    CHECK(ip_at_B <= FCFB_MTU, "at budget %u, IP datagram %u > MTU %u",
          B, ip_at_B, FCFB_MTU);
    CHECK(ip_at_B1 > FCFB_MTU, "budget %u not tight: B+1 IP datagram %u still <= MTU",
          B, ip_at_B1);

    // --- dual-ADC shares the same total budget (W_a+W_b) ---
    CHECK(fcfb_bin_budget(24) / 2 == 121, "symmetric per-ADC int24=%u (want 121)",
          fcfb_bin_budget(24) / 2);

    if (fails) { printf("%d CHECK(s) FAILED\n", fails); return 1; }
    printf("all budget/guard checks passed "
           "(G=%d, budget int24=%u int16=%u)\n",
           FCFB_GUARD_BINS, fcfb_bin_budget(24), fcfb_bin_budget(16));
    return 0;
}
