# fcfb board-side server. Clean-room; standard library only.
#
#   fcfb_server   board (ARM) TCP-control + UDP-data server (C, cross-compiled)
#   test_budget   host-side unit check of the budget/guard admission predicates
#
# fcfb_server is cross-compiled for the TRX-duo (Zynq 7010) ARM/Linux; set
# CROSS_CC to your Buildroot toolchain, e.g.
#   make CROSS_CC=/path/to/buildroot/output/host/bin/arm-buildroot-linux-gnueabihf-gcc fcfb_server

CC       ?= cc
CFLAGS   ?= -O2 -Wall -Wextra
CROSS_CC ?= arm-buildroot-linux-gnueabihf-gcc

all: test_budget

# board server (cross-compiled, static). -pthread: the board-info (BINF) responder
# runs on its own thread so it answers even while the single-session port is streaming.
fcfb_server: fcfb_server.c fcfb_net.h fcfb_zcudp_abi.h
	$(CROSS_CC) -O2 -static -Wall -Wextra -pthread -o $@ fcfb_server.c

# SD-image board binary: the defaults the shipped image expects --
#   int24 on-wire (matches the int24 wideband bitstream) and jumbo MTU 3980
#   (matches the overlay's eth0 default, etc/network/if-pre-up.d/mtu-jumbo).
# -u/-b still override at runtime. Copy the result over board/overlay/root/fcfb_server.
FCFB_SD_DEFS ?= -DFCFB_DEFAULT_BIN_WIDTH=24 -DFCFB_MTU=3980u
fcfb_server_sd: fcfb_server.c fcfb_net.h fcfb_zcudp_abi.h
	$(CROSS_CC) -O2 -static -Wall -Wextra -pthread $(FCFB_SD_DEFS) -o $@ fcfb_server.c

# host-side unit check: budget/guard admission predicates (fcfb_net.h)
test_budget: test_budget.c fcfb_net.h
	$(CC) $(CFLAGS) -o $@ test_budget.c

test: test_budget
	./test_budget

clean:
	rm -f fcfb_server fcfb_server_sd test_budget

.PHONY: all clean test
