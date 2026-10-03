# SPDX-License-Identifier: MIT
# Copyright (C) 2026  fcfb project
"""Register-bus sim of the fcfb Stage-1 register map + SVD smoke.

Exercises the custom register bus (ren/address -> rdata one cycle later;
wstrobe/wdata write) on the control and fcfb banks to confirm the fcfb field
definitions read/write as intended, and that the SVD generates.
"""
from amaranth import *
from amaranth.sim import Simulator

from maia_fcfb import registers as R


async def _read(ctx, bank, addr):
    ctx.set(bank.address, addr)
    ctx.set(bank.ren, 1)
    await ctx.tick()
    ctx.set(bank.ren, 0)
    return ctx.get(bank.rdata)


async def _write(ctx, bank, addr, wdata, wstrobe):
    ctx.set(bank.address, addr)
    ctx.set(bank.wdata, wdata)
    ctx.set(bank.wstrobe, wstrobe)
    await ctx.tick()
    ctx.set(bank.wstrobe, 0)


def test_control_bank():
    bank = R.control_registers()
    seen = {}

    async def tb(ctx):
        seen['product_id'] = await _read(ctx, bank, 0b00)
        seen['version'] = await _read(ctx, bank, 0b01)
        seen['reset_default'] = await _read(ctx, bank, 0b10)
        # clear sdr_reset
        await _write(ctx, bank, 0b10, 0, 0b0001)
        seen['reset_cleared'] = await _read(ctx, bank, 0b10)
        # interrupt: pulse the dma sticky input
        ctx.set(bank['interrupts']['dma'], 1)
        await ctx.tick()
        ctx.set(bank['interrupts']['dma'], 0)
        await ctx.tick()
        seen['interrupt'] = ctx.get(bank['interrupts'].interrupt)
        seen['interrupt_read'] = await _read(ctx, bank, 0b11)

    sim = Simulator(bank)
    sim.add_clock(8e-9)
    sim.add_testbench(tb)
    sim.run()

    major, minor, bugfix = (int(x) for x in R.VERSION.split('.'))
    assert seen['product_id'] == R.PRODUCT_ID
    assert seen['version'] == (
        (R.PLATFORM << 24) | (major << 16) | (minor << 8) | bugfix)
    assert seen['reset_default'] == 1
    assert seen['reset_cleared'] == 0
    assert seen['interrupt'] == 1
    assert seen['interrupt_read'] == 1


def test_fcfb_bank():
    ol2, shift_width = 12, 5
    bank = R.fcfb_registers(ol2, shift_width)
    seen = {}

    # 'run': enable[0] (RW), mask_busy[1] (R input).
    # 'mask_load' (0b101): addr[0:12], keep_a[12], keep_b[13], load[14] (Wpulse),
    #                      clear[15] (Wpulse).
    addr, keep_a, keep_b = 697, 1, 0
    mask_word = (addr | (keep_a << ol2) | (keep_b << (ol2 + 1))
                 | (1 << (ol2 + 2)))                       # load pulse

    async def tb(ctx):
        seen['run_default'] = await _read(ctx, bank, 0b00)
        # enable = 1
        await _write(ctx, bank, 0b00, 0b1, 0b0001)
        seen['run_enabled'] = await _read(ctx, bank, 0b00)
        # mask_busy is an R input: drive it, read it back in bit 1
        ctx.set(bank['run']['mask_busy'], 1)
        seen['run_busy'] = await _read(ctx, bank, 0b00)
        ctx.set(bank['run']['mask_busy'], 0)
        # quant.shift
        await _write(ctx, bank, 0b01, 3, 0b0001)
        seen['shift'] = await _read(ctx, bank, 0b01)
        # dma_control write-pulse self-clears next cycle
        await _write(ctx, bank, 0b10, 0b01, 0b0001)
        seen['dma_start'] = ctx.get(bank['dma_control']['dma_start'])
        await ctx.tick()
        seen['dma_start_cleared'] = ctx.get(bank['dma_control']['dma_start'])
        # dma_next_address is a read-only input; drive it and read back.
        ctx.set(bank['dma_next_address']['next_address'], 0x1a00_2000)
        seen['next_address'] = await _read(ctx, bank, 0b11)
        # dma_base (0b110): RW runtime AXI base (Option B / udmabuf).  Default is
        # the compile-time base; write a udmabuf-like physical address and read
        # it back.
        seen['dma_base_default'] = await _read(ctx, bank, 0b110)
        await _write(ctx, bank, 0b110, 0x3000_0000, 0b1111)
        seen['dma_base'] = await _read(ctx, bank, 0b110)
        # mask_load: write addr+keep_a+load; addr/keep hold, load self-clears.
        await _write(ctx, bank, 0b101, mask_word, 0b1111)
        seen['mask_addr'] = ctx.get(bank['mask_load']['addr'])
        seen['mask_keep_a'] = ctx.get(bank['mask_load']['keep_a'])
        seen['mask_load_pulse'] = ctx.get(bank['mask_load']['load'])
        await ctx.tick()
        seen['mask_load_cleared'] = ctx.get(bank['mask_load']['load'])
        # clear pulse (bit ol2+3)
        await _write(ctx, bank, 0b101, 1 << (ol2 + 3), 0b1111)
        seen['mask_clear_pulse'] = ctx.get(bank['mask_load']['clear'])
        await ctx.tick()
        seen['mask_clear_cleared'] = ctx.get(bank['mask_load']['clear'])

    sim = Simulator(bank)
    sim.add_clock(8e-9)
    sim.add_testbench(tb)
    sim.run()

    assert seen['run_default'] == 0
    assert (seen['run_enabled'] & 0b1) == 1
    assert ((seen['run_busy'] >> 1) & 0b1) == 1
    assert (seen['shift'] & ((1 << shift_width) - 1)) == 3
    assert seen['dma_start'] == 1
    assert seen['dma_start_cleared'] == 0
    assert seen['next_address'] == 0x1a00_2000
    assert seen['dma_base_default'] == 0x1000_0000
    assert seen['dma_base'] == 0x3000_0000
    assert seen['mask_addr'] == addr
    assert seen['mask_keep_a'] == keep_a
    assert seen['mask_load_pulse'] == 1
    assert seen['mask_load_cleared'] == 0
    assert seen['mask_clear_pulse'] == 1
    assert seen['mask_clear_cleared'] == 0


def test_param_bank():
    # Read-only params bank: every field is a build constant read back verbatim.
    bank = R.param_registers(r_hop=3125, t_frames=4, n_fft=4096, wmax=512,
                             bin_width=24, n_dds=1, build_id=0x9615567c)
    seen = {}

    async def tb(ctx):
        seen['magic'] = await _read(ctx, bank, 0b0000)
        seen['version'] = await _read(ctx, bank, 0b0001)
        seen['r'] = await _read(ctx, bank, 0b0010)
        seen['t'] = await _read(ctx, bank, 0b0011)
        seen['n'] = await _read(ctx, bank, 0b0100)
        seen['wmax'] = await _read(ctx, bank, 0b0101)
        seen['bin_width'] = await _read(ctx, bank, 0b0110)
        seen['n_dds'] = await _read(ctx, bank, 0b0111)
        seen['features'] = await _read(ctx, bank, 0b1000)
        seen['build_lo'] = await _read(ctx, bank, 0b1001)
        seen['build_hi'] = await _read(ctx, bank, 0b1010)

    sim = Simulator(bank)
    sim.add_clock(8e-9)
    sim.add_testbench(tb)
    sim.run()

    assert seen['magic'] == R.PARAM_BLOCK_MAGIC
    assert (seen['version'] & 0xffff) == R.PARAM_BLOCK_VER
    assert seen['r'] == 3125
    assert seen['t'] == 4
    assert seen['n'] == 4096
    assert seen['wmax'] == 512
    assert seen['bin_width'] == 24
    assert seen['n_dds'] == 1
    assert seen['features'] == 0
    assert seen['build_lo'] == 0x9615567c
    assert seen['build_hi'] == 0


def _reg_names(bank):
    return {r.name for r in bank.registers.values()}


def test_dma_base_presence_and_init():
    # dma_base shares slot 0b110 with dds2, so it is mapped only for the builds
    # that use the udmabuf zero-copy path (n_dds < 2), and drops out for the
    # two-tone (n_dds >= 2) bench builds, which keep the compile-time base.
    for n_dds in (0, 1):
        assert 'dma_base' in _reg_names(R.fcfb_registers(n_dds=n_dds))
    for n_dds in (2, 3):
        names = _reg_names(R.fcfb_registers(n_dds=n_dds))
        assert 'dma_base' not in names
        assert 'dds2' in names
    # init is overridable (default is the compile-time base).
    assert 'dma_base' in _reg_names(
        R.fcfb_registers(n_dds=0, dma_base_init=0x2000_0000))


def test_dma_laps_presence():
    # dma_laps shares slot 0b111 with dds3, so -- like dma_base -- it is mapped
    # only for the udmabuf zero-copy builds (n_dds < 2, i.e. production n_dds=0
    # and the single-tone HW-verify n_dds=1) and drops out for the three-tone
    # bench build (n_dds >= 3), which keeps dds3 there instead.
    for n_dds in (0, 1):
        names = _reg_names(R.fcfb_registers(n_dds=n_dds))
        assert 'dma_laps' in names
        assert 'dds3' not in names
    names = _reg_names(R.fcfb_registers(n_dds=3))
    assert 'dma_laps' not in names
    assert 'dds3' in names


def test_svd_smoke():
    control = R.control_registers()
    fcfb = R.fcfb_registers()
    params = R.param_registers(r_hop=3125, t_frames=4, n_fft=4096, wmax=512,
                              bin_width=24, n_dds=1)
    svd = R.register_map(control, fcfb, params).svd()
    assert b'product_id' in svd
    assert b'mask_load' in svd
    assert b'fcfb Stage-1' in svd
    assert b'param_magic' in svd and b'r_hop' in svd
