# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""Constrained-random programs, configs and stimulus, checked in lockstep
against the reference model every cycle."""

import os
import random

import cocotb

NC = int(os.environ.get("PG_NCORES", "3"))
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

from lockstep import Lockstep
from pg_host import Host, creg, G_CTRL, G_IRQ, G_IRQMASK, G_BANKMAP, FIFO

SEEDS = int(os.environ.get("FUZZ_SEEDS", "6"))
CYCLES = int(os.environ.get("FUZZ_CYCLES", "6000"))


def rand_instr(rng, lo=0, hi=127, here=None):
    """One random instruction, biased towards making progress."""
    ss = rng.randrange(4)
    dly = rng.choice([0, 0, 0, 0, 1, 2, 5])
    kind = rng.choices(
        ["jmp", "wait", "in", "out", "line_in", "line_out", "pp", "mov", "irq",
         "set", "add", "ctl", "rsvd"],
        weights=[10, 4, 7, 8, 9, 9, 5, 8, 3, 9, 4, 5, 1])[0]
    if kind == "jmp":
        cond = rng.choice([0, 1, 2, 2, 3, 4, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])
        if here is not None and here < hi and rng.random() < 0.85:
            tgt = rng.randrange(here + 1, hi + 1)      # mostly forward
        else:
            tgt = rng.randrange(lo, hi + 1)
        op, arg = 0, (cond << 7) | tgt
    elif kind == "wait":
        to = 1 if rng.random() < 0.9 else 0
        op, arg = 1, (rng.randrange(2) << 10) | (rng.randrange(4) << 8) | (to << 7) | rng.randrange(8)
    elif kind in ("in", "line_in"):
        src = 4 if kind == "line_in" else rng.randrange(8)
        op, arg = 2, (src << 8) | rng.choice([1, 1, 2, 3, 4, 7, 8, 13, 16, 24, 31, 0])
    elif kind in ("out", "line_out"):
        dst = 4 if kind == "line_out" else rng.choice([0, 0, 1, 2, 3, 5, 6, 7])
        op, arg = 3, (dst << 8) | rng.choice([1, 1, 2, 3, 4, 7, 8, 13, 16, 24, 31, 0])
        if dst == 6 and rng.random() < 0.8:
            dst = 3
            arg = (dst << 8) | (arg & 31)
    elif kind == "pp":
        block = 1 if rng.random() < 0.25 else 0
        op, arg = 4, (rng.randrange(2) << 7) | (rng.randrange(2) << 6) | (block << 5)
    elif kind == "mov":
        dst = rng.choice([0, 1, 1, 2, 2, 3, 4, 4, 5, 6, 7])
        op, arg = 5, (dst << 8) | (rng.randrange(4) << 6) | (rng.randrange(8) << 3)
        if dst == 5 and rng.random() < 0.8:      # mov pc mostly harmless
            arg = (1 << 8) | (arg & 0xFF)
    elif kind == "irq":
        wait = 1 if rng.random() < 0.15 else 0
        op, arg = 6, (rng.randrange(2) << 6) | (wait << 5) | (rng.randrange(2) << 4) | rng.randrange(8)
    elif kind == "set":
        dst = rng.choice([0, 1, 2, 2, 2, 3])
        v = rng.randrange(256) if dst != 2 else rng.randrange(12)   # small Y for timeouts
        op, arg = 7, (dst << 8) | v
    elif kind == "add":
        op, arg = 8, rng.randrange(1 << 11)
    elif kind == "ctl":
        op, arg = 9, rng.randrange(1 << 9)
        if rng.random() < 0.6:
            arg |= 0x77                      # enable crc/stuff/code
    else:
        op, arg = rng.randrange(10, 16), rng.randrange(1 << 11)
    return (op << 16) | (ss << 14) | (dly << 11) | (arg & 0x7FF)


def rand_cfg(rng, core):
    lo = core * 32 if rng.random() < 0.7 else rng.randrange(128)
    hi = min(lo + rng.randrange(1, 32), 127)
    exec_ = (lo | (hi << 8) | (rng.randrange(2) << 15) | (rng.randrange(4) << 16) |
             (rng.randrange(8) << 18) | (rng.randrange(8) << 21))
    if rng.random() < 0.1:
        exec_ |= (1 << 31) | (rng.randrange(lo, hi + 1) << 24)
    shift = rng.randrange(1 << 14)
    shift &= ~3
    shift |= (rng.random() < 0.4) | ((rng.random() < 0.5) << 1)   # autopull, autopush
    if rng.random() < 0.5:
        shift = (shift & ~0x3F0) | (rng.choice([1, 8, 16, 0]) << 4)
    pin = rng.randrange(1 << 12)
    pm = rng.getrandbits(40)
    line = (rng.randrange(3) | (rng.randrange(2) << 2) | (rng.randrange(2) << 3) |
            (rng.randrange(8) << 4) | (rng.randrange(2) << 7) | (rng.randrange(2) << 8) |
            (rng.randrange(32) << 9) | (rng.randrange(2) << 14) | (rng.randrange(6) << 16))
    return [rng.choice([0, 0, 0, 1, 3]), exec_, shift, pin,
            pm & 0xFFFFF, pm >> 20, line, rng.getrandbits(32)]


async def stimulus(dut, host, rng, stop):
    v = 0
    while not stop[0]:
        await ClockCycles(dut.clk, rng.choice([1, 2, 3, 7, 15]))
        v ^= 1 << rng.randrange(8)
        dut.uio_in.value = v
        if rng.random() < 0.3:
            host.ui_hi = rng.randrange(32)


@cocotb.test()
async def test_random_lockstep(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    base = int(os.environ.get("FUZZ_SEED", "1"))
    total = 0
    for seed in range(base, base + SEEDS):
        rng = random.Random(seed)
        dut.rst_n.value = 0
        dut.ena.value = 1
        dut.uio_in.value = 0
        host = Host(dut, half=rng.choice([4, 5, 7]))
        ls = Lockstep(dut)
        task = cocotb.start_soon(ls.run())
        await ClockCycles(dut.clk, 5)
        dut.rst_n.value = 1
        await RisingEdge(dut.clk)

        # fill all of memory so no core ever fetches an unwritten word
        prog = []
        for b in range(4):
            prog += [rand_instr(rng, 32 * b, 32 * b + 31, 32 * b + a) for a in range(32)]
        await host.load_program(prog)
        for c in range(NC):
            await host.write(creg(c, 0), rand_cfg(rng, c))
            if rng.random() < 0.5:
                await host.write(creg(c, 10), [rng.getrandbits(32), rng.getrandbits(32)])
        if rng.random() < 0.3:
            await host.write(G_BANKMAP, rng.getrandbits(8))
        await host.write(G_IRQMASK, rng.getrandbits(16))
        await host.push(rng.randrange(NC), [rng.getrandbits(32) for _ in range(rng.randrange(1, 5))])

        stop = [False]
        cocotb.start_soon(stimulus(dut, host, rng, stop))
        await host.ctrl(en=(1 << NC) - 1)

        start = ls.cycles
        while ls.cycles - start < CYCLES and ls.running:
            r = rng.random()
            c = rng.randrange(NC)
            if r < 0.25:
                await host.push(c, [rng.getrandbits(32) for _ in range(rng.randrange(1, 4))])
            elif r < 0.50:
                await host.pop(c, 4)
            elif r < 0.55:
                await host.write(G_IRQ, rng.getrandbits(16))
            elif r < 0.62:
                # inject an instruction (often a jump to break a stall)
                if rng.random() < 0.5:
                    await host.write(creg(c, 15), rng.randrange(32 * c, 32 * c + 32))
                else:
                    await host.write(creg(c, 15), rand_instr(rng, 32 * c, 32 * c + 31))
            elif r < 0.66:
                await host.ctrl(en=15 if rng.random() < 0.8 else rng.randrange(16), restart=rng.randrange(16) if rng.random() < 0.3 else 0,
                                step=rng.randrange(16) if rng.random() < 0.3 else 0, divsync=rng.randrange(16),
                                fclr=rng.randrange(16) if rng.random() < 0.2 else 0)
            elif r < 0.70:
                await host.write(creg(c, rng.choice([8, 9, 10, 11, 12, 13])), rng.getrandbits(32))
            elif r < 0.73:
                await host.write(creg(c, 0), rand_cfg(rng, c))
            else:
                await ClockCycles(dut.clk, rng.randrange(20, 300))
        stop[0] = True
        ls.running = False
        await ClockCycles(dut.clk, 2)
        task.cancel()
        dut._log.info(f"seed {seed}: {ls.cycles} cycles in lockstep, {len(ls.errors)} errors")
        total += ls.cycles
        ls.check()
    dut._log.info(f"total {total} cycles compared")
    from polyglot.model import COVERAGE
    dut._log.info("coverage: " + ", ".join(f"{k}={v}" for k, v in sorted(COVERAGE.items())))
