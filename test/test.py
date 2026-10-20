# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""Smoke tests: host interface and a minimal program (run in RTL and GL)."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from pg_host import Host, reset, G_ID, G_CTRL, creg, R_X, R_PC, R_PMLO, R_INSTR


@cocotb.test()
async def test_id_and_registers(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    host = Host(dut)
    await reset(dut)

    assert await host.read(G_ID) == 0x504C5947, "ID register"

    # X is read/write through the host port
    await host.write(creg(1, R_X), 0xDEADBEEF)
    assert await host.read(creg(1, R_X)) == 0xDEADBEEF

    # burst read with auto-increment: PC, X of core 1
    pc, x = await host.read(creg(1, R_PC), 2)
    assert pc == 0 and x == 0xDEADBEEF


@cocotb.test()
async def test_toggle_pin(dut):
    """Core 0: SET PINDIRS 1; loop: SET PINS 1 [3]; SET PINS 0 [3]."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    host = Host(dut)
    await reset(dut)

    SET = 0x7
    def enc(op, arg, delay=0):
        return (op << 16) | ((delay & 0x1F) << 11) | arg

    prog = [
        enc(SET, (3 << 8) | 1),         # set pindirs, 1
        enc(SET, (0 << 8) | 1, 3),      # set pins, 1 [3]
        enc(SET, (0 << 8) | 0, 3),      # set pins, 0 [3]
    ]
    await host.load_program(prog)
    # wrap 1..2, vpin0 -> physical pin 2 (uio[2])
    await host.write(creg(0, 1), (1 << 0) | (2 << 8))
    await host.write(creg(0, R_PMLO), 2)
    # read back instruction 1 through the INSTR register while halted
    await host.write(creg(0, R_PC), 1)
    assert await host.read(creg(0, R_INSTR)) == prog[1]
    await host.write(creg(0, R_PC), 0)
    await host.write(G_CTRL, 1)

    await ClockCycles(dut.clk, 20)
    assert (int(dut.uio_oe.value) >> 2) & 1 == 1
    seen = []
    for _ in range(32):
        await ClockCycles(dut.clk, 1)
        seen.append((int(dut.uio_out.value) >> 2) & 1)
    # 4 cycles high, 4 cycles low: every interior run is exactly 4 long
    s = "".join(map(str, seen))
    runs = [len(r) for r in s.replace("01", "0 1").replace("10", "1 0").split()]
    assert len(runs) >= 6 and all(r == 4 for r in runs[1:-1]), s
