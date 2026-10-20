# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""Protocol firmware tests against bus-functional device models.

These are black-box (pins + host SPI only), so they also run on the
gate-level netlist. In RTL simulation the reference model additionally
runs in lockstep for the whole test.
"""

import os
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sw"))
from polyglot.asm import assemble                      # noqa: E402
from polyglot import programs as P                      # noqa: E402
from polyglot.regs import (creg, R_PC, R_INSTR, G_IRQ, G_BANKMAP, IO,   # noqa: E402
                           IN, OUT)

from bfm import Board, UartMonitor, UartDriver, SpiSlave, I2CEeprom  # noqa: E402
from pg_host import Host, reset                       # noqa: E402

GL = os.environ.get("GATES") == "yes"


class Env:
    def __init__(self, dut):
        self.dut = dut

    async def start(self):
        dut = self.dut
        cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
        self.host = Host(dut)
        self.board = Board(dut, self.host)
        self.ls = None
        if not GL:
            from lockstep import Lockstep
            self.ls = Lockstep(dut)
            cocotb.start_soon(self.ls.run())
        await reset(dut)
        cocotb.start_soon(self.board.run())
        return self

    async def load(self, core, prog, cfg):
        await self.host.load_program(prog.words, base=prog.origin)
        await self.host.write(creg(core, 0), cfg.words())
        await self.host.write(creg(core, R_PC), prog.wrap_target)

    async def inject(self, core, text):
        await self.host.write(creg(core, R_INSTR), assemble(text).words[0])

    def reader(self, core):
        """Background task that drains a core's RX FIFO into a list."""
        got = []

        async def run():
            while True:
                _, rx = await self.host.fifo_levels(core)
                if rx:
                    w = await self.host.pop(core, rx)
                    got.extend(w if isinstance(w, list) else [w])
                await ClockCycles(self.dut.clk, 100)
        cocotb.start_soon(run())
        return got

    async def until(self, cond, timeout=400000, what="condition"):
        for _ in range(timeout // 200):
            if cond():
                return
            await ClockCycles(self.dut.clk, 200)
        raise AssertionError(f"timeout waiting for {what() if callable(what) else what}")

    async def wait_rx(self, core, n, timeout=200000):
        got = []
        for _ in range(timeout // 500):
            _, rx = await self.host.fifo_levels(core)
            if rx:
                got += [await self.host.pop(core) for _ in range(rx)]
            if len(got) >= n:
                return got
            await ClockCycles(self.dut.clk, 400)
        extra = ("\n" + "\n".join(self.ls.errors[:12])) if self.ls and self.ls.errors else ""
        raise AssertionError(f"timeout waiting for {n} words, got {got}{extra}")

    def finish(self):
        if self.ls:
            self.ls.running = False
            self.dut._log.info(f"lockstep: {self.ls.cycles} cycles compared")
            self.ls.check()


@cocotb.test()
async def test_uart(dut):
    """UART TX on uo[2] looped back into an RX core, plus an external RX."""
    env = await Env(dut).start()
    host = env.host
    bit = 32                                    # clkdiv 3 -> 8 ticks x 4 clk
    tx, rx = P.uart_tx_program(0), P.uart_rx_packed_program(32)
    rx2 = P.uart_rx_program(64)
    await env.load(0, tx, P.uart_config(tx, 3, OUT[2]))
    await env.load(1, rx, P.uart_config(rx, 3, OUT[2], rx=True, packed=True))
    await env.load(2, rx2, P.uart_config(rx2, 3, IN[3], rx=True))
    mon = UartMonitor(lambda b: b.uo(2), bit)
    drv = UartDriver(3, bit)
    env.board.devices += [mon, drv]
    # bring the line up to idle-high before enabling the receivers: an
    # undriven output-only pad reads 0, which looks like a start bit
    await host.ctrl(en=0b0001)
    await env.inject(0, "set pins, 1")
    await env.inject(0, "set pindirs, 1")
    await host.ctrl(en=0b0111)

    msg = b"Polyglot speaks UART!..."           # multiple of 4 for the packed RX
    got1, got2 = env.reader(1), env.reader(2)
    drv.send(b"\x55\xa3\x00\xff")
    await host.push_wait(0, list(msg))
    await env.until(lambda: 4 * len(got1) >= len(msg) and len(got2) >= 4,
                    what=lambda: f"UART bytes: rx1={P.unpack_bytes(got1, 99)} rx2={got2} line={bytes(mon.bytes)}")
    assert P.unpack_bytes(got1, len(msg)) == msg, got1
    assert bytes(mon.bytes) == msg, mon.bytes
    assert got2 == [0x55, 0xA3, 0x00, 0xFF], got2

    # framing error -> IRQ flag 4 + core 2 (relative IRQ)
    drv.send(b"\x42", bad_stop=True)
    while not drv.idle:
        await ClockCycles(dut.clk, 100)
    await ClockCycles(dut.clk, 200)
    assert (await host.read(G_IRQ)) & (1 << 6), "framing error IRQ"
    env.finish()


@cocotb.test()
async def test_spi_master(dut):
    env = await Env(dut).start()
    host = env.host
    prog = P.spi_master_program(0)
    cfg = P.spi_master_config(prog, clkdiv=1, mosi=IO[0], sck=IO[1], miso=IO[2], cs=IO[3])
    await env.load(0, prog, cfg)
    reply = [0xA5, 0x3C, 0x01, 0x80]
    dev = SpiSlave(mosi=0, sck=1, miso=2, cs=3, reply=reply)
    env.board.devices.append(dev)
    env.board.pullup[3] = True
    await host.ctrl(en=1)
    await env.inject(0, "set pins, 8")          # CS high, SCK low
    await env.inject(0, "set pindirs, 11")      # MOSI, SCK, CS outputs
    await env.inject(0, "set pins, 0")          # CS low
    out = [0x9F, 0x00, 0x55, 0xFF]
    await host.push(0, [b << 24 for b in out])
    got = await env.wait_rx(0, 4)
    await env.inject(0, "set pins, 8")          # CS high
    assert dev.rx == out, dev.rx
    assert got == reply, got
    env.finish()


@cocotb.test()
async def test_i2c_eeprom(dut):
    """I2C master firmware (two banks) against an EEPROM model with
    clock stretching: write 3 bytes, read them back, probe a missing
    device."""
    env = await Env(dut).start()
    host = env.host
    prog = P.i2c_master_program(0)
    await env.load(0, prog, P.i2c_master_config(prog, clkdiv=2, sda=IO[4], scl=IO[5]))
    await host.write(G_BANKMAP, 0b11_10_00_00)   # core 0 owns banks 0 and 1
    dev = I2CEeprom(sda=4, scl=5, addr=0x50, stretch=25)
    env.board.devices.append(dev)
    env.board.pullup[4] = env.board.pullup[5] = True
    await host.ctrl(en=1)
    c = P.I2CCommands(prog)

    # write 0xDE 0xAD 0x42 at address 0x10
    await host.push(0, [c.start(), c.write(0xA0), c.write(0x10), c.write(0xDE)])
    await host.push_wait(0, [c.write(0xAD), c.write(0x42), c.stop()])
    acks = await env.wait_rx(0, 5)
    assert acks == [0, 0, 0, 0, 0], (acks, dev.log)
    assert dev.mem[0x10:0x13] == [0xDE, 0xAD, 0x42]

    # random read: set pointer, repeated start, read 3 (ACK, ACK, NACK)
    await host.push_wait(0, [c.start(), c.write(0xA0), c.write(0x10), c.start()])
    await host.push_wait(0, [c.write(0xA1), c.read(True), c.read(True), c.read(False), c.stop()])
    got = await env.wait_rx(0, 6)
    assert got == [0, 0, 0, 0xDE, 0xAD, 0x42], got

    # nobody at 0x3C: NACK
    await host.push(0, [c.start(), c.write(0x3C << 1), c.stop()])
    nack = await env.wait_rx(0, 1)
    assert nack == [1], nack
    dut._log.info("bus log: " + " ".join(dev.log))
    env.finish()
