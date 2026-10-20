# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""Stretch-goal protocols: USB low speed, 10BASE-T, CAN, logic analyser."""

import os
import sys
import zlib

import cocotb
from cocotb.triggers import ClockCycles

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "sw"))
from polyglot import programs as P                       # noqa: E402
from polyglot.regs import IO, G_IRQ, G_BANKMAP, ZERO      # noqa: E402

from bfm_wire import (UsbLsMonitor, UsbLsDriver, EthTxMonitor,  # noqa: E402
                      CanTxMonitor, crc16_usb, crc15_can)
from test_protocols import Env                             # noqa: E402


@cocotb.test()
async def test_usb_lowspeed(dut):
    """usb_tx sends DATA0 packets (heavy bit stuffing) that usb_rx on a
    second core receives off the same pads; an independent decoder checks
    the wire. Then an externally generated packet is received."""
    env = await Env(dut).start()
    host, board = env.host, env.board
    board.pullup[0] = True                       # 1.5k on D- : low-speed J idle
    tx, rx = P.usb_tx_program(0), P.usb_rx_program(32)
    await env.load(0, tx, P.usb_config(tx, IO[0], IO[1]))
    await env.load(1, rx, P.usb_config(rx, IO[0], IO[1], rx=True))
    mon = UsbLsMonitor(0, 1)
    board.devices.append(mon)
    rxw = env.reader(1)
    await host.ctrl(en=0b011)

    payload = bytes([0xFF, 0xFF, 0x00, 0x7E, 0x3F, 0xFC, 0x01, 0x80])
    await host.push_wait(0, P.usb_tx_words(0xC3, payload))       # DATA0
    await env.until(lambda: len(mon.packets) == 1 and len(rxw) >= 6,
                    what=lambda: f"USB packet: mon={mon.packets} rx={[hex(w) for w in rxw]}")
    pkt = mon.packets[0]
    crc = crc16_usb(payload)
    assert pkt["data"] == bytes([0x80, 0xC3]) + payload + bytes([crc & 0xFF, crc >> 8]), pkt
    assert pkt["stuff_errors"] == 0 and not mon.errors, (pkt, mon.errors)
    assert 60 <= pkt["se0"] <= 68, f"EOP SE0 length {pkt['se0']}"
    data, res = P.usb_rx_decode(rxw)
    assert data == pkt["data"], data.hex()
    assert res == 0xB001, f"CRC16 residual {res:#x}"
    assert (await host.read(G_IRQ)) & 0b101 == 0b101, "tx/rx done IRQs (0 and 1+1)"

    # an external packet (from the "host side") into the receiver
    await host.write(G_IRQ, 0xFF)
    drv = UsbLsDriver(0, 1)
    board.devices.append(drv)
    rxw.clear()
    ext = bytes([0x3F, 0x3F, 0x3F, 0xAA])
    c2 = crc16_usb(ext)
    drv.send(bytes([0x80, 0x4B]) + ext + bytes([c2 & 0xFF, c2 >> 8]))   # DATA1
    await env.until(lambda: len(rxw) >= 5, what=lambda: f"rx {rxw}")
    data, res = P.usb_rx_decode(rxw)
    assert data == bytes([0x80, 0x4B]) + ext + bytes([c2 & 0xFF, c2 >> 8]), data.hex()
    assert res == 0xB001
    env.finish()


@cocotb.test()
async def test_ethernet_10base_t_tx(dut):
    env = await Env(dut).start()
    host, board = env.host, env.board
    prog = P.eth_tx_program(0, frame=32)
    await env.load(0, prog, P.eth_tx_config(prog, IO[2], IO[3]))
    await host.write(G_BANKMAP, 0b11_10_00_00)   # core 0: code bank 0, frame bank 1
    frame = (bytes.fromhex("ffffffffffff" "02a1b2c3d4e5" "88b5") +
             b"Polyglot says hello over 10BASE-T!".ljust(46, b"."))   # 60-byte minimum frame
    await host.load_program(P.eth_frame_words(frame), base=32)
    fcs = zlib.crc32(frame).to_bytes(4, "little")
    mon = EthTxMonitor(2, 3, nbits=8 * (8 + len(frame) + 4))
    board.devices.append(mon)
    await host.ctrl(en=1)
    await host.write(G_IRQ, 1 << 8)              # set IRQ flag 0: send
    await env.until(lambda: mon.frames, what="ethernet frame")
    got = mon.frames[0]
    assert got[:8] == b"\x55" * 7 + b"\xd5", got[:8].hex()
    assert got[8:-4] == frame, got.hex()
    assert got[-4:] == fcs, f"FCS {got[-4:].hex()} expected {fcs.hex()}"
    assert mon.diff_errors == 0, mon.diff_errors
    assert (await host.read(G_IRQ)) & 0b11 == 0b10, "trigger consumed, done flag set"
    env.finish()


@cocotb.test()
async def test_can_tx(dut):
    env = await Env(dut).start()
    host, board = env.host, env.board
    prog = P.can_tx_program(0)
    await env.load(0, prog, P.can_tx_config(prog, IO[6]))
    mon = CanTxMonitor(lambda b: b.io(6))
    board.devices.append(mon)
    board.pullup[6] = True                       # recessive bus
    await host.ctrl(en=1)
    await env.inject(0, "set pins, 1")
    await env.inject(0, "set pindirs, 1")
    frames = [(0x123, bytes([0x00, 0x00, 0x00, 0xFF])), (0x7FF, b"\xf0\x0f")]
    for ident, data in frames:
        await host.push_wait(0, P.can_tx_words(ident, data))
    await env.until(lambda: len(mon.frames) == 2, what=lambda: f"CAN frames {mon.frames}")
    for (ident, data), f in zip(frames, mon.frames):
        assert f["ident"] == ident and f["data"] == data, f
        assert f["crc"] == f["crc_calc"], f
        assert f["stuff_errors"] == 0, f
        assert f["tail"] == [1] * 10, f
    env.finish()


@cocotb.test()
async def test_logic_analyzer(dut):
    """Edge capture with 32-bit timestamps on four uio pins."""
    env = await Env(dut).start()
    host, board = env.host, env.board
    prog = P.logic_analyzer_program(0)
    await env.load(0, prog, P.logic_analyzer_config(prog, [IO[4], IO[5], IO[6], IO[7]]))
    got = env.reader(0)
    await host.ctrl(en=1)
    await ClockCycles(dut.clk, 50)
    edges = [(0b0001, 300), (0b0011, 137), (0b0010, 1000), (0b1010, 61), (0b0000, 500)]
    for val, gap in edges:
        await ClockCycles(dut.clk, gap)
        for k in range(4):
            board.push[4 + k] = (val >> k) & 1
    await env.until(lambda: len(got) >= len(edges), what=lambda: f"LA {got}")
    states = [w & 0xFF for w in got]
    stamps = [w >> 8 for w in got]
    assert [s & 0xF for s in states] == [v for v, _ in edges], states
    deltas = [b - a for a, b in zip(stamps, stamps[1:])]
    # the 3-cycle polling loop gives a few cycles of jitter
    assert all(abs(d - g) <= 4 for d, (_, g) in zip(deltas, edges[1:])), \
        (deltas, [g for _, g in edges[1:]])
    env.finish()
