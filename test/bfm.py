# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""Board model and protocol device models (bus-functional models).

Board resolves the chip's pads every clock: uio pins can have pull-ups,
open-drain pull-downs and push-pull drivers from devices; ui[7:3] are
driven through the host driver. Devices see pad levels and act once per
system clock, so all timing below is in clock cycles.
"""

from cocotb.triggers import FallingEdge


class Board:
    def __init__(self, dut, host):
        self.dut = dut
        self.host = host
        self.devices = []
        self.pullup = [False] * 8
        self.od_low = {}           # (device, pin) -> bool
        self.push = {}             # pin -> value (uio, push-pull from devices)
        self.ui = {}               # ui index (3..7) -> value
        self.level = [0] * 8       # resolved uio levels
        self.cycle = 0
        self.running = True

    def uo(self, n):
        return (int(self.dut.uo_out.value) >> n) & 1

    def io(self, n):
        return self.level[n]

    def chip_drives(self, n):
        return (int(self.dut.uio_oe.value) >> n) & 1

    def _resolve(self):
        oe = int(self.dut.uio_oe.value)
        out = int(self.dut.uio_out.value)
        v = 0
        for n in range(8):
            if (oe >> n) & 1:
                lvl = (out >> n) & 1
            elif n in self.push:
                lvl = self.push[n]
            else:
                lvl = 1 if self.pullup[n] else 0
            if any(low for (d, p), low in self.od_low.items() if p == n):
                lvl = 0
            self.level[n] = lvl
            v |= lvl << n
        return v

    async def run(self):
        while self.running:
            await FallingEdge(self.dut.clk)
            self._resolve()
            for d in self.devices:
                d.tick(self)
            self.dut.uio_in.value = self._resolve()
            if self.ui:
                hi = 0
                for n, val in self.ui.items():
                    hi |= (val & 1) << (n - 3)
                self.host.set_inputs(hi)
            self.cycle += 1


# ---------------------------------------------------------------------------
class UartMonitor:
    """Decodes 8N1 frames from a pin getter."""

    def __init__(self, get, bit):
        self.get, self.bit = get, bit
        self.bytes, self.errors = [], 0
        self.t = None
        self.prev = 1

    def tick(self, b):
        v = self.get(b)
        if self.t is None:
            if self.prev == 1 and v == 0:
                self.t, self.val = 0, 0
        else:
            self.t += 1
            k = (self.t - self.bit // 2) // self.bit      # bit index at mid-bit
            if (self.t - self.bit // 2) % self.bit == 0:
                if k == 0 and v != 0:
                    self.t = None                      # glitch
                elif 1 <= k <= 8:
                    self.val |= v << (k - 1)
                elif k == 9:
                    if v == 1:
                        self.bytes.append(self.val)
                    else:
                        self.errors += 1
                    self.t = None
        self.prev = v


class UartDriver:
    """Sends 8N1 frames on a ui input (index 3..7)."""

    def __init__(self, ui_index, bit):
        self.n, self.bit = ui_index, bit
        self.queue = []
        self.bits, self.t = [], 0

    def send(self, data, bad_stop=False):
        for byte in data:
            self.queue.append([0] + [(byte >> i) & 1 for i in range(8)] + [0 if bad_stop else 1, 1])

    def tick(self, b):
        if not self.bits and self.queue:
            self.bits, self.t = self.queue.pop(0), 0
        if self.bits:
            b.ui[self.n] = self.bits[self.t // self.bit]
            self.t += 1
            if self.t >= self.bit * len(self.bits):
                self.bits = []
        else:
            b.ui[self.n] = 1

    @property
    def idle(self):
        return not self.bits and not self.queue


# ---------------------------------------------------------------------------
class SpiSlave:
    """Mode-0 SPI slave on uio pins; replies from a byte list."""

    def __init__(self, mosi, sck, miso, cs, reply):
        self.p = dict(mosi=mosi, sck=sck, miso=miso, cs=cs)
        self.reply = list(reply)
        self.rx, self.cur, self.n, self.out = [], 0, 0, 0
        self.prev_sck, self.selected = 0, False

    def tick(self, b):
        cs = b.io(self.p["cs"])
        sck = b.io(self.p["sck"])
        if cs:
            self.selected = False
            b.push[self.p["miso"]] = 0
        else:
            if not self.selected:
                self.selected, self.n, self.cur = True, 0, 0
                self.out = self.reply.pop(0) if self.reply else 0
                b.push[self.p["miso"]] = (self.out >> 7) & 1
            if sck and not self.prev_sck:                 # rising: sample
                self.cur = (self.cur << 1) | b.io(self.p["mosi"])
                self.n += 1
                if self.n == 8:
                    self.rx.append(self.cur)
                    self.cur, self.n = 0, 0
            if not sck and self.prev_sck:                 # falling: shift
                if self.n == 0:
                    self.out = self.reply.pop(0) if self.reply else 0
                    b.push[self.p["miso"]] = (self.out >> 7) & 1
                else:
                    b.push[self.p["miso"]] = (self.out >> (7 - self.n)) & 1
        self.prev_sck = sck


# ---------------------------------------------------------------------------
class I2CEeprom:
    """I2C slave: 256-byte memory with an address pointer. Optional clock
    stretching (holds SCL low for `stretch` cycles after every ACK bit)."""

    def __init__(self, sda, scl, addr=0x50, stretch=0):
        self.sda, self.scl, self.addr = sda, scl, addr
        self.mem = [0xFF] * 256
        self.ptr = 0
        self.stretch = stretch
        self.state = "idle"
        self.prev = (1, 1)
        self.log = []
        self.hold = 0
        self.sda_low = False

    def tick(self, b):
        scl, sda = b.io(self.scl), b.io(self.sda)
        pscl, psda = self.prev
        if self.hold:
            self.hold -= 1
            b.od_low[(self, self.scl)] = self.hold > 0
        if scl and pscl and psda and not sda:
            self.log.append("S")
            # bit = -1: the SCL fall that ends the START is not a data bit
            self.state, self.bit, self.byte, self.first = "addr", -1, 0, True
            self._sda(b, False)
        elif scl and pscl and not psda and sda:
            self.log.append("P")
            self.state = "idle"
            self._sda(b, False)
        elif self.state != "idle":
            if scl and not pscl:                          # rising edge
                if self.bit < 8 and self.state in ("addr", "wdata"):
                    self.byte = (self.byte << 1) | sda
                elif self.bit == 8 and self.state == "rdata":
                    self.master_ack = (sda == 0)
            elif not scl and pscl:                        # falling edge
                k = self.bit
                self.bit += 1
                if k < 0:
                    pass
                elif k == 7:
                    if self.state == "addr":
                        if (self.byte >> 1) == self.addr:
                            self.rw = self.byte & 1
                            self.log.append(f"A{self.byte:02x}")
                            self._sda(b, True)
                        else:
                            self.log.append(f"N{self.byte:02x}")
                            self.state = "idle"
                    elif self.state == "wdata":
                        if self.first:
                            self.ptr = self.byte
                            self.first = False
                        else:
                            self.mem[self.ptr] = self.byte
                            self.ptr = (self.ptr + 1) & 0xFF
                        self.log.append(f"W{self.byte:02x}")
                        self._sda(b, True)
                    else:
                        self._sda(b, False)               # release for master ACK
                elif k == 8:
                    self._sda(b, False)
                    self.bit, self.byte = 0, 0
                    if self.stretch:
                        self.hold = self.stretch
                        b.od_low[(self, self.scl)] = True
                    if self.state == "addr":
                        self.state = "rdata" if self.rw else "wdata"
                        if self.rw:
                            self._load(b)
                    elif self.state == "rdata":
                        if self.master_ack:
                            self._load(b)
                        else:
                            self.state = "idle"
                elif self.state == "rdata" and k < 7:
                    self._sda(b, not ((self.out >> (6 - k)) & 1))
        self.prev = (scl, sda)

    def _load(self, b):
        self.out = self.mem[self.ptr]
        self.ptr = (self.ptr + 1) & 0xFF
        self.log.append(f"R{self.out:02x}")
        self._sda(b, not ((self.out >> 7) & 1))

    def _sda(self, b, low):
        b.od_low[(self, self.sda)] = low
