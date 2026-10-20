# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""Fast firmware simulation on the reference model (no RTL simulator).

    s = ModelSim()
    s.load(0, prog, cfg)
    s.ctrl(en=1)
    s.push(0, words)
    s.run(10000, on_cycle=lambda s: ...)

Pads: uio pins resolve as (chip drives ? chip value : external/pull-up).
Register writes are applied as single-cycle host strobes, so timing
matches the chip (minus SPI transfer time).
"""

from .model import Chip
from .regs import creg, R_PC, G_CTRL, FIFO, ctrl_word


class ModelSim:
    def __init__(self, ncores=3):
        self.chip = Chip(ncores)
        self.ext = {}            # uio pin -> driven value
        self.pullup = set()
        self.ui = 0
        self.cycle = 0
        self.trace = []

    def pads(self):
        out = self.chip.outputs()
        v = 0
        for n in range(8):
            if (out["uio_oe"] >> n) & 1:
                lvl = (out["uio_out"] >> n) & 1
            elif n in self.ext:
                lvl = self.ext[n]
            else:
                lvl = 1 if n in self.pullup else 0
            v |= lvl << n
        return v

    def step(self, wr=None, rd=None):
        self.chip.step(self.ui | 0x02, self.pads(), wr=wr, rd=rd)
        self.cycle += 1

    def write(self, addr, data):
        if isinstance(data, int):
            data = [data]
        for i, d in enumerate(data):
            a = addr if (addr & 0xFC) == FIFO else addr + i
            self.step(wr=(a, d))
            self.step()
            self.step()

    def load(self, core, prog, cfg):
        self.write(prog.origin, prog.words)
        self.write(creg(core, 0), cfg.words())
        self.write(creg(core, R_PC), prog.wrap_target)

    def ctrl(self, **kw):
        self.write(G_CTRL, ctrl_word(**kw))

    def push(self, core, words):
        for w in words:
            while self.chip.txf[core].full:
                self.step()
            self.write(FIFO + core, w)

    def pop_all(self, core):
        out = []
        while self.chip.rxf[core].q:
            out.append(self.chip.read(FIFO + core))
            self.step(rd=FIFO + core)
        return out

    def run(self, n, on_cycle=None):
        for _ in range(n):
            if on_cycle:
                on_cycle(self)
            self.step()
