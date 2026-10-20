# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""Lockstep checker: runs the Python reference model next to the RTL.

Every falling clock edge it compares the RTL's pins and every core's
architectural state against the model, then feeds the model the inputs
and host-bus strobes the RTL will sample at the next rising edge.
"""

import os
import sys

from cocotb.triggers import FallingEdge

sys.path.insert(0, os.environ.get("POLYGLOT_SW", os.path.join(os.path.dirname(__file__), "..", "sw")))
from polyglot.model import Chip  # noqa: E402

CORE_FIELDS = {
    # model field: RTL signal
    "pc": "pc", "x": "x", "y": "y", "isr": "isr", "osr": "osr", "crc": "crc",
    "isr_cnt": "isr_cnt", "osr_cnt": "osr_cnt", "delay": "delay_cnt",
    "div": "div_cnt", "vout": "vout", "voe": "voe", "to": "to_flag",
    "armed": "irq_armed", "inj_pend": "inj_pend", "brk_skip": "brk_skip",
    "stalled": "stalled", "lflags": "lflags", "run_cnt": "run_cnt",
    "run_val": "run_val", "nrzi_prev": "nrzi_prev", "manch_pend": "manch_pend",
    "last_bit": "last_bit", "stuff_flag": "stuff_flag",
}


class Lockstep:
    def __init__(self, dut, check_state=True):
        self.dut = dut
        self.top = dut.user_project
        self.model = Chip()
        self.check_state = check_state
        self.cycles = 0
        self.errors = []
        self.running = True
        self._cores = [self.top.g_core[i].u_core for i in range(4)]

    def _v(self, sig):
        return int(sig.value)

    def compare(self):
        m = self.model
        dut = self.dut
        out = m.outputs()
        got = dict(uo_out=self._v(dut.uo_out), uio_out=self._v(dut.uio_out),
                   uio_oe=self._v(dut.uio_oe))
        # uo_out[0] (MISO) comes from the SPI block, which the model does not
        # own; it is fed back in so phys_in matches.
        for k in got:
            if got[k] != out[k]:
                self.errors.append(f"cycle {self.cycles}: {k} rtl={got[k]:#x} model={out[k]:#x}")
        if self._v(self.top.irq_flags) != m.irq:
            self.errors.append(f"cycle {self.cycles}: irq rtl={self._v(self.top.irq_flags):#x} model={m.irq:#x}")
        if self._v(self.top.core_en) != m.core_en:
            self.errors.append(f"cycle {self.cycles}: core_en rtl={self._v(self.top.core_en):#x} model={m.core_en:#x}")
        if not self.check_state:
            return
        for i, rc in enumerate(self._cores):
            mc = m.cores[i]
            for mf, rf in CORE_FIELDS.items():
                rv = self._v(getattr(rc, rf))
                mv = getattr(mc, mf)
                if rv != mv:
                    self.errors.append(
                        f"cycle {self.cycles}: core{i}.{mf} rtl={rv:#x} model={mv:#x} pc={mc.pc}")
            # pending manchester timer details only matter while pending
            if mc.manch_pend:
                for mf, rf in (("manch_cnt", "manch_cnt"), ("manch_lvl", "manch_lvl")):
                    rv, mv = self._v(getattr(rc, rf)), getattr(mc, mf)
                    if rv != mv:
                        self.errors.append(f"cycle {self.cycles}: core{i}.{mf} rtl={rv} model={mv}")
            for i2, (rt, mt) in enumerate(((self.top.g_core[i].u_txf.cnt, len(m.txf[i].q)),
                                           (self.top.g_core[i].u_rxf.cnt, len(m.rxf[i].q)))):
                if self._v(rt) != mt:
                    self.errors.append(f"cycle {self.cycles}: core{i} {'tx' if i2 == 0 else 'rx'} level rtl={self._v(rt)} model={mt}")

    async def run(self, max_errors=10):
        top = self.top
        while self.running:
            await FallingEdge(self.dut.clk)
            if not self.running:
                break
            if not self._v(self.dut.rst_n):
                self.model = Chip()
                continue
            self.model.miso = self._v(top.miso)
            self.compare()
            if len(self.errors) >= max_errors:
                self.running = False
                break
            wr = (self._v(top.h_waddr), self._v(top.h_wdata)) if self._v(top.h_wr) else None
            rd = self._v(top.h_addr) if self._v(top.h_rd) else None
            self.model.step(self._v(self.dut.ui_in), self._v(self.dut.uio_in),
                            wr=wr, rd=rd, miso=self._v(top.miso))
            self.cycles += 1

    def check(self):
        assert not self.errors, "lockstep mismatches:\n" + "\n".join(self.errors[:20])
