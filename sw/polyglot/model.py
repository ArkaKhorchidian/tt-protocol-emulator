# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""Cycle-accurate reference model of the Polyglot chip (docs/ISA.md).

Written from the specification, instruction by instruction, independently
of the RTL's datapath structure. The cocotb lockstep test steps this model
alongside the RTL every clock and compares pins and architectural state.

Timing contract (cycle t = the clock period ending at rising edge t+1):
  * pad inputs pass through a 2-flop synchroniser
  * a host register write strobed in cycle t takes effect at edge t+1;
    instruction-memory and TX-FIFO (latch) writes become visible in t+2
  * CTRL restart/step/divsync/fifo-clear pulses act at edge t+2
"""

M32 = 0xFFFFFFFF


def rev32(v):
    return int(f"{v & M32:032b}"[::-1], 2)


def bswap32(v):
    return int.from_bytes((v & M32).to_bytes(4, "little"), "big")


def rotl8(v, s):
    v &= 0xFF
    return ((v << s) | (v >> (8 - s))) & 0xFF if s else v


def rotr8(v, s):
    v &= 0xFF
    return ((v >> s) | (v << (8 - s))) & 0xFF if s else v


def mask(n):
    return M32 if n >= 32 else (1 << n) - 1


class Cfg:
    """Decoded per-core configuration registers."""

    def __init__(self):
        self.clkdiv = 0
        self.exec_ = 0x00001F00
        self.shift = 0x000C
        self.pin = 0
        self.pmlo = 0
        self.pmhi = 0
        self.line = 0
        self.poly = 0

    def write(self, reg, v):
        if reg == 0: self.clkdiv = v & 0xFFFF
        elif reg == 1: self.exec_ = v & M32
        elif reg == 2: self.shift = v & 0x7FFF
        elif reg == 3: self.pin = v & 0xFFF
        elif reg == 4: self.pmlo = v & 0xFFFFF
        elif reg == 5: self.pmhi = v & 0xFFFFF
        elif reg == 6: self.line = v & 0xFFFFFF
        elif reg == 7: self.poly = v & M32

    # exec
    wrap_bot = property(lambda s: s.exec_ & 0x7F)
    wrap_top = property(lambda s: (s.exec_ >> 8) & 0x7F)
    side_dirs = property(lambda s: (s.exec_ >> 15) & 1)
    side_mode = property(lambda s: (s.exec_ >> 16) & 3)
    side_base = property(lambda s: (s.exec_ >> 18) & 7)
    jmp_pin = property(lambda s: (s.exec_ >> 21) & 7)
    brk_addr = property(lambda s: (s.exec_ >> 24) & 0x7F)
    brk_en = property(lambda s: (s.exec_ >> 31) & 1)
    # shift
    autopull = property(lambda s: s.shift & 1)
    autopush = property(lambda s: (s.shift >> 1) & 1)
    out_right = property(lambda s: (s.shift >> 2) & 1)
    in_right = property(lambda s: (s.shift >> 3) & 1)
    pull_th = property(lambda s: ((s.shift >> 4) & 31) or 32)
    push_th = property(lambda s: ((s.shift >> 9) & 31) or 32)
    pull_imem = property(lambda s: (s.shift >> 14) & 1)
    # pins
    out_base = property(lambda s: s.pin & 7)
    set_base = property(lambda s: (s.pin >> 3) & 7)
    set_count = property(lambda s: ((s.pin >> 6) & 7) + 1)
    in_base = property(lambda s: (s.pin >> 9) & 7)
    # line coder
    line_mode = property(lambda s: s.line & 3)
    line_inv = property(lambda s: (s.line >> 2) & 1)
    line_diff = property(lambda s: (s.line >> 3) & 1)
    stuff_len = property(lambda s: (s.line >> 4) & 7)
    stuff_any = property(lambda s: (s.line >> 7) & 1)
    crc_msb = property(lambda s: (s.line >> 8) & 1)
    crc_w = property(lambda s: ((s.line >> 9) & 31) + 1)
    line_idle = property(lambda s: (s.line >> 14) & 1)
    manch_half = property(lambda s: (s.line >> 16) & 0xFF)

    def pinmap(self, k):
        v = (self.pmhi << 20) | self.pmlo
        return (v >> (5 * k)) & 31


COVERAGE = {}     # (event) -> count, filled while stepping


def cov(key):
    COVERAGE[key] = COVERAGE.get(key, 0) + 1


class Core:
    FIELDS = ("pc dptr x y isr osr crc isr_cnt osr_cnt delay div vin_prev vout voe "
              "to armed inj_pend inj_instr brk_skip stalled lflags run_cnt "
              "run_val nrzi_prev manch_pend manch_cnt manch_lvl last_bit "
              "stuff_flag").split()

    def __init__(self, cid):
        self.cid = cid
        self.cfg = Cfg()
        for f in self.FIELDS:
            setattr(self, f, 0)
        self.osr_cnt = 32

    def snapshot(self):
        return {f: getattr(self, f) for f in self.FIELDS}

    def status(self):
        s = self
        return (s.pc | (s.stalled << 7) | (s.to << 8) | (((s.lflags >> 3) & 1) << 9) |
                (s.stuff_flag << 10) | (s.last_bit << 11) | (s.brk_skip << 12) |
                ((s.lflags & 1) << 13) | (((s.lflags >> 1) & 1) << 14) |
                (((s.lflags >> 2) & 1) << 15) | (s.osr_cnt << 16) | (s.isr_cnt << 24))

    def do_restart(self):
        self.isr = 0; self.osr = 0; self.isr_cnt = 0; self.osr_cnt = 32
        self.delay = 0; self.div = 0; self.to = 0; self.armed = 0
        self.inj_pend = 0; self.brk_skip = 0; self.stalled = 0; self.lflags = 0
        self.run_cnt = 0; self.run_val = 0; self.nrzi_prev = self.cfg.line_idle
        self.manch_pend = 0; self.stuff_flag = 0

    # ------------------------------------------------------------------
    def step(self, en, step, vin, fetched, tx_empty, tx_data, rx_full, irq, time, dmem=0):
        """Compute one clock. Returns (next_state_dict, effects)."""
        c = self.cfg
        s = self
        n = self.snapshot()
        fifo_empty = tx_empty              # for the txne jump condition
        if c.pull_imem:                    # pulls come from imem[dptr]
            tx_empty, tx_data = False, dmem
        eff = dict(tx_pop=False, rx_push=None, irq_set=0, irq_clr=0, brk_hit=False)

        run_tick = en and s.div == 0
        tick = run_tick or step or (s.inj_pend and not en)
        no_delay = s.delay == 0
        at_brk = (c.brk_en and s.pc == c.brk_addr and not s.inj_pend and
                  not s.brk_skip and not step)
        brk_hit = run_tick and no_delay and at_brk
        eff["brk_hit"] = brk_hit
        if brk_hit:
            cov("breakpoint")
        exe = tick and no_delay and not brk_hit

        # clock divider
        if not en:
            n["div"] = 0
        elif s.div == 0:
            n["div"] = c.clkdiv
        else:
            n["div"] = s.div - 1

        if tick:
            n["vin_prev"] = vin

        # Manchester second half-bit: fires on every tick
        line_drive = None              # level to put on the line (pre-inversion)
        if tick and s.manch_pend:
            if s.manch_cnt <= 1:
                n["manch_pend"] = 0
                n["manch_cnt"] = 0
                line_drive = s.manch_lvl
            else:
                n["manch_cnt"] = s.manch_cnt - 1

        if tick and not no_delay:
            n["delay"] = s.delay - 1

        if brk_hit:
            n["brk_skip"] = 1

        vout, voe = s.vout, s.voe
        if not exe:
            if line_drive is not None:
                vout = self._line(vout, line_drive)
            n["vout"], n["voe"] = vout, voe
            return n, eff

        instr = s.inj_instr if s.inj_pend else fetched
        op = (instr >> 16) & 15
        ss = (instr >> 14) & 3
        dly = (instr >> 11) & 7
        arg = instr & 0x7FF
        hi = (arg >> 8) & 7

        sm = c.side_mode
        if sm == 0:
            delay_val, side_en, side_two = (ss << 3) | dly, False, False
        elif sm == 1:
            delay_val, side_en, side_two = ((ss >> 1) << 3) | dly, True, False
        elif sm == 2:
            delay_val, side_en, side_two = dly, True, True
        else:
            delay_val, side_en, side_two = dly, bool(ss >> 1), False

        cnt = (arg & 31) or 32
        stall = False
        jump = None
        pin_write = None               # (is_dir, base, count, data)

        stuff_active = (s.lflags >> 1) & 1 and c.stuff_len != 0
        stuff_due = stuff_active and s.run_cnt == c.stuff_len
        code_nrzi = (s.lflags >> 2) & 1 and c.line_mode == 1
        code_manch = (s.lflags >> 2) & 1 and c.line_mode == 2
        vin_rot = rotr8(vin, c.in_base)

        def src_val(sel, line_bit=None):
            return {0: vin_rot, 1: s.x, 2: s.y, 3: 0,
                    4: (line_bit if line_bit is not None else s.crc),
                    5: time, 6: s.isr, 7: s.osr}[sel]

        def crc_bit(b):
            w = c.crc_w
            if c.crc_msb:
                fb = ((s.crc >> (w - 1)) & 1) ^ b
                v = (s.crc << 1) ^ (c.poly if fb else 0)
            else:
                fb = (s.crc & 1) ^ b
                v = (s.crc >> 1) ^ (c.poly if fb else 0)
            return v & mask(w)

        def run_track(b):
            if not c.stuff_any:
                n["run_val"] = 1
                n["run_cnt"] = min(s.run_cnt + 1, 7) if b else 0
            elif b == s.run_val and s.run_cnt != 0:
                n["run_cnt"] = min(s.run_cnt + 1, 7)
            else:
                n["run_val"] = b
                n["run_cnt"] = 1

        # -------------------------------------------------------- JMP
        if op == 0:
            cc = (arg >> 7) & 15
            if cc == 2: n["x"] = (s.x - 1) & M32
            if cc == 4: n["y"] = (s.y - 1) & M32
            cond = [True, s.x == 0, s.x != 0, s.y == 0, s.y != 0, s.x != s.y,
                    bool((vin >> c.jmp_pin) & 1), not ((vin >> c.jmp_pin) & 1),
                    s.osr_cnt < c.pull_th, not fifo_empty, bool(s.to),
                    bool(stuff_due), bool((s.lflags >> 3) & 1), s.x < s.y,
                    bool(s.last_bit), s.isr_cnt >= c.push_th][cc]
            if cond:
                jump = arg & 0x7F

        # -------------------------------------------------------- WAIT
        elif op == 1:
            pol, wsrc, to_en, idx = (arg >> 10) & 1, (arg >> 8) & 3, (arg >> 7) & 1, arg & 7
            if wsrc == 0:
                ok = ((vin >> idx) & 1) == pol
            elif wsrc == 1:
                ok = ((irq >> idx) & 1) == pol
            elif wsrc == 2:
                ok = ((vin >> idx) & 1) == pol and ((s.vin_prev >> idx) & 1) != pol
            else:
                ok = True
            if ok:
                if to_en: n["to"] = 0
                if wsrc == 1 and pol: eff["irq_clr"] |= 1 << idx
            elif to_en and s.y == 0:
                n["to"] = 1
            else:
                stall = True
                if to_en:
                    n["y"] = (s.y - 1) & M32     # survives the stall

        # -------------------------------------------------------- IN
        elif op == 2:
            line = hi == 4
            if line:
                lvl = ((vin >> c.in_base) & 1) ^ c.line_inv
                b = int(lvl == s.nrzi_prev) if code_nrzi else lvl
                cnt = 1
                val = b
            else:
                val = src_val(hi)
            is_stuff = line and stuff_due
            if not is_stuff:
                v = val & mask(cnt)
                if cnt == 32:
                    new = val & M32
                elif c.in_right:
                    new = ((s.isr >> cnt) | (v << (32 - cnt))) & M32
                else:
                    new = ((s.isr << cnt) | v) & M32
                new_cnt = min(s.isr_cnt + cnt, 32)
                push = c.autopush and new_cnt >= c.push_th
                if push and rx_full:
                    stall = True
                elif push:
                    eff["rx_push"] = new
                    n["isr"], n["isr_cnt"] = 0, 0
                else:
                    n["isr"], n["isr_cnt"] = new, new_cnt
            if line and not stall:
                n["nrzi_prev"] = lvl
                n["stuff_flag"] = int(is_stuff)
                if is_stuff:
                    if b == s.run_val:
                        n["lflags"] |= 8          # stuff violation
                else:
                    n["last_bit"] = b
                    if s.lflags & 1:
                        n["crc"] = crc_bit(b)
                if stuff_active:
                    run_track(b)

        # -------------------------------------------------------- OUT
        elif op == 3:
            line = hi == 4
            if line:
                cnt = 1
            if line and stuff_due:
                b = 1 - s.run_val                 # stuff bit, consumes nothing
                n["stuff_flag"] = 1
                run_track(b)
                line_drive = self._tx_level(b, code_nrzi, code_manch, n)
            else:
                osr, ocnt = s.osr, s.osr_cnt
                refill = c.autopull and ocnt >= c.pull_th
                if refill and tx_empty:
                    stall = True
                else:
                    if refill:
                        eff["tx_pop"] = True
                        osr, ocnt = tx_data, 0
                        if c.pull_imem:
                            n["dptr"] = (s.dptr + 1) & 0x7F
                    if c.out_right:
                        data = osr & mask(cnt)
                        n["osr"] = 0 if cnt == 32 else osr >> cnt
                    else:
                        data = osr >> (32 - cnt)
                        n["osr"] = 0 if cnt == 32 else (osr << cnt) & M32
                    n["osr_cnt"] = min(ocnt + cnt, 32)
                    if hi in (0, 5):
                        pin_write = (hi == 5, c.out_base, cnt, data)
                    elif hi == 1: n["x"] = data
                    elif hi == 2: n["y"] = data
                    elif hi == 4:
                        b = data & 1
                        n["stuff_flag"] = 0
                        n["last_bit"] = b
                        if s.lflags & 1:
                            n["crc"] = crc_bit(b)
                        if stuff_active:
                            run_track(b)
                        line_drive = self._tx_level(b, code_nrzi, code_manch, n)
                    elif hi == 6: jump = data & 0x7F
                    elif hi == 7: n["isr"], n["isr_cnt"] = data, cnt

        # -------------------------------------------------------- PUSH/PULL
        elif op == 4:
            pull, ifc, block = (arg >> 7) & 1, (arg >> 6) & 1, (arg >> 5) & 1
            if not pull:
                if ifc and s.isr_cnt < c.push_th:
                    pass
                elif rx_full:
                    if block: stall = True
                    else: n["isr"], n["isr_cnt"] = 0, 0
                else:
                    eff["rx_push"] = s.isr
                    n["isr"], n["isr_cnt"] = 0, 0
            else:
                if ifc and s.osr_cnt < c.pull_th:
                    pass
                elif tx_empty:
                    if block: stall = True
                    else: n["osr"], n["osr_cnt"] = s.x, 0
                else:
                    eff["tx_pop"] = True
                    n["osr"], n["osr_cnt"] = tx_data, 0
                    if c.pull_imem:
                        n["dptr"] = (s.dptr + 1) & 0x7F

        # -------------------------------------------------------- MOV
        elif op == 5:
            v = src_val((arg >> 3) & 7)
            mop = (arg >> 6) & 3
            v = [v, ~v & M32, rev32(v), bswap32(v)][mop]
            if hi == 0: pin_write = (False, c.out_base, 8, v)
            elif hi == 1: n["x"] = v
            elif hi == 2: n["y"] = v
            elif hi == 3: pin_write = (True, c.out_base, 8, v)
            elif hi == 4: n["crc"] = v & mask(c.crc_w)
            elif hi == 5: jump = v & 0x7F
            elif hi == 6: n["isr"], n["isr_cnt"] = v, 0
            else: n["osr"], n["osr_cnt"] = v, 0

        # -------------------------------------------------------- IRQ
        elif op == 6:
            idx = arg & 7
            if (arg >> 4) & 1:
                idx = (idx & 4) | ((idx + self.cid) & 3)
            if (arg >> 6) & 1:
                eff["irq_clr"] |= 1 << idx
            elif (arg >> 5) & 1:
                if not s.armed:
                    eff["irq_set"] |= 1 << idx
                    n["armed"] = 1
                    stall = True
                elif (irq >> idx) & 1:
                    stall = True
                else:
                    n["armed"] = 0
            else:
                eff["irq_set"] |= 1 << idx

        # -------------------------------------------------------- SET
        elif op == 7:
            v = arg & 0xFF
            if hi == 0: pin_write = (False, c.set_base, c.set_count, v)
            elif hi == 1: n["x"] = v
            elif hi == 2: n["y"] = v
            elif hi == 3: pin_write = (True, c.set_base, c.set_count, v)
            elif hi == 4: n["dptr"] = v & 0x7F

        # -------------------------------------------------------- ADD
        elif op == 8:
            imm = arg & 0x3FF
            if imm & 0x200:
                imm -= 0x400
            if (arg >> 10) & 1: n["y"] = (s.y + imm) & M32
            else: n["x"] = (s.x + imm) & M32

        # -------------------------------------------------------- CTL
        elif op == 9:
            m, v = (arg >> 4) & 15, arg & 15
            n["lflags"] = (s.lflags & ~m & 15) | (v & m)
            if (arg >> 8) & 1:
                n["run_cnt"] = 0
                n["run_val"] = 0
                n["nrzi_prev"] = c.line_idle
                n["manch_pend"] = 0
                n["stuff_flag"] = 0

        # ---------------------------------------------------- stall rollback
        if stall:
            keep = {f: getattr(s, f) for f in self.FIELDS}
            # only these survive a stall: timers, divider, side-set (below),
            # WAIT-timeout Y decrement, IRQ-wait arming
            for f in ("div", "vin_prev", "manch_pend", "manch_cnt", "brk_skip"):
                keep[f] = n[f]
            if op == 1:
                keep["y"] = n["y"]
            if op == 6:
                keep["armed"] = n["armed"]
            else:
                eff["irq_set"] = 0
            eff["irq_clr"] = 0
            eff["tx_pop"] = False
            eff["rx_push"] = None
            n = keep
            # a stalled OUT LINE does not drive; the Manchester timer still may
            if not (op == 3 and hi == 4) or line_drive is None:
                pass
        n["stalled"] = int(stall)
        OPN = ["jmp", "wait", "in", "out", "pp", "mov", "irq", "set", "add", "ctl"]
        name = OPN[op] if op < 10 else "rsvd"
        cov(f"{name}{'.stall' if stall else ''}")
        if op in (2, 3) and hi == 4:
            cov(f"line.{name}{'.stuff' if (stuff_due) else ''}{'.nrzi' if code_nrzi else '.manch' if code_manch else ''}")
        if op == 0 and jump is not None:
            cov("jmp.taken")
        if eff["rx_push"] is not None and not stall:
            cov("rx_push")
        if eff["tx_pop"] and not stall:
            cov("tx_pop")
        if side_en:
            cov("sideset")
        if s.inj_pend:
            cov("injected")

        # ---------------------------------------------------- pins
        if line_drive is not None:
            vout = self._line(vout, line_drive)
        if pin_write and not stall:
            is_dir, base, count, data = pin_write
            m = rotl8(mask(min(count, 8)) & 0xFF, base)
            d = rotl8(data & 0xFF, base)
            if is_dir: voe = (voe & ~m) | (d & m)
            else: vout = (vout & ~m) | (d & m)
        if side_en:
            m = rotl8(3 if side_two else 1, c.side_base)
            d = rotl8(ss & (3 if side_two else 1), c.side_base)
            if c.side_dirs: voe = (voe & ~m) | (d & m)
            else: vout = (vout & ~m) | (d & m)
        n["vout"], n["voe"] = vout & 0xFF, voe & 0xFF

        # ---------------------------------------------------- commit
        if not stall:
            n["delay"] = delay_val
            n["brk_skip"] = 0
            n["inj_pend"] = 0
            if jump is not None:
                n["pc"] = jump
            elif s.inj_pend:
                n["pc"] = s.pc
            elif s.pc == c.wrap_top:
                n["pc"] = c.wrap_bot
            else:
                n["pc"] = (s.pc + 1) & 0x7F
        return n, eff

    def _tx_level(self, b, nrzi, manch, n):
        c = self.cfg
        n["manch_cnt"] = c.manch_half
        n["manch_lvl"] = b
        if nrzi:
            lvl = self.nrzi_prev if b else 1 - self.nrzi_prev
            n["nrzi_prev"] = lvl
            n["manch_pend"] = 0
            return lvl
        if manch:
            n["manch_pend"] = 1
            return 1 - b
        n["manch_pend"] = 0
        return b

    def _line(self, vout, level):
        c = self.cfg
        lv = level ^ c.line_inv
        b = c.out_base
        vout = (vout & ~(1 << b)) | (lv << b)
        if c.line_diff:
            b1 = (b + 1) & 7
            vout = (vout & ~(1 << b1)) | ((1 - lv) << b1)
        return vout & 0xFF


class TxFifo:
    """Latch FIFO: entries become visible one cycle after the push."""

    def __init__(self, present=True):
        self.q, self.pend, self.present = [], None, present

    @property
    def full(self):
        return not self.present or len(self.q) + (self.pend is not None) >= 4


RX_DEPTH = 2


class RxFifo:
    def __init__(self):
        self.q = []

    @property
    def full(self):
        return len(self.q) >= RX_DEPTH


NCORES = 3


class Chip:
    def __init__(self, ncores=NCORES):
        self.ncores = ncores
        self.cores = [Core(i) for i in range(4)]
        self.imem = [None] * 128          # None = never written (X in RTL)
        self.imem_pend = None
        self.txf = [TxFifo(i < ncores) for i in range(4)]
        self.rxf = [RxFifo() for _ in range(4)]
        self.core_en = 0
        self.halted = 0
        self.irq_mask = 0
        self.bankmap = 0xE4
        self.irq = 0
        self.timer = 0
        self.pulses = (0, 0, 0, 0)        # restart, step, divsync, fclr
        self.sync1 = 0x0200
        self.sync2 = 0x0200
        self.miso = 0

    # -------- combinational views of the current state --------
    def drive(self):
        """Returns (drv_val, drv_oe) as 32-bit physical vectors."""
        val = oe = 0
        for c in self.cores:
            for k in range(8):
                if (c.voe >> k) & 1:
                    p = c.cfg.pinmap(k)
                    oe |= 1 << p
                    val |= ((c.vout >> k) & 1) << p
        return val, oe

    def irq_pin(self):
        rx_ne = sum((len(f.q) > 0) << i for i, f in enumerate(self.rxf))
        tx_nf = sum((not f.full) << i for i, f in enumerate(self.txf))
        m = self.irq_mask
        return int(bool((self.irq & m & 0xFF) | (rx_ne & (m >> 8) & 15) |
                        (tx_nf & (m >> 12) & 15)))

    def outputs(self):
        val, oe = self.drive()
        uo = (((val >> 18) & 0x3F) << 2) | (self.irq_pin() << 1) | self.miso
        return dict(uo_out=uo, uio_out=val & 0xFF, uio_oe=oe & 0xFF)

    def phys_in(self):
        val, _ = self.drive()
        uo = self.outputs()["uo_out"]
        return ((val >> 24) & 0xFF) << 24 | uo << 16 | self.sync2

    def bank_word(self, bank):
        """Read port of a bank: owner's pc if it executes there, else its dptr."""
        owner = (self.bankmap >> (2 * bank)) & 3
        if owner < self.ncores:
            o = self.cores[owner]
            a = o.pc if (o.pc >> 5) == bank else o.dptr
        else:
            a = 0
        w = self.imem[bank * 32 + (a & 31)]
        return 0 if w is None else w

    def fetch(self, cid):
        return self.bank_word(self.cores[cid].pc >> 5)

    def fetch_data(self, cid):
        return self.bank_word(self.cores[cid].dptr >> 5)

    # -------- host access --------
    def read(self, addr):
        """Combinational host read (no side effects)."""
        if addr & 0xC0 == 0x80:
            c = self.cores[(addr >> 4) & 3]
            r = addr & 15
            if c.cid >= self.ncores:
                return 0
            return {8: c.crc, 9: c.pc, 10: c.x, 11: c.y, 12: c.isr, 13: c.osr,
                    14: c.status(), 15: self.fetch_raw(c.cid)}.get(r, 0)
        if addr & 0xF0 == 0xC0:
            r = addr & 15
            if r == 0: return 0x504C5947
            if r == 1: return (self.halted << 8) | self.core_en
            if r == 2: return self.irq
            if r == 3: return self.irq_mask
            if r == 4: return self.timer
            if r == 5:
                v = 0
                for i in range(4):
                    v |= ((self.cores[i].stalled << 7) | (((self.halted >> i) & 1) << 6) |
                          (len(self.rxf[i].q) << 3) | len(self.txf[i].q)) << (8 * i)
                return v
            if r == 6: return self.bankmap
            if r == 7: return self.phys_in()
            return 0
        if addr & 0xFC == 0xD0:
            q = self.rxf[addr & 3].q
            return q[0] if q else 0
        return 0

    def fetch_raw(self, cid):
        return self.fetch(cid)

    # -------- one clock --------
    def step(self, ui_in, uio_in, wr=None, rd=None, miso=0):
        """Advance one rising edge.

        wr: (addr, data) host write strobe present during this cycle
        rd: addr of a host read commit (FIFO pop) during this cycle
        miso: value of the SPI MISO flop during this cycle (not modelled)
        """
        self.miso = miso
        phys = self.phys_in()
        restart, step, divsync, fclr = self.pulses
        new_pulses = (0, 0, 0, 0)

        # ---- cores ----
        results = []
        for i, c in enumerate(self.cores[:self.ncores]):
            vin = 0
            for k in range(8):
                vin |= ((phys >> c.cfg.pinmap(k)) & 1) << k
            tf, rf = self.txf[i], self.rxf[i]
            tx_empty = len(tf.q) == 0
            results.append(c.step(
                en=bool((self.core_en >> i) & 1), step=bool((step >> i) & 1),
                vin=vin, fetched=self.fetch(i), dmem=self.fetch_data(i), tx_empty=tx_empty,
                tx_data=tf.q[0] if tf.q else 0, rx_full=rf.full,
                irq=self.irq, time=self.timer))

        # ---- next state (all decisions use pre-edge state) ----
        brk = sum(r[1]["brk_hit"] << i for i, r in enumerate(results))
        iset = iclr = 0
        for n, eff in results:
            iset |= eff["irq_set"]
            iclr |= eff["irq_clr"]
        tx_full_pre = [f.full for f in self.txf]
        rx_empty_pre = [len(f.q) == 0 for f in self.rxf]

        for i, (n, eff) in enumerate(results):
            tf, rf = self.txf[i], self.rxf[i]
            if eff["tx_pop"] and tf.q and not self.cores[i].cfg.pull_imem:
                tf.q.pop(0)
            if eff["rx_push"] is not None and not rf.full:
                rf.q.append(eff["rx_push"])
            if tf.pend is not None:            # latch write completes
                tf.q.append(tf.pend)
                tf.pend = None

        if rd is not None and rd & 0xFC == 0xD0 and not rx_empty_pre[rd & 3]:
            self.rxf[rd & 3].q.pop(0)

        for i, c in enumerate(self.cores[:self.ncores]):
            for f, v in results[i][0].items():
                setattr(c, f, v)
            if (restart >> i) & 1:
                c.do_restart()
            if (divsync >> i) & 1:
                c.div = 0
            if (fclr >> i) & 1:
                self.txf[i] = TxFifo(True)
                self.rxf[i] = RxFifo()

        if self.imem_pend is not None:
            a, d = self.imem_pend
            self.imem[a] = d
            self.imem_pend = None

        hset = hclr = 0
        halted_pre = self.halted
        self.core_en &= ~brk & 15
        self.halted |= brk
        self.timer = (self.timer + 1) & M32

        # ---- host write takes effect at this edge ----
        if wr is not None:
            addr, data = wr
            data &= M32
            if addr < 0x80:
                self.imem_pend = (addr, data & 0xFFFFF)
            elif addr & 0xC0 == 0x80 and ((addr >> 4) & 3) < self.ncores:
                c = self.cores[(addr >> 4) & 3]
                r = addr & 15
                if r < 8: c.cfg.write(r, data)
                elif r == 8: c.crc = data
                elif r == 9: c.pc = data & 0x7F
                elif r == 10: c.x = data
                elif r == 11: c.y = data
                elif r == 12: c.isr, c.isr_cnt = data, 0
                elif r == 13: c.osr, c.osr_cnt = data, 0
                elif r == 15: c.inj_instr, c.inj_pend = data & 0xFFFFF, 1
            elif addr & 0xF0 == 0xC0:
                r = addr & 15
                if r == 1:
                    self.core_en = data & ((1 << self.ncores) - 1)
                    self.halted = halted_pre & ~data & 15
                    new_pulses = ((data >> 4) & 15, (data >> 8) & 15,
                                  (data >> 12) & 15, (data >> 16) & 15)
                elif r == 2:
                    hclr, hset = data & 0xFF, (data >> 8) & 0xFF
                elif r == 3: self.irq_mask = data & 0xFFFF
                elif r == 4: self.timer = data
                elif r == 6: self.bankmap = data & 0xFF
            elif addr & 0xFC == 0xD0:
                i = addr & 3
                if not tx_full_pre[i] and not (fclr >> i) & 1:
                    self.txf[i].pend = data
        self.irq = ((self.irq & ~(iclr | hclr)) | iset | hset) & 0xFF
        self.pulses = new_pulses

        # synchroniser
        self.sync2 = self.sync1
        self.sync1 = ((ui_in & 0xFF) << 8) | (uio_in & 0xFF)
