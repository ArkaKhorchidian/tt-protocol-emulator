# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""Register map and configuration-word builders (docs/ISA.md, "Registers")."""

from dataclasses import dataclass, field

# ---- physical pin indices (crossbar space) ----
IO = list(range(0, 8))                       # uio[0..7]  bidirectional
UI = list(range(8, 16))                      # ui[0..7]   inputs (ui[0..2] = host SPI)
UO = list(range(16, 24))                     # uo[0..7]   outputs (uo[0..1] = host)
NET = list(range(24, 32))                    # internal nets between cores
IN = {n: 8 + n for n in range(3, 8)}         # ui[3..7]  free inputs
OUT = {n: 16 + n for n in range(2, 8)}       # uo[2..7]  free outputs
ZERO = NET[7]                                # an undriven net reads 0

# ---- addresses ----
IMEM = 0x00
R_CLKDIV, R_EXEC, R_SHIFT, R_PIN, R_PMLO, R_PMHI, R_LINE, R_POLY = range(8)
R_CRC, R_PC, R_X, R_Y, R_ISR, R_OSR, R_STATUS, R_INSTR = range(8, 16)
G_ID, G_CTRL, G_IRQ, G_IRQMASK, G_TIME, G_FIFOSTAT, G_BANKMAP, G_PINS = range(0xC0, 0xC8)
FIFO = 0xD0


def creg(core, reg):
    return 0x80 + 16 * core + reg


# ---- line modes ----
NRZ, NRZI, MANCHESTER = 0, 1, 2


@dataclass
class CoreConfig:
    clkdiv: int = 0
    wrap_bottom: int = 0
    wrap_top: int = 31
    side_mode: int = 0
    side_dirs: bool = False
    side_base: int = 0
    jmp_pin: int = 0
    brk: int = None
    autopull: bool = False
    autopush: bool = False
    out_right: bool = True          # True: LSB first
    in_right: bool = True           # True: bits enter at the MSB end
    pull_thresh: int = 32
    push_thresh: int = 32
    out_base: int = 0
    set_base: int = 0
    set_count: int = 1
    in_base: int = 0
    pins: list = field(default_factory=lambda: [ZERO] * 8)   # vpin -> physical
    line_mode: int = NRZ
    line_invert: bool = False
    line_diff: bool = False
    stuff_len: int = 0
    stuff_any: bool = False         # False: count runs of 1s (USB); True: any (CAN)
    crc_msb_first: bool = False
    crc_width: int = 16
    line_idle: int = 0
    manch_half: int = 0
    crc_poly: int = 0

    def words(self):
        """Values for core registers 0..7."""
        ex = (self.wrap_bottom & 0x7F) | ((self.wrap_top & 0x7F) << 8) | \
             (int(self.side_dirs) << 15) | ((self.side_mode & 3) << 16) | \
             ((self.side_base & 7) << 18) | ((self.jmp_pin & 7) << 21)
        if self.brk is not None:
            ex |= (1 << 31) | ((self.brk & 0x7F) << 24)
        sh = int(self.autopull) | (int(self.autopush) << 1) | (int(self.out_right) << 2) | \
             (int(self.in_right) << 3) | ((self.pull_thresh & 31) << 4) | \
             ((self.push_thresh & 31) << 9)
        pn = (self.out_base & 7) | ((self.set_base & 7) << 3) | \
             (((self.set_count - 1) & 7) << 6) | ((self.in_base & 7) << 9)
        pm = 0
        for k, p in enumerate((list(self.pins) + [ZERO] * 8)[:8]):
            pm |= (p & 31) << (5 * k)
        ln = (self.line_mode & 3) | (int(self.line_invert) << 2) | (int(self.line_diff) << 3) | \
             ((self.stuff_len & 7) << 4) | (int(self.stuff_any) << 7) | \
             (int(self.crc_msb_first) << 8) | (((self.crc_width - 1) & 31) << 9) | \
             ((self.line_idle & 1) << 14) | ((self.manch_half & 0xFF) << 16)
        return [self.clkdiv & 0xFFFF, ex, sh, pn, pm & 0xFFFFF, pm >> 20, ln,
                self.crc_poly & 0xFFFFFFFF]

    def from_program(self, prog):
        """Take wrap bounds and side-set mode from an assembled Program."""
        self.wrap_bottom = prog.wrap_target
        self.wrap_top = prog.wrap
        self.side_mode = prog.side_mode
        self.side_dirs = prog.side_dirs
        return self


def ctrl_word(en=0, restart=0, step=0, divsync=0, fclr=0):
    return en | (restart << 4) | (step << 8) | (divsync << 12) | (fclr << 16)
