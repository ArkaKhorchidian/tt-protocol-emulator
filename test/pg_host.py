# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""cocotb host driver for the Polyglot SPI register interface.

Bit-bangs SPI mode 0 on ui_in[0]=SCK, ui_in[1]=CS_n, ui_in[2]=MOSI and
samples MISO on uo_out[0]. ui_in[7:3] stay available to tests as protocol
inputs through `set_inputs`.
"""

from cocotb.triggers import ClockCycles, RisingEdge

# register map
IMEM = 0x00
CORE = 0x80
R_CLKDIV, R_EXEC, R_SHIFT, R_PIN, R_PMLO, R_PMHI, R_LINE, R_POLY = range(8)
R_CRC, R_PC, R_X, R_Y, R_ISR, R_OSR, R_STATUS, R_INSTR = range(8, 16)
G_ID, G_CTRL, G_IRQ, G_IRQMASK, G_TIME, G_FIFOSTAT, G_BANKMAP, G_PINS = (
    0xC0 + i for i in range(8))
FIFO = 0xD0


def creg(core, reg):
    return CORE + 16 * core + reg


class Host:
    def __init__(self, dut, half=5):
        self.dut = dut
        self.half = half          # SCK half period in system clocks
        self.ui_hi = 0            # ui_in[7:3]
        self._ui(sck=0, cs=1, mosi=0)

    def _ui(self, sck, cs, mosi):
        self._sck, self._cs, self._mosi = sck, cs, mosi
        self.dut.ui_in.value = (self.ui_hi << 3) | (mosi << 2) | (cs << 1) | sck

    def set_inputs(self, v):
        """Drive ui_in[7:3] (5 bits)."""
        self.ui_hi = v & 0x1F
        self._ui(self._sck, self._cs, self._mosi)

    async def xfer(self, data):
        clk = self.dut.clk
        out = []
        self._ui(0, 0, 0)
        await ClockCycles(clk, self.half)
        for byte in data:
            rx = 0
            for i in range(7, -1, -1):
                self._ui(0, 0, (byte >> i) & 1)
                await ClockCycles(clk, self.half)
                rx = (rx << 1) | (int(self.dut.uo_out.value) & 1)
                self._ui(1, 0, (byte >> i) & 1)
                await ClockCycles(clk, self.half)
            out.append(rx)
        self._ui(0, 0, 0)
        await ClockCycles(clk, self.half)
        self._ui(0, 1, 0)
        await ClockCycles(clk, self.half)
        return out

    async def write(self, addr, words):
        if isinstance(words, int):
            words = [words]
        payload = [0x02, addr & 0xFF]
        for w in words:
            payload += list((w & 0xFFFFFFFF).to_bytes(4, "big"))
        await self.xfer(payload)

    async def read(self, addr, n=1):
        rx = await self.xfer([0x03, addr & 0xFF, 0x00] + [0] * (4 * n))
        data = rx[3:]
        words = [int.from_bytes(bytes(data[4 * i:4 * i + 4]), "big") for i in range(n)]
        return words if n > 1 else words[0]

    # ---- convenience ----
    async def load_program(self, words, base=0):
        await self.write(IMEM + base, words)

    async def ctrl(self, en=0, restart=0, step=0, divsync=0, fclr=0):
        await self.write(G_CTRL, en | (restart << 4) | (step << 8) |
                         (divsync << 12) | (fclr << 16))

    async def push(self, core, words):
        if isinstance(words, int):
            words = [words]
        await self.write(FIFO + core, words)

    async def pop(self, core, n=1):
        return await self.read(FIFO + core, n)

    async def fifo_levels(self, core):
        st = await self.read(G_FIFOSTAT)
        b = (st >> (8 * core)) & 0xFF
        return b & 7, (b >> 3) & 7    # tx, rx


async def reset(dut, cycles=10):
    dut.ena.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, cycles)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)
