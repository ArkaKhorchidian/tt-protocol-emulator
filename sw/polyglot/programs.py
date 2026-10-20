# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""Protocol firmware for Polyglot cores.

Each `*_program(origin)` returns an assembled Program; each `*_config(...)`
returns a CoreConfig. Pin arguments are physical crossbar indices (see
regs.py). Host-side helpers that encode FIFO words live next to them.
"""

from .asm import assemble
from .regs import CoreConfig, NRZI, MANCHESTER, ZERO

# ---------------------------------------------------------------------------
# UART (8N1, LSB first). One bit = 8 core ticks: clkdiv = f_clk/(8*baud) - 1
# ---------------------------------------------------------------------------
UART_TX = """
.program uart_tx
.side_set 1 opt
.wrap_target
    pull            side 1 [7]      ; idle/stop bit high, wait for a byte
    set x, 7        side 0 [7]      ; start bit
bit:
    out pins, 1                     ; data bit, LSB first
    jmp x--, bit    [6]
.wrap
"""

UART_RX = """
.program uart_rx
.wrap_target
start:
    wait 0 pin 0                    ; start bit edge
    set x, 7        [10]            ; to the middle of data bit 0 (1.5 bits)
bit:
    in pins, 1
    jmp x--, bit    [6]
    jmp pin, good                   ; stop bit must be high
    irq 4 rel                       ; framing error -> IRQ 4+core
    wait 1 pin 0
    jmp start
good:
    in null, 24                     ; right-align the byte
    push
.wrap
"""


# Packed receiver: autopush every 32 bits, so 4 bytes share one FIFO word
# (byte 0 in bits [7:0]). Quarter of the host bandwidth of uart_rx.
UART_RX_PACKED = """
.program uart_rx_packed
.wrap_target
start:
    wait 0 pin 0
    set x, 7        [10]
bit:
    in pins, 1                      ; autopush at 32 bits
    jmp x--, bit    [6]
    jmp pin, start                  ; good stop bit
    irq 4 rel                       ; framing error
    wait 1 pin 0
.wrap
"""


def uart_tx_program(origin=0):
    return assemble(UART_TX, origin)


def uart_rx_program(origin=0):
    return assemble(UART_RX, origin)


def uart_rx_packed_program(origin=0):
    return assemble(UART_RX_PACKED, origin)


def uart_config(prog, clkdiv, pin, rx=False, packed=False):
    c = CoreConfig(clkdiv=clkdiv).from_program(prog)
    c.pins = [pin] + [ZERO] * 7
    if rx:
        c.jmp_pin = 0
    if packed:
        c.autopush = True
        c.push_thresh = 32
    return c


def unpack_bytes(words, n):
    out = []
    for w in words:
        out += [(w >> (8 * i)) & 0xFF for i in range(4)]
    return bytes(out[:n])


# ---------------------------------------------------------------------------
# SPI master, mode 0, 8-bit, MSB first, full duplex.
# vpin0 = MOSI, vpin1 = SCK, vpin2 = MISO, vpin3 = CS_n (host-controlled
# through injected SET instructions). SCK = tick/4.
# Host pushes (byte << 24); received bytes come back right-aligned.
# ---------------------------------------------------------------------------
SPI_MASTER = """
.program spi_master
.side_set 1
.wrap_target
    out pins, 1     side 0 [1]      ; shift MOSI on the falling edge
    in pins, 1      side 1 [1]      ; sample MISO on the rising edge
.wrap
"""


def spi_master_program(origin=0):
    return assemble(SPI_MASTER, origin)


def spi_master_config(prog, clkdiv, mosi, sck, miso, cs):
    c = CoreConfig(clkdiv=clkdiv).from_program(prog)
    c.pins = [mosi, sck, miso, cs] + [ZERO] * 4
    c.side_base = 1
    c.out_base = 0
    c.in_base = 2
    c.autopull = c.autopush = True
    c.pull_thresh = c.push_thresh = 8
    c.out_right = False
    c.in_right = False
    c.set_base, c.set_count = 0, 4
    return c


# ---------------------------------------------------------------------------
# I2C master (open drain through pindirs). vpin0 = SDA, vpin1 = SCL.
# The host sends one command word per bus action; bits [6:0] are the
# absolute address of the handler (so the dispatch is a single `out pc, 7`).
# Clock stretching is honoured with a per-command timeout budget in Y.
# 45 instructions: needs two instruction banks.
# ---------------------------------------------------------------------------
I2C_MASTER = """
.program i2c_master
.side_set 1 opt pindirs             ; side = SCL, 1 pulls it low
.wrap_target
cmd:
    pull
    set y, 255                      ; clock-stretch budget (ticks)
    out pc, 7                       ; dispatch to handler
start:                              ; (repeated) START
    nop             side 1 [1]      ; SCL low
    set pindirs, 0         [1]      ; release SDA
    nop             side 0          ; release SCL
    wait 1 pin 1 timeout   [3]      ; wait for SCL high (stretching)
    set pindirs, 1         [3]      ; SDA falls while SCL high
    jmp cmd         side 1          ; SCL low
stop:
    nop             side 1 [1]
    set pindirs, 1         [1]      ; SDA low
    nop             side 0
    wait 1 pin 1 timeout   [3]
    set pindirs, 0         [3]      ; SDA rises while SCL high
    jmp !y, err
    jmp cmd
write:
    set x, 7
wbit:
    nop             side 1 [1]      ; SCL low
    out pindirs, 1         [1]      ; data (pre-inverted by host)
    nop             side 0
    wait 1 pin 1 timeout   [3]
    jmp x--, wbit
    nop             side 1 [1]
    set pindirs, 0         [1]      ; release SDA for ACK
    nop             side 0
    wait 1 pin 1 timeout   [2]
    in pins, 1                      ; ACK bit (0 = ACK)
    push            side 1
    jmp !y, err
    jmp cmd
read:
    set x, 7
    nop             side 1 [1]
    set pindirs, 0         [1]      ; release SDA
rbit:
    nop             side 0
    wait 1 pin 1 timeout   [2]
    in pins, 1
    nop             side 1 [3]
    jmp x--, rbit
    out null, 8
    out pindirs, 1         [1]      ; ACK (drive) / NACK (release)
    nop             side 0
    wait 1 pin 1 timeout   [3]
    nop             side 1 [1]
    set pindirs, 0
    push
    jmp !y, err
    jmp cmd
err:
    irq 0                           ; SCL stuck low: tell the host
    jmp cmd
.wrap
"""


def i2c_master_program(origin=0):
    return assemble(I2C_MASTER, origin)


def i2c_master_config(prog, clkdiv, sda, scl):
    c = CoreConfig(clkdiv=clkdiv).from_program(prog)
    c.pins = [sda, scl] + [ZERO] * 6
    c.side_base = 1
    c.set_base, c.set_count = 0, 1
    c.out_base = 0
    c.in_base = 0
    c.out_right = True
    c.in_right = False
    return c


def _rev8(b):
    return int(f"{b & 0xFF:08b}"[::-1], 2)


class I2CCommands:
    """Encodes host FIFO words for the i2c_master program."""

    def __init__(self, prog):
        self.l = prog.labels

    def start(self):
        return self.l["start"]

    def stop(self):
        return self.l["stop"]

    def write(self, byte):
        # LSB-first shifter emits MSB first after a bit reverse; pindirs
        # are inverted data (1 = drive low).
        return self.l["write"] | ((~_rev8(byte) & 0xFF) << 7)

    def read(self, ack=True):
        return self.l["read"] | (int(ack) << 15)
