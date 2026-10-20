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


# ---------------------------------------------------------------------------
# USB low speed (1.5 Mbit/s): one bit = 32 ticks (48 MHz clock, CLKDIV 0).
# vpin0 = D-, vpin1 = D+. Low-speed J = D- high. The line coder does NRZI,
# stuffing after six 1s and CRC16; the `stuffdue` checks keep every bit
# exactly 32 ticks long, stuff bits included.
#
# TX FIFO: [payload_bits - 1] then the bit stream SYNC, PID, payload packed
# LSB first into 32-bit words. CRC16 and EOP are generated in hardware.
# ---------------------------------------------------------------------------
USB_TX = """
.program usb_tx
.wrap_target
    out x, 32               ; payload bits - 1 (blocks on the FIFO)
    set pins, 1             ; J
    set pindirs, 3
    ctl reset, code on, stuff on, crc off, err clear
    mov crc, ~null          ; CRC16 seed 0xFFFF
    set y, 14
hdr:                        ; SYNC + PID: 16 bits, never stuffed
    out line, 1     [30]
    jmp y--, hdr
    out line, 1     [29]
    ctl crc on
pay:
    jmp stuffdue, pstuff    ;  1
    out line, 1     [27]    ; 29
    jmp x--, ppad           ; 30
    mov osr, ~crc           ; 31  CRC16 goes out inverted, LSB first
    set y, 15               ; 32
crc:
    jmp stuffdue, cstuff
    out line, 1     [27]
    jmp y--, cpad
    jmp stuffdue, tstuff [1] ; a stuff bit may follow the last CRC bit
eop:
    set pins, 0     [31]    ; SE0
    out null, 32    [31]    ; SE0 (and empty the OSR)
    set pins, 1     [31]    ; J
    set pindirs, 0
    irq 0 rel               ; packet sent
.wrap
ppad:
    jmp pay         [1]
pstuff:
    out line, 1     [29]
    jmp pay
cpad:
    jmp crc         [1]
cstuff:
    out line, 1     [29]
    jmp crc
tstuff:
    out line, 1     [30]
    jmp eop
"""

# RX: wait for the J->K edge of SYNC, sample mid-bit, stop at SE0.
# vpins 2-7 must read 0 (map them to ZERO) so `mov y, pins` == 0 means SE0.
# Bits are packed 32 per FIFO word (autopush), so a whole low-speed packet
# fits in the 4-deep FIFO. At EOP it pushes: the partial word, ~(payload
# bit count), then the CRC16 register (0xB001 = good). IRQ 1+core.
USB_RX = """
.program usb_rx
.wrap_target
    ctl reset, code on, stuff on, crc off, err clear
    mov crc, ~null
    wait 0 edge 0           ; J -> K: start of SYNC
    set x, 15       [14]    ; to the middle of the first bit
hdr:                        ; SYNC + PID (x ends at -1)
    mov y, pins
    jmp !y, eop
    in line, 1
    jmp x--, hdr    [28]
    ctl crc on
bit:
    mov y, pins
    jmp !y, eop             ; SE0
    jmp stuffdue, sbit
    in line, 1              ; NRZI decode + CRC in hardware
    jmp x--, bit    [27]    ; count payload bits
sbit:
    in line, 1              ; stuff bit: dropped (and checked) by hardware
    jmp bit         [27]
eop:
    push                    ; partial word (bits at the top)
    mov isr, x
    push
    mov isr, crc
    push
    irq 1 rel
    wait 1 pin 0            ; back to J
.wrap
"""


def usb_tx_program(origin=0):
    return assemble(USB_TX, origin)


def usb_rx_program(origin=0):
    return assemble(USB_RX, origin)


def usb_config(prog, dminus, dplus, rx=False):
    c = CoreConfig().from_program(prog)
    c.pins = [dminus, dplus] + [ZERO] * 6
    c.out_base = c.in_base = 0
    c.set_base, c.set_count = 0, 2
    c.line_mode = NRZI
    c.line_diff = not rx
    c.line_idle = 1                     # J
    c.stuff_len = 6
    c.crc_width, c.crc_poly = 16, 0xA001
    c.autopull, c.pull_thresh = True, 32
    if rx:
        c.autopush, c.push_thresh = True, 32
    return c


def usb_rx_decode(words):
    """Unpack usb_rx FIFO output -> (bytes incl. SYNC/PID, crc16 register)."""
    *data, partial, nx, crc = words
    nbits = 16 + ((~nx) & 0xFFFFFFFF)
    bits = []
    for w in data:
        bits += [(w >> i) & 1 for i in range(32)]
    rem = nbits - len(bits)
    bits += [(partial >> (32 - rem + i)) & 1 for i in range(rem)] if rem > 0 else []
    return bytes(sum(bits[8 * k + j] << j for j in range(8)) for k in range(nbits // 8)), crc


def pack_bits(bits):
    words = []
    for i in range(0, len(bits), 32):
        w = 0
        for j, b in enumerate(bits[i:i + 32]):
            w |= (b & 1) << j
        words.append(w)
    return words


def bytes_to_bits(data):
    return [(b >> i) & 1 for b in data for i in range(8)]


def usb_tx_words(pid, payload):
    """FIFO words for usb_tx: a data packet with the given PID byte."""
    bits = bytes_to_bits(bytes([0x80, pid]) + bytes(payload))
    return [len(payload) * 8 - 1] + pack_bits(bits)


# ---------------------------------------------------------------------------
# 10BASE-T transmit: Manchester at 10 Mbit/s = 4 ticks per bit (40 MHz).
# vpin0 = TD+, vpin1 = TD- (differential). CRC-32 FCS in hardware.
#
# A whole frame is far too fast for the SPI host to stream, so the frame
# lives in an instruction-memory bank the core owns (PULL from imem, 20 bits
# per word): [frame_bits - 1], then preamble + SFD + frame, LSB first.
# Setting IRQ flag 0 sends it; IRQ 1+core signals completion.
# ---------------------------------------------------------------------------
ETH_TX = """
.program eth_tx
.wrap_target
    wait 1 irq 0            ; host: "send"
    set dptr, {frame}
    out x, 20               ; frame bits (after SFD) - 1
    set pindirs, 3
    ctl reset, code on, crc off
    mov crc, ~null
    set y, 62
pre:                        ; preamble + SFD: 64 bits
    out line, 1     [2]
    jmp y--, pre
    out line, 1     [1]     ; 2
    ctl crc on              ; 3
    set y, 17               ; 4
pay:
    out line, 1             ; 1
    jmp x--, ppad           ; 2
    ctl crc off             ; 3  freeze the CRC
    mov osr, ~crc           ; 4  FCS: inverted CRC, LSB first
    out line, 1             ; FCS bit 0
    mov isr, ~crc           ; the OSR autopulls after 20 bits, so stage
    in null, 20             ; FCS bits 20..31 in the ISR
    set x, 11
fcs1:                       ; FCS bits 1..18
    out line, 1     [2]
    jmp y--, fcs1
    out line, 1             ; FCS bit 19
    mov osr, isr            ; FCS bits 20..31
    nop             [1]
fcs2:
    out line, 1     [2]
    jmp x--, fcs2
    set pins, 1     [9]     ; TP_IDL: positive for 250 ns
    set pins, 0
    set pindirs, 0
    irq 1 rel
.wrap
ppad:
    jmp pay         [1]
"""


def eth_tx_program(origin=0, frame=32):
    return assemble(ETH_TX.format(frame=frame), origin)


def eth_tx_config(prog, tdp, tdn):
    c = CoreConfig().from_program(prog)
    c.pins = [tdp, tdn] + [ZERO] * 6
    c.out_base = 0
    c.set_base, c.set_count = 0, 2
    c.line_mode = MANCHESTER
    c.line_diff = True
    c.manch_half = 2
    c.crc_width, c.crc_poly = 32, 0xEDB88320
    c.autopull, c.pull_thresh, c.pull_imem = True, 20, True
    return c


def pack_bits20(bits):
    return [sum((b & 1) << j for j, b in enumerate(bits[i:i + 20]))
            for i in range(0, len(bits), 20)]


def eth_frame_words(frame):
    """Instruction-memory image of a frame for eth_tx (max 31 words)."""
    bits = bytes_to_bits(b"\x55" * 7 + b"\xd5" + bytes(frame))
    words = [len(frame) * 8 - 1] + pack_bits20(bits)
    assert len(words) <= 32, "frame too large for one bank"
    return words


# ---------------------------------------------------------------------------
# CAN 2.0A transmit (NRZ, stuff after 5 identical bits, CRC-15).
# One bit = 16 ticks. vpin0 = TXD (0 = dominant).
# TX FIFO: [bits - 1] then SOF..DATA in transmit order, packed LSB first.
# The CRC is computed MSB first and sent MSB first via a bit reverse.
# ---------------------------------------------------------------------------
CAN_TX = """
.program can_tx
.wrap_target
    out x, 32               ; SOF..DATA bits - 1
    set pins, 1             ; recessive
    set pindirs, 1
    ctl reset, stuff on, crc on, err clear
    mov crc, null
    set y, 14
pay:
    jmp stuffdue, ps        ;  1
    out line, 1     [11]    ; 13
    jmp x--, ppad           ; 14
    mov osr, ::crc          ; 15  crc[14] lands in bit 17
    out null, 17            ; 16
crc:
    jmp stuffdue, cs        ;  1
    out line, 1     [11]    ; 13
    jmp y--, cpad           ; 14
    jmp stuffdue, ts [2]    ; 16 (a stuff bit may follow the last CRC bit)
tail:
    set pins, 1     [15]    ; CRC delimiter
    set y, 11       [15]    ; ACK slot (recessive from the transmitter)
eof:                        ; ACK delimiter + 7 EOF + 3 IFS + 1
    jmp y--, eof    [15]
    irq 0 rel
.wrap
ppad:
    jmp pay         [1]
ps:
    out line, 1     [13]
    jmp pay
cpad:
    jmp crc         [1]
cs:
    out line, 1     [13]
    jmp crc
ts:
    out line, 1     [14]
    jmp tail
"""


def can_tx_program(origin=0):
    return assemble(CAN_TX, origin)


def can_tx_config(prog, txd, clkdiv=0):
    c = CoreConfig(clkdiv=clkdiv).from_program(prog)
    c.pins = [txd] + [ZERO] * 7
    c.out_base = 0
    c.set_base, c.set_count = 0, 1
    c.stuff_len, c.stuff_any = 5, True
    c.crc_msb_first, c.crc_width, c.crc_poly = True, 15, 0x4599
    c.autopull, c.pull_thresh = True, 32
    return c


def can_frame_bits(ident, data):
    """SOF..DATA of a standard data frame (unstuffed, transmit order)."""
    bits = [0]
    bits += [(ident >> (10 - i)) & 1 for i in range(11)]
    bits += [0, 0, 0]                                  # RTR, IDE, r0
    bits += [(len(data) >> (3 - i)) & 1 for i in range(4)]
    for b in data:
        bits += [(b >> (7 - i)) & 1 for i in range(8)]
    return bits


def can_tx_words(ident, data):
    bits = can_frame_bits(ident, data)
    return [len(bits) - 1] + pack_bits(bits)


# ---------------------------------------------------------------------------
# Logic analyser with timestamps: records (time, pins) only when any of
# the 8 vpins changes. 4-cycle sampling loop at CLKDIV 0.
# ---------------------------------------------------------------------------
# One FIFO word per event: bits [7:0] = new pin state, [31:8] = low 24 bits
# of the cycle counter (wraps every 2^24 cycles; the host extends it).
LOGIC_ANALYZER = """
.program logic_analyzer
    mov y, pins
.wrap_target
loop:
    mov x, pins
    jmp x!=y, change
    jmp loop
change:
    in x, 8
    in time, 24
    push noblock            ; dropped if the host falls behind
    mov y, x
.wrap
"""


def logic_analyzer_program(origin=0):
    return assemble(LOGIC_ANALYZER, origin)


def logic_analyzer_config(prog, pins):
    c = CoreConfig().from_program(prog)
    c.pins = (list(pins) + [ZERO] * 8)[:8]
    return c
