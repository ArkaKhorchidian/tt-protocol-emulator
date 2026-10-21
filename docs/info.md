<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

Polyglot is a programmable **protocol emulator**: three small, cycle-exact
cores that bit-bang hardware protocols from firmware, in the spirit of the
RP2040 PIO and TI PRU, with a few things done differently.

```
 host SPI ──► register file ──► 128 x 20-bit instruction memory (4 banks)
                 │   │                 │ one read port per bank
                 │   └── TX/RX FIFOs ─►├──► core 0 ─┐
                 │       (4 deep)      ├──► core 1 ─┼─► pin crossbar ──► uio[7:0], uo[7:2]
                 │                     └──► core 2 ─┘   (any vpin to      ui[7:3]
                 └── IRQ flags, 32-bit timestamp         any pad / net)
```

Each core executes one 20-bit instruction per tick (clock / (CLKDIV+1)):
`JMP WAIT IN OUT PUSH PULL MOV IRQ SET ADD CTL`, each with a delay field
and optional side-set. The core has X/Y scratch registers, 32-bit
input/output shift registers with autopush/autopull, and blocking FIFOs to
the host. Timing is deterministic, so a protocol is just a program.

What is different from PIO:

* **Hardware line coder per core.** `OUT LINE`/`IN LINE` pass bits through
  a programmable CRC (any polynomial, 1–32 bits, MSB or LSB first),
  **automatic bit-stuffing insertion/removal** (USB style: after N ones;
  CAN style: after N identical bits), NRZI, Manchester and differential
  output. USB low-speed, CAN and 10BASE-T framing become small programs.
  A `stuffdue` jump condition keeps bit loops exact when a stuff bit
  goes out.
* **Virtual pins.** Each core addresses 8 virtual pins; each one maps to
  any pad or to one of 8 internal nets, so cores can be chained (one core
  generates a clock or decodes a line, another consumes it).
* **Timeouts and edges.** `WAIT ... timeout` gives up after Y ticks and sets
  a flag (e.g. I2C clock-stretch timeout). `WAIT edge` waits for a fresh
  edge rather than a level.
* **Banked memory.** 4 banks × 32 instructions. Bank ownership is
  configurable, so one core can run a 64- or 96-instruction program.
* **Timestamps.** `MOV/IN ... time` reads a 32-bit cycle counter, for
  edge-capture logic analysis and pulse measurement.
* **Debug.** Breakpoints, single-step, instruction injection and full
  register readback over the host port.

The full instruction set and register map are in
[docs/ISA.md](ISA.md); firmware for UART, SPI, I2C and more is in
`sw/polyglot/programs.py`.

### Host interface

SPI mode 0 on `ui[0]` SCK, `ui[1]` CS_n, `ui[2]` MOSI, `uo[0]` MISO.
Keep SCK at or below clk/8.

* write: `0x02, ADDR, D31..D0 [, next word ...]`
* read:  `0x03, ADDR, dummy, D31..D0 [, ...]`

The address auto-increments, except on the FIFO ports `0xD0+core`, which
stream. `uo[1]` is an interrupt line (IRQ flags / FIFO status, maskable).

| Address | Register |
|---|---|
| 0x00–0x7F | instruction memory (write only) |
| 0x80 + 16·core + r | core registers: CLKDIV, EXEC, SHIFT, PIN, PINMAP×2, LINE, POLY (write only); CRC, PC, X, Y, ISR, OSR, STATUS, INSTR (read/write) |
| 0xC0 | ID = 0x504C5947 ("PLYG") |
| 0xC1 | CTRL: enable, restart, step, clock-divider sync, FIFO clear |
| 0xC2–0xC7 | IRQ flags, IRQ mask, timer, FIFO status, bank map, pad state |
| 0xD0–0xD2 | TX FIFO push (write) / RX FIFO pop (read) |

## How to test

1. Drive the host SPI from the demo board's RP2040 (or any MCU) and read
   register `0xC0`. It must return `0x504C5947`.
2. Load a program into instruction memory, configure the core (pin map,
   wrap, shift and line settings), write CTRL to enable it, and push data
   into its FIFO. For example, with `uart_tx` on core 0 and vpin0 mapped to
   physical pin 18 (`uo[2]`), bytes pushed to `0xD0` come out as 8N1 serial
   at clk/(8·(CLKDIV+1)) baud.
3. The Python package in `sw/polyglot` contains the assembler, a
   cycle-accurate reference model and ready-made firmware. The cocotb
   tests in `test/` show complete host sequences for UART, SPI and I2C.

Clocking: timing is signed off at 33 MHz at the slow corner
(1.08 V, 125 °C). Typical silicon should run faster. The USB low-speed
firmware assumes 48 MHz (32 ticks per bit) and 10BASE-T needs 40 MHz
(4 ticks per bit); lower clocks work with adjusted firmware delays.

Notes: unused output-only pads read 0 until a core drives them, so set
idle levels before enabling receivers. Pad inputs pass through a 2-flop
synchroniser, so use CLKDIV ≥ 1 for SPI so MISO samples have margin.

## External hardware

None required. Any 3.3 V peripheral can be attached: UART adapters, SPI
flash, I2C sensors (with pull-ups on the `uio` pins), and so on. USB
low-speed needs series resistors and a 1.5 kΩ pull-up. 10BASE-T transmit
needs the usual resistor/transformer network.
