![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg)

# Polyglot: a programmable protocol-emulator ASIC

An entry for the [Jane Street protocol-emulator ASIC competition](https://blog.janestreet.com/protocol-emulator-asic-competition/),
built for Tiny Tapeout on IHP SG13CMOS5L (6x4 tiles).

Polyglot is three small cycle-exact cores that speak hardware protocols
from firmware. They are PIO/PRU-like, with the parts that make "hard"
protocols hard moved into a programmable **line coder**: CRC, bit-stuffing,
NRZI, Manchester and differential signalling. With that, the same chip
does UART, SPI and I2C, plus USB low-speed, CAN and 10BASE-T framing.

| Protocol | Firmware | Verified against |
|---|---|---|
| UART 8N1 TX / RX (+ packed RX, framing-error IRQ) | `uart_tx`, `uart_rx`, `uart_rx_packed` | UART encoder/decoder model |
| SPI master, mode 0 | `spi_master` | SPI slave model |
| I2C master: START/rSTART/STOP, ACK/NACK, clock stretching with timeout | `i2c_master` (2 banks) | I2C EEPROM model with clock stretching |
| USB 1.1 low speed TX + RX: NRZI, stuffing, CRC16, EOP | `usb_tx`, `usb_rx` | independent NRZI/destuff/CRC decoder and encoder |
| 10BASE-T TX: Manchester differential, CRC-32 FCS, 60-byte frame from on-chip memory | `eth_tx` | independent Manchester decoder + `zlib.crc32` |
| CAN 2.0A TX: 5-bit stuffing, CRC-15 | `can_tx` | independent CAN decoder |
| Logic analyser with 24-bit timestamps per edge | `logic_analyzer` | stimulus timing |

Every protocol test also runs the cycle-accurate reference model in
lockstep with the RTL (see Verification).

## What is different from RP2040 PIO

* **Hardware line coder per core.** `out line, 1` / `in line, 1` pass a bit
  through a CRC of any width and polynomial (1–32 bits, either bit order),
  automatic stuff-bit insertion and removal (USB-style runs of 1s or
  CAN-style runs of either value), NRZI, Manchester and differential
  output. A `stuffdue` jump condition lets bit loops keep exact timing
  when the hardware inserts a stuff bit. On PIO this work falls to the CPU
  or needs extra state machines.
* **Code/data memory.** 128×20-bit instruction memory in 4 banks with
  configurable ownership: one core can run a program of up to 128 instructions, or
  stream data (`set dptr` + pull-from-imem) out of a bank it owns. That is
  how a whole Ethernet frame goes out at 10 Mbit/s, far beyond what the
  SPI host link could feed.
* **Virtual-pin crossbar.** Any of a core's 8 vpins can map to any pad or
  to one of 8 internal nets for core-to-core wiring, not contiguous pin
  windows.
* **Robustness primitives.** `wait ... timeout` (Y-counted, with a
  `timeout` jump condition), `wait edge`, a global 32-bit timestamp
  source, signed `add`, `x<y`.
* **Debuggability.** Breakpoints, single-step, instruction injection and
  register readback over the host port.

Full ISA and register map: [docs/ISA.md](docs/ISA.md). Datasheet:
[docs/info.md](docs/info.md).

## Architecture

```
                 ┌───────────────────────────── instruction memory ─────────────────────────────┐
 host SPI ──►    │ bank0 32x20 │ bank1 32x20 │ bank2 32x20 │ bank3 32x20   (latches + ICGs)     │
 (ui0-2, uo0)    └──────┬──────────────┬─────────────┬─────────────┬──────────────────────────────┘
     │                  │ one read port per bank, addressed by its owner's PC or DPTR
     ▼            ┌─────▼─────┐  ┌─────▼─────┐  ┌─────▼─────┐
 registers ──────►│  core 0   │  │  core 1   │  │  core 2   │  each: X Y ISR OSR CRC, delay,
 TX/RX FIFOs ◄───►│ +line     │  │ +line     │  │ +line     │  clkdiv, side-set, 8 vpins
 IRQ flags, TIME  │  coder    │  │  coder    │  │  coder    │
                  └─────┬─────┘  └─────┬─────┘  └─────┬─────┘
                        └──── pin crossbar (24 vpins → 14 pads + 8 nets) ────► uio[7:0] uo[7:2]
                                                                                    ◄── ui[7:3] uio
```

Implementation notes:
* **Area-driven choices, all measured by synthesizing variants against
  the cmos5l liberty.**
  * Latch-based instruction memory and host-written FIFOs, with
    glitch-free writes through integrated clock-gate cells.
  * Single funnel shifters for both shift directions, using bit-reversal
    tricks.
  * Write-only configuration registers.
  * The first 4-core build hit routing congestion of 1.3 in placement, so
    the tape-out build uses 3 cores (`PG_NCORES`). 4 cores fit an 8x4 tile.
* The host port is an oversampled SPI slave with a speculative read-ahead
  that only pops a FIFO once the host actually clocks the word.

## Verification

* **Reference model**: `sw/polyglot/model.py` is a cycle-accurate Python
  model of the whole chip, written per instruction from the spec rather
  than transliterated from the RTL.
* **Lockstep co-simulation**: `test/lockstep.py` compares every core's
  architectural state (PC, X, Y, ISR/OSR and counts, CRC, line-coder state,
  pins, FIFO levels, IRQ flags) against the model on every clock. It runs
  under every RTL test.
* **Constrained-random fuzzing**: `test/test_fuzz.py` generates random
  programs, configurations, pad stimulus and host traffic (FIFO
  push/pop, instruction injection, restarts, breakpoints, single steps,
  bank remaps). Coverage counters report instruction and line-coder
  activity.
* **Mutation testing** of the checker. Six deliberately planted model
  bugs, including an off-by-one in autopush, NRZI polarity, and the
  stuff-run reset, are all detected.
* **Formal**: FIFO bookkeeping invariants are proved by k-induction with
  Yosys' SAT solver (`formal/run.sh`). The assertions live in the RTL
  under `ifdef FORMAL`, and a deliberately false bound is shown to fail.
* **Protocol tests** against independent bus-functional models
  (`test/bfm.py`, `test/bfm_wire.py`). These are black-box, pins and SPI
  only, so they also apply to the gate-level netlist.
* Design bugs these found: a host read of an empty FIFO returned stale
  data. Firmware/ISA insights they drove: the `stuffdue` condition and the
  pull-from-imem streaming mode.

Run everything:

```bash
cd test && make
```

`FUZZ_SEEDS`/`FUZZ_CYCLES` scale the fuzzer. `sw/polyglot/sim.py` runs
firmware on the model alone, for fast iteration.

## Software

```python
from polyglot.asm import assemble
from polyglot import programs, regs
prog = programs.uart_tx_program(origin=0)
cfg  = programs.uart_config(prog, clkdiv=3, pin=regs.OUT[2])
# load prog.words at IMEM 0x00, cfg.words() at core regs 0-7, enable via CTRL
```

## Bring-up on the Tiny Tapeout demo board

`sw/micropython/polyglot_tt.py` drives the host SPI from the demo board's
RP2040 through the TT MicroPython SDK. `python -m polyglot.export uart`
produces a loadable image (program, configuration, bank map and
initialisation instructions) for it.

## Layout

| Path | Contents |
|---|---|
| `src/` | RTL: `project.v` (top, registers, crossbar), `pg_core.v`, `pg_imem.v`, `pg_fifo*.v`, `pg_host_spi.v` |
| `sw/polyglot/` | assembler, reference model, register helpers, protocol firmware, model simulator, image exporter |
| `sw/micropython/` | demo-board host driver |
| `formal/` | k-induction proofs |
| `test/` | cocotb tests, lockstep checker, device models |
| `synth/` | quick Yosys area estimate against the cmos5l liberty |
| `docs/` | datasheet and ISA reference |

## License

Apache-2.0
