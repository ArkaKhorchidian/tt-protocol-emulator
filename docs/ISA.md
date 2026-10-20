# Polyglot ISA and programmer's reference

This document is the specification. The RTL (`src/`) and the reference
model (`sw/polyglot/model.py`) both implement it, and the lockstep tests
check them against each other on every clock.

## 1. Execution model

Polyglot has 3 cores (the RTL supports 4 with `PG_NCORES=4`). Each core
advances on a **tick**: every system clock when `CLKDIV = 0`, otherwise once
every `CLKDIV + 1` clocks. On each tick a core either:

* counts down its **delay** counter (set by the previous instruction), or
* executes the instruction at `PC` (or an injected instruction, §6).

An instruction that cannot complete **stalls**. It is retried on the next
tick, no architectural state changes, and its delay does not start until
it completes. Three effects do happen while stalled:

* side-set is applied on every attempt,
* a `WAIT ... timeout` decrements Y on every attempt,
* `IRQ wait` raises its flag on the first attempt.

After a non-jump instruction completes, `PC = (PC == WRAP_TOP) ? WRAP_BOTTOM : PC + 1`.
Wrapping is free, so it costs no cycle.

Instruction memory has 128 words of 20 bits in 4 banks of 32. A core fetches
from bank `PC[6:5]`. Each bank has one read port, owned by one core
(`BANKMAP`). The port is addressed by the owner's `PC` when the owner
executes from that bank, otherwise by the owner's data pointer `DPTR`. So
a core can run a program of up to 128 instructions across the banks it
owns, or stream data out of an owned bank it is not executing (§4.4).

## 2. Instruction format

```
 19   16 15 14 13   11 10                       0
+-------+-----+-------+--------------------------+
|  op   | SS  | delay |          args            |
+-------+-----+-------+--------------------------+
```

`SS` is interpreted according to the core's side-set mode (`EXEC.side_mode`):

| mode | side pins | delay range | meaning of SS |
|---|---|---|---|
| 0 | none | 0–31 | delay bits 4:3 |
| 1 | 1 | 0–15 | SS[0] = side value, SS[1] = delay bit 3 |
| 2 | 2 | 0–7 | SS[1:0] = values for side pins 1, 0 |
| 3 | 1 optional | 0–7 | SS[1] = enable, SS[0] = value |

Side pins are vpins `side_base` and `side_base+1`. With `EXEC.side_dirs`
set, side-set writes the pin directions instead of the values.

## 3. Instructions

### JMP `jmp [cond,] target` — op 0
`args = cond[10:7], target[6:0]`. The target is an absolute address (0–127).

| cond | mnemonic | jumps when |
|---|---|---|
| 0 | (always) | always |
| 1 | `!x` | X == 0 |
| 2 | `x--` | X != 0; X is decremented regardless |
| 3 | `!y` | Y == 0 |
| 4 | `y--` | Y != 0; Y is decremented regardless |
| 5 | `x!=y` | X != Y |
| 6 | `pin` | vpin `jmp_pin` is 1 |
| 7 | `!pin` | vpin `jmp_pin` is 0 |
| 8 | `!osre` | OSR count < pull threshold |
| 9 | `txne` | TX FIFO not empty |
| 10 | `timeout` | last timeout-WAIT gave up |
| 11 | `stuffdue` | the next LINE bit will be a stuff bit |
| 12 | `err` | line error flag (stuff violation) set |
| 13 | `x<y` | X < Y (unsigned) |
| 14 | `lastbit` | last LINE data bit was 1 |
| 15 | `isrfull` | ISR count ≥ push threshold |

### WAIT `wait <pol> <pin|irq|edge> <idx> [timeout]` — op 1
`args = pol[10], src[9:8], T[7], idx[2:0]`

* `pin`: wait until vpin `idx` == pol.
* `irq`: wait until IRQ flag `idx` == pol. With pol = 1 the flag is
  cleared when the wait completes.
* `edge`: wait for a fresh transition to `pol` on vpin `idx`. The vpin must
  equal pol now and must have differed on the previous tick.

With `timeout` (T = 1), each unsuccessful tick decrements Y. If the
condition is still false when Y is 0, the WAIT completes anyway and sets
the timeout flag. A WAIT whose condition is met clears the flag.

### IN `in <src>, <n>` — op 2
`args = src[10:8], n[4:0]` (n = 0 means 32). Shifts n bits of the source
into the ISR. If `in_right`, bits enter at the MSB and the ISR shifts right.
Otherwise they enter at the LSB and it shifts left. The ISR count
saturates at 32. With autopush, when the count reaches the push threshold
the ISR is pushed to the RX FIFO and cleared in the same tick. If the FIFO
is full, the IN stalls.

| src | 0 pins | 1 x | 2 y | 3 null | 4 line | 5 time | 6 isr | 7 osr |
|---|---|---|---|---|---|---|---|---|

`pins` reads vpins rotated by `in_base`. `time` is the global 32-bit
cycle counter. `line` receives one bit through the line decoder (§5).

### OUT `out <dst>, <n>` — op 3
Shifts n bits out of the OSR (LSB end if `out_right`, else MSB end). If
autopull is on and the OSR count is at least the pull threshold, the OSR
is refilled first, in the same tick. The source is the TX FIFO, or
`imem[DPTR++]` in pull-from-imem mode. If the FIFO is empty, the OUT stalls.

| dst | 0 pins | 1 x | 2 y | 3 null | 4 line | 5 pindirs | 6 pc | 7 isr |
|---|---|---|---|---|---|---|---|---|

`pins`/`pindirs` write n bits (at most 8) starting at vpin `out_base`,
wrapping. `pc` jumps to the data. `isr` sets ISR = data, count = n.
`line` sends one bit through the line encoder (§5).

### PUSH / PULL — op 4
`args = pull[7], if[6], block[5]`

* `push [iffull] [block|noblock]`: ISR → RX FIFO, ISR cleared. `iffull`
  does nothing unless count ≥ threshold. If the FIFO is full, `block`
  stalls and `noblock` drops the data (the ISR is still cleared).
* `pull [ifempty] [block|noblock]`: TX FIFO (or `imem[DPTR++]`) → OSR.
  If the FIFO is empty, `noblock` copies X into the OSR.

### MOV `mov <dst>, [op]<src>` — op 5
`args = dst[10:8], op[7:6], src[5:3]`

| | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| dst | pins | x | y | pindirs | crc | pc | isr | osr |
| src | pins | x | y | null | crc | time | isr | osr |

op: none, `~` invert, `::` bit-reverse, `><` byte-swap. Writing ISR or OSR
resets its count to 0. Writing CRC masks to the CRC width.

### IRQ `irq [set|wait|clear] <idx> [rel]` — op 6
`args = clear[6], wait[5], rel[4], idx[2:0]`. There are 8 flags shared by
all cores and the host. `rel` adds the core number to idx[1:0]. `wait`
raises the flag, then stalls until someone clears it.

### SET `set <dst>, <imm8>` — op 7
`args = dst[10:8], imm[7:0]`. Destinations: 0 `pins` and 3 `pindirs`
(`set_count` bits from `set_base`), 1 `x`, 2 `y`, 4 `dptr` (imm[6:0]).

### ADD `add <x|y>, <simm10>` — op 8
`args = dst[10], imm[9:0]` (signed −512…511).

### CTL `ctl <clauses>` — op 9
`args = reset[8], mask[7:4], value[3:0]`. Writes the line-coder flags
`{err, code, stuff, crc}` selected by the mask. `reset` clears the stuff
run tracker, sets the NRZI level to `LINE.idle`, and cancels a pending
Manchester half-bit. Example: `ctl reset, code on, stuff on, crc off`.

Opcodes 10–15 are reserved and execute as NOP. `nop` assembles to `mov y, y`.

## 4. Data path details

### 4.1 Pins
Each core has 8 **virtual pins**. `PINMAP` maps each vpin to a physical
index for both input and output:

| index | 0–7 | 8–15 | 16–23 | 24–31 |
|---|---|---|---|---|
| pin | `uio[0..7]` | `ui[0..7]` (input) | `uo[0..7]` (output; readback on input) | internal nets 0–7 |

A vpin drives its physical pin while its direction bit is 1. Several
drivers on one pin are wired-OR. `uo[0]` and `uo[1]` are reserved for the
host. An undriven `uo` pin outputs 0, and an undriven net reads 0. Pad
inputs pass through a 2-flop synchroniser. Nets and `uo` readback are
seen one clock after they are driven.

### 4.2 Autopull timing
Refills happen on demand: an OUT that needs bits when count ≥ threshold
refills and shifts in the same tick. So `!osre` reports an empty OSR even
with autopull enabled. Use `txne` to test whether more data is queued.

### 4.3 FIFOs
Each core has 4-word TX and RX FIFOs. A host push becomes visible to the
core 2 cycles later, because TX storage is latch-based (§7).

### 4.4 Streaming from instruction memory
With `SHIFT.pull_imem` set, PULL and autopull read `imem[DPTR]` (20 bits,
zero-extended) and post-increment DPTR. They never stall. `DPTR` must point
into a bank the core owns but is not executing from. Typically you use
pull threshold 20 and pack bitstreams 20 bits per word.

## 5. Line coder

`OUT LINE` / `IN LINE` move one bit between the OSR/ISR and vpin
`out_base`/`in_base` through this pipeline:

```
TX: OSR bit ─► CRC update ─► stuff insertion ─► NRZI / Manchester ─► invert ─► vpin (+ inverted vpin+1 if diff)
RX: vpin ─► invert ─► NRZI decode ─► stuff removal/check ─► CRC update ─► ISR
```

* **CRC**: Galois LFSR, width W = 1–32 (`LINE.crc_w`), polynomial `POLY`.
  LSB-first (reflected): `fb = crc[0]^b; crc = (crc>>1) ^ (fb ? POLY : 0)`.
  MSB-first: `fb = crc[W-1]^b; crc = (crc<<1) ^ (fb ? POLY : 0)`.
  Masked to W bits. It updates only on data bits, and only when the CRC
  flag is on.
* **Stuffing** (`stuff_len` N = 1–7, flag on): a run tracker counts
  consecutive 1s (`stuff_any` = 0, USB) or consecutive identical bits
  (`stuff_any` = 1, CAN). When the run reaches N, the next LINE operation is
  a stuff bit, the complement of the run value. On TX, the OUT emits it
  without consuming an OSR bit. On RX, the IN discards it without shifting,
  and sets the error flag if it is not the complement. The stuff bit counts
  toward the next run. `stuffdue` tests this condition in advance, so bit
  loops keep exact timing.
* **NRZI** (mode 1): a 0 toggles the line and a 1 holds it (USB).
* **Manchester** (mode 2): the OUT drives ~b immediately, and b after
  `manch_half` ticks (IEEE 802.3: a 1 is low→high).
* **diff**: vpin `out_base+1` carries the inverse level.

## 6. Debug features

* **INSTR** (core reg 15) write: injects an instruction, which runs on the
  next tick ahead of the fetched one and does not advance PC. It runs even
  on a disabled core. INSTR read returns `imem[PC]`, so you can read memory
  back by halting the core and setting PC.
* **Breakpoint**: `EXEC.brk_en/brk_addr`. When the core is about to execute
  `brk_addr`, it is disabled before executing it, and its `halted` bit is
  set. Resuming executes that instruction.
* **Step**: `CTRL.step` gives a disabled core exactly one tick.
* PC, X, Y, ISR, OSR, CRC and STATUS are readable and writable at any time.

## 7. Registers

Per core `c`, base `0x80 + 16·c`:

| r | name | fields |
|---|---|---|
| 0 | CLKDIV | [15:0] tick every CLKDIV+1 clocks |
| 1 | EXEC | [6:0] wrap_bottom, [14:8] wrap_top, [15] side_dirs, [17:16] side_mode, [20:18] side_base, [23:21] jmp_pin, [30:24] brk_addr, [31] brk_en |
| 2 | SHIFT | [0] autopull, [1] autopush, [2] out_right, [3] in_right, [8:4] pull_th (0=32), [13:9] push_th (0=32), [14] pull_imem |
| 3 | PIN | [2:0] out_base, [5:3] set_base, [8:6] set_count−1, [11:9] in_base |
| 4/5 | PINMAP | 5 bits per vpin: vpins 0–3 in reg 4, vpins 4–7 in reg 5 |
| 6 | LINE | [1:0] mode (NRZ/NRZI/Manchester), [2] invert, [3] diff, [6:4] stuff_len, [7] stuff_any, [8] crc_msb, [13:9] crc_w−1, [14] idle, [23:16] manch_half |
| 7 | POLY | CRC polynomial |
| 8 | CRC | r/w |
| 9 | PC | r/w |
| 10/11 | X / Y | r/w |
| 12/13 | ISR / OSR | r/w (write clears the count) |
| 14 | STATUS | [6:0] pc, [7] stalled, [8] timeout, [9] err, [10] stuff, [11] lastbit, [12] brk_skip, [13] crc_en, [14] stuff_en, [15] code_en, [21:16] osr_cnt, [29:24] isr_cnt |
| 15 | INSTR | write: inject; read: imem[PC] |

Registers 0–7 are write-only (the host keeps a shadow copy).

Global:

| addr | name | fields |
|---|---|---|
| 0xC0 | ID | 0x504C5947 |
| 0xC1 | CTRL | [3:0] enable (r/w), [7:4] restart, [11:8] step, [15:12] clkdiv sync, [19:16] FIFO clear (pulses); read [11:8] halted |
| 0xC2 | IRQ | read flags; write [7:0] clear, [15:8] set |
| 0xC3 | IRQMASK | [7:0] flags, [11:8] RX not empty, [15:12] TX not full → `uo[1]` |
| 0xC4 | TIME | 32-bit cycle counter (r/w) |
| 0xC5 | FIFOSTAT | byte per core: [2:0] TX level, [5:3] RX level, [6] halted, [7] stalled |
| 0xC6 | BANKMAP | 2 bits per bank: owning core (reset 0xE4) |
| 0xC7 | PINS | synchronised pad/net state (phys_in) |
| 0xD0+c | FIFO | write: push TX; read: pop RX (0 when empty) |
| 0x00–0x7F | IMEM | write only |
