# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""Assembler for the Polyglot protocol-emulator ISA (docs/ISA.md).

    from polyglot.asm import assemble
    prog = assemble('''
        .side_set 1 opt
        .wrap_target
            pull           side 1 [7]
            set x, 7       side 0 [7]
        bit:
            out pins, 1
            jmp x--, bit   [6]
        .wrap
    ''')
    prog.words, prog.wrap_target, prog.wrap, prog.side_mode

Syntax summary (case-insensitive, ';' or '//' comments):
    jmp   [cond,] target
    wait  <0|1> <pin|irq|edge> <idx> [timeout]
    in    <pins|x|y|null|line|time|isr|osr>, <1..32>
    out   <pins|x|y|null|line|pindirs|pc|isr>, <1..32>
    push  [iffull] [block|noblock]
    pull  [ifempty] [block|noblock]
    mov   <pins|x|y|pindirs|crc|pc|isr|osr>, [~|!|::|><]<pins|x|y|null|crc|time|isr|osr>
    irq   [set|wait|clear] <0..7> [rel]
    set   <pins|x|y|pindirs>, <0..255>
    add   <x|y>, <-512..511>
    ctl   [crc on|off] [stuff on|off] [code on|off] [err clear|set] [reset]  (comma separated)
    nop
  modifiers:  side <v>   [delay]
  directives: .program name  .side_set <1|2> [opt] [pindirs]  .wrap_target  .wrap
              .origin <n>  .define NAME value  .word value
"""

import re
from dataclasses import dataclass, field

OP = dict(jmp=0, wait=1, in_=2, out=3, pp=4, mov=5, irq=6, set=7, add=8, ctl=9)

JMP_COND = {
    "": 0, "always": 0, "!x": 1, "x--": 2, "!y": 3, "y--": 4, "x!=y": 5,
    "pin": 6, "!pin": 7, "!osre": 8, "txne": 9, "timeout": 10, "stuffdue": 11,
    "err": 12, "x<y": 13, "lastbit": 14, "isrfull": 15,
}
IN_SRC = dict(pins=0, x=1, y=2, null=3, line=4, time=5, isr=6, osr=7)
OUT_DST = dict(pins=0, x=1, y=2, null=3, line=4, pindirs=5, pc=6, isr=7)
MOV_DST = dict(pins=0, x=1, y=2, pindirs=3, crc=4, pc=5, isr=6, osr=7)
MOV_SRC = dict(pins=0, x=1, y=2, null=3, crc=4, time=5, isr=6, osr=7)
SET_DST = dict(pins=0, x=1, y=2, pindirs=3)
WAIT_SRC = dict(pin=0, gpio=0, irq=1, edge=2)


class AsmError(Exception):
    pass


@dataclass
class Program:
    words: list
    wrap_target: int = 0
    wrap: int = 0
    side_mode: int = 0          # 0 none, 1 one pin, 2 two pins, 3 one pin optional
    side_dirs: bool = False
    labels: dict = field(default_factory=dict)
    origin: int = 0
    name: str = ""
    source_lines: list = field(default_factory=list)   # (addr, text)


def _int(tok, defines):
    tok = tok.strip()
    if tok in defines:
        return defines[tok]
    try:
        return int(tok, 0)
    except ValueError:
        raise AsmError(f"bad number '{tok}'")


def assemble(src, origin=0):
    defines = {}
    labels = {}
    side_mode = 0
    side_dirs = False
    name = ""
    items = []            # (addr, mnemonic, args, side, delay, lineno, text)
    wrap_target = None
    wrap = None
    pc = origin

    # ---- pass 1: labels, directives ----
    for lineno, raw in enumerate(src.splitlines(), 1):
        line = re.split(r";|//", raw)[0].strip()
        if not line:
            continue
        while True:
            m = re.match(r"^([A-Za-z_]\w*):\s*(.*)$", line)
            if not m:
                break
            labels[m.group(1)] = pc
            line = m.group(2).strip()
        if not line:
            continue
        if line.startswith("."):
            parts = line.split()
            d = parts[0].lower()
            if d == ".program":
                name = parts[1] if len(parts) > 1 else ""
            elif d == ".side_set":
                n = int(parts[1])
                opt = "opt" in [p.lower() for p in parts[2:]]
                side_dirs = "pindirs" in [p.lower() for p in parts[2:]]
                if n == 1 and opt:
                    side_mode = 3
                elif n == 1:
                    side_mode = 1
                elif n == 2 and not opt:
                    side_mode = 2
                else:
                    raise AsmError(f"line {lineno}: unsupported .side_set")
            elif d == ".wrap_target":
                wrap_target = pc
            elif d == ".wrap":
                wrap = pc - 1
            elif d == ".origin":
                pc = origin = _int(parts[1], defines)
            elif d == ".define":
                defines[parts[1]] = _int(parts[2], defines)
            elif d == ".word":
                items.append((pc, ".word", parts[1], None, 0, lineno, raw.strip()))
                pc += 1
            else:
                raise AsmError(f"line {lineno}: unknown directive {d}")
            continue
        items.append((pc, None, line, None, None, lineno, raw.strip()))
        pc += 1

    # ---- pass 2: encode ----
    words = []
    src_lines = []
    max_delay = {0: 31, 1: 15, 2: 7, 3: 7}[side_mode]
    for addr, kind, text, _, _, lineno, raw in items:
        if kind == ".word":
            words.append(_int(text, {**defines, **labels}) & 0xFFFFF)
            src_lines.append((addr, raw))
            continue
        try:
            words.append(_encode(text, side_mode, max_delay, {**defines, **labels}))
        except AsmError as e:
            raise AsmError(f"line {lineno}: {e}: {raw}") from None
        src_lines.append((addr, raw))

    if wrap_target is None:
        wrap_target = origin
    if wrap is None:
        wrap = origin + len(words) - 1
    return Program(words, wrap_target, wrap, side_mode, side_dirs, labels,
                   origin, name, src_lines)


def _encode(text, side_mode, max_delay, syms):
    # strip delay and side-set modifiers
    delay = 0
    m = re.search(r"\[\s*([^\]]+)\]\s*$", text)
    if m:
        delay = _int(m.group(1), syms)
        text = text[:m.start()].strip()
    side = None
    m = re.search(r"\bside\s+(\S+)\s*$", text, re.I)
    if m:
        side = _int(m.group(1), syms)
        text = text[:m.start()].strip()
    if delay < 0 or delay > max_delay:
        raise AsmError(f"delay {delay} out of range (max {max_delay})")

    if side_mode == 0:
        if side is not None:
            raise AsmError("side-set used without .side_set")
        ss, dly = delay >> 3, delay & 7
    elif side_mode == 1:
        if side is None:
            raise AsmError("side-set value required")
        ss, dly = ((delay >> 3) << 1) | (side & 1), delay & 7
    elif side_mode == 2:
        if side is None:
            raise AsmError("side-set value required")
        ss, dly = side & 3, delay
    else:
        ss, dly = (0 if side is None else (2 | (side & 1))), delay

    mnem, _, rest = text.partition(" ")
    mnem = mnem.lower()
    args = [a.strip() for a in rest.split(",")] if rest.strip() else []
    low = [a.lower() for a in args]

    def word(op, arg):
        return (op << 16) | (ss << 14) | (dly << 11) | (arg & 0x7FF)

    if mnem == "nop":
        return word(OP["mov"], (MOV_DST["y"] << 8) | MOV_SRC["y"] << 3)

    if mnem == "jmp":
        if len(args) == 1:
            cond, target = "", args[0]
        elif len(args) == 2:
            cond, target = low[0].replace(" ", ""), args[1]
        else:
            raise AsmError("jmp takes [cond,] target")
        if cond not in JMP_COND:
            raise AsmError(f"unknown jmp condition '{cond}'")
        addr = _int(target, syms)
        if not 0 <= addr < 128:
            raise AsmError("jump target out of range")
        return word(OP["jmp"], (JMP_COND[cond] << 7) | addr)

    if mnem == "wait":
        toks = rest.lower().replace(",", " ").split()
        if len(toks) < 3:
            raise AsmError("wait <pol> <pin|irq|edge> <idx> [timeout]")
        pol = _int(toks[0], syms)
        srcn = WAIT_SRC.get(toks[1])
        if srcn is None:
            raise AsmError(f"unknown wait source '{toks[1]}'")
        idx = _int(toks[2], syms)
        to = 1 if (len(toks) > 3 and toks[3] == "timeout") else 0
        if not (0 <= idx < 8 and pol in (0, 1)):
            raise AsmError("wait operand out of range")
        return word(OP["wait"], (pol << 10) | (srcn << 8) | (to << 7) | idx)

    if mnem == "in":
        if len(args) != 2 or low[0] not in IN_SRC:
            raise AsmError("in <src>, <count>")
        n = _int(args[1], syms)
        if not 1 <= n <= 32:
            raise AsmError("count must be 1..32")
        return word(OP["in_"], (IN_SRC[low[0]] << 8) | (n & 31))

    if mnem == "out":
        if len(args) != 2 or low[0] not in OUT_DST:
            raise AsmError("out <dst>, <count>")
        n = _int(args[1], syms)
        if not 1 <= n <= 32:
            raise AsmError("count must be 1..32")
        return word(OP["out"], (OUT_DST[low[0]] << 8) | (n & 31))

    if mnem in ("push", "pull"):
        toks = rest.lower().split()
        cond = ("iffull" in toks) if mnem == "push" else ("ifempty" in toks)
        block = "noblock" not in toks
        for t in toks:
            if t not in ("iffull", "ifempty", "block", "noblock"):
                raise AsmError(f"bad {mnem} option '{t}'")
        return word(OP["pp"], ((mnem == "pull") << 7) | (cond << 6) | (block << 5))

    if mnem == "mov":
        if len(args) != 2 or low[0] not in MOV_DST:
            raise AsmError("mov <dst>, [op]<src>")
        s = low[1].replace(" ", "")
        mop = 0
        for pre, code in (("::", 2), ("><", 3), ("~", 1), ("!", 1)):
            if s.startswith(pre):
                mop, s = code, s[len(pre):]
                break
        if s not in MOV_SRC:
            raise AsmError(f"unknown mov source '{s}'")
        return word(OP["mov"], (MOV_DST[low[0]] << 8) | (mop << 6) | (MOV_SRC[s] << 3))

    if mnem == "irq":
        toks = rest.lower().split()
        clear = wait = rel = 0
        idx = None
        for t in toks:
            if t == "set":
                pass
            elif t == "nowait":
                pass
            elif t == "wait":
                wait = 1
            elif t == "clear":
                clear = 1
            elif t == "rel":
                rel = 1
            else:
                idx = _int(t, syms)
        if idx is None or not 0 <= idx < 8:
            raise AsmError("irq index 0..7 required")
        return word(OP["irq"], (clear << 6) | (wait << 5) | (rel << 4) | idx)

    if mnem == "set":
        if len(args) != 2 or low[0] not in SET_DST:
            raise AsmError("set <dst>, <imm>")
        v = _int(args[1], syms)
        if not 0 <= v <= 255:
            raise AsmError("set immediate must be 0..255")
        return word(OP["set"], (SET_DST[low[0]] << 8) | v)

    if mnem == "add":
        if len(args) != 2 or low[0] not in ("x", "y"):
            raise AsmError("add <x|y>, <imm>")
        v = _int(args[1], syms)
        if not -512 <= v <= 511:
            raise AsmError("add immediate must be -512..511")
        return word(OP["add"], ((low[0] == "y") << 10) | (v & 0x3FF))

    if mnem == "ctl":
        mask = val = rst = 0
        bits = dict(crc=0, stuff=1, code=2, err=3)
        for a in low:
            t = a.split()
            if t == ["reset"]:
                rst = 1
                continue
            if len(t) != 2 or t[0] not in bits:
                raise AsmError(f"bad ctl clause '{a}'")
            b = bits[t[0]]
            on = t[1] in ("on", "set", "1")
            if not on and t[1] not in ("off", "clear", "0"):
                raise AsmError(f"bad ctl value '{t[1]}'")
            mask |= 1 << b
            val |= on << b
        return word(OP["ctl"], (rst << 8) | (mask << 4) | val)

    raise AsmError(f"unknown mnemonic '{mnem}'")


def disassemble(w, side_mode=0):
    """Best-effort single-instruction disassembly (for traces)."""
    op, ss, dly, arg = (w >> 16) & 15, (w >> 14) & 3, (w >> 11) & 7, w & 0x7FF
    hi = (arg >> 8) & 7
    inv = lambda d, v: next((k for k, x in d.items() if x == v), "?")
    if op == 0:
        c = inv(JMP_COND, (arg >> 7) & 15)
        s = f"jmp {c + ', ' if c else ''}{arg & 127}"
    elif op == 1:
        s = f"wait {(arg >> 10) & 1} {['pin', 'irq', 'edge', 'none'][(arg >> 8) & 3]} {arg & 7}" + (" timeout" if arg & 0x80 else "")
    elif op == 2:
        s = f"in {inv(IN_SRC, hi)}, {(arg & 31) or 32}"
    elif op == 3:
        s = f"out {inv(OUT_DST, hi)}, {(arg & 31) or 32}"
    elif op == 4:
        s = ("pull" if arg & 0x80 else "push") + (" if" if arg & 0x40 else "") + (" block" if arg & 0x20 else " noblock")
    elif op == 5:
        s = f"mov {inv(MOV_DST, hi)}, {['', '~', '::', '><'][(arg >> 6) & 3]}{inv(MOV_SRC, (arg >> 3) & 7)}"
    elif op == 6:
        s = f"irq {'clear ' if arg & 0x40 else 'wait ' if arg & 0x20 else ''}{arg & 7}{' rel' if arg & 0x10 else ''}"
    elif op == 7:
        s = f"set {inv(SET_DST, hi)}, {arg & 255}"
    elif op == 8:
        v = arg & 0x3FF
        s = f"add {'y' if arg & 0x400 else 'x'}, {v - 1024 if v & 512 else v}"
    elif op == 9:
        s = f"ctl mask={(arg >> 4) & 15:x} val={arg & 15:x}{' reset' if arg & 0x100 else ''}"
    else:
        s = f".word {w:#x}"
    return s + (f" ss={ss}" if ss else "") + (f" [{dly}]" if dly else "")
