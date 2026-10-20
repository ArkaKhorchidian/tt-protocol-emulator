# SPDX-FileCopyrightText: © 2026 Arka Khorchidian
# SPDX-License-Identifier: Apache-2.0
"""Independent reference encoders/decoders for USB-LS, 10BASE-T and CAN.

These operate on per-clock pad samples and share no code with the chip
model, so they check the firmware + line coder against the protocol
definitions themselves.
"""


def crc16_usb(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc ^ 0xFFFF


def crc15_can(bits):
    crc = 0
    for b in bits:
        nxt = b ^ ((crc >> 14) & 1)
        crc = (crc << 1) & 0x7FFF
        if nxt:
            crc ^= 0x4599
    return crc


def bits_to_bytes(bits):
    return bytes(sum(bits[8 * i + j] << j for j in range(8)) for i in range(len(bits) // 8))


# ---------------------------------------------------------------------------
class UsbLsMonitor:
    """Decodes low-speed packets from D-/D+ levels (bit = `bit` clocks)."""

    def __init__(self, dm, dp, bit=32):
        self.dm, self.dp, self.bit = dm, dp, bit
        self.packets, self.errors = [], []
        self.state, self.prev = "idle", (1, 0)

    def tick(self, b):
        dm, dp = b.io(self.dm), b.io(self.dp)
        if self.state == "idle":
            if self.prev == (1, 0) and (dm, dp) == (0, 1):      # J -> K
                self.state, self.t, self.raw, self.se0 = "rx", 0, [], 0
        else:
            self.t += 1
            if dm == 0 and dp == 0:
                self.se0 += 1
            elif self.se0:
                # end of EOP: SE0 must have lasted about two bit times
                n = self.se0
                self._finish(n)
                self.state = "idle"
            if self.state == "rx" and not self.se0 and self.t % self.bit == self.bit // 2:
                if dm == dp:
                    self.errors.append("diff violation")
                self.raw.append(dm)
        self.prev = (dm, dp)

    def _finish(self, se0_len):
        lvl, bits, ones, stuff_errs = 1, [], 0, 0
        for r in self.raw:
            b = 1 if r == lvl else 0
            lvl = r
            if ones == 6:
                if b != 0:
                    stuff_errs += 1
                ones = 0
                continue
            bits.append(b)
            ones = ones + 1 if b else 0
        data = bits_to_bytes(bits)
        self.packets.append(dict(data=data, se0=se0_len, stuff_errors=stuff_errs,
                                 nbits=len(bits)))


def usb_ls_encode(packet_bytes):
    """Line levels (D- per bit, J=1) for SYNC..CRC bytes, with stuffing."""
    bits, ones = [], 0
    for byte in packet_bytes:
        for i in range(8):
            b = (byte >> i) & 1
            bits.append(b)
            ones = ones + 1 if b else 0
            if ones == 6:
                bits.append(0)
                ones = 0
    lvl, out = 1, []
    for b in bits:
        if b == 0:
            lvl ^= 1
        out.append(lvl)
    return out


class UsbLsDriver:
    """Drives a low-speed packet onto D-/D+ (push-pull), then EOP and J."""

    def __init__(self, dm, dp, bit=32):
        self.dm, self.dp, self.bit = dm, dp, bit
        self.seq = []
        self.t = 0

    def send(self, packet_bytes):
        levels = usb_ls_encode(packet_bytes)
        seq = [(l, 1 - l) for l in levels] + [(0, 0), (0, 0), (1, 0)]
        self.seq += [s for s in seq for _ in range(self.bit)]

    def tick(self, b):
        if self.seq:
            dm, dp = self.seq.pop(0)
            b.push[self.dm], b.push[self.dp] = dm, dp
        else:
            b.push.pop(self.dm, None)
            b.push.pop(self.dp, None)

    @property
    def idle(self):
        return not self.seq


# ---------------------------------------------------------------------------
class EthTxMonitor:
    """Decodes a 10BASE-T Manchester frame from TD+/TD- (bit = 4 clocks)."""

    def __init__(self, tdp, tdn, nbits, bit=4):
        self.tdp, self.tdn, self.nbits, self.bit = tdp, tdn, nbits, bit
        self.samples = None
        self.diff_errors = 0
        self.frames = []

    def tick(self, b):
        p, n = b.io(self.tdp), b.io(self.tdn)
        if self.samples is None:
            if b.chip_drives(self.tdp) and p != n:
                self.samples = []
        if self.samples is not None and len(self.samples) < self.nbits * self.bit:
            if p == n:
                self.diff_errors += 1
            self.samples.append(p)
            if len(self.samples) == self.nbits * self.bit:
                bits = []
                for k in range(self.nbits):
                    first = self.samples[k * self.bit]
                    second = self.samples[k * self.bit + self.bit // 2]
                    if first == second:
                        self.diff_errors += 1          # no mid-bit transition
                    bits.append(second)                # IEEE 802.3: 1 = low->high
                self.frames.append(bits_to_bytes(bits))


# ---------------------------------------------------------------------------
class CanTxMonitor:
    """Decodes a standard CAN data frame from TXD (bit = `bit` clocks)."""

    def __init__(self, pin_get, bit=16):
        self.get, self.bit = pin_get, bit
        self.frames, self.state, self.prev = [], "idle", 1

    def tick(self, b):
        v = self.get(b)
        if self.state == "idle":
            if self.prev == 1 and v == 0:
                self.state, self.t, self.raw = "rx", 0, []
        else:
            if self.t % self.bit == self.bit // 2:
                self.raw.append(v)
            self.t += 1
            if len(self.raw) > 250 or (len(self.raw) > 30 and self.raw[-20:] == [1] * 20):
                self._finish()
                self.state = "idle"
        self.prev = v

    def _finish(self):
        raw, bits, run, val, i, stuff_errors = self.raw, [], 0, None, 0, 0
        out = dict(stuff_errors=0)
        # destuff up to the end of the CRC field
        while i < len(raw):
            b = raw[i]
            if run == 5:
                if b == val:
                    stuff_errors += 1
                run, val = 1, b
                i += 1
                continue
            bits.append(b)
            run = run + 1 if b == val else 1
            val = b
            i += 1
            if len(bits) >= 19:
                dlc = int("".join(map(str, bits[15:19])), 2)
                if len(bits) == 19 + 8 * dlc + 15:
                    break
        if run == 5:                       # stuff bit after the last CRC bit
            if i < len(raw) and raw[i] == val:
                stuff_errors += 1
            i += 1
        tail = raw[i:i + 10]
        dlc = int("".join(map(str, bits[15:19])), 2)
        n = 19 + 8 * dlc
        out.update(
            ident=int("".join(map(str, bits[1:12])), 2), dlc=dlc,
            data=bytes(int("".join(map(str, bits[19 + 8 * k:27 + 8 * k])), 2) for k in range(dlc)),
            crc=int("".join(map(str, bits[n:n + 15])), 2),
            crc_calc=crc15_can(bits[:n]), stuff_errors=stuff_errors, tail=tail)
        self.frames.append(out)
