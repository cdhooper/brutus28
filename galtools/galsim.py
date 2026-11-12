"""Fuse-level GAL16V8 / GAL22V10 simulator (scalar, one vector at a time).

Fuse semantics follow the Atmel WinCUPL device library, checked against
lincupl fuse plots and its csim simulator:

GAL16V8 (QF 2194): 64 rows x 32 columns, 8 rows per OLMC, OLMC 0 = pin 19.
  Columns 2k/2k+1 = true/complement of slot k; even slots are pins 2..9,
  odd slots are OLMC feedback 19..12.  XOR(n) 2048+n (1 = active high),
  AC1(n) 2120+n, PTD 2128+row (0 = term disabled), SYN 2192, AC0 2193.
  registered mode (SYN 0, AC0 1): pin 1 clock, pin 11 /OE for registers;
      AC1 0: D = sum ^ XOR, pin = !Q, feedback = !Q;
      AC1 1: row 0 is OE, rows 1..7 sum, feedback from the pin.
  complex (SYN 1, AC0 1): every OLMC like the registered-mode AC1 1 case;
      feedback slots of pins 19 and 12 carry pins 1 and 11.
  simple (SYN 1, AC0 0): AC1 0 output always enabled (8 rows), AC1 1 input;
      feedback slots carry 1, 19, 18, 17, 14, 13, 12, 11 (19..12).
GAL22V10 (QF 5892): 132 rows x 44 columns; row 0 AR, row 131 SP, OLMCs
  pins 23..14 with an OE row and 8,10,12,14,16,16,14,12,10,8 terms.
  Columns: pin 1, fb23, pin 2, fb22, ... pin 10, fb14, pin 11, pin 13.
  S0 = 5808+2n (1 = active high), S1 = 5809+2n (1 = combinational).
  registered: Q <= sum on pin 1 rising edge (SP sets, AR clears async),
      pin = Q if S0 else !Q, feedback = !Q;  combinational: feedback = pin.
Registers are 0 after power-up.

A pin that the chip does not drive reads the external (weak) drive.
"""

import re

PTS22 = [8, 10, 12, 14, 16, 16, 14, 12, 10, 8]


def read_jed(path):
    with open(path, errors="replace") as f:
        s = f.read()
    qf = int(re.search(r"\*\s*QF\s*(\d+)", s).group(1))
    df = re.search(r"\*\s*F\s*([01])", s)
    f = [int(df.group(1)) if df else 0] * qf
    for m in re.finditer(r"\*\s*L\s*(\d+)\s+([01\s]+)", s):
        a = int(m.group(1))
        for i, c in enumerate(re.sub(r"\s", "", m.group(2))):
            f[a + i] = int(c)
    return f


class Olmc:
    def __init__(self, pin, rows, oe_row, registered, active_high, mode):
        self.pin, self.rows, self.oe_row = pin, rows, oe_row
        self.registered, self.active_high, self.mode = registered, active_high, mode


class GAL:
    def __init__(self, src):
        f = read_jed(src) if isinstance(src, str) else list(src)
        self.fuses = f
        if len(f) == 2194:
            self._init16(f)
        elif len(f) in (5892, 5828):
            self._init22(f)
        else:
            raise ValueError("unsupported fuse count %d" % len(f))
        self.power_up()

    # -- fuse map decoding -------------------------------------------------

    def _rowmask(self, f, base, ncols):
        m = 0
        for c in range(ncols):
            if f[base + c] == 0:
                m |= 1 << c
        return m

    def _init16(self, f):
        self.kind, self.npins, self.vcc, self.gnd = "GAL16V8", 20, 20, 10
        self.ncols = 32
        syn, ac0 = f[2192], f[2193]
        if (syn, ac0) == (0, 1):
            self.mode = "registered"
        elif (syn, ac0) == (1, 1):
            self.mode = "complex"
        elif (syn, ac0) == (1, 0):
            self.mode = "simple"
        else:
            raise ValueError("GAL16V8 SYN=0 AC0=0 is not a valid mode")
        rows = []
        for r in range(64):
            rows.append(self._rowmask(f, 32 * r, 32) if f[2128 + r] else None)
        self.rows = rows
        self.ar_row = self.sp_row = None
        self.olmcs = []
        slots = []
        for k in range(8):
            slots.append(("pin", 2 + k))
            slots.append(("fb", 19 - k))
        simple_src = {19: 1, 18: 19, 17: 18, 16: 17, 15: 14, 14: 13, 13: 12, 12: 11}
        cols = []
        for kind, n in slots:
            if kind == "pin":
                cols.append(("pin", n))
            elif self.mode == "simple":
                cols.append(("pin", simple_src[n]))
            elif self.mode == "complex" and n in (19, 12):
                cols.append(("pin", 1 if n == 19 else 11))
            else:
                cols.append(("fb", n))
        self.cols = cols
        for i in range(8):
            pin = 19 - i
            xor, ac1 = f[2048 + i], f[2120 + i]
            base = 8 * i
            if self.mode == "registered" and ac1 == 0:
                o = Olmc(pin, list(range(base, base + 8)), None, True, xor == 1, "reg")
            elif self.mode in ("registered", "complex"):
                o = Olmc(pin, list(range(base + 1, base + 8)), base, False, xor == 1, "comb")
            elif ac1 == 0:
                o = Olmc(pin, list(range(base, base + 8)), "always", False, xor == 1, "comb")
            else:
                o = Olmc(pin, [], "never", False, xor == 1, "input")
            self.olmcs.append(o)
        self.clock_pin = 1 if self.mode == "registered" else None
        self.oe_pin = 11 if self.mode == "registered" else None

    def _init22(self, f):
        self.kind, self.npins, self.vcc, self.gnd = "GAL22V10", 24, 24, 12
        self.ncols = 44
        self.mode = "22v10"
        self.rows = [self._rowmask(f, 44 * r, 44) for r in range(132)]
        self.ar_row, self.sp_row = 0, 131
        cols = []
        for k in range(10):
            cols.append(("pin", 1 + k))
            cols.append(("fb", 23 - k))
        cols.append(("pin", 11))
        cols.append(("pin", 13))
        self.cols = cols
        self.olmcs = []
        r = 1
        for i, n in enumerate(PTS22):
            pin = 23 - i
            s0, s1 = f[5808 + 2 * i], f[5809 + 2 * i]
            o = Olmc(pin, list(range(r + 1, r + 1 + n)), r, s1 == 0, s0 == 1,
                     "reg" if s1 == 0 else "comb")
            self.olmcs.append(o)
            r += 1 + n
        assert r == 131
        self.clock_pin, self.oe_pin = 1, None

    # -- simulation --------------------------------------------------------

    def power_up(self):
        self.q = {o.pin: 0 for o in self.olmcs}
        self.ext = {}
        self.levels = {}
        self.driven = set()
        self.unstable = set()

    def signal_pins(self):
        return [p for p in range(1, self.npins + 1) if p not in (self.vcc, self.gnd)]

    def _colvals(self, levels):
        v = 0
        for k, (kind, n) in enumerate(self.cols):
            bit = (1 - self.q[n]) if kind == "fb" and self._olmc(n).registered else levels[n]
            v |= (1 << (2 * k)) if bit else (1 << (2 * k + 1))
        return v

    def _olmc(self, pin):
        for o in self.olmcs:
            if o.pin == pin:
                return o
        raise KeyError(pin)

    def _term(self, row, cv):
        m = self.rows[row]
        return m is not None and (m & ~cv) == 0

    def _sum(self, o, cv):
        return int(any(self._term(r, cv) for r in o.rows))

    def _outputs(self, levels):
        """Pins the chip drives and their values, for the given pin levels."""
        cv = self._colvals(levels)
        drive = {}
        for o in self.olmcs:
            if o.mode == "input":
                continue
            if o.mode == "reg" and self.kind == "GAL16V8":
                if levels[11] == 0:
                    drive[o.pin] = 1 - self.q[o.pin]
                continue
            if o.oe_row == "always":
                en = True
            elif o.oe_row == "never":
                en = False
            else:
                en = self._term(o.oe_row, cv)
            if not en:
                continue
            if o.mode == "reg":
                drive[o.pin] = self.q[o.pin] if o.active_high else 1 - self.q[o.pin]
            else:
                s = self._sum(o, cv)
                drive[o.pin] = s if o.active_high else 1 - s
        return drive, cv

    def settle(self, ext):
        """Apply external drive (dict pin -> 0/1 for every signal pin) and
        settle.  Returns (levels, driven-set)."""
        self.ext = dict(ext)
        pins = self.signal_pins()
        levels = dict(self.levels) if self.levels else {p: ext.get(p, 0) for p in pins}
        for p in pins:
            if p not in self.driven:
                levels[p] = ext.get(p, 0)
        drive = {}
        self.unstable = set()
        seen = []
        for it in range(200):
            if self.ar_row is not None:
                cv = self._colvals(levels)
                if self._term(self.ar_row, cv):
                    for o in self.olmcs:
                        if o.registered:
                            self.q[o.pin] = 0
            drive, _ = self._outputs(levels)
            new = {p: drive.get(p, ext.get(p, 0)) for p in pins}
            if new == levels:
                break
            if it < 50:
                levels = new
                continue
            # no convergence with simultaneous updates: change one pin at a
            # time (unequal delays); remember the pins that keep toggling
            ch = [p for p in pins if new[p] != levels[p]]
            seen.extend(ch)
            levels = dict(levels)
            levels[ch[it % len(ch)]] = new[ch[it % len(ch)]]
        else:
            self.unstable = set(seen)
        self.levels, self.driven = levels, set(drive)
        return levels, set(drive)

    def apply(self, ext):
        """Like settle, but a 0 -> 1 change on the clock pin clocks the
        registers with the levels before the edge."""
        cp = self.clock_pin
        if cp and self.levels and self.levels.get(cp) == 0 and ext.get(cp, 0) == 1:
            cv = self._colvals(self.levels)
            sp = self.sp_row is not None and self._term(self.sp_row, cv)
            nq = {}
            for o in self.olmcs:
                if not o.registered:
                    continue
                s = self._sum(o, cv)
                if self.kind == "GAL16V8":
                    nq[o.pin] = s ^ (1 if o.active_high else 0)
                else:
                    nq[o.pin] = 1 if sp else s
            self.q.update(nq)
        return self.settle(ext)
