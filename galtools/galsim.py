"""Fuse-level GAL16V8 / GAL20V8 / GAL22V10 simulator (scalar, one vector at
a time).

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
GAL20V8 (QF 2706): the same macrocells and modes with 40 columns, OLMC 0 =
  pin 22, pin 13 /OE in registered mode; XOR 2560+n, AC1 2632+n, PTD
  2640+row, SYN 2704, AC0 2705.  Its columns per mode are in GAL.V8, as
  CUPL fuses them.
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
        if len(f) in self.V8:
            self._init_v8(f)
        elif len(f) in (5892, 5828):
            self._init22(f)
        else:
            raise ValueError("unsupported fuse count %d" % len(f))
        # for the simulation: per array column, the pin it reads and its
        # true and complement bits (register feedback apart); per OLMC, the
        # product terms that can be true
        self._pins = [p for p in range(1, self.npins + 1) if p not in (self.vcc, self.gnd)]
        self._lvcols, self._fbcols = [], []
        for k, (kind, n) in enumerate(self.cols):
            reg = kind == "fb" and self._olmc(n).registered
            (self._fbcols if reg else self._lvcols).append((n, 1 << (2 * k), 1 << (2 * k + 1)))
        for o in self.olmcs:
            o.live = [self.rows[r] for r in o.rows if self.satisfiable(r)]
        self.power_up()

    # -- fuse map decoding -------------------------------------------------

    def _rowmask(self, f, base, ncols):
        m = 0
        for c in range(ncols):
            if f[base + c] == 0:
                m |= 1 << c
        return m

    # GAL16V8 and GAL20V8: the same macrocells and modes.  Per device: pins,
    # VCC, GND, array columns, the first fuse of XOR, AC1, PTD and SYN, the
    # pin of OLMC 0, the output enable pin of the registered mode, and per
    # mode what the array's column pairs read: a pin, or ("fb", pin) for an
    # OLMC's feedback (the register's in a registered OLMC, else its pin).
    # The GAL20V8's columns are as CUPL fuses them for g20v8s, g20v8ma and
    # g20v8ms.
    V8 = {
        2194: dict(kind="GAL16V8", npins=20, vcc=20, gnd=10, ncols=32, xor=2048, ac1=2120,
                   ptd=2128, syn=2192, top=19, oe=11, cols=dict(
                       registered=[2, "f19", 3, "f18", 4, "f17", 5, "f16", 6, "f15", 7, "f14",
                                   8, "f13", 9, "f12"],
                       complex=[2, 1, 3, "f18", 4, "f17", 5, "f16", 6, "f15", 7, "f14",
                                8, "f13", 9, 11],
                       simple=[2, 1, 3, 19, 4, 18, 5, 17, 6, 14, 7, 13, 8, 12, 9, 11])),
        2706: dict(kind="GAL20V8", npins=24, vcc=24, gnd=12, ncols=40, xor=2560, ac1=2632,
                   ptd=2640, syn=2704, top=22, oe=13, cols=dict(
                       registered=[2, 23, 3, "f22", 4, "f21", 5, "f20", 6, "f19", 7, "f18",
                                   8, "f17", 9, "f16", 10, "f15", 11, 14],
                       complex=[2, 1, 3, 23, 4, "f21", 5, "f20", 6, "f19", 7, "f18",
                                8, "f17", 9, "f16", 10, 14, 11, 13],
                       simple=[2, 1, 3, 23, 4, 22, 5, 21, 6, 20, 7, 17, 8, 16, 9, 15,
                               10, 14, 11, 13])),
    }

    def _init_v8(self, f):
        d = self.V8[len(f)]
        self.kind, self.npins, self.vcc, self.gnd = d["kind"], d["npins"], d["vcc"], d["gnd"]
        self.ncols = d["ncols"]
        syn, ac0 = f[d["syn"]], f[d["syn"] + 1]
        if (syn, ac0) == (0, 1):
            self.mode = "registered"
        elif (syn, ac0) == (1, 1):
            self.mode = "complex"
        elif (syn, ac0) == (1, 0):
            self.mode = "simple"
        else:
            raise ValueError("%s SYN=0 AC0=0 is not a valid mode" % self.kind)
        self.rows = [self._rowmask(f, self.ncols * r, self.ncols) if f[d["ptd"] + r] else None
                     for r in range(64)]
        self.ar_row = self.sp_row = None
        self.cols = [("fb", int(c[1:])) if isinstance(c, str) else ("pin", c)
                     for c in d["cols"][self.mode]]
        self.olmcs = []
        for i in range(8):
            pin = d["top"] - i
            xor, ac1 = f[d["xor"] + i], f[d["ac1"] + i]
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
        self.oe_pin = d["oe"] if self.mode == "registered" else None

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

    # -- structure ---------------------------------------------------------

    def satisfiable(self, row):
        """Whether product term `row` can be true (it has no column with
        both its true and complement fuse connected)."""
        m = self.rows[row]
        if m is None:
            return False
        return all((m >> (2 * k)) & 3 != 3 for k in range(self.ncols // 2))

    def _row_pins(self, row):
        """Pins whose level product term `row` reads (not register feedback)."""
        m = self.rows[row]
        return {n for k, (kind, n) in enumerate(self.cols)
                if (m >> (2 * k)) & 3 and not (kind == "fb" and self._olmc(n).registered)}

    def async_parts(self):
        """What can change the chip's state without a clock edge, as a list
        of descriptions ([] for a purely clocked design): the 22V10's
        asynchronous reset term, and combinational outputs fed back into
        the array in a loop, which can hold a value (latches)."""
        parts = []
        if (self.ar_row is not None and self.satisfiable(self.ar_row)
                and any(o.registered and any(self.satisfiable(r) for r in o.rows)
                        for o in self.olmcs)):
            parts.append("an asynchronous reset term")
        comb = {o.pin: o for o in self.olmcs if o.mode == "comb"}
        uses = {}
        for p, o in comb.items():
            rows = list(o.rows) + ([o.oe_row] if isinstance(o.oe_row, int) else [])
            uses[p] = set()
            for r in rows:
                if self.satisfiable(r):
                    uses[p] |= self._row_pins(r) & set(comb)
        # pins on a loop: those that reach themselves through `uses`
        loop = []
        for p in sorted(comb):
            seen, todo = set(), list(uses[p])
            while todo:
                q = todo.pop()
                if q not in seen:
                    seen.add(q)
                    todo.extend(uses[q])
            if p in seen:
                loop.append(p)
        if loop:
            parts.append("combinational feedback through pin%s %s" % (
                "s" if len(loop) > 1 else "", ", ".join(map(str, loop))))
        return parts

    # -- simulation --------------------------------------------------------

    def power_up(self):
        self.q = {o.pin: 0 for o in self.olmcs}
        self.ext = {}
        self.levels = {}
        self.driven = set()
        self.unstable = set()

    def signal_pins(self):
        return list(self._pins)

    def _colvals(self, levels):
        v = 0
        for n, t, c in self._lvcols:
            v |= t if levels[n] else c
        q = self.q
        for n, t, c in self._fbcols:
            v |= c if q[n] else t
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
        ncv = ~cv
        for m in o.live:
            if not m & ncv:
                return 1
        return 0

    def _outputs(self, levels):
        """Pins the chip drives and their values, for the given pin levels."""
        cv = self._colvals(levels)
        drive = {}
        for o in self.olmcs:
            if o.mode == "input":
                continue
            if o.mode == "reg" and self.oe_pin:
                if levels[self.oe_pin] == 0:
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
        pins = self._pins
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
                if self.kind != "GAL22V10":
                    nq[o.pin] = s ^ (1 if o.active_high else 0)
                else:
                    nq[o.pin] = 1 if sp else s
            self.q.update(nq)
        return self.settle(ext)
