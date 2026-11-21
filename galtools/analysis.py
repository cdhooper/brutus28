"""Black-box reverse engineering of a GAL16V8 / GAL20V8 / GAL22V10 from rig
captures.

1. discover: one exhaustive walk over every signal pin but pin 1 (and the
   pins the user fixed) at power-up.  An I/O pin whose driven value varies
   with the inputs is combinational; the others (inputs, registers,
   constants) are state candidates.  The enumerated pins V are the
   dedicated inputs, the candidates never driven, and the tri-state
   combinational pins.
2. explore: a state is, per candidate, the value it drives or Z, read with
   probe vectors (an enabling input vector per candidate, from the walks).
   States are found breadth-first from power-up: each state gets an
   exhaustive walk and a survey (a spread sample of the inputs clocked in
   once), then a full sweep of every input.  Full sweeps get return paths:
   clocked routes from other known states back, so the firmware rarely has
   to power-cycle.  If a new state drives a candidate that no probe
   enables, probes are added and the exploration restarts.
   Outputs with a constant value and an enable are not enumerated; if
   something reads one where the GAL lets it go (_let_go_inputs), it is
   enumerated too, and the exploration runs again.
3. assign: the would-be value of every registered pin per state; a pin
   that is never enabled in a state takes the complement of the only value
   it is ever seen with.
4. equations: per output function (value, output enable, register input)
   the support from single-variable flips, then a greedy two-level
   minimiser on both polarities; CUPL out.

Pin 1 is held low except for clock edges.  If clocking never changes the
state, the design is taken as combinational and pin 1 is enumerated too.
Limits: a combinational dependence on pin 1 of a registered 22V10 is not
captured; registers that never reach a pin are not seen.

A chip that changes state without a clock is detected: the power-up walk
is repeated in other orders (Analysis.ORDERS), and must read the same;
every other state's walk must leave the state as it found it; and in the
transition sweeps, reading the new state must not change it (the
firmware reads the clocked vector before and after the probes).  The
GAL22V10's asynchronous reset term, which clears every register, shows as
a return to the power-up state.  It is learned (_learn_ar) and explored
around: every write avoids its inputs, probes lie outside them, and the
states where it can fire are walked without them.  Anything else that
changes state without a clock (latches built from combinational feedback)
stops this analysis with AsyncDesign; reglatch.RegLatchAnalysis goes on
from there.  It keeps the pins that change without a clock in self.hist:
they are no state candidates, and the transition sweeps do not read them.
"""

import textwrap
from functools import reduce

from bitsim import litmasks
from rig import (CLOCK, MAX_CAND, MAX_PATH, MAX_PROBES, MAX_RET, MAX_RET_LEN, cube_set,
                 popcount, survey_mul)


class AsyncDesign(Exception):
    """The chip changes state without a clock edge in a way the analysis
    does not model."""


class Limit(RuntimeError):
    """The chip is beyond what the analysis can do, or the setup is wrong;
    the message says how."""


class LatchSuspected(AsyncDesign):
    """The power-up walk depends on its order, as latches built from
    combinational feedback make it (latch.LatchAnalysis takes over).
    pins: those that read differently."""

    def __init__(self, message, pins=()):
        super().__init__(message)
        self.pins = tuple(pins)


class Unsteady(AsyncDesign):
    """The same walk read differently twice (.pins): a bad contact, a
    floating pin, or an output that does not settle (osc.OscAnalysis)."""

    def __init__(self, msg, pins=()):
        super().__init__(msg)
        self.pins = list(pins)


class _Restart(Exception):
    """Explore again from scratch (the asynchronous reset term is known now)."""


DEV = {
    "GAL16V8": {"ded": [1, 2, 3, 4, 5, 6, 7, 8, 9, 11], "io": list(range(12, 20)),
                "cupl": "g16v8", "oe": 11, "modes": ("g16v8ma", "g16v8ms"),
                "fed": range(13, 19)},
    "GAL20V8": {"ded": list(range(1, 12)) + [13, 14, 23], "io": list(range(15, 23)),
                "cupl": "g20v8", "oe": 13, "modes": ("g20v8ma", "g20v8ms"),
                "fed": range(16, 22)},
    "GAL22V10": {"ded": list(range(1, 12)) + [13], "io": list(range(14, 24)),
                 "cupl": "g22v10"},
}


# -- two-level minimiser (from the MACH111 tester's mtest) -------------------

def _cube_set(care, val, k, full, v1):
    s = full
    for b in range(k):
        if care >> b & 1:
            s &= v1[b] if val >> b & 1 else full ^ v1[b]
    return s


def sop(on, off, k, order=None):
    """Greedy cover of the minterm bitset `on` avoiding `off` over k
    variables; [(care, val)] cubes.  A cube grown from a minterm drops
    the variable that takes in most of what is not covered yet; of
    several that take in as much, the first in order (default: the
    lowest)."""
    if not on:
        return []
    full, v1 = litmasks(k)
    if not off:
        return [(0, 0)]
    unc, cubes = on, []
    if order is None:
        order = range(k)
    while unc:
        m = (unc & -unc).bit_length() - 1
        care, val, cur = (1 << k) - 1, m, 1 << m
        while True:
            best = None
            for b in order:
                if not care >> b & 1:
                    continue
                c2 = care & ~(1 << b)
                v2 = val & c2
                s2 = _cube_set(c2, v2, k, full, v1)
                if s2 & off:
                    continue
                gain = popcount(s2 & unc)
                if best is None or gain > best[0]:
                    best = (gain, c2, v2, s2)
            if best is None:
                break
            _, care, val, cur = best
        cubes.append([care, val, cur])
        unc &= ~cur
    cubes.sort(key=lambda c: popcount(c[2] & on))
    i = 0
    while i < len(cubes):
        others = 0
        for j, c in enumerate(cubes):
            if j != i:
                others |= c[2]
        if not (cubes[i][2] & on & ~others):
            cubes.pop(i)
        else:
            i += 1
    return [(c[0], c[1]) for c in cubes]


def fewer(cubes, on, off, k, rounds=6):
    """A cover of `on` that avoids `off`, with fewer cubes if it finds
    one: reduce every cube to the points that only it covers, expand it
    again (it may now grow another way and take in a neighbour), drop the
    cubes the others cover, and again, as the Espresso minimiser does.
    For a function that is known only in part and has more cubes than its
    macrocell has product terms: the chip's own cover fits."""
    full, v1 = litmasks(k)

    def cs(care, val):
        return _cube_set(care, val, k, full, v1)

    def around(points):
        care = val = 0
        for b in range(k):
            if not points & ~v1[b] & full:
                care |= 1 << b
                val |= 1 << b
            elif not points & v1[b]:
                care |= 1 << b
        return care, val

    def lean(F):
        F.sort(key=lambda c: popcount(c[2] & on))
        i = 0
        while i < len(F):
            others = 0
            for j, c in enumerate(F):
                if j != i:
                    others |= c[2]
            if F[i][2] & on & ~others:
                i += 1
            else:
                F.pop(i)
    F = [(c, v, cs(c, v)) for c, v in cubes]
    best = list(F)
    for r in range(rounds):
        F.sort(key=lambda c: -popcount(c[2] & on))
        i = 0
        while i < len(F):
            others = 0
            for j, c in enumerate(F):
                if j != i:
                    others |= c[2]
            mine = F[i][2] & on & ~others
            if not mine:
                F.pop(i)
                continue
            c, v = around(mine)
            F[i] = (c, v, cs(c, v))
            i += 1
        F.sort(key=lambda c: popcount(c[2] & on))
        for i in range(len(F)):
            care, val, s = F[i]
            while True:
                top = None
                for b in range(k):
                    if care >> b & 1:
                        c2 = care & ~(1 << b)
                        s2 = cs(c2, val & c2)
                        if s2 & off:
                            continue
                        gain = popcount(s2 & on)
                        if top is None or gain > top[0]:
                            top = (gain, c2, val & c2, s2)
                if top is None:
                    break
                _, care, val, s = top
            F[i] = (care, val, s)
        lean(F)
        if len(F) < len(best):
            best = list(F)
        elif r:
            break
    return [(c, v) for c, v, _ in best]


def sop_lits(on, off, lits, full):
    """A greedy cover of on (context -> the x) that avoids off, as
    products of the literals lits ((text, {context: the x where it is
    true})): cubes as lists of literal numbers.  From an uncovered
    point's literals, those are dropped one by one that keep the
    product out of off, the one first that covers most."""
    unc = {c: v for c, v in on.items() if v}
    ctxs = [c for c in set(on) | set(off)]

    def product(cube):
        out = {}
        for c in ctxs:
            v = full
            for i in cube:
                v &= lits[i][1].get(c, 0)
                if not v:
                    break
            out[c] = v
        return out
    cubes = []
    while unc:
        c0 = next(iter(unc))
        x0 = lowest(unc[c0])
        cube = [i for i, (_, m) in enumerate(lits) if m.get(c0, 0) >> x0 & 1]
        while len(cube) > 0:
            # the products without each literal: before[i] & after[i + 1]
            k = len(cube)
            before = [dict.fromkeys(ctxs, full)]
            for i in cube:
                m = lits[i][1]
                before.append({c: before[-1][c] & m.get(c, 0) for c in ctxs})
            after = [dict.fromkeys(ctxs, full)]
            for i in reversed(cube):
                m = lits[i][1]
                after.append({c: after[-1][c] & m.get(c, 0) for c in ctxs})
            after.reverse()
            best = None
            for t in range(k):
                gain, ok = 0, True
                for c in ctxs:
                    v = before[t][c] & after[t + 1][c]
                    if v & off.get(c, 0):
                        ok = False
                        break
                    u = unc.get(c)
                    if u:
                        gain += popcount(v & u)
                if ok and (best is None or gain > best[0]):
                    best = (gain, t)
            if best is None:
                break
            del cube[best[1]]
        cov = product(cube)
        for c in list(unc):
            unc[c] &= ~cov[c]
            if not unc[c]:
                del unc[c]
        cubes.append((cube, cov))
    # a cube that the others cover between them is not needed
    i = 0
    while i < len(cubes):
        need = False
        for c, v in on.items():
            rest = 0
            for j, (_, cov) in enumerate(cubes):
                if j != i:
                    rest |= cov[c]
            if v & cubes[i][1][c] & ~rest:
                need = True
                break
        if need:
            i += 1
        else:
            cubes.pop(i)
    return [c for c, _ in cubes]


def tight(cubes, on, k, own=None):
    """The cubes, claiming less where the function is known only in part.

    sop() grows a cube as far as `off` allows.  Where every point is known
    to be on or off, that is the function.  Where some were never seen (a
    latch's excitation is known only at the points the walks went
    through), a cube then also takes in points that nothing was seen of:
    a latch held by P4 & ... got a term without P4, because no walk had
    let pin 4 fall in just that corner, and the rebuilt chip kept the
    latch where the original lets go.

    - Every cube gets the literals that hold at every point of `on`: if
      the function was never seen on with pin 4 low, no cube says it is.
    - A cube that reads variable `own` (a latch's own pin: a term that
      holds the latch) is shrunk to the smallest cube around the points
      of `on` it covers.  What holds a latch is seen at every point where
      it rests, so there is much to go by.  The other cubes (what sets
      it) are seen only in the steps that set it; shrunk like that, they
      generalise so little that checking the steps keeps adding to them,
      and the design no longer fits the chip."""
    full, v1 = litmasks(k)
    if not on:
        return cubes

    def around(points):
        care = val = 0
        for b in range(k):
            if not points & ~v1[b] & full:
                care |= 1 << b
                val |= 1 << b
            elif not points & v1[b]:
                care |= 1 << b
        return care, val
    care, val = around(on)
    out = []
    for c, v in cubes:
        got = _cube_set(c, v, k, full, v1) & on
        if own is not None and c >> own & 1 and got:
            out.append(around(got))
        else:
            out.append((c | care, v | val))
    return out


# -- helpers -----------------------------------------------------------------

def swap(f, b, v1):
    """Bitset f with the halves of variable b exchanged."""
    s = 1 << b
    return ((f & v1[b]) >> s) | ((f & ~v1[b]) << s)


def lowest(xs):
    return (xs & -xs).bit_length() - 1


def _pins(ps):
    return "pin%s %s" % ("s" if len(ps) > 1 else "", ", ".join(map(str, ps)))


def _state(s):
    """A state as text: the candidates' values, then (after a slash) the
    levels of the latch pins that are part of it."""
    return "".join("/" + "".join(map(str, v)) if isinstance(v, tuple) else str(v) for v in s)


def _cube(c, name):
    return " & ".join(("" if v else "!") + name(p) for p, v in sorted(c.items()))


HOW = ("/* Recovered by galre from captures of the chip.\n"
       "   Compile with: cupl -jaxm4 %s.pld */")
KEEP = ("/* A latch has product terms that only hold it while an input changes.\n"
        "   Minimising would remove them. */")


class Table:
    """One state's exhaustive walk: per pin, driven and value bitsets over
    the x in care[p].  A walk that skipped the x outside `care` knows
    nothing there, and an enumerated pin's drive is told from the read
    of its neighbour x ^ bit, so it needs both."""

    def __init__(self, V, la, lb, full, v1, care=None):
        care = full if care is None else care
        self.la, self.lb, self.walked = la, lb, care
        self.driven, self.value, self.care = {}, {}, {}
        for p in la:
            if p in V:
                b = V.index(p)
                c = care & swap(care, b, v1) & full
                sw = swap(la[p], b, v1) & full
                self.driven[p] = (full ^ (la[p] ^ sw)) & c
            else:
                c = care
                self.driven[p] = (full ^ (la[p] ^ lb[p])) & c
            self.value[p] = la[p] & c
            self.care[p] = c


class Analysis:
    def __init__(self, rig, device, fixed=None, survey=64, max_states=1024, log=print):
        self.rig, self.device, self.dev, self.log = rig, device, DEV[device], log
        self.pins = rig.pins
        self.fixed = dict(fixed or {})
        self.base = dict(self.fixed)
        self.survey = survey
        self.max_states = max_states
        self.clock_in_v = CLOCK in self.fixed
        self.late_comb = {}       # pin -> enumerate it (tri-state)
        self.hist = set()         # pins that change without a clock (reglatch)
        self.mixed = False        # the chip holds state without a clock too (reglatch)
        self.bidir = set()        # pins of hist that the GAL does not always drive
        self._power_up = {}
        self.ar = None            # the asynchronous reset term's inputs, pin -> value
        self.ar_set = 0           # the x in it
        self.ar_cand = []         # its pins that would be state candidates
        self.oe_const = []        # registers with a constant input (reglatch, pin 11 held)
        self.lsig = ()            # latch pins whose levels are part of a state (reglatch)
        self.notes = []           # what a reader of the source should know, as comments
        self.cvar = {}            # state -> its sweeps with the latches at other levels
        self.cmap = {}            # state -> the state with latch levels that stands for it
        self._wsig = {}
        self.ar_on = set()        # states in which it fires
        self.ar_regs = {}         # its register literals, register pin -> value
        self.ar_dc = set()
        self.scover = {}          # covers that read other outputs' pins (_read_outputs)

    # 1. discovery -----------------------------------------------------------
    def discover(self):
        V0 = [p for p in self.pins
              if (p != CLOCK or self.clock_in_v) and p not in self.fixed]
        full, v1 = litmasks(len(V0))
        if self.clock_in_v not in self._power_up:
            la, lb, _ = self.rig.walk(V0, self.base, [])
            self.walk0 = V0, la
            self._check_order(V0, la, lb)
            self._power_up[self.clock_in_v] = Table(V0, la, lb, full, v1)
        t = self._power_up[self.clock_in_v]
        for p in sorted(self.fixed):
            if t.value[p] != (full if self.fixed[p] else 0):
                raise Limit("pin %d is held at %d, but the GAL drives it; do not hold "
                                   "it (--fix)" % (p, self.fixed[p]))
        self.comb0, self.cand = [], []
        for p in self.dev["io"]:
            if p in self.fixed:
                continue                      # held: an input at a known value
            if p in self.hist:
                continue                      # a latch or one of its outputs
            d = t.driven[p]
            v = t.value[p] & d
            if (d and v not in (0, d)) or p in self.late_comb:
                self.comb0.append(p)          # varies with the inputs
            else:
                self.cand.append(p)           # input, register or constant
        tri = [p for p in self.comb0 if full ^ t.driven[p] or self.late_comb.get(p)]
        never = [p for p in self.cand if not t.driven[p]]
        ded = [p for p in self.dev["ded"]
               if (p != CLOCK or self.clock_in_v) and p not in self.fixed]
        self.V = sorted(set(ded + never + tri) | self.bidir)
        self.full, self.v1 = litmasks(len(self.V))
        if self.ar:
            if any(p not in self.V for p in self.ar):
                raise AsyncDesign("the asynchronous reset term reads pins that are no longer "
                                  "enumerated")
            self.ar_set = cube_set(self.ar, self.V, self.base, self.full, self.v1)
            # A state's signature is read with every candidate driven both
            # ways, to see whether the GAL drives it.  On a pin that the
            # reset term reads, that would fire the term.
            self.ar_cand = [p for p in self.cand if p in self.ar]
            self.cand = [p for p in self.cand if p not in self.ar]
        self.log("power-up walk: combinational %s (tri-state %s), state candidates %s "
                 "(never driven %s)%s" % (
                     self.comb0, tri, self.cand, never,
                     ", changing without a clock %s" % sorted(self.hist) if self.hist else ""))
        self.log("enumerated pins (%d): %s%s" % (
            len(self.V), self.V,
            ", fixed %s" % self.fixed if self.fixed else ""))
        self.probe_x = []

    # Walk orders that make a latch see its inputs change in another order
    # than counting does: descending, bit-reversed (the pin that changed
    # fastest changes slowest) and both.  Tried on set/reset and transparent
    # latches with their inputs on every pair of pins, and on set-only bits,
    # together they show every latch that can leave its power-up value;
    # no two of them do.
    # In all of these, a pin rises together with every faster pin falling
    # (or the reverse), so a latch that holds only while another signal
    # stays as it is never gets to hold: in a GAL from a graphics card,
    # !P14 = !P8 # !P14 & !P13 held only if pin 8 rose alone.  Gray code
    # orders change one pin at a time; the mixed ones do so with most
    # pins changing often (rig.mix_bits).
    ORDERS = (("descending", dict(mul=-1)), ("bit-reversed", dict(rev=1)),
              ("bit-reversed descending", dict(mul=-1, rev=1)),
              ("Gray code", dict(gray=1)), ("mixed Gray code", dict(gray=1, mix=1)),
              ("another mixed Gray code", dict(gray=1, mix=2)))

    def _check_order(self, V, la, lb):
        """Walk the power-up state again in other orders.  A clocked chip
        reads the same, whatever the order.  If it does not, another walk
        in counting order tells state that changes without a clock from
        readings that do not repeat."""
        n = len(V)
        if n < 2:
            return
        found, pins = None, set()
        for order, kw in self.ORDERS:
            kw = dict(kw)
            if "mul" in kw:
                kw["mul"] &= (1 << n) - 1
            la2, lb2, _ = self.rig.walk(V, self.base, [], **kw)
            d = {p: (la[p] ^ la2[p]) | (lb[p] ^ lb2[p]) for p in la if p not in self.hist}
            d = {p: v for p, v in d.items() if v}
            if d and found is None:
                found = order, d
            pins.update(d)                    # every order: each may show other pins
        if found is None:
            return
        name, diff = found
        la3, lb3, _ = self.rig.walk(V, self.base, [])
        unsteady = sorted(p for p in la if p not in self.hist
                          and (la[p] ^ la3[p]) | (lb[p] ^ lb3[p]))
        if unsteady:
            raise Unsteady("the same walk read differently twice on %s: a bad contact, "
                           "a floating pin or an oscillating output" % _pins(unsteady),
                           unsteady)
        xs = reduce(lambda a, b: a | b, diff.values())
        x = lowest(xs)
        raise LatchSuspected(
            "%s read differently when the inputs are walked in %s order (%d of %d "
            "input vectors, the first %s): the chip holds state without a clock, like a "
            "latch built from combinational feedback" % (
                _pins(sorted(diff)), name, popcount(xs), 1 << n,
                " ".join("P%d=%d" % (p, x >> i & 1) for i, p in enumerate(V))),
            pins=sorted(pins))

    def _disturbed(self, path, sigs, around=False):
        before, after = (self._sig_obs(o) for o in sigs)
        if (not around and self.ar is None and self.device == "GAL22V10"
                and self._reg(after) == self._reg(self.power_up_state)):
            self._learn_ar(path)
        raise AsyncDesign(
            "walking the inputs in state %s (%d clocks from power-up)%s changed it to %s: the "
            "chip changes state without a clock%s" % (
                _state(before), len(path),
                " around the asynchronous reset term" if around else "", _state(after),
                " in another way too" if self.ar else ""))

    def _flagged(self, path, x, pins=()):
        """Reading the state that clocking x in from the path state leads
        to changed it (pins read differently)."""
        if self.ar is None and self.device == "GAL22V10" and len(path) < MAX_PATH:
            self._learn_ar(path + (x,))
        raise AsyncDesign(
            "reading the state %d clocks from power-up (the last with %s) changed it: the chip "
            "changes state without a clock" % (len(path) + 1, self._inputs(x)))

    def _inputs(self, x):
        return " ".join("P%d=%d" % (p, x >> i & 1) for i, p in enumerate(self.V))

    def _gray_last(self, sets, x1):
        """For each bitset, the x in it that a Gray code walk from x1
        reaches last (None if it is empty)."""
        N = 1 << len(self.V)
        bs = [b.to_bytes((N + 7) // 8, "little") for b in sets]
        out = [None] * len(sets)
        todo = [k for k, b in enumerate(sets) if b]
        for i in range(N - 1, -1, -1):
            if not todo:
                break
            x = i ^ (i >> 1) ^ x1
            for k in list(todo):
                if bs[k][x >> 3] >> (x & 7) & 1:
                    out[k] = x
                    todo.remove(k)
        return out

    def _learn_ar(self, path):
        """The state at the end of path returns to the power-up state
        without a clock: find the inputs of the asynchronous reset term,
        then explore again around them.  The reset clears every register,
        so it shows only in other states, and it is a product term: it
        fires in a state for the x in a cube C (the same in every state in
        which it fires, from its register literals).

        1. Walk the state in Gray code order from the last vector of the
           path, so one pin changes per write and no momentary mix of pin
           values can fire the term.  The reads differ from the power-up
           walk up to the reset, and agree after it.  The last vector
           that reads differently on an output, d, tells the state from
           the power-up state.
        2. For each enumerated pin p and value v: walk the state again,
           skipping and avoiding the x with p = v, then read d.  The state
           survives exactly when C lies within p = v: p = v is a literal
           of C."""
        V, n = self.V, len(self.V)
        drives = self.drives(path)
        x1 = path[-1]
        what = "the state %d clocks from power-up (the last with %s)" % (len(path),
                                                                          self._inputs(x1))
        la, lb, _ = self.rig.walk(V, self.base, drives, gray=1, start=x1)
        la0, lb0 = self._raw0
        diff = out = 0
        for p in la:
            if p in self.hist:
                continue
            diff |= (la[p] ^ la0[p]) | (lb[p] ^ lb0[p])
            if p not in V and p != CLOCK:
                out |= la[p] ^ la0[p]
        last, d = self._gray_last([diff, out], x1)
        if last is None:
            raise AsyncDesign("%s changed without a clock, but walked in Gray code order it reads "
                              "like the power-up state throughout: the chip changes state "
                              "without a clock in a way galre does not model" % what)
        if last == (1 << (n - 1)) ^ x1:          # the walk's last vector
            raise AsyncDesign("%s changed without a clock, but a walk in Gray code order (one "
                              "pin changing at a time) leaves it alone: the chip changes state "
                              "without a clock in a way galre does not model" % what)
        if d is None:
            raise AsyncDesign("%s returns to the power-up state without a clock, but no output "
                              "tells the two states apart" % what)
        dpins = [p for p in sorted(la) if p not in V and p != CLOCK and p not in self.hist
                 and (la[p] ^ la0[p]) >> d & 1][:MAX_CAND]
        want = tuple(la[p] >> d & 1 for p in dpins)
        probe = [self._drive(d)]
        lits = {}
        for b, p in enumerate(V):
            for v in (1 ^ x1 >> b & 1, x1 >> b & 1):
                k = {p: v}
                _, _, sigs = self.rig.walk(V, self.base, drives, gray=1, start=x1, skip=k,
                                           avoid=k, probes=probe, cand=dpins, data=False)
                if tuple(a for a, _ in sigs[1][0]) == want:
                    lits[p] = v
                    break
        cset = cube_set(lits, V, self.base, self.full, self.v1)
        if not lits or (cset >> x1 | cset >> d) & 1:
            raise AsyncDesign("%s returns to the power-up state without a clock, but not for "
                              "the inputs of one product term" % what)
        self.ar = lits
        self.log("asynchronous reset: %s returns to the power-up state when %s; exploring "
                 "again around it" % (what, _cube(lits, self.name)))
        raise _Restart()

    def _drive(self, x):
        d = dict(self.base)
        for i, p in enumerate(self.V):
            d[p] = x >> i & 1
        return d

    def drives(self, path):
        return [self._drive(x) for x in path]

    def probes(self):
        return [self._drive(x) for x in self.probe_x]

    def _add_probes(self, t):
        """An enabling input vector for every candidate the table drives,
        outside the asynchronous reset term."""
        added = False
        for r in self.cand:
            d = t.driven[r] & ~self.ar_set
            if d and not any(d >> x & 1 for x in self.probe_x):
                if len(self.probe_x) == MAX_PROBES:
                    raise Limit("more than %d probe vectors needed to tell the states apart" %
                                MAX_PROBES)
                self.probe_x.append(lowest(d))
                added = True
        return added

    # 2. exploration ---------------------------------------------------------
    # With lsig, a state is the candidates' values and the levels of the
    # latch pins (RegLatchAnalysis, where a clock edge goes one way or
    # another by a latch).  The chip is then left the same way after
    # every clocked step, in a path as in a sweep (rig, psig), and an
    # observation ends with the latch levels.
    def _wctx(self):
        """Options of a walk from a state, with lsig."""
        if self.lsig and self.probe_x:
            return dict(probes=self.probes(), cand=self.cand, psig=True, also=self.lsig)
        return {}

    def _tctx(self):
        """Options of a sweep: the latch pins are left out of the reads,
        or with lsig read, without flagging a vector."""
        if self.lsig and self.probe_x:
            return dict(psig=True, also=self.lsig, nochk=sorted(self.hist))
        return dict(ignore=sorted(self.hist))

    def _reg(self, s):
        """A state without the latch levels."""
        return s[:len(self.cand)]

    def _robs(self, obs):
        """An observation without the latch levels."""
        return obs[:len(self.probe_x)]

    def _sig_obs(self, obs):
        tail = obs[len(self.probe_x):] if self.lsig else ()
        sig = []
        for i, r in enumerate(self.cand):
            v = "Z"
            for pr in self._robs(obs) if tail else obs:
                a, b = pr[i]
                if a == b:
                    v = a
                    break
            sig.append(v)
        return tuple(sig) + tail

    def _sig_path(self, path, t):
        """The state at the end of path, from its table and (with lsig)
        the observation before its walk."""
        obs = self._wsig.get(path)
        tail = obs[len(self.probe_x):] if obs and self.lsig else ()
        return self._sig_table(t) + tail

    def _sig_table(self, t):
        sig = []
        for r in self.cand:
            d = t.driven[r]
            if not d:
                sig.append("Z")
                continue
            v = t.value[r] & d
            sig.append(0 if v == 0 else 1 if v == d else "?")
        return tuple(sig)

    def _walk(self, path):
        """The state's table.  Where the asynchronous reset term fires (the
        walk returns to the power-up state), the state is walked again
        without its inputs."""
        if path not in self._walks:
            if len(path) > MAX_PATH:
                raise Limit("a state is %d clocks from power-up, more than the firmware's "
                            "%d" % (len(path), MAX_PATH))
            care = None
            if path and self.probe_x:
                drives = self.drives(path)
                kw = self._wctx() or dict(probes=self.probes(), cand=self.cand)
                la, lb, sigs = self.rig.walk(self.V, self.base, drives, avoid=self.ar, **kw)
                if self._robs(sigs[0]) != self._robs(sigs[1]):
                    if not self.ar or (self._reg(self._sig_obs(sigs[1]))
                                       != self._reg(self.power_up_state)):
                        self._disturbed(path, sigs)
                    la, lb, sigs = self.rig.walk(self.V, self.base, drives, skip=self.ar,
                                                 avoid=self.ar, **kw)
                    if self._robs(sigs[0]) != self._robs(sigs[1]):
                        self._disturbed(path, sigs, around=True)
                    care = self.full ^ self.ar_set
                    self.ar_paths.add(path)
                self._wsig[path] = sigs[0]
                self._one_at_a_time(path, la, lb, care)
            else:
                la, lb, _ = self.rig.walk(self.V, self.base, self.drives(path))
                if not path:
                    self._raw0 = la, lb
            self._walks[path] = Table(self.V, la, lb, self.full, self.v1, care)
        return self._walks[path]

    def _one_at_a_time(self, path, la, lb, care):
        """Walk the state once more, in a mixed Gray code order from the
        vector the path left the chip at, so one pin changes at a time.
        It must read as in counting order: a latch that holds only while
        the other pins keep still shows here (see ORDERS).  The power-up
        state gets this in _check_order."""
        n = len(self.V)
        if n < 2:
            return
        kw = dict(skip=self.ar, avoid=self.ar) if care is not None else dict(avoid=self.ar)
        kw.update(self._wctx())
        la2, lb2, _ = self.rig.walk(self.V, self.base, self.drives(path), gray=1,
                                    mix=1 + len(path), start=path[-1], **kw)
        m = self.full if care is None else care
        pins = sorted(p for p in la if p not in self.hist and p != CLOCK
                      and ((la[p] ^ la2[p]) | (lb[p] ^ lb2[p])) & m)
        if pins:
            x = lowest(reduce(lambda a, b: a | b,
                              (((la[p] ^ la2[p]) | (lb[p] ^ lb2[p])) & m for p in pins)))
            self._history_dependent(pins, AsyncDesign(
                "%s read differently in the state %d clocks from power-up when one input "
                "changes at a time (the first at %s): the chip holds state without a "
                "clock there" % (_pins(pins), len(path), self._inputs(x))))

    def _history_dependent(self, pins, err):
        raise err

    def _returns(self, s):
        """(observation, drives) routes from known states back to s."""
        s = self.cmap.get(s, s)
        hop = {s: None}
        frontier = [s]
        while frontier:
            nxt = []
            for b in frontier:
                for a, m in self.edges.items():
                    if a in hop or b not in m:
                        continue
                    hop[a] = (lowest(m[b]), b)
                    nxt.append(a)
            frontier = nxt
        weight = self.edges.get(s, {})
        out = []
        for a in sorted(hop, key=lambda a: -popcount(weight.get(a, 0))):
            if a == s:
                continue
            path, c = [], a
            while c != s:
                x, c = hop[c]
                path.append(x)
            if len(path) > MAX_RET_LEN:
                continue
            for o in self.obs_of.get(a, ()):
                out.append((o, self.drives(path)))
        return out[:MAX_RET]

    def _sweep(self, s, count, queue):
        """Clock every (or a sample of the) input vector in from state s.
        False if a new state needs more probes."""
        path, _ = self.states[s]
        n = len(self.V)
        if count is not None and count >= 1 << n:
            count = None
        res = self.rig.transitions(self.V, self.base, self.drives(path), self.probes(),
                                   self.cand, count=count,
                                   mul=1 if count is None else survey_mul(n),
                                   returns=self._returns(s), avoid=self.ar, chk=True,
                                   pre=self.mixed, **self._tctx())
        obs, flags = res[0], res[1]
        if self.mixed and count is None:
            self.pre[s] = res[2]              # the pin levels each x was clocked in with
        if flags:
            self._flagged(path, *flags[0])
        m = self.edges.setdefault(s, {})
        if count is None:
            self.trans[s] = {}
        for o, xs in obs.items():
            s2 = self._sig_obs(o)
            for s3, x3 in m.items():
                if s3 != s2 and x3 & xs and not self.mixed:
                    self._two_ways(
                        "clocking %s in from state %s led to state %s once and to %s once: "
                        "the next state depends on more than the inputs and the registers "
                        "(a register input that reads a latch), or the readings do not "
                        "repeat" % (self._inputs(lowest(x3 & xs)), _state(s), _state(s3),
                                    _state(s2)))
            self.obs_of.setdefault(s2, set()).add(o)
            m[s2] = m.get(s2, 0) | xs
            if count is None:
                self.trans[s][s2] = self.trans[s].get(s2, 0) | xs
            if s2 in self.states:
                continue
            p2 = path + (lowest(xs),)
            t = self._walk(p2)
            st = self._sig_path(p2, t)
            if st != s2:
                if self._add_probes(t):
                    return False
                late = [r for r, v in zip(self.cand, st) if v == "?"]
                if not late:
                    self._unrepeatable(
                        "clocking %s in from state %s led to state %s, but the same clocks "
                        "from power-up lead to %s: the chip has state that no pin shows, or "
                        "its readings do not repeat" % (
                            self._inputs(lowest(xs)), _state(s), _state(s2), _state(st)))
                # constant at power-up, input-dependent here: combinational
                for r in late:
                    self.late_comb[r] = bool(self.full ^ t.driven[r])
                self.log("pins %s depend on the inputs in state %s: combinational" % (late, s2))
                self.rediscover = True
                return False
            self.states[s2] = (p2, t)
            queue.append(s2)
            if len(self.states) > self.max_states:
                raise Limit("more than %d states (--max-states)" % self.max_states)
        return True

    def _explore_once(self):
        self._walks, self._wsig, self.cmap, self.cvar = {}, {}, {}, {}
        self.ar_paths = set()
        t0 = self._walk(())
        self._add_probes(t0)
        if self._wctx():
            # the power-up state as a sweep leaves it: after the signature reads
            self._wsig[()] = self.rig.walk(self.V, self.base, [], data=False,
                                           **self._wctx())[2][0]
        s0 = self._sig_path((), t0)
        self.power_up_state = s0
        self.states = {s0: ((), t0)}
        self.trans, self.edges, self.obs_of, self.pre = {}, {}, {}, {}
        if self.clock_in_v:
            self.trans[s0] = {s0: self.full}
            return True
        if not self.probe_x:
            self.log("no candidate is ever driven: no clocked state to see")
            self.trans[s0] = {s0: self.full}
            return True
        survey = None if self.survey is None or self.survey >= 1 << len(self.V) else self.survey
        queue = [s0]
        while True:
            while queue:
                s = queue.pop(0)
                if not self._sweep(s, survey, queue):
                    return False
            todo = [s for s in self.states if s not in self.trans]
            if not todo:
                return True
            if not self._sweep(todo[-1], None, queue):
                return False

    def explore(self, max_rounds=24):
        for rnd in range(max_rounds):
            self.rediscover = False
            try:
                done = self._explore_once()
            except _Restart:
                self.discover()           # new probes, outside the reset term
                continue
            if done:
                break
            if self.rediscover:
                self.discover()
            else:
                self.log("round %d: a new state drives a pin no probe enables; %d probes "
                         "now, exploring again" % (rnd + 1, len(self.probe_x)))
        else:
            raise Limit("the probe vectors that tell the states apart did not settle")
        self._fold()
        for r in self.ar_cand:
            if any(t.driven[r] for _, t in self.states.values()):
                raise AsyncDesign("the asynchronous reset term reads pin %d, which the chip "
                                  "drives in some states: galre does not model that" % r)
        self.inputs, self.reg, self.comb = list(self.ar_cand), [], list(self.comb0)
        for i, r in enumerate(self.cand):
            sigs = {sig[i] for sig in self.states}
            if all(not t.driven[r] for _, t in self.states.values()):
                self.inputs.append(r)
            elif sigs <= {0, 1, "Z"} and (len(sigs - {"Z"}) == 2 or "Z" in sigs):
                self.reg.append(r)
            else:
                self.comb.append(r)       # constant or input-dependent
        for i, r in enumerate(self.cand):
            # A register's feedback comes from the register, not from its
            # pin.  A pin that shows one level where it is driven, and that
            # other logic reads where the chip lets it go, is a constant
            # output with an enable (an open-drain copy of a register,
            # say), if the other registers tell the states apart.
            if (r in self.reg and r in self.V
                    and len({s[i] for s in self.states} - {"Z"}) == 1 and self._pin_read(r)):
                rest = [k for k, q in enumerate(self.cand) if q in self.reg and q != r]
                if len({tuple(s[k] for k in rest) for s in self.states}) == len(self.states):
                    self.reg.remove(r)
                    self.comb.append(r)
        oe = self.dev.get("oe")
        if oe is not None and self.fixed.get(oe) == 0 and not self.clock_in_v:
            # Held low, pin 11 (13) enables every register: a pin that is
            # not always driven has an enable of its own.
            for r in list(self.reg):
                if any(t.driven[r] != t.care[r] for _, t in self.states.values()):
                    self.reg.remove(r)
                    self.comb.append(r)
            for r in self.oe_const:
                if r in self.comb:
                    self.comb.remove(r)
                    self.reg.append(r)
        if oe in self.V and not self.clock_in_v:
            # A registered GAL16V8's or GAL20V8's registers are enabled by
            # the OE pin (11, 13) and by nothing else.  A pin that changes with the state but is not
            # driven exactly while pin 11 is low is a combinational output
            # that reads the registers: it has an enable of its own.
            b = self.V.index(oe)
            low = self.full ^ self.v1[b]
            for r in list(self.reg):
                if any(t.driven[r] != t.care[r] & low for _, t in self.states.values()):
                    self.reg.remove(r)
                    self.comb.append(r)
        self.log("%d states reachable from power-up; inputs %s, registered %s, "
                 "combinational %s" % (len(self.states), self.inputs, self.reg, self.comb))
        self.ar_on = {s for s, (path, _) in self.states.items() if path in self.ar_paths}
        if self.ar:
            self.log("the asynchronous reset term fires in %d of them" % len(self.ar_on))

    def _pin_read(self, r):
        """Does a transition or another pin depend on the level of the
        enumerated pin r where the chip lets it go?"""
        b = self.V.index(r)
        for s, (_, t) in self.states.items():
            free = (t.care[r] ^ t.driven[r]) & self.full
            if not free:
                continue
            for xs in self.trans.get(s, {}).values():
                if (xs ^ swap(xs, b, self.v1)) & free:
                    return True
            for p in t.value:
                if p in self.V or p == CLOCK or p in self.hist:
                    continue
                d, v = t.driven[p], t.value[p]
                ds, vs = swap(d, b, self.v1), swap(v, b, self.v1)
                if ((d ^ ds) | (v ^ vs) & d & ds) & free & t.care[p] & swap(t.care[p], b, self.v1):
                    return True
        return False

    def _unrepeatable(self, msg):
        """The same clocks led to two states."""
        raise AsyncDesign(msg)

    def _fold(self):
        """With lsig, the exploration went by register values and latch
        levels together.  From here on a state is the register values
        again: the first state found with them stands for it (its path,
        table and sweep), and the sweeps of the others are kept as what
        its clock edges do with the latches at other levels (cvar)."""
        n = len(self.cand)
        if not any(len(c) > n for c in self.states):
            return
        first = {}
        for c in self.states:
            first.setdefault(c[:n], c)
        states, trans, pre, cvar = {}, {}, {}, {}
        for c, (path, t) in self.states.items():
            r, tr = c[:n], {}
            for c2, xs in self.trans[c].items():
                tr[c2[:n]] = tr.get(c2[:n], 0) | xs
            if first[r] == c:
                states[r], trans[r] = (path, t), tr
                if c in self.pre:
                    pre[r] = self.pre[c]
            else:
                cvar.setdefault(r, []).append((path, t, tr, self.pre.get(c)))
        self.log("%d states by registers and latch levels (%s), %d by the registers alone" % (
            len(self.states), _pins(self.lsig), len(states)))
        self.states, self.trans, self.pre, self.cvar, self.cmap = states, trans, pre, cvar, first
        self.power_up_state = self.power_up_state[:n]

    def run(self):
        """discover + explore + assign + equations; treats pin 1 as an input
        when clocking never changes anything."""
        self.discover()
        self.explore()
        if (not self.reg and len(self.states) == 1 and not self.clock_in_v
                and CLOCK not in self.fixed):
            self._unclocked()
        while self._let_go_inputs():
            self.discover()
            self.explore()
        self.assign()
        self.asynchronous()
        return self.equations()

    def _let_go_inputs(self):
        """Outputs with a constant value and an enable (X = 'b'0; X.oe = ...)
        looked like state candidates, so they were never enumerated: where
        the GAL lets one go, it sat at its base level, and in every second
        read at the other level together with all the others.  If another
        pin, an enable or a register input reads such a pin, it is an
        input too, and what is learned is wrong where it is let go at the
        other level.

        To find out, every state is walked (and swept, if there are
        registers) with all of them held at the level they do not drive,
        so that each shows where it is driven; then, for each, once more
        with that one at the level it drives.  One whose level makes a
        difference to any other pin or to a transition is enumerated from
        the next round on (True: discover and explore again)."""
        held = {}
        for r in self.cand:
            if r not in self.comb or r in self.late_comb or r in self.hist or r in self.V:
                continue
            tables = [t for _, t in self.states.values()]
            if not any(t.care[r] ^ t.driven[r] for t in tables):
                continue                              # always driven
            vals = {v for t in tables for v in (0, 1)
                    if (t.value[r] if v else ~t.value[r]) & t.driven[r]}
            if len(vals) == 1:
                held[r] = 1 - vals.pop()
        if not held:
            return False
        base = dict(self.base)
        base.update(held)

        def look(b, s, path, t):
            kw = dict(skip=self.ar if s in self.ar_on else None,
                      avoid=self.ar if path else None)
            kw.update(self._wctx())
            la, lb, _ = self.rig.walk(self.V, b, self.drives(path), **kw)
            tr = None
            if self.reg and self.probe_x:
                obs = self.rig.transitions(self.V, b, self.drives(path), self.probes(),
                                           self.cand, avoid=self.ar, **self._tctx())[0]
                tr = {}
                for o, xs in obs.items():
                    s2 = self._reg(self._sig_obs(o))
                    tr[s2] = tr.get(s2, 0) | xs
            return la, lb, tr
        found = set()
        for s, (path, t) in self.states.items():
            la0, lb0, tr0 = look(base, s, path, t)
            for q in sorted(held):
                if q in found:
                    continue
                b = dict(base)
                b[q] = 1 - held[q]
                la, lb, tr = look(b, s, path, t)
                if tr != tr0 or any(((la[p] ^ la0[p]) | (lb[p] ^ lb0[p])) & t.walked
                                    for p in la if p not in (q, CLOCK) and p not in self.hist):
                    found.add(q)
        if not found:
            return False
        for q in found:
            self.late_comb[q] = True
        self.log("%s %s an enable, and other logic reads %s where the GAL lets go: "
                 "enumerating %s too, and exploring again" % (
                     _pins(sorted(found)), "has" if len(found) == 1 else "have",
                     "it" if len(found) == 1 else "them", "it" if len(found) == 1 else "them"))
        return True

    def _two_ways(self, msg):
        """The survey and the full sweep of a state disagree."""
        raise AsyncDesign(msg)

    def _unclocked(self):
        self.log("clocking never changes the state: combinational design, "
                 "enumerating pin 1 as an input")
        self.clock_in_v = True
        self.discover()
        self.explore()

    def asynchronous(self):
        """What the exploration left out (reglatch.RegLatchAnalysis: the
        pins that change without a clock)."""

    # 3. would-be register values per state ---------------------------------
    def assign(self):
        regs = sorted(self.reg)
        idx = [self.cand.index(r) for r in regs]
        seen = {r: set() for r in regs}
        for s in self.states:
            for r, i in zip(regs, idx):
                if s[i] in (0, 1):
                    seen[r].add(s[i])
        self.W = {}
        free = {}                       # state -> registers whose value nothing shows
        for s in self.states:
            w = []
            for k, (r, i) in enumerate(zip(regs, idx)):
                v = s[i]
                if v in (0, 1):
                    w.append(v)
                elif len(seen[r]) == 1:
                    w.append(1 - next(iter(seen[r])))
                else:
                    w.append(None)
                    free.setdefault(s, []).append(k)
            self.W[s] = tuple(w)
        if free:
            self._assign_hidden(regs, free)
        if len(set(self.W.values())) != len(self.W):
            raise Limit("two states show the same register values: the chip has state that "
                        "no pin shows")

    def _assign_hidden(self, regs, free):
        """A register that shows both values elsewhere does not reach its
        pin in some state (its enable is off whatever the inputs).  The
        exploration found nothing there that depends on it: the same
        clocks always led to the same states.  So any value will do, as
        long as the register input loads that value on the way in, which
        the equations then say.  Take the value it has when cleared
        (after power-up), or the other if a state with that value and the
        same other registers exists."""
        s0 = self.power_up_state
        pv = [0 if self.W[s0][k] is None else self.W[s0][k] for k in range(len(regs))]
        taken = {w for w in self.W.values() if None not in w}
        for s, ks in free.items():
            for m in range(1 << len(ks)):
                w = list(self.W[s])
                for t, k in enumerate(ks):
                    w[k] = pv[k] ^ (m >> t & 1)
                if tuple(w) not in taken:
                    break
            else:
                raise Limit("the register%s on %s do%s not show in state %s, and every value "
                            "is another state's" % (
                                "s" if len(ks) > 1 else "", _pins([regs[k] for k in ks]),
                                "" if len(ks) > 1 else "es", _state(s)))
            self.W[s] = tuple(w)
            taken.add(tuple(w))
        hid = sorted({regs[k] for ks in free.values() for k in ks})
        self.log("the register%s on %s do%s not show in %d states (output not enabled there, "
                 "whatever the inputs), and nothing seen depends on %s there: taken as %s" % (
                     "s" if len(hid) > 1 else "", _pins(hid), "" if len(hid) > 1 else "es",
                     len(free), "them" if len(hid) > 1 else "it",
                     "cleared where no other state has those values"))

    # 4. equations -------------------------------------------------------------
    def functions(self):
        """(pin, kind) -> per state (on, off) bitsets over x."""
        F = {}
        regs = sorted(self.reg)
        for p in self.comb:
            val, oe = {}, {}
            for s, (_, t) in self.states.items():
                d = t.driven[p]
                val[s] = (t.value[p] & d, (self.full ^ t.value[p]) & d)
                oe[s] = (d, t.care[p] ^ d)
            F[(p, "")] = val
            if any(o[1] for o in oe.values()):
                F[(p, "oe")] = oe
        for i, r in enumerate(regs):
            F[(r, "d")] = self._dfunction(i)
            if self.device == "GAL22V10":
                oe = {}
                for s, (_, t) in self.states.items():
                    d = t.driven[r]
                    oe[s] = (d, t.care[r] ^ d)
                if any(o[1] for o in oe.values()):
                    F[(r, "oe")] = oe
        return F

    def _dfunction(self, i):
        """Per state, the (on, off) sets of the i-th register's input."""
        dd = {}
        for s, m in self.trans.items():
            on = off = 0
            for s2, xs in m.items():
                if self.W[s2][i]:
                    on |= xs
                else:
                    off |= xs
            if s in self.ar_dc:
                # the reset clears the registers first: the edge loads
                # from the power-up state, whatever the state's inputs
                on &= ~self.ar_set
                off &= ~self.ar_set
            dd[s] = (on, off)
        return dd

    def _eq_space(self):
        """What the equations are written over: the pins EV, with their
        bitset masks; and the variables that are constant in a context
        (cvars, the registered pins; CW gives a context's values, and a
        context is a state)."""
        self.EV, self.efull, self.ev1 = self.V, self.full, self.v1
        self.cvars, self.CW = sorted(self.reg), self.W

    def _support(self, f):
        """Variables (index into EV, then cvars) f depends on."""
        n = len(self.EV)
        sup = set()
        for s, (on, off) in f.items():
            for b in range(n):
                if b not in sup and (on & swap(off, b, self.ev1)) & self.efull:
                    sup.add(b)
        sl = list(f)
        for i in range(len(self.cvars)):
            for a in sl:
                for b in sl:
                    wa, wb = self.CW[a], self.CW[b]
                    if sum(x != y for x, y in zip(wa, wb)) == 1 and wa[i] != wb[i] and wa[i] == 0:
                        if (f[a][0] & f[b][1]) or (f[a][1] & f[b][0]):
                            sup.add(n + i)
        return sorted(sup)

    def _project(self, f, sup):
        """on/off minterm bitsets over the support, by splitting the x
        bitsets on each support variable."""
        n = len(self.EV)
        vpos = [(j, v) for j, v in enumerate(sup) if v < n]
        on = off = 0
        for s, (fo, ff) in f.items():
            w = self.CW[s]
            wbits = 0
            for j, v in enumerate(sup):
                if v >= n and w[v - n]:
                    wbits |= 1 << j
            for which, bs in ((0, fo), (1, ff)):
                groups = [(wbits, bs)] if bs else []
                for j, v in vpos:
                    ng = []
                    for m, g in groups:
                        a, b = g & ~self.ev1[v], g & self.ev1[v]
                        if a:
                            ng.append((m, a))
                        if b:
                            ng.append((m | 1 << j, b))
                    groups = ng
                acc = 0
                for m, _ in groups:
                    acc |= 1 << m
                if which == 0:
                    on |= acc
                else:
                    off |= acc
        return on, off

    def name(self, p):
        return "P%d" % p

    def equations(self):
        """[(pin, kind, inverted, expression, cubes)]"""
        self.ar_dc = set()          # states whose next state the reset term decides in C
        if self.ar:
            self._ar_term()
            s0 = self.power_up_state
            self.ar_dc = set(self.ar_on)
            if all(self.W[s0][sorted(self.reg).index(r)] == v for r, v in self.ar_regs.items()):
                self.ar_dc.add(s0)
        self._eq_space()
        F = self.functions()
        regs = sorted(self.reg)
        names = [self.name(p) for p in self.EV] + [self.name(r) for r in self.cvars]
        self.F, self.cover, eqs = F, {}, []
        self.nterms = {}            # (pin, kind) -> (product terms, the macrocell's)
        for (p, kind), f in F.items():
            sup = self._support(f)
            while True:
                on, off = self._project(f, sup)
                if not on & off:
                    break
                extra = [i for i in range(len(self.EV) + len(self.cvars)) if i not in sup]
                if not extra:
                    raise Limit("pin %d %s is not a function of the inputs and "
                                "registers" % (p, kind))
                sup = sorted(sup + [extra[-1]])
            k = len(sup)
            pos = sop(on, off, k)
            neg = sop(off, on, k)
            if self.mixed:
                # known only where the walks went: of the covers that agree
                # with it, a smaller one if this one does not fit
                limit = self._budget(p, kind)
                order = self._drop_first(p, kind, sup)
                if len(pos) > limit:
                    pos = min(pos, sop(on, off, k, order), key=len)
                    if len(pos) > limit:
                        pos = fewer(pos, on, off, k)
                if len(neg) > limit:
                    neg = min(neg, sop(off, on, k, order), key=len)
                    if len(neg) > limit:
                        neg = fewer(neg, off, on, k)
                # ... and claim no more than that
                own = self._own(p, kind, sup)
                pos, neg = tight(pos, on, k, own), tight(neg, off, k, own)
                self.nterms[(p, kind)] = (min(len(pos), len(neg)), limit)
            pos, neg = self._hazards(p, kind, sup, pos, neg, on, off)
            if kind == "d" and self.device == "GAL22V10":
                # a cleared register (power-up, reset term) shows the output
                # polarity on the pin: CUPL makes a negated .d active low
                inv = bool(self.W[self.power_up_state][regs.index(p)])
            else:
                inv = len(neg) < len(pos)
            cubes = neg if inv else pos
            self.cover[(p, kind)] = (sup, cubes, inv)

            def lit(c):
                if c[0] == 0:
                    return "'b'1"
                return " & ".join(("" if c[1] >> j & 1 else "!") + names[sup[j]]
                                  for j in range(k) if c[0] >> j & 1)
            expr = "\n    # ".join(lit(c) for c in cubes) if cubes else "'b'0"
            if kind == "oe" and len(cubes) > 1 and not (
                    inv and all(c[0] & (c[0] - 1) == 0 for c in cubes)):
                # an enable is one product term: over the inputs it is not,
                # so it reads another output's pin
                term = self._one_term(p, f)
                if term:
                    eqs.append((p, kind, False, term, 1))
                    continue
            eqs.append((p, kind, inv, expr, len(cubes)))
        self.scover = {}
        if not self.mixed:
            eqs = self._read_outputs(F, eqs)
        if self.dev.get("oe") and regs and not self.clock_in_v:
            # A registered GAL16V8's pin 11 enables its registers and is no
            # array input.  An output that is enabled by it and never
            # changes is a register that always loads the same value.
            n11 = "!" + self.name(self.dev["oe"])
            fixed = {p for p, kind, inv, expr, _ in eqs
                     if kind == "oe" and not inv and expr == n11}
            fixed = {p for p, kind, _, expr, _ in eqs
                     if p in fixed and kind == "" and expr.startswith("'b'")}
            eqs = [(p, "d" if p in fixed and kind == "" else kind, inv, expr, n)
                   for p, kind, inv, expr, n in eqs if not (p in fixed and kind == "oe")]
        self.eqs = sorted(eqs)
        return self.eqs

    # product terms of a GAL22V10's macrocells, pins 23 down to 14
    TERMS22 = (8, 10, 12, 14, 16, 16, 14, 12, 10, 8)

    def _drop_first(self, p, kind, sup):
        """The order in which a cube of pin p's cover drops variables that
        take in equally much, for a second cover when the first does not
        fit (None: as they come)."""
        return None

    def _budget(self, p, kind):
        """The product terms a function of pin p may have.  A GAL16V8 or
        GAL20V8 has eight per macrocell, of which the enable takes one
        unless the output is registered."""
        if self.device == "GAL22V10":
            return self.TERMS22[23 - p]
        return 8 if kind == "d" else 7

    def _read_outputs(self, F, eqs):
        """The functions that need more product terms over the inputs and
        registers than their macrocell has, written again over the inputs,
        the registers and the pins of other outputs: the original read
        another output there (!P15 = !P21 & ..., with pin 21 a sum of four
        terms itself).  Only outputs that are always driven and written
        over the inputs themselves are read, so nothing reads in a loop."""
        over = [(p, kind) for p, kind, _, _, n in eqs
                if kind in ("", "d") and n > self._budget(p, kind)]
        if not over:
            return eqs
        fed = self.dev["io"] if self.reg or "fed" not in self.dev else self.dev["fed"]
        ys = [y for y in self.comb if y in fed and y not in self.EV
              and (y, "") in F and (y, "oe") not in F and (y, "") not in over]
        lits = []                           # (text, {context: the x where it is true})
        ctxs = list(self.CW)
        for b, q in enumerate(self.EV):
            m = self.ev1[b]
            lits.append((self.name(q), {c: m for c in ctxs}))
            lits.append(("!" + self.name(q), {c: self.efull ^ m for c in ctxs}))
        for i, q in enumerate(self.cvars):
            lits.append((self.name(q), {c: self.efull if self.CW[c][i] else 0 for c in ctxs}))
            lits.append(("!" + self.name(q), {c: 0 if self.CW[c][i] else self.efull for c in ctxs}))
        for y in ys:
            lv = self._levels(y)
            lits.append((self.name(y), {c: lv[c][0] for c in ctxs if c in lv}))
            lits.append(("!" + self.name(y), {c: lv[c][1] for c in ctxs if c in lv}))
        out = []
        for p, kind, inv, expr, n in eqs:
            if (p, kind) in over:
                f = F[(p, kind)]
                on = {c: v[0] for c, v in f.items()}
                off = {c: v[1] for c, v in f.items()}
                pos, neg = self._sop_lits(on, off, lits), self._sop_lits(off, on, lits)
                if kind == "d" and self.device == "GAL22V10":
                    cubes, inv2 = (neg, True) if inv else (pos, False)
                else:
                    cubes, inv2 = (neg, True) if len(neg) < len(pos) else (pos, False)
                if len(cubes) < n:
                    read = sorted({y for y in ys for c in cubes for i in c
                                   if lits[i][0].lstrip("!") == self.name(y)})
                    self.log("pin %d%s needs %d product terms over the inputs, more than "
                             "its %d: %d with the pin%s of %s" % (
                                 p, " ." + kind if kind else "", n, self._budget(p, kind),
                                 len(cubes), "s" if len(read) > 1 else "",
                                 ", ".join(map(str, read)) or "no other output"))
                    self.scover[(p, kind)] = (cubes, inv2, lits)
                    expr = "\n    # ".join(" & ".join(lits[i][0] for i in c) or "'b'1"
                                           for c in cubes)
                    inv, n = inv2, len(cubes)
            out.append((p, kind, inv, expr, n))
        return out

    def _sop_lits(self, on, off, lits):
        return sop_lits(on, off, lits, self.efull)

    def _levels(self, y):
        """Where output y is known to be high and low, per context:
        {context: (high, low)} over the x of the equations."""
        return {s: (t.value[y] & t.driven[y], t.driven[y] & ~t.value[y])
                for s, (_, t) in self.states.items()}

    def _one_term(self, p, f):
        """Function f of pin p (per context, on and off sets) as one product
        term over the inputs, the registers and the levels of the other
        outputs, or None: the literals that hold wherever f is on, less
        those not needed to keep it off elsewhere."""
        ctxs = [c for c in f if f[c][0] or f[c][1]]
        lits = []                       # (text, {context: the x at which it is true})
        for b, q in enumerate(self.EV):
            if q == p:
                continue
            for val in (1, 0):
                m = self.ev1[b] if val else self.efull ^ self.ev1[b]
                if all(not f[c][0] & ~m for c in ctxs):
                    lits.append((("" if val else "!") + self.name(q), {c: m for c in ctxs}))
        for i, q in enumerate(self.cvars):
            vals = {self.CW[c][i] for c in ctxs if f[c][0]}
            if len(vals) == 1 and q != p:
                v = vals.pop()
                lits.append((("" if v else "!") + self.name(q),
                             {c: self.efull if self.CW[c][i] == v else 0 for c in ctxs}))
        outs = [y for y in self.comb if y != p and y not in self.EV and y not in self.cvars]
        for y in outs:
            lv = self._levels(y)
            if not all(c in lv for c in ctxs):
                continue
            for val in (1, 0):
                if all(not f[c][0] & ~lv[c][1 - val] for c in ctxs):
                    # true where the level is not known to be the other one
                    lits.append((("" if val else "!") + self.name(y),
                                 {c: self.efull ^ lv[c][val] for c in ctxs}))

        def off(ls):
            for c in ctxs:
                bits = f[c][1]
                for _, m in ls:
                    bits &= m[c]
                    if not bits:
                        break
                if bits:
                    return True
            return False
        if not lits or off(lits):
            return None
        for item in list(lits):
            rest = [x for x in lits if x is not item]
            if rest and not off(rest):
                lits = rest
        return " & ".join(t for t, _ in lits)

    def _latch_names(self):
        """The outputs CUPL must not minimise (reglatch: the latches)."""
        return []

    def _own(self, p, kind, sup):
        """Where in sup the function's own pin is (reglatch: a latch reads
        itself), or None."""
        return None

    def _hazards(self, p, kind, sup, pos, neg, on, off):
        """The two covers of a function, with whatever its kind needs
        beyond covering (reglatch: a latch must not glitch)."""
        return pos, neg

    def _ar_term(self):
        """The register literals of the asynchronous reset term: true in the
        states in which it fires, false in the others.  In the power-up
        state, which it cannot change, it must be false if clocking in one
        of its x leaves that state."""
        regs = sorted(self.reg)
        s0 = self.power_up_state
        on = [self.W[s] for s in self.ar_on]
        if not on:
            raise AsyncDesign("the asynchronous reset term fired once, but in no state "
                              "explored since")
        off = [self.W[s] for s in self.states if s not in self.ar_on and s != s0]
        if any(xs & self.ar_set for s2, xs in self.trans.get(s0, {}).items() if s2 != s0):
            off.append(self.W[s0])

        def covers(lits, w):
            return all(w[i] == v for i, v in lits.items())
        lits = {i: on[0][i] for i in range(len(regs)) if all(w[i] == on[0][i] for w in on)}
        if any(covers(lits, w) for w in off):
            raise AsyncDesign("the asynchronous reset term fires in states %s, but not in %s: "
                              "not one product term of the inputs and registers" % (
                                  ", ".join(_state(s) for s in sorted(self.ar_on, key=str)),
                                  ", ".join(_state(s) for s in self.states
                                            if s not in self.ar_on and s != s0)))
        for i in sorted(lits):
            fewer = {j: v for j, v in lits.items() if j != i}
            if not any(covers(fewer, w) for w in off):
                lits = fewer
        self.ar_regs = {regs[i]: v for i, v in lits.items()}

    def ar_expr(self):
        """The asynchronous reset term in CUPL, or None."""
        if not self.ar:
            return None
        c = dict(self.ar)
        c.update(self.ar_regs)
        return _cube(c, self.name)

    def check_equations(self):
        """Every function's cover against its captured on/off sets; a list
        of (pin, kind, state) that disagree (empty when all hold)."""
        bad = []
        for key, f in self.F.items():
            for s, (on, off) in f.items():
                cov = self._scov(key, s) if key in self.scover else self._cov(key, self.CW[s])
                if (on & ~cov) or (off & cov):
                    bad.append(key + (s,))
        return bad

    def _scov(self, key, s):
        """The x for which function key's cover over the inputs and other
        outputs' pins (_read_outputs) is true in context s."""
        cubes, inv, lits = self.scover[key]
        cov = 0
        for cube in cubes:
            v = self.efull
            for i in cube:
                v &= lits[i][1].get(s, 0)
            cov |= v
        return self.efull ^ cov if inv else cov

    def _cov(self, key, w):
        """The x for which function key's cover is true, in a context
        whose cvars have the values w."""
        sup, cubes, inv = self.cover[key]
        n = len(self.EV)
        cov = 0
        for care, val in cubes:
            c = self.efull
            for j, v in enumerate(sup):
                if not care >> j & 1:
                    continue
                want = val >> j & 1
                if v < n:
                    c &= self.ev1[v] if want else self.efull ^ self.ev1[v]
                elif w[v - n] != want:
                    c = 0
                    break
            cov |= c
        return self.efull ^ cov if inv else cov

    def cupl(self, title="galre"):
        regs = sorted(self.reg)
        clocked = bool(regs) and not self.clock_in_v
        lines = ["Name %s; Partno x; Date x; Revision 1; Designer galre; Company x; "
                 "Assembly x; Location x;" % title,
                 "Device %s;" % self.dev["cupl"], "", HOW % title, ""]
        if clocked:
            lines.append("Pin 1 = CLK;")
        for p in sorted(set(self.EV) | set(self.reg) | set(self.comb)):
            if p == CLOCK and clocked:
                continue
            if clocked and p == self.dev.get("oe"):
                lines.append("Pin %d = !OE;" % p)
                continue
            lines.append("Pin %d = %s;" % (p, self.name(p)))
        lines.append("")
        latches = self._latch_names()
        if latches:
            lines += [KEEP, "MIN [%s] = 0;" % ", ".join(latches), ""]
        for note in self.notes:
            text = textwrap.wrap(note, 72)
            lines += ["/* " + text[0]] + ["   " + t for t in text[1:]]
            lines[-1] += " */"
            lines.append("")
        for p, kind, inv, expr, n in self.eqs:
            if self.dev.get("oe") and clocked and p in regs and kind == "oe":
                continue
            lhs = self.name(p) + ("." + kind if kind else "")
            if kind == "oe" and inv:
                lines.append("%s = !(%s);" % (lhs, expr))
            else:
                lines.append("%s%s = %s;" % ("!" if inv else "", lhs, expr))
        if self.ar:
            # one product term in the GAL22V10, shared by every register
            for r in regs:
                lines.append("%s.ar = %s;" % (self.name(r), self.ar_expr()))
        return "\n".join(lines) + "\n"
