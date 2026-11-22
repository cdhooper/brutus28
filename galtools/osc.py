"""Outputs that do not settle: an output that feeds back inverted.

An output that feeds back inverted has no level while its loop is closed:
with P17 = !P1 & !P5 # !P17 & !P5, pin 17 is its own complement while
pin 1 is high and pin 5 low, and toggles as fast as the chip lets it.  A
read then catches an arbitrary level, and the same walk reads differently
twice.  That is all analysis.py sees, and it stops (Unsteady).

But where a pin does not settle is itself what there is to know.  Read
often enough, every vector tells for every output whether it settles
there, and at which level.  Then:

- A pin that does not settle at x can only be the complement of itself
  there (a loop pin): its equation is 1 at (x, pin low) and 0 at (x, pin
  high).  Where it settles, the equation is its level.
- Another output that is unsteady only where a loop pin is, is a
  function of that pin there: it is taken to follow it.  Following its
  complement explains the same reads (the phase is not seen); of the
  two, the one with fewer product terms and literals is written, and the
  choice is noted.
- An output that settles where a loop pin does not is the same at both
  levels of that pin.

Every vector is read REPEAT times in each of a few walk orders (the same
job again: a recording answers it in the order recorded).  An oscillator
fails to settle at the same inputs every time; a pin that is unsteady at
other vectors in another order is a bad contact or a floating pin, and
galre says so.

A pin that toggles is not at both levels equally often.  On a GAL16V8
(osc16v8, 2026-11) the loop pin read high in 45 of 100 reads, the output
of the gate behind it in 19.  REPEAT reads of a vector are then all the
same now and then (there in one walk order of nine), and the vector
looks settled in that order.  So after the first orders galre
counts how often that happened and reads as many more as it takes for a
vector to be unlikely to look settled in all of them (_orders_needed).
A pin that looks settled in a walk order at more than half of the
vectors at which it is unsteady in another cannot be told from a bad
contact, and is refused.

Limits: one loop pin per set of vectors (no ring through several pins);
no other state (where the pins settle, their levels must be the same in
every walk order); every output always driven.  The frequency is the
chip's own.  A loop of one gate may toggle too fast for a pin behind it
to swing at all; that pin then reads as a level, and nothing shows it.
"""

import math
import textwrap

from analysis import HOW, AsyncDesign, sop, _pins
from bitsim import litmasks
from latch import LatchAnalysis, _Walk
from rig import popcount


class OscAnalysis(LatchAnalysis):
    REPEAT = 12        # reads of every vector per walk order
    ORDERS = [(0, 0, 0, 0), (0, 1, 0, 0), (0, 0, 0, 1), (0, 0, 0, 2)]
    MAX_ORDERS = 32    # walk orders at most
    MISSED = 0.01      # the chance taken that some vector at which a pin does not settle
                       # reads the same in every walk order

    def _order(self, i):
        """The i-th walk order: ORDERS, then further mixed Gray codes."""
        return self.ORDERS[i] if i < len(self.ORDERS) else (0, i & 1, 0, i)

    def _orders_needed(self, un, per, log):
        """How many walk orders to read, after those in per (per pin, the
        vectors unsteady in each).  A pin that toggles can be at one of
        its levels so seldom that REPEAT reads of a vector are all the
        same now and then.  From the share of its unsteady vectors that
        looked settled in the worst order, r: after k orders a vector
        has looked settled in all of them with a chance of r^k.  A share
        over a half is not an oscillator that reads can pin down, and is
        what a bad contact looks like: each order then shows other
        vectors."""
        need = len(next(iter(per.values())))
        for p in self.outputs:
            total = popcount(un[p])
            if not total:
                continue
            miss, d = max((popcount(un[p] & ~d), d) for d in per[p])
            if miss > 2 + total // 2:
                x = ((un[p] & ~d) & -(un[p] & ~d)).bit_length() - 1
                raise AsyncDesign(
                    "pin %d reads unsteadily at %d input vectors, and settles at %d of them "
                    "in another walk order (the first: %s): a bad contact or a floating pin "
                    "rather than an oscillating output" % (p, total, miss, self._inputs(x)))
            if not miss:
                continue
            k = math.ceil(math.log(total / self.MISSED) / -math.log(miss / total))
            if k > need:
                log("pin %d looks settled in one walk order at %d of the %d input vectors at "
                    "which it does not settle (it is seldom at one of its levels): reading "
                    "%d walk orders, not %d" % (p, miss, total, min(k, self.MAX_ORDERS), need))
                need = k
        return min(need, self.MAX_ORDERS)

    def _steadiness(self, rig, log=lambda msg: None):
        """Per output: (settles high, settles low, does not settle), as
        bitsets over the input vectors, from every walk order read
        REPEAT times; and the walk orders read."""
        N = (1 << (1 << self.n)) - 1
        hi = {p: 0 for p in self.outputs}
        lo = {p: 0 for p in self.outputs}
        un = {p: 0 for p in self.outputs}
        per = {p: [] for p in self.outputs}
        orders = len(self.ORDERS)
        while len(per[self.outputs[0]]) < orders:
            rot, rev, start, mix = self._order(len(per[self.outputs[0]]))
            first, differ = None, {p: 0 for p in self.outputs}
            for _ in range(self.REPEAT):
                la, _, _ = rig.walk(self.V, self.wbase, [], gray=1, rot=rot, rev=rev,
                                    start=start, mix=mix)
                if first is None:
                    first = la
                for p in self.outputs:
                    differ[p] |= la[p] ^ first[p]
            for p in self.outputs:
                per[p].append(differ[p])
                un[p] |= differ[p]
                hi[p] |= first[p] & ~differ[p] & N
                lo[p] |= ~first[p] & ~differ[p] & N
            if len(per[self.outputs[0]]) == orders:
                # An oscillator fails to settle at the same inputs every time
                orders = self._orders_needed(un, per, log)
        for p in self.outputs:
            hi[p] &= ~un[p]
            lo[p] &= ~un[p]
            both = hi[p] & lo[p]
            if both:
                x = (both & -both).bit_length() - 1
                raise AsyncDesign(
                    "pin %d settles at %s, but high in one walk order and low in another: "
                    "the chip holds state as well as oscillating, which galre does not "
                    "model" % (p, self._inputs(x)))
        # with the outputs driven the other way: one that settles reads the same
        base = dict(self.wbase)
        base.update({p: 1 for p in self.outputs})
        la, _, _ = rig.walk(self.V, base, [], gray=1)
        tri = [p for p in self.outputs if (la[p] ^ hi[p]) & (hi[p] | lo[p])]
        if tri:
            raise AsyncDesign("outputs %s are not always driven, which the model of outputs "
                              "that do not settle does not cover" % tri)
        return hi, lo, un, [self._order(i) for i in range(orders)]

    def run(self):
        self.roles()
        self.hi, self.lo, self.un, orders = self._steadiness(self.rig, self.log)
        hi, lo, un = self.hi, self.lo, self.un
        self.walks = [_Walk(c, [], {}) for c in orders]
        n = self.n
        unsteady = [p for p in self.outputs if un[p]]
        if not unsteady:
            raise AsyncDesign("a walk read differently twice, but walked one input at a time "
                              "every pin settles: several inputs changing at once matter")
        # the loop pins: unsteady at more vectors than any pin that is unsteady with them
        loops = []
        for p in unsteady:
            if any(un[q] != un[p] and un[q] & un[p] == un[p] for q in unsteady):
                continue
            if any(un[q] == un[p] for q in loops):
                continue                     # a copy of one: the lower pin is taken
            loops.append(p)
        for a in loops:
            for b in loops:
                if a < b and un[a] & un[b]:
                    raise AsyncDesign("pins %d and %d both fail to settle at some of the same "
                                      "inputs, neither everywhere the other does: a ring "
                                      "through several pins, which galre does not model" % (a, b))
        self.L = loops
        m = len(loops)
        k = n + m
        full, v1 = litmasks(k)

        def spread(bits):
            out = 0
            for lv in range(1 << m):
                out |= bits << (lv << n)
            return out

        level = [v1[n + j] for j in range(m)]
        # where each loop pin can be: at its level where it settles, at both where not
        can = [spread(hi[p]) & level[j] | spread(lo[p]) & (full ^ level[j]) | spread(un[p])
               for j, p in enumerate(loops)]
        rest = full
        for c in can:
            rest &= c
        self.follows, self.cover, self.rcover = {}, {}, {}
        notes = []

        def best(on, off):
            pos, neg = sop(on, off, k), sop(off, on, k)
            return (neg, True) if len(neg) < len(pos) else (pos, False)

        def size(cov):
            return (len(cov[0]), sum(popcount(c) for c, _ in cov[0]))

        for p in self.outputs:
            if p in loops:
                j = loops.index(p)
                others = full
                for i, c in enumerate(can):
                    if i != j:
                        others &= c
                # its own complement where it does not settle; its level where it does,
                # from either level (it comes there from an arbitrary one)
                on = (spread(un[p]) & (full ^ level[j]) | spread(hi[p])) & others
                off = (spread(un[p]) & level[j] | spread(lo[p])) & others
                self.cover[p] = best(on, off)
                continue
            lead = [j for j, q in enumerate(loops) if un[p] & un[q]]
            if len(lead) > 1 or (lead and un[p] & ~un[loops[lead[0]]]) or (un[p] and not lead):
                raise AsyncDesign("pin %d does not settle at inputs at which no single pin "
                                  "that feeds back does" % p)
            on, off = spread(hi[p]) & rest, spread(lo[p]) & rest
            if not lead:
                self.cover[p] = best(on, off)
                continue
            j = lead[0]
            u = spread(un[p]) & rest
            with_it = best(on | u & level[j], off | u & (full ^ level[j]))
            against = best(on | u & (full ^ level[j]), off | u & level[j])
            self.follows[p] = loops[j]
            if size(against) < size(with_it):
                self.cover[p] = against
                notes.append("%s is taken to follow the complement of %s where that does not "
                             "settle; following %s itself explains the reads as well (the "
                             "phase is not seen)." % (self.name(p), self.name(loops[j]),
                                                      self.name(loops[j])))
            else:
                self.cover[p] = with_it
                notes.append("%s is taken to follow %s where that does not settle; following "
                             "its complement explains the reads as well (the phase is not "
                             "seen)." % (self.name(p), self.name(loops[j])))
        for p in loops:
            others = [q for q in unsteady if q != p and un[q] & un[p]]
            self.log("pin %d does not settle at %d of %d input vectors%s: its own complement "
                     "there, an oscillator" % (
                         p, popcount(un[p]), 1 << n,
                         "" if not others else ", and %s with it at some" % _pins(others)))
        self.notes = [
            "%s %s not settle at some inputs: there the equation is the pin's own "
            "complement, and the pin toggles as fast as the chip lets it.  This was "
            "inferred from where the pins settle and where they do not, not read; the "
            "frequency is the chip's own." % (
                ", ".join(self.name(p) for p in loops), "does" if m == 1 else "do")] + notes
        for note in notes:
            self.log(note)
        return self.equations()

    def check_equations(self):
        """The covers against what was seen: [(pin, x)] where they differ."""
        n, m = self.n, len(self.L)
        bad = []
        N = (1 << (1 << n)) - 1
        for p, (cubes, inv) in self.cover.items():
            per = []
            for lv in range(1 << m):
                bits = 0
                for care, val in cubes:
                    if (care >> n) & ~(lv ^ (val >> n)) != care >> n:
                        continue
                    b = N
                    for i in range(n):
                        if care >> i & 1:
                            w = 0
                            for x in range(1 << n):
                                if (x >> i & 1) == (val >> i & 1):
                                    w |= 1 << x
                            b &= w
                    bits |= b
                per.append(bits ^ N if inv else bits)
            if p in self.L:
                j = self.L.index(p)
                for lv in range(1 << m):
                    own = lv >> j & 1
                    want_hi = self.hi[p] | (self.un[p] if not own else 0)
                    want_lo = self.lo[p] | (self.un[p] if own else 0)
                    if per[lv] & want_lo or (per[lv] ^ N) & want_hi:
                        bad.append((p, lv))
        return bad

    def check_jed(self, jed, notes=None):
        """The rebuilt chip through the same reads: every output must
        settle at the same vectors, at the same levels."""
        from rig import SimRig
        try:
            hi, lo, un, _ = self._steadiness(SimRig(jed))
        except AsyncDesign as e:
            return [("the rebuilt chip", str(e))]
        bad = []
        for p in self.outputs:
            d = (hi[p] ^ self.hi[p]) | (lo[p] ^ self.lo[p]) | (un[p] ^ self.un[p])
            if d:
                x = (d & -d).bit_length() - 1
                bad.append(("pin %d" % p, "%d vectors, the first %s" % (popcount(d),
                                                                      self._inputs(x))))
        return bad

    def cupl(self, title="galre"):
        lines = ["Name %s; Partno x; Date x; Revision 1; Designer galre; Company x; "
                 "Assembly x; Location x;" % title,
                 "Device %s;" % self.dev["cupl"], "", HOW % title, ""]
        for p in sorted(set(self.V) | set(self.outputs)):
            lines.append("Pin %d = %s;" % (p, self.name(p)))
        lines += ["", "MIN [%s] = 0;" % ", ".join(self.name(p) for p in self.L), ""]
        for note in self.notes:
            text = textwrap.wrap(note, 72)
            lines += ["/* " + text[0]] + ["   " + t for t in text[1:]]
            lines[-1] += " */"
            lines.append("")
        for p, _, inv, expr, _ in self.eqs:
            lines.append("%s%s = %s;" % ("!" if inv else "", self.name(p), expr))
        return "\n".join(lines) + "\n"
