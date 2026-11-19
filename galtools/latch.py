"""Latches built from combinational feedback (GAL16V8, GAL22V10).

A combinational output fed back into the array can hold a value: its level
depends on the history of the inputs, not only on their present value.
The analysis in analysis.py sees this as a power-up walk that reads
differently in another order (LatchSuspected), and hands over to
LatchAnalysis, which models a design without registers:

- The outputs whose level depends on the history (two walks read them
  differently at the same x) are the latch pins L, less those that read
  as a function of x and the others: an output of the latches, such as
  Y & !Z, or the second output of a cross-coupled pair.  Every output is
  a function of the inputs x and the latch levels.  For a latch pin j,
  g_j(x, L) is its excitation: the chip rests where g(x, L) = L, and
  moves otherwise.
- The inputs are walked in Gray code order from power-up, so one pin
  changes per write and no momentary mix of old and new values (the
  firmware's port writes) reaches the chip, in several bit orders (rot,
  rev), so that every pin gets to change often and in many latch states.
  Each step from a resting (x', L') to x says that (x', L') rests, and that
  from (x, L') the latches settle at L.  If one latch changes, g(x, L') = L.
  If several do, galre fires them in rounds: those of a round together
  (every state on the way excites the ones not yet flipped), the rounds
  one after the other.  Of the splits that contradict nothing seen, it
  takes the one that best agrees with the steps that change one latch,
  so latches with a shared enable flip together, and a latch that reads
  another flips after it.
- Each output's on and off sets over (x, L) are minimised with the greedy
  minimiser of analysis.py, on both polarities, over the variables the
  output is seen to depend on (a single-variable flip that changes it);
  where the walks did not go, it does not depend on the others either.
  A latch's cover also gets, for every observed step that keeps it at 1
  (at 0 for a negated cover), a product term that holds through the
  step, so the rebuilt latch cannot glitch there (a static hazard can
  flip a latch).
- galre learns from walks in every bit order, then walks from other start
  vectors until the learned circuit predicts two walks in a row that it
  was not learned from (each from its first read), learning again from
  those it gets wrong.  Last, it checks that no output is ever left
  undriven.

Not modelled: registers together with latches (they make the same x and
latch levels lead to different places, and galre stops), latches that
react only to several inputs changing at once, and the latches' state at
power-up, which the rebuilt chip may settle differently (a cross-coupled
pair races there); verify compares walks from the first read where the
rebuilt chip agrees.
"""

from analysis import DEV, HOW, KEEP, AsyncDesign, _cube_set, popcount, sop, swap
from bitsim import litmasks
from rig import bitrev, gray_order, survey_mul

_BITS = bytes.maketrans(b"\x00\x01", b"01")
_ONES = bytes.maketrans(b"\x00\x01\x02", b"001")
_ZEROS = bytes.maketrans(b"\x00\x01\x02", b"010")


def _to_bytes(bits, n):
    """A bitset over 2^n vectors as one byte (0/1) per vector."""
    N = 1 << n
    return bytearray(int(c) for c in format(bits, "0%db" % N)[::-1]) if N else bytearray()


def _from_bytes(b, table=_BITS):
    """A bitset from one byte per vector (0/1, or a table mapping to '0'/'1')."""
    s = bytes(b).translate(table)
    return int(s[::-1], 2) if s else 0


class _Walk:
    """One Gray code walk: the order of x and the output levels."""

    def __init__(self, cfg, xs, levels):
        self.cfg, self.xs, self.levels = cfg, xs, levels   # levels: pin -> bytearray over x


def _sym(bits, b, v1, full):
    """bits made independent of variable b (either half set sets both)."""
    s = 1 << b
    m = (bits & ~v1[b] & full) | ((bits & v1[b]) >> s)
    return m | (m << s)


def _splits(bits):
    """Every way to fire bits in rounds: ordered set partitions."""
    if not bits:
        yield []
        return
    k = len(bits)
    for mask in range(1, 1 << k):
        first = [bits[t] for t in range(k) if mask >> t & 1]
        for tail in _splits([bits[t] for t in range(k) if not mask >> t & 1]):
            yield [first] + tail


def _rounds(x, lp, rounds, n):
    """The points (x, latch state) a step from lp passes through when it
    fires rounds, with the latch state each must excite, and the moves
    between them as (a, b, latch state, latches moving) for the hazard
    pairs."""
    pts, edges = [], []
    cur = lp
    for k, rnd in enumerate(rounds):
        nxt = tuple(rounds[k + 1]) if k + 1 < len(rounds) else ()
        target = cur
        for b in rnd:
            target ^= 1 << b
        for sub in range(1 << len(rnd)):
            s = cur
            for t, b in enumerate(rnd):
                if sub >> t & 1:
                    s ^= 1 << b
            if s == target:
                continue
            i = x | s << n
            pts.append((i, target))
            for t, b in enumerate(rnd):
                if not sub >> t & 1:
                    s2 = s ^ 1 << b
                    edges.append((i, x | s2 << n, target, (b,) + (nxt if s2 == target else ())))
        cur = target
    return pts, edges


class LatchAnalysis:
    def __init__(self, rig, device, fixed=None, log=print, walk0=None, max_walks=48,
                 confirm=2):
        self.rig, self.device, self.dev, self.log = rig, device, DEV[device], log
        self.pins = rig.pins
        self.fixed = dict(fixed or {})
        self.base = dict(self.fixed)
        self.walk0 = walk0              # (V0, la) of the power-up walk, for the pin roles
        self.max_walks, self.confirm = max_walks, confirm
        self.clock_in_v = True
        self.reg = []
        self.walks = []

    # pin roles --------------------------------------------------------------
    def roles(self):
        """Inputs: the dedicated inputs (pin 1 too) and the I/O pins that
        read their own drive in every vector of the power-up walk.  The
        other I/O pins are outputs."""
        V0, la = self.walk0
        _, v1 = litmasks(len(V0))
        self.outputs = [p for p in self.dev["io"] if p not in self.fixed
                        and not (p in V0 and la[p] == v1[V0.index(p)])]
        io_in = [p for p in self.dev["io"] if p not in self.fixed and p not in self.outputs]
        self.V = sorted([p for p in self.dev["ded"] if p not in self.fixed] + io_in)
        self.n = len(self.V)
        self.wbase = dict(self.base)
        self.wbase.update({p: 0 for p in self.outputs})
        self.log("latch model: inputs %s, outputs %s" % (self.V, self.outputs))

    def configs(self):
        """(rot, rev, start, mix) of the walks to learn from, all from
        power-up: every rotation of the usual Gray code, spread out, both
        ways, and some mixed Gray codes, in which most pins change often
        (rig.mix_bits)."""
        n = self.n
        nb = max(1, (n - 1).bit_length())
        rots = sorted(range(n), key=lambda r: (bitrev(r, nb), r))
        return [(r, v, 0, 0) for r in rots for v in (0, 1)] + [(0, 0, 0, k) for k in (1, 2, 3, 4)]

    def checks(self):
        """Walks to check the model with: mixed Gray codes from other start
        vectors."""
        n, m = self.n, (1 << self.n) - 1
        mul = survey_mul(n)
        k = 0
        while True:
            k += 1
            yield ((k * 3) % n, k & 1, (k * mul) & m, 100 + k)

    def need(self):
        """How many check walks in a row the model must predict: confirm,
        or as many as make 16384 steps (at most 64)."""
        return max(self.confirm, min(64, -(-16384 >> self.n)))

    def _walk(self, cfg):
        rot, rev, start, mix = cfg
        la, _, _ = self.rig.walk(self.V, self.wbase, [], gray=1, rot=rot, rev=rev, start=start,
                                 mix=mix)
        xs = list(gray_order(self.n, rot, rev, start, mix))
        return _Walk(cfg, xs, {p: _to_bytes(la[p], self.n) for p in self.outputs})

    # learning ---------------------------------------------------------------
    def _latch_pins(self):
        """Outputs that read differently at the same x in two walks, less
        those that read as a function of x and the others (an output of
        the latches, not one itself)."""
        lp = []
        for p in self.outputs:
            ref = self.walks[0].levels[p]
            if any(w.levels[p] != ref for w in self.walks[1:]):
                lp.append(p)
        changed = True
        while changed:
            changed = False
            for p in list(lp):
                rest = [q for q in lp if q != p]
                seen = {}
                if all(seen.setdefault((x,) + tuple(w.levels[q][x] for q in rest),
                                       w.levels[p][x]) == w.levels[p][x]
                       for w in self.walks for x in w.xs):
                    lp.remove(p)
                    changed = True
        return lp

    def _lstate(self, w, x):
        return sum(w.levels[p][x] << j for j, p in enumerate(self.L))

    def learn(self):
        """Learn the latch pins and every output's function from self.walks."""
        self.L = self._latch_pins()
        if not self.L:
            raise AsyncDesign("the power-up walk depends on its order, but walks that change "
                              "one input at a time do not: the chip reacts to several inputs "
                              "changing at once, which galre does not model")
        n, m = self.n, len(self.L)
        size = 1 << (n + m)
        g = [bytearray(size) for _ in range(m)]          # 0 unknown, 1 low, 2 high
        f = {p: bytearray(size) for p in self.outputs if p not in self.L}
        pairs = [set() for _ in range(m)]               # (a, b, value) steps that hold g_j

        def put(arr, i, v, what):
            old = arr[i]
            if old == 0:
                arr[i] = v + 1
            elif old != v + 1:
                x, lv = i & ((1 << n) - 1), i >> n
                raise AsyncDesign(
                    "%s at %s with latch pins %s at %s is %d once and %d once: not a circuit "
                    "of latches on those pins (registers, or a latch that is not seen?)" % (
                        what, self._inputs(x), self.L,
                        "".join(str(lv >> j & 1) for j in range(m)), old - 1, v))

        def rest(x, lv, w):
            i = x | lv << n
            for j in range(m):
                put(g[j], i, lv >> j & 1, "the excitation of pin %d" % self.L[j])
            for p, arr in f.items():
                put(arr, i, w.levels[p][x], "pin %d" % p)

        def hold(a, b, lv, moving=()):
            """g_j is lv's bit j at both points, for the j not moving."""
            for j in range(m):
                if j not in moving:
                    pairs[j].add((min(a, b), max(a, b), lv >> j & 1))

        # What a step proves.  The chip is seen at rest only: before the
        # step at (x', L'), after it at (x, L).  At rest every latch is
        # excited to the level it has (rest()).
        # - Nothing changed: the same, at (x, L'), and no latch may glitch
        #   between the two points (hold).
        # - One latch changed: it is taken to be excited at (x, L').  What
        #   the others do there is NOT known: one of them may be excited
        #   for one gate delay and fall back when the first has moved (in
        #   a board's GAL, pin 14 pulsed low while pin 18 rose).  Recording
        #   "holds" there bends its equation.
        # - Several changed: in which order, and through which levels, is
        #   not seen.
        # So the circuit is learned from the first two, and every step is
        # then checked: the learned circuit, every excited latch moving at
        # once as in the chip's simulator (one gate delay each), must come
        # to rest where the chip did.  Where it does not, the step says
        # more: a latch that did not change but moves in the learned
        # circuit holds at (x, L'); a step with several changes is split
        # into rounds (_step).  Then all are checked again.
        steps = set()
        for w in self.walks:
            xp = w.xs[0]
            lp = self._lstate(w, xp)
            rest(xp, lp, w)
            for x in w.xs[1:]:
                lc = self._lstate(w, x)
                rest(x, lc, w)
                e = x | lp << n
                ch = lc ^ lp
                if not ch:
                    hold(xp | lp << n, e, lp)
                else:
                    if ch & (ch - 1) == 0:
                        q = ch.bit_length() - 1
                        put(g[q], e, lc >> q & 1, "the excitation of pin %d" % self.L[q])
                    steps.add((xp, x, lp, lc))
                xp, lp = x, lc
        self.g, self.f, self.pairs = g, f, pairs
        self._minimise()
        self._tables()
        k = n + m
        full, v1 = litmasks(k)
        todo = sorted(steps)
        self.races = sum(1 for t in todo if (t[2] ^ t[3]) & ((t[2] ^ t[3]) - 1))
        for _ in range(24):
            wrong = [t for t in todo if self._settle(t[1], t[2]) != t[3]]
            if not wrong:
                break
            proj = None
            for xp, x, lp, lc in wrong:
                ch = lc ^ lp
                e = x | lp << n
                if ch & (ch - 1) == 0:
                    # the latches that the learned circuit moves, but the chip did not
                    for j in range(m):
                        if j != ch.bit_length() - 1 and self.tab[self.L[j]][lp][x] != lp >> j & 1:
                            put(g[j], e, lp >> j & 1, "the excitation of pin %d" % self.L[j])
                    continue
                if proj is None:
                    proj = [self._supported(_from_bytes(g[j], _ONES), _from_bytes(g[j], _ZEROS),
                                            k, full, v1) for j in range(m)]
                if not self._step(g, proj, x, lp, lc, hold, xp | lp << n):
                    bits = [self.L[j] for j in range(m) if ch >> j & 1]
                    raise AsyncDesign("latch pins %s change together at %s, in no order that "
                                      "agrees with the other walks" % (bits, self._inputs(x)))
            done = {t for t in wrong if (t[2] ^ t[3]) & ((t[2] ^ t[3]) - 1)}
            todo = [t for t in todo if t not in done]
            self._minimise()
            self._tables()
        else:
            xp, x, lp, lc = wrong[0]
            raise AsyncDesign(
                "%d steps are not explained by one circuit of latches (the first: pins %s "
                "change at %s): what happens there depends on more than the levels seen" % (
                    len(wrong), [self.L[j] for j in range(m) if (lp ^ lc) >> j & 1],
                    self._inputs(x)))

    def _settle(self, x, lv):
        """Where the learned latches come to rest with the inputs at x,
        from the levels lv: every excited latch moves at once, as in the
        chip's simulator (one gate delay each); if that does not settle,
        one at a time.  None if they never rest."""
        m = len(self.L)
        gt = [self.tab[p] for p in self.L]
        rm = range(m)
        for _ in range(m + 6):
            nxt = 0
            for j in rm:
                nxt |= gt[j][lv][x] << j
            if nxt == lv:
                return lv
            lv = nxt
        for _ in range(4 * m + 8):
            exc = [j for j in rm if gt[j][lv][x] != lv >> j & 1]
            if not exc:
                return lv
            lv ^= 1 << exc[0]
        return None

    def _step(self, g, proj, x, lp, lc, hold, prev):
        """Several latches changed in one step: fire them in rounds, the
        latches of a round together (every state on the way excites those
        not yet flipped), the rounds one after the other.  Of the ways to
        split them that contradict nothing seen, take the one that best
        agrees with what the steps changing one latch show (proj: their on
        and off sets, over the variables they depend on), then the one
        with the fewest rounds.  False if there is none."""
        n, m = self.n, len(self.L)
        bits = [j for j in range(m) if (lp ^ lc) >> j & 1]
        cands = [self._greedy(proj, x, lp, lc, bits)]
        if len(bits) <= 4:
            cands += list(_splits(bits))
        best = None
        for idx, rounds in enumerate(cands):
            pts, edges = _rounds(x, lp, rounds, n)
            if any(g[j][i] not in (0, (req >> j & 1) + 1) for i, req in pts for j in range(m)):
                continue
            bad = sum(proj[j][1 if req >> j & 1 else 0] >> i & 1      # the off set for a 1
                      for i, req in pts for j in range(m))
            key = (bad, len(rounds), idx)
            if best is None or key < best[0]:
                best = (key, rounds, pts, edges)
        if best is None:
            return False
        _, rounds, pts, edges = best
        for i, req in pts:
            for j in range(m):
                g[j][i] = (req >> j & 1) + 1
        hold(prev, x | lp << n, lp, tuple(rounds[0]))
        for a, b, lv, moving in edges:
            hold(a, b, lv, moving)
        return True

    def _greedy(self, proj, x, lp, lc, bits):
        """Rounds as proj predicts them: first the latches it sees excited,
        else those it does not see resting."""
        n = self.n
        cur, left, rounds = lp, list(bits), []
        while left:
            i = x | cur << n
            exc = [b for b in left if proj[b][0 if lc >> b & 1 else 1] >> i & 1]
            exc = exc or [b for b in left if not proj[b][1 if lc >> b & 1 else 0] >> i & 1]
            exc = exc or list(left)
            rounds.append(exc)
            for b in exc:
                cur ^= 1 << b
            left = [b for b in left if b not in exc]
        return rounds

    def _supported(self, on, off, k, full, v1):
        """on and off made independent of the variables no single flip shows
        the function depending on; more variables if that contradicts."""
        sup = {b for b in range(k) if on & swap(off, b, v1) & full}
        while True:
            a, b = on, off
            for v in range(k):
                if v not in sup:
                    a, b = _sym(a, v, v1, full), _sym(b, v, v1, full)
            if not a & b:
                return a, b
            best = None
            for v in range(k):
                if v not in sup:
                    a2, b2 = on, off
                    for u in range(k):
                        if u not in sup and u != v:
                            a2, b2 = _sym(a2, u, v1, full), _sym(b2, u, v1, full)
                    c = popcount(a2 & b2)
                    if best is None or c < best[0]:
                        best = (c, v)
            sup.add(best[1])

    def _minimise(self):
        n, m = self.n, len(self.L)
        k = n + m
        full, v1 = litmasks(k)
        self.cover = {}
        for j, p in enumerate(self.L):
            on, off = self._supported(_from_bytes(self.g[j], _ONES),
                                      _from_bytes(self.g[j], _ZEROS), k, full, v1)
            best = None
            for inv in (False, True):
                a, b = (off, on) if inv else (on, off)
                cubes = sop(a, b, k)
                cubes = self._hazard_free(cubes, [(x, y) for x, y, v in self.pairs[j]
                                                  if v == (0 if inv else 1)], b, k, full, v1)
                if best is None or len(cubes) < len(best[0]):
                    best = (cubes, inv)
            self.cover[p] = best
        for p, arr in self.f.items():
            on, off = self._supported(_from_bytes(arr, _ONES), _from_bytes(arr, _ZEROS),
                                      k, full, v1)
            pos, neg = sop(on, off, k), sop(off, on, k)
            self.cover[p] = (neg, True) if len(neg) < len(pos) else (pos, False)

    @staticmethod
    def _hazard_free(cubes, pairs, off, k, full, v1):
        """Add, for every pair of neighbouring points that no single cube
        covers, a cube through both, as large as off allows."""
        cubes = list(cubes)

        def covers(c, a):
            return a & c[0] == c[1]
        for a, b in sorted(pairs):
            if any(covers(c, a) and covers(c, b) for c in cubes):
                continue
            care = ((1 << k) - 1) & ~(a ^ b)
            val = a & care
            if _cube_set(care, val, k, full, v1) & off:
                raise RuntimeError("hazard pair %x %x crosses the off set" % (a, b))
            for bit in range(k):
                if care >> bit & 1:
                    c2 = care & ~(1 << bit)
                    if not _cube_set(c2, val & c2, k, full, v1) & off:
                        care, val = c2, val & c2
            cubes.append((care, val))
        return cubes

    # the learned circuit ----------------------------------------------------
    def _tables(self):
        """Per output and latch state, its value for every x (bytes)."""
        n, m = self.n, len(self.L)
        fulln, v1n = litmasks(n)
        self.tab = {}
        for p, (cubes, inv) in self.cover.items():
            per = []
            for lv in range(1 << m):
                bits = 0
                for care, val in cubes:
                    if (care >> n) & ~(lv ^ (val >> n)) != care >> n:
                        continue                    # a latch literal is false
                    xm = (1 << n) - 1
                    bits |= _cube_set(care & xm, val & xm, n, fulln, v1n)
                if inv:
                    bits ^= fulln
                per.append(_to_bytes(bits, n))
            self.tab[p] = per

    def predict(self, w):
        """The first step of walk w the learned circuit gets wrong, as
        (index, x, pin, read, predicted), or None."""
        m = len(self.L)
        gt = [self.tab[p] for p in self.L]
        others = [p for p in self.outputs if p not in self.L]
        lv = self._lstate(w, w.xs[0])
        for i, x in enumerate(w.xs):
            lv = self._settle(x, lv)
            if lv is None:
                return (i, x, None, None, None)
            for j, p in enumerate(self.L):
                if w.levels[p][x] != lv >> j & 1:
                    return (i, x, p, w.levels[p][x], lv >> j & 1)
            for p in others:
                if w.levels[p][x] != self.tab[p][lv][x]:
                    return (i, x, p, w.levels[p][x], self.tab[p][lv][x])
        return None

    def _inputs(self, x):
        return " ".join("P%d=%d" % (p, x >> i & 1) for i, p in enumerate(self.V))

    # run --------------------------------------------------------------------
    def run(self):
        self.roles()
        if self.n < 2:
            raise AsyncDesign("too few inputs to walk in different orders")
        self.walks = [self._walk(c) for c in self.configs()]
        self.learn()
        self._tables()
        good, need = 0, self.need()
        limit = len(self.walks) + max(self.max_walks, 3 * need)
        for c in self.checks():
            if good >= need or len(self.walks) >= limit:
                break
            w = self._walk(c)
            self.walks.append(w)
            miss = self.predict(w)
            if miss is None:
                good += 1
                continue
            good = 0
            self.log("walk with rot=%d rev=%d start=%x mix=%d: step %d (%s) %s; learning "
                     "again from %d walks" % (
                         c + (miss[0], self._inputs(miss[1]),
                              "does not settle" if miss[2] is None else
                              "pin %d reads %d, predicted %d" % miss[2:], len(self.walks))))
            self.learn()
            self._tables()
        bad = self.check_equations()
        if bad:
            raise AsyncDesign("the latch model does not predict the walks: %s" % bad[:3])
        if good < need:
            raise AsyncDesign("the latch model did not predict %d walks in a row within %d "
                              "walks" % (need, len(self.walks)))
        self._check_driven()
        self.log("latch model: latch pins %s, combinational %s, from %d walks" % (
            self.L, [p for p in self.outputs if p not in self.L], len(self.walks)))
        return self.equations()

    def _check_driven(self):
        """One more walk, each vector read again with the outputs' drive
        inverted: an output must read the same, so the GAL drives it."""
        base = dict(self.base)
        la, lb, _ = self.rig.walk(self.V, base, [], gray=1)
        tri = [p for p in self.outputs if la[p] != lb[p]]
        if tri:
            raise AsyncDesign("outputs %s are not always driven, which the latch model does "
                              "not cover" % tri)

    def check_equations(self):
        """Walks the learned circuit gets wrong: [(rot, rev, start, step, x)]."""
        out = []
        for w in self.walks:
            miss = self.predict(w)
            if miss:
                out.append(w.cfg + miss[:2])
        return out

    # output -----------------------------------------------------------------
    def name(self, p):
        return "P%d" % p

    def equations(self):
        names = [self.name(p) for p in self.V] + [self.name(p) for p in self.L]
        k = len(names)
        eqs = []
        for p, (cubes, inv) in sorted(self.cover.items()):
            def lit(c):
                if c[0] == 0:
                    return "'b'1"
                return " & ".join(("" if c[1] >> j & 1 else "!") + names[j]
                                  for j in range(k) if c[0] >> j & 1)
            expr = "\n    # ".join(lit(c) for c in cubes) if cubes else "'b'0"
            eqs.append((p, "", inv, expr, len(cubes)))
        self.eqs = eqs
        return eqs

    def cupl(self, title="galre"):
        lines = ["Name %s; Partno x; Date x; Revision 1; Designer galre; Company x; "
                 "Assembly x; Location x;" % title,
                 "Device %s;" % self.dev["cupl"], "", HOW % title, ""]
        for p in sorted(set(self.V) | set(self.outputs)):
            lines.append("Pin %d = %s;" % (p, self.name(p)))
        lines += ["", KEEP, "MIN [%s] = 0;" % ", ".join(self.name(p) for p in self.L), ""]
        for p, _, inv, expr, _ in self.eqs:
            lines.append("%s%s = %s;" % ("!" if inv else "", self.name(p), expr))
        return "\n".join(lines) + "\n"
