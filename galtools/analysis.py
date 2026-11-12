"""Black-box reverse engineering of a GAL16V8 / GAL22V10 from rig captures.

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
3. assign: the would-be value of every registered pin per state; a pin
   that is never enabled in a state takes the complement of the only value
   it is ever seen with.
4. equations: per output function (value, output enable, register input)
   the support from single-variable flips, then a greedy two-level
   minimiser on both polarities; CUPL out.

Pin 1 is held low except for clock edges.  If clocking never changes the
state, the design is taken as combinational and pin 1 is enumerated too.
Limits: a combinational dependence on pin 1 of a registered 22V10 is not
captured; registers that never reach a pin are not seen; asynchronous
reset terms and latches built from feedback disturb the walks.
"""

from bitsim import litmasks
from rig import CLOCK, MAX_PATH, MAX_PROBES, MAX_RET, MAX_RET_LEN, popcount, survey_mul

DEV = {
    "GAL16V8": {"ded": [1, 2, 3, 4, 5, 6, 7, 8, 9, 11], "io": list(range(12, 20)),
                "cupl": "g16v8"},
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


def sop(on, off, k):
    """Greedy cover of the minterm bitset `on` avoiding `off` over k
    variables; [(care, val)] cubes."""
    if not on:
        return []
    full, v1 = litmasks(k)
    if not off:
        return [(0, 0)]
    unc, cubes = on, []
    while unc:
        m = (unc & -unc).bit_length() - 1
        care, val, cur = (1 << k) - 1, m, 1 << m
        while True:
            best = None
            for b in range(k):
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


# -- helpers -----------------------------------------------------------------

def swap(f, b, v1):
    """Bitset f with the halves of variable b exchanged."""
    s = 1 << b
    return ((f & v1[b]) >> s) | ((f & ~v1[b]) << s)


def lowest(xs):
    return (xs & -xs).bit_length() - 1


class Table:
    """One state's exhaustive walk: per pin, driven and value bitsets."""

    def __init__(self, V, la, lb, full, v1):
        self.driven, self.value = {}, {}
        for p in la:
            if p in V:
                b = V.index(p)
                sw = swap(la[p], b, v1) & full
                self.driven[p] = full ^ (la[p] ^ sw)
            else:
                self.driven[p] = full ^ (la[p] ^ lb[p])
            self.value[p] = la[p]


class Analysis:
    def __init__(self, rig, device, fixed=None, survey=64, max_states=64, log=print):
        self.rig, self.device, self.dev, self.log = rig, device, DEV[device], log
        self.pins = rig.pins
        self.fixed = dict(fixed or {})
        self.base = dict(self.fixed)
        self.survey = survey
        self.max_states = max_states
        self.clock_in_v = CLOCK in self.fixed
        self.late_comb = {}       # pin -> enumerate it (tri-state)
        self._power_up = {}

    # 1. discovery -----------------------------------------------------------
    def discover(self):
        V0 = [p for p in self.pins
              if (p != CLOCK or self.clock_in_v) and p not in self.fixed]
        full, v1 = litmasks(len(V0))
        if self.clock_in_v not in self._power_up:
            self._power_up[self.clock_in_v] = Table(V0, *self.rig.walk(V0, self.base, []), full, v1)
        t = self._power_up[self.clock_in_v]
        self.comb0, self.cand = [], []
        for p in self.dev["io"]:
            d = t.driven[p]
            v = t.value[p] & d
            if (d and v not in (0, d)) or p in self.late_comb:
                self.comb0.append(p)          # varies with the inputs
            else:
                self.cand.append(p)           # input, register or constant
        tri = [p for p in self.comb0 if full ^ t.driven[p] or self.late_comb.get(p)]
        never = [p for p in self.cand if not t.driven[p] and p not in self.fixed]
        ded = [p for p in self.dev["ded"]
               if (p != CLOCK or self.clock_in_v) and p not in self.fixed]
        self.V = sorted(ded + never + [p for p in tri if p not in self.fixed])
        self.full, self.v1 = litmasks(len(self.V))
        self.log("power-up walk: combinational %s (tri-state %s), state candidates %s "
                 "(never driven %s)" % (self.comb0, tri, self.cand, never))
        self.log("enumerated pins (%d): %s%s" % (
            len(self.V), self.V,
            ", fixed %s" % self.fixed if self.fixed else ""))
        self.probe_x = []

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
        """An enabling input vector for every candidate the table drives."""
        added = False
        for r in self.cand:
            d = t.driven[r]
            if d and not any(d >> x & 1 for x in self.probe_x):
                if len(self.probe_x) == MAX_PROBES:
                    raise RuntimeError("more than %d probe vectors needed" % MAX_PROBES)
                self.probe_x.append(lowest(d))
                added = True
        return added

    # 2. exploration ---------------------------------------------------------
    def _sig_obs(self, obs):
        sig = []
        for i, r in enumerate(self.cand):
            v = "Z"
            for pr in obs:
                a, b = pr[i]
                if a == b:
                    v = a
                    break
            sig.append(v)
        return tuple(sig)

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
        if path not in self._walks:
            if len(path) > MAX_PATH:
                raise RuntimeError("state needs a path of %d clocks" % len(path))
            la, lb = self.rig.walk(self.V, self.base, self.drives(path))
            self._walks[path] = Table(self.V, la, lb, self.full, self.v1)
        return self._walks[path]

    def _returns(self, s):
        """(observation, drives) routes from known states back to s."""
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
        obs = self.rig.transitions(self.V, self.base, self.drives(path), self.probes(),
                                   self.cand, count=count,
                                   mul=1 if count is None else survey_mul(n),
                                   returns=self._returns(s))
        m = self.edges.setdefault(s, {})
        if count is None:
            self.trans[s] = {}
        for o, xs in obs.items():
            s2 = self._sig_obs(o)
            self.obs_of.setdefault(s2, set()).add(o)
            m[s2] = m.get(s2, 0) | xs
            if count is None:
                self.trans[s][s2] = self.trans[s].get(s2, 0) | xs
            if s2 in self.states:
                continue
            p2 = path + (lowest(xs),)
            t = self._walk(p2)
            st = self._sig_table(t)
            if st != s2:
                if self._add_probes(t):
                    return False
                late = [r for r, v in zip(self.cand, st) if v == "?"]
                if not late:
                    raise RuntimeError("state %s walks as %s" % (s2, st))
                # constant at power-up, input-dependent here: combinational
                for r in late:
                    self.late_comb[r] = bool(self.full ^ t.driven[r])
                self.log("pins %s depend on the inputs in state %s: combinational" % (late, s2))
                self.rediscover = True
                return False
            self.states[s2] = (p2, t)
            queue.append(s2)
            if len(self.states) > self.max_states:
                raise RuntimeError("more than %d states" % self.max_states)
        return True

    def _explore_once(self):
        self._walks = {}
        t0 = self._walk(())
        self._add_probes(t0)
        s0 = self._sig_table(t0)
        self.power_up_state = s0
        self.states = {s0: ((), t0)}
        self.trans, self.edges, self.obs_of = {}, {}, {}
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
            if self._explore_once():
                break
            if self.rediscover:
                self.discover()
            else:
                self.log("round %d: a new state drives a pin no probe enables; %d probes "
                         "now, exploring again" % (rnd + 1, len(self.probe_x)))
        else:
            raise RuntimeError("probe set did not converge")
        self.inputs, self.reg, self.comb = [], [], list(self.comb0)
        for i, r in enumerate(self.cand):
            sigs = {sig[i] for sig in self.states}
            if all(not t.driven[r] for _, t in self.states.values()):
                self.inputs.append(r)
            elif sigs <= {0, 1, "Z"} and (len(sigs - {"Z"}) == 2 or "Z" in sigs):
                self.reg.append(r)
            else:
                self.comb.append(r)       # constant or input-dependent
        self.log("%d states reachable from power-up; inputs %s, registered %s, "
                 "combinational %s" % (len(self.states), self.inputs, self.reg, self.comb))

    def run(self):
        """discover + explore + assign + equations; treats pin 1 as an input
        when clocking never changes anything."""
        self.discover()
        self.explore()
        if (not self.reg and len(self.states) == 1 and not self.clock_in_v
                and CLOCK not in self.fixed):
            self.log("clocking never changes the state: combinational design, "
                     "enumerating pin 1 as an input")
            self.clock_in_v = True
            self.discover()
            self.explore()
        self.assign()
        return self.equations()

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
        for s in self.states:
            w = []
            for r, i in zip(regs, idx):
                v = s[i]
                if v in (0, 1):
                    w.append(v)
                elif len(seen[r]) == 1:
                    w.append(1 - next(iter(seen[r])))
                else:
                    raise RuntimeError("register on pin %d is hidden in state %s" % (r, s))
            self.W[s] = tuple(w)
        if len(set(self.W.values())) != len(self.W):
            raise RuntimeError("two states have the same register values")

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
                oe[s] = (d, self.full ^ d)
            F[(p, "")] = val
            if any(o[1] for o in oe.values()):
                F[(p, "oe")] = oe
        for i, r in enumerate(regs):
            dd = {}
            for s, m in self.trans.items():
                on = off = 0
                for s2, xs in m.items():
                    if self.W[s2][i]:
                        on |= xs
                    else:
                        off |= xs
                dd[s] = (on, off)
            F[(r, "d")] = dd
            if self.device == "GAL22V10":
                oe = {}
                for s, (_, t) in self.states.items():
                    d = t.driven[r]
                    oe[s] = (d, self.full ^ d)
                if any(o[1] for o in oe.values()):
                    F[(r, "oe")] = oe
        return F

    def _support(self, f):
        """Variables (index into V, then registers) f depends on."""
        n, regs = len(self.V), sorted(self.reg)
        sup = set()
        for s, (on, off) in f.items():
            for b in range(n):
                if b not in sup and (on & swap(off, b, self.v1)) & self.full:
                    sup.add(b)
        sl = list(f)
        for i in range(len(regs)):
            for a in sl:
                for b in sl:
                    wa, wb = self.W[a], self.W[b]
                    if sum(x != y for x, y in zip(wa, wb)) == 1 and wa[i] != wb[i] and wa[i] == 0:
                        if (f[a][0] & f[b][1]) or (f[a][1] & f[b][0]):
                            sup.add(n + i)
        return sorted(sup)

    def _project(self, f, sup):
        """on/off minterm bitsets over the support, by splitting the x
        bitsets on each support variable."""
        n = len(self.V)
        vpos = [(j, v) for j, v in enumerate(sup) if v < n]
        on = off = 0
        for s, (fo, ff) in f.items():
            w = self.W[s]
            wbits = 0
            for j, v in enumerate(sup):
                if v >= n and w[v - n]:
                    wbits |= 1 << j
            for which, bs in ((0, fo), (1, ff)):
                groups = [(wbits, bs)] if bs else []
                for j, v in vpos:
                    ng = []
                    for m, g in groups:
                        a, b = g & ~self.v1[v], g & self.v1[v]
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
        F = self.functions()
        regs = sorted(self.reg)
        names = [self.name(p) for p in self.V] + [self.name(r) for r in regs]
        self.F, self.cover, eqs = F, {}, []
        for (p, kind), f in F.items():
            sup = self._support(f)
            while True:
                on, off = self._project(f, sup)
                if not on & off:
                    break
                extra = [i for i in range(len(self.V) + len(regs)) if i not in sup]
                if not extra:
                    raise RuntimeError("pin %d %s is not a function of the inputs and "
                                       "registers" % (p, kind))
                sup = sorted(sup + [extra[-1]])
            k = len(sup)
            pos = sop(on, off, k)
            neg = sop(off, on, k)
            inv = len(neg) < len(pos)
            cubes = neg if inv else pos
            self.cover[(p, kind)] = (sup, cubes, inv)

            def lit(c):
                if c[0] == 0:
                    return "'b'1"
                return " & ".join(("" if c[1] >> j & 1 else "!") + names[sup[j]]
                                  for j in range(k) if c[0] >> j & 1)
            expr = "\n    # ".join(lit(c) for c in cubes) if cubes else "'b'0"
            eqs.append((p, kind, inv, expr, len(cubes)))
        self.eqs = sorted(eqs)
        return self.eqs

    def check_equations(self):
        """Every function's cover against its captured on/off sets; a list
        of (pin, kind, state) that disagree (empty when all hold)."""
        n, bad = len(self.V), []
        for key, f in self.F.items():
            sup, cubes, inv = self.cover[key]
            for s, (on, off) in f.items():
                cov = 0
                for care, val in cubes:
                    c = self.full
                    for j, v in enumerate(sup):
                        if not care >> j & 1:
                            continue
                        want = val >> j & 1
                        if v < n:
                            c &= self.v1[v] if want else self.full ^ self.v1[v]
                        elif self.W[s][v - n] != want:
                            c = 0
                    cov |= c
                if inv:
                    cov = self.full ^ cov
                if (on & ~cov) or (off & cov):
                    bad.append(key + (s,))
        return bad

    def cupl(self, title="galre"):
        regs = sorted(self.reg)
        clocked = bool(regs) and not self.clock_in_v
        lines = ["Name %s; Partno x; Date x; Revision 1; Designer galre; Company x; "
                 "Assembly x; Location x;" % title,
                 "Device %s;" % self.dev["cupl"], ""]
        if clocked:
            lines.append("Pin 1 = CLK;")
        for p in sorted(set(self.V) | set(self.reg) | set(self.comb)):
            if self.device == "GAL16V8" and clocked and p == 11:
                lines.append("Pin 11 = !OE;")
                continue
            lines.append("Pin %d = %s;" % (p, self.name(p)))
        lines.append("")
        for p, kind, inv, expr, n in self.eqs:
            if self.device == "GAL16V8" and clocked and p in regs and kind == "oe":
                continue
            lhs = self.name(p) + ("." + kind if kind else "")
            if kind == "oe" and inv:
                lines.append("%s = !(%s);" % (lhs, expr))
            else:
                lines.append("%s%s = %s;" % ("!" if inv else "", lhs, expr))
        return "\n".join(lines) + "\n"
