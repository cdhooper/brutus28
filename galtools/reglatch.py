"""Registers and latches in one design (GAL16V8, GAL22V10).

A registered design can also hold state in a combinational output that is
fed back into the array (a latch), and a GAL22V10's product terms can read
pin 1, the clock, like any other input.  An example is a chip select that
is set while the clock is high and a register is set, and holds itself
until the register or an input lets go.  analysis.Analysis stops at such a
chip: after a clock edge, reading the new state (the probes change the
inputs) changes a pin without a clock.  RegLatchAnalysis goes on:

1. The pins that change without a clock (self.hist) are no state
   candidates, and the transition sweeps do not read them.  What is left
   are the registers, explored as usual: the states, the transitions, the
   asynchronous reset term.  Each full sweep also keeps the level of every
   pin at each clock edge (self.pre): a register input may read a latch.
   A pin of self.hist that the GAL does not always drive (a latch on a
   bidirectional pin) stays enumerated (self.bidir): it is an input too.
2. Walks with pin 1 enumerated.  The chip is walked in Gray code order
   (one pin changes per write) over pin 1 and the enumerated pins, so the
   walk clocks the chip whenever it raises pin 1.  The walks are: from
   every state, two with pin 1 changing slowest (all inputs with pin 1
   high, then all with pin 1 low, going around the reset term where it
   would fire); from power-up, one for every other position of pin 1 in
   the order, both ways; then walks from other states and start vectors,
   until the learned circuit predicts some in a row.
3. The register state at every vector of a walk (_interpret).  A clock
   edge with the outputs at the levels the state's sweep had at that x
   goes where the sweep went.  With other levels it is taken on trust,
   and every read checks the registered pins that are driven: one that
   reads otherwise shows that the register loaded the other value, which
   is kept (self.rise).  So the register inputs are learned with the latch
   pins as inputs, where the registers show.
4. The outputs whose level depends on the history at the same inputs,
   pin 1 level and register state are the latch pins, less those that
   are a function of these and the other latch pins.  A context is a
   register state with the levels of the latch pins.  Every other output,
   every output enable and every register input is a function of the
   inputs and pin 1 in each context.  A latch pin's excitation is learned
   as in latch.py: a vector at which the chip rests excites the levels it
   has; a step that changes one pin, or clocks, excites what the latches
   settle to, from the levels they had.
   An enumerated latch pin (bidirectional) is driven by the GAL only
   where its enable is true.  The enable is one product term: it must
   hold every read at which the pin is not at the level Brutus drives it
   to, and none at which the pin follows its drive.  Where the GAL lets
   the pin go, it is an input at the level it is driven to.  Nothing
   depends on Brutus's drive of such a pin as well as on its level.
5. The equations are found as in analysis.py, over pin 1, the enumerated
   pins, the registers and the latch pins.  A latch's cover gets a
   product term for every observed step that holds it, so the rebuilt
   latch cannot glitch there.

A design without registers goes the same way, with pin 1 as one more
input: latches with output enables, which latch.py does not take.

Limits: a clock edge that goes another way because of a latch is learned
only if a register that differs shows on its pin before the next edge;
a state that is reached only that way is not explored.  The reset term
must be one product term of inputs and registers.  A clock edge is taken
to change pin 1 and the registers at the same moment, as the simulator
does.  An enable or an equation may come out with other literals than
the original where a latch and its condition always go together; it is
the same at every level the walks saw.  Like latch.py, the result holds
for inputs that change one at a time, and the latches' level at power-up
is not modelled.
"""

import sys
from array import array

from analysis import Analysis, AsyncDesign, Limit, _Restart, _pins, _state, lowest, swap
from bitsim import litmasks
from latch import LatchAnalysis, _ONES, _ZEROS, _from_bytes
from rig import CLOCK, bitrev, cube_set, gray_arrays, survey_mul

_T01 = bytes.maketrans(b"01", b"\x00\x01")


def _cells(mask, N):
    """A bitset over N vectors as a number with 16 bits per vector (0/1)."""
    ba = bytearray(2 * N)
    ba[0::2] = _b01(mask, N)
    return int.from_bytes(ba, "little")


def _states(acc, N):
    """An array with a 16 bit number per vector from such a number."""
    a = array("H")
    a.frombytes(acc.to_bytes(2 * N, "little"))
    if sys.byteorder != "little":
        a.byteswap()
    return a


def _where(sid, v):
    """The vectors (as a bitset) at which the array of 16 bit numbers
    sid is v."""
    if sys.byteorder != "little":
        sid = array("H", sid)
        sid.byteswap()
    raw = sid.tobytes()
    lo, hi = bytearray(b"0" * 256), bytearray(b"0" * 256)
    lo[v & 255] = hi[v >> 8] = 49
    a = raw[0::2].translate(bytes(lo))
    b = raw[1::2].translate(bytes(hi))
    return int(a[::-1], 2) & int(b[::-1], 2) if a else 0


def _b01(bits, N):
    """A bitset over N vectors as one byte (0/1) per vector."""
    return format(bits, "0%db" % N)[::-1].encode().translate(_T01)


def _spread(bits):
    """Bit k of bits at bit 2k: a bitset over x as one over (pin 1 low, x)."""
    return int("0".join(format(bits, "b")), 2)


class _Walk:
    """One walk over (pin 1, V) and what the register model says of it:
    la, lb   pin -> bitset over the vectors (second read: pins outside the
             enumeration driven the other way)
    smask    state index -> the vectors read in that state
    order    the vectors in the order written (None for a state's table:
             its vectors are resting points only), clean[k]: one pin
             changed since the vector before
    sid      per vector, the state index + 1 (0: not read, or not known)"""

    def __init__(self, cfg, la, lb):
        self.cfg, self.la, self.lb = cfg, la, lb
        self.order = self.clean = self.sid = None
        self.smask = {}
        self.cut = None
        self.unsure = self.surprises = 0


class _Again(Exception):
    """Start over: a pin that changes without a clock is not always driven,
    so it is enumerated from now on."""


class _Flat:
    """latch.LatchAnalysis's handling of a step that flips several latches,
    with (pin 1, V, state index) as its inputs."""

    def __init__(self, n, L):
        self.n, self.L = n, L

    _step = LatchAnalysis._step
    _greedy = LatchAnalysis._greedy
    _supported = LatchAnalysis._supported


class RegLatchAnalysis(Analysis):
    PLAIN, AROUND, AVOID = 0, 1, 2      # how a walk treats the reset term

    def __init__(self, rig, device, fixed=None, survey=64, max_states=1024, log=print,
                 hist=(), max_checks=48, confirm=2):
        super().__init__(rig, device, fixed=fixed, survey=survey, max_states=max_states,
                         log=log)
        self.hist = {p for p in hist if p in self.dev["io"] and p not in self.fixed}
        self.mixed = bool(self.hist)
        self.max_checks, self.confirm = max_checks, confirm
        self.L, self.O, self.lwalks = [], [], []
        self.readpins = set()     # outputs with an enable whose level other logic reads
        self.rise = {}            # clock edges a walk showed: (state, x, levels) -> state
        self._dtab = None         # the register inputs learned so far, for _guess

    # 1. exploration without the pins that change without a clock ------------
    def _history(self, pins, err):
        """pins changed without a clock, and it is not the reset term."""
        new = sorted(p for p in pins if p in self.dev["io"] and p not in self.hist
                     and p not in self.fixed)
        if not new:
            raise err
        self.hist.update(new)
        self.bidir.update(p for p in new if p in self.V)      # also an input at times
        self.mixed = True
        self.log("asynchronous behaviour: %s" % err)
        self.log("%s changed: taking %s for latches built from combinational feedback, or "
                 "outputs of latches, and exploring the registers again without" % (
                     _pins(new), "it" if len(new) == 1 else "them"))
        raise _Restart()

    def _history_dependent(self, pins, err):
        self._history(pins, err)

    def _flagged(self, path, x, pins=()):
        try:
            super()._flagged(path, x, pins)
        except AsyncDesign as e:
            self._history(pins, e)

    def _disturbed(self, path, sigs, around=False):
        try:
            super()._disturbed(path, sigs, around)
        except AsyncDesign as e:
            before, after = (self._sig_obs(o) for o in sigs)
            self._history([r for r, a, b in zip(self.cand, before, after) if a != b], e)

    def _two_ways(self, msg):
        self.mixed = True
        self.log("asynchronous behaviour: %s" % msg)
        self.log("taking it for a register input that reads a latch, and exploring again "
                 "with the pin levels at every clock edge")
        raise _Restart()

    def _unclocked(self):
        if self.mixed:
            # latches without registers: pin 1 is one more input of theirs
            self.log("clocking changes no register")
            return
        super()._unclocked()

    def _let_go_inputs(self):
        """A pin that other logic reads where the GAL lets it go is handled
        like a latch on a bidirectional pin: the equations read its level,
        whoever drives it (walks with pin 1 enumerated, as for latches)."""
        before = set(self.late_comb)
        if not super()._let_go_inputs():
            return False
        self.readpins |= set(self.late_comb) - before
        self.mixed = True
        if self.clock_in_v and CLOCK not in self.fixed:
            self.clock_in_v = False
        return True

    def name(self, p):
        return "CLK" if p == CLOCK and self.mixed and self.reg else super().name(p)

    def run(self):
        while True:
            try:
                return super().run()
            except _Again:
                pass

    # 2. walks with pin 1 enumerated ------------------------------------------
    def _model(self):
        """The registers as the exploration found them: per state and x,
        the next state; where the reset term fires."""
        n = len(self.V)
        N = 1 << n
        self.S = list(self.states)
        self.sidx = {s: i for i, s in enumerate(self.S)}
        self.s0 = self.sidx[self.power_up_state]
        self.Vc = [CLOCK] + self.V
        self.n1 = n + 1
        self.cfull, self.cv1 = litmasks(self.n1)
        self.nxt = []
        for s in self.S:
            acc = 0
            for s2, xs in self.trans[s].items():
                acc += (self.sidx[s2] + 1) * _cells(xs, N)
            self.nxt.append(_states(acc, N))
        self.fire, self.arb = bytes(len(self.S)), None
        if self.ar:
            self._ar_term()
            regs = sorted(self.reg)
            s0 = self.power_up_state
            on = set(self.ar_on)
            if all(self.W[s0][regs.index(r)] == v for r, v in self.ar_regs.items()):
                on.add(s0)
            self.fire = bytes(1 if s in on else 0 for s in self.S)
            self.arb = _b01(self.ar_set, N)
        self.O = sorted(set(self.comb) | self.hist)
        self.comb = list(self.O)
        self.widx = {self.W[s]: i for i, s in enumerate(self.S)}
        # the outputs' levels each x was clocked in with, in each state's sweep
        self.prek = [self._pack(self.pre[s], N) if s in self.pre else None for s in self.S]
        self.rise, self._dtab = {}, None

    def _pack(self, la, N):
        """The outputs' levels per vector, 8 pins to a byte (in the order
        of self.O)."""
        out = []
        for c in range(0, len(self.O), 8):
            acc = 0
            for j, p in enumerate(self.O[c:c + 8]):
                acc |= int.from_bytes(_b01(la[p], N), "little") << j
            out.append(acc.to_bytes(N, "little"))
        return out

    def _table_walk(self, s):
        """A state's exhaustive walk (pin 1 low) as resting points."""
        _, t = self.states[s]
        w = _Walk(("table", self.sidx[s]), {p: _spread(v) for p, v in t.la.items()},
                  {p: _spread(v) for p, v in t.lb.items()})
        w.smask = {self.sidx[s]: _spread(t.walked)}
        return w

    def _configs(self):
        """(state index, rot, rev, start, mode, mix) of the walks to learn
        from: from every state two with pin 1 changing slowest, and one in
        a mixed Gray code (most pins, pin 1 too, change often:
        rig.mix_bits); from power-up every other rotation, and more mixed
        ones."""
        n1 = self.n1
        out = []
        for s, (path, _) in self.states.items():
            a = self.sidx[s]
            start = (path[-1] << 1 | 1) if path else 0
            mode = self.AROUND if path and self.fire[a] else self.PLAIN
            out += [(a, 1, 0, start, mode, 0), (a, 0, 1, start, mode, 0)]     # pin 1 slowest
            out.append((a, 0, 0, start, self.AVOID if self.ar and path else self.PLAIN, 1 + a))
        nb = max(1, (n1 - 1).bit_length())
        for r in sorted(range(n1), key=lambda r: (bitrev(r, nb), r)):
            for v in (0, 1):
                if (r, v) not in ((1, 0), (0, 1)):
                    out.append((self.s0, r, v, 0, self.PLAIN, 0))
        out += [(self.s0, 0, 0, 0, self.PLAIN, 0x1000 + k) for k in (1, 2, 3)]
        return out

    def _need(self):
        """How many check walks in a row the learned circuit must predict:
        confirm, or as many as make 16384 steps (at most 64)."""
        return max(self.confirm, min(64, -(-16384 >> self.n1)))

    def _checks(self):
        """Walks to check the learned circuit with: from every state in
        turn, from other start vectors, pin 1 anywhere in the order."""
        n1 = self.n1
        mask, mul = (1 << n1) - 1, survey_mul(n1)
        k = 0
        while True:
            k += 1
            a = k % len(self.S)
            path = self.states[self.S[a]][0]
            start = (k * mul) & mask
            start = start | 1 if path else start & ~1
            yield (a, (k * 3) % n1, k & 1, start,
                   self.AVOID if self.ar and path else self.PLAIN, 0x2000 + k)

    def _event(self, a, rot, rev, start, mode, mix=0):
        path = self.states[self.S[a]][0]
        kw = {}
        if mode == self.AROUND:
            kw = dict(skip=self.ar, avoid=self.ar)
        elif mode == self.AVOID:
            kw = dict(avoid=self.ar)
        la, lb, _ = self.rig.walk(self.Vc, self.base, self.drives(path), gray=1, rot=rot,
                                  rev=rev, start=start, mix=mix, **kw)
        w = _Walk((a, rot, rev, start, mode, mix), la, lb)
        self._let_go(w)
        self._late_outputs(w)
        self._interpret(w)
        self._check_registers(w)
        return w

    def _guess(self, b, x, kb):
        """The state a clock edge leads to that neither a sweep nor a walk
        has shown with these output levels: by the register inputs learned
        so far, else as the sweep went (None if unknown)."""
        if self._dtab is None:
            t = self.nxt[b][x]
            return t - 1 if t else None
        lv = 0
        for j, (c, bit) in enumerate(self.lpos):
            lv |= (kb[c] >> bit & 1) << j
        wn = []
        for i, r in enumerate(sorted(self.reg)):
            t = self._dtab.get((i, b, lv))
            if t is None:
                t = self._dtab[(i, b, lv)] = _b01(self._cov((r, "d"), self.CW[(b, lv)]),
                                                  1 << self.n1)
            wn.append(t[x << 1])
        return self.widx.get(tuple(wn))

    def _late_outputs(self, w):
        """An enumerated pin that the exploration took for an input (never
        driven with pin 1 low) is not at the level it is driven to
        somewhere in this walk: the GAL drives it, with an enable that
        reads pin 1.  It is an output from now on, and the analysis starts
        over."""
        new = []
        for b, p in enumerate(self.Vc):
            if p in self.dev["io"] and p not in self.O and p not in self.reg \
                    and (w.la[p] ^ self.cv1[b]) & self.cfull:
                new.append(p)
        if new:
            for p in new:
                self.late_comb[p] = True
            self.log("%s %s driven in a walk with pin 1 high, and never with pin 1 low: "
                     "taking %s for %s too, and starting again" % (
                         _pins(new), "is" if len(new) == 1 else "are",
                         "it" if len(new) == 1 else "them",
                         "an output" if len(new) == 1 else "outputs"))
            raise _Again()

    def _let_go(self, w):
        """A pin that changes without a clock and is not enumerated must
        be driven throughout: driven the other way in the second read, it
        would be an input that changes with every vector.  If the walk
        shows it undriven, it is enumerated from now on, and the analysis
        starts over."""
        seen = 0
        for m in w.smask.values():
            seen |= m
        if w.order is not None:
            seen = self.cfull if not seen else seen
        new = sorted(p for p in self.hist if p not in self.Vc and p not in self.base
                     and (w.la[p] ^ w.lb[p]) & seen)
        if new:
            self.bidir.update(new)
            self.log("%s %s not always driven: enumerating %s too, and starting again" % (
                _pins(new), "is" if len(new) == 1 else "are",
                "it" if len(new) == 1 else "them"))
            raise _Again()

    def _interpret(self, w):
        """The register state at every vector of the walk.  A clock edge
        with the outputs at the levels the state's sweep had at that x
        goes where the sweep went.  With other levels (a latch stands
        otherwise), it goes where a walk showed it going (self.rise), or
        else, on trust, where _guess says.  Every read checks the
        registered pins that are driven.  One that reads otherwise before
        the next edge corrects an edge taken on trust: that register
        loaded the other value, the edge is kept in self.rise, and the
        walk is followed again from it."""
        a0, rot, rev, start, mode, mix = w.cfg
        n1 = self.n1
        N = 1 << n1
        path = self.states[self.S[a0]][0]
        skipb = None
        if mode == self.AROUND:
            skipb = _b01(cube_set(self.ar, self.Vc, self.base, self.cfull, self.cv1), N)
        vecs = gray_arrays(n1, rot, rev, start, mix)[0]
        if skipb:
            vecs = [xp for xp in vecs if not skipb[xp]]
        key = self._pack(w.la, N)
        nk = range(len(key))
        regs = [(i, r) for i, r in enumerate(sorted(self.reg))
                if r not in self.Vc and r not in self.base]
        drv = [_b01(~(w.la[r] ^ w.lb[r]) & self.cfull, N) for _, r in regs]
        lvl = [_b01(w.la[r], N) for _, r in regs]
        want = [[self.W[s][i] for i, _ in regs] for s in self.S]
        nr = range(len(regs))
        nxt, fire, arb, s0, prek = self.nxt, self.fire, self.arb, self.s0, self.prek
        sid = array("H", bytes(2 * N))
        clean = bytearray(len(vecs))
        fix = {}              # step of an edge -> [state it led to, registers corrected]
        trust = []            # edges taken on trust: [step, state, pins, last_ok, edge]
        w.cut, w.unsure, w.surprises = None, 0, 0
        n = len(vecs)
        i, a = 0, a0
        pxp = (path[-1] << 1 | 1) if path else 0
        last_ok = [-1] * len(regs)
        edge = -1             # the step of the last clock edge
        while i < n:
            xp = vecs[i]
            d = xp ^ pxp
            x = xp >> 1
            single = d & (d - 1) == 0
            if d & xp & 1:                             # pin 1 rises
                if not single:
                    w.cut, n = "pin 1 rises as the inputs change", i
                    break
                b = a
                if arb and fire[b] and arb[x]:
                    b = s0                             # reset before the edge
                if arb and fire[b] and arb[x]:
                    a = b                              # and held in reset
                    del trust[:]
                else:
                    kb = tuple(k[pxp] for k in key)
                    pk = prek[b]
                    if i in fix:
                        a2 = fix[i][0]
                        trust.append([i, a, pxp, list(last_ok), (b, x, kb)])
                    elif pk is None or all(pk[c][x] == kb[c] for c in nk):
                        a2 = nxt[b][x] - 1
                    else:
                        a2 = self.rise.get((b, x, kb))
                        if a2 is None:
                            a2 = self._guess(b, x, kb)
                            fix[i] = [a2, set(), self._dtab is not None]
                            trust.append([i, a, pxp, list(last_ok), (b, x, kb)])
                            w.unsure += 1
                    if a2 is None or a2 < 0:
                        w.cut, n = "no transition seen from state %s with %s" % (
                            _state(self.S[b]), self._inputs(x)), i
                        break
                    if arb and fire[a2] and arb[x]:
                        a2 = s0
                    a = a2
                edge = i
            elif arb and fire[a] and arb[x]:
                a = s0
                del trust[:]                           # the registers are known again
            wa = want[a]
            bad = -1
            for j in nr:
                if drv[j][xp]:
                    if lvl[j][xp] == wa[j]:
                        last_ok[j] = i
                    else:
                        bad = j
                        break
            if bad >= 0:
                cands = [t for t in trust if t[0] > last_ok[bad]]
                if not cands:
                    raise AsyncDesign(
                        "pin %d reads %d at %s in a walk that enumerates pin 1, where the "
                        "transitions lead to state %s: the registers do not follow the "
                        "transitions the sweeps and the walks showed" % (
                            regs[bad][1], lvl[bad][xp], self._cinputs(xp), _state(self.S[a])))
                t = cands[-1]
                f = fix[t[0]]
                wn = list(self.W[self.S[f[0]]]) if f[0] is not None else None
                # every register this read shows, as it shows it
                diff = {j for j in nr if drv[j][xp] and lvl[j][xp] != wa[j]}
                if len(cands) > 1 or t[0] != edge or diff & f[1] or wn is None:
                    # which edge it was cannot be told: leave the walk there
                    w.cut, n = "the registers left the learned transitions", cands[0][0]
                    break
                for j in diff:
                    wn[regs[j][0]] = lvl[j][xp]
                a2 = self.widx.get(tuple(wn))
                if a2 is None:
                    raise AsyncDesign(
                        "clocking %s in from state %s with a latch at another level than in "
                        "the sweeps leads to a state the sweeps did not find (registers at "
                        "%s): galre does not explore states that are reached only that "
                        "way" % (self._inputs(x), _state(self.S[t[4][0]]),
                                 "".join(map(str, wn))))
                if f[2]:
                    w.surprises += 1
                f[0] = a2
                f[1] |= diff
                i, a, pxp, last_ok = t[0], t[1], t[2], list(t[3])
                del trust[trust.index(t):]
                continue
            sid[xp] = a + 1
            clean[i] = 1 if d and single else 0
            pxp = xp
            i += 1
        for k in range(n, len(vecs)):
            sid[vecs[k]] = 0
        # what the walk taught about clock edges
        for step, f in fix.items():
            if step < n and f[1]:
                xp = vecs[step]
                kb = tuple(k[vecs[step - 1] if step else 0] for k in key)
                b = sid[vecs[step - 1]] - 1 if step else a0
                if arb and fire[b] and arb[xp >> 1]:
                    b = s0
                old = self.rise.setdefault((b, xp >> 1, kb), f[0])
                if old != f[0]:
                    raise AsyncDesign(
                        "clocking %s in from state %s led to state %s once and to %s once, "
                        "with the same output levels" % (
                            self._inputs(xp >> 1), _state(self.S[b]), _state(self.S[old]),
                            _state(self.S[f[0]])))
        w.order, w.clean, w.sid = array("I", vecs[:n]), clean[:n], sid
        w.smask = {b - 1: _where(sid, b) for b in set(sid) - {0}}

    def _cinputs(self, xp):
        return " ".join("%s=%d" % (self.name(p), xp >> i & 1) for i, p in enumerate(self.Vc))

    def _check_registers(self, w):
        """A registered pin that is driven reads what the state says."""
        for i, r in enumerate(sorted(self.reg)):
            if r in self.Vc or r in self.base:
                continue
            drv = ~(w.la[r] ^ w.lb[r])
            for a, m in w.smask.items():
                want = self.W[self.S[a]][i]
                bad = (w.la[r] ^ (self.cfull if want else 0)) & drv & m
                if bad:
                    raise AsyncDesign(
                        "pin %d reads %d at %s in a walk that enumerates pin 1, where the "
                        "transitions lead to state %s: the registers do not follow the "
                        "transitions the sweeps showed (a register input that reads a "
                        "latch, or readings that do not repeat)" % (
                            r, 1 - want, self._cinputs(lowest(bad)), _state(self.S[a])))

    # 3. latch pins, contexts, functions ---------------------------------------
    def _contexts(self, w, L):
        """[(state index, latch levels, the vectors of w read in them)]"""
        out = []
        for a, m in w.smask.items():
            groups = [(0, m)]
            for j, q in enumerate(L):
                ng = []
                for lv, g in groups:
                    hi = g & w.la[q]
                    if g ^ hi:
                        ng.append((lv, g ^ hi))
                    if hi:
                        ng.append((lv | 1 << j, hi))
                groups = ng
            out += [(a, lv, g) for lv, g in groups]
        return out

    def _latch_pins(self):
        """Outputs that read differently at the same vector in the same
        state, less those that read as a function of the vector, the
        state and the others (an output of the latches, not one itself)."""
        def function(p, L):
            acc = {}
            for w in self.lwalks:
                for a, lv, m in self._contexts(w, L):
                    f = acc.setdefault((a, lv), [0, 0])
                    f[0] |= w.la[p] & m
                    f[1] |= ~w.la[p] & m
            return all(not on & off for on, off in acc.values())
        lp = [p for p in self.O if not function(p, [])]
        changed = True
        while changed:
            changed = False
            for p in list(lp):
                if function(p, [q for q in lp if q != p]):
                    lp.remove(p)
                    changed = True
        return lp

    def _learn(self):
        """The latch pins, and their excitation from every walk: g[j] over
        (vector, state index, latch levels), 0 unknown, 1 low, 2 high."""
        self.L = self._latch_pins()
        self.L += [q for q in sorted(self.readpins) if q not in self.L and q in self.Vc]
        m, n1 = len(self.L), self.n1
        N = 1 << n1
        self.nn = nn = n1 + max(1, (len(self.S) - 1).bit_length())
        size = 1 << (nn + m)
        g = [bytearray(size) for _ in range(m)]
        self.g, self.hz = g, [{} for _ in range(m)]
        # latch pins that are enumerated: the GAL drives them only at times
        self.B = [q for q in self.L if q in self.Vc]
        self.oecube, self._oebits = {q: self._enable_cube(q) for q in self.B}, {}
        bj = [(self.L.index(q), self.Vc.index(q)) for q in self.B]
        # nothing depends on Brutus's drive of such a pin, only on its level:
        # samples are kept with those drive bits at 0
        canon = ~sum(1 << b for _, b in bj)
        if not m:
            return

        def put(j, i, v):
            old = g[j][i]
            if old == 0:
                g[j][i] = v + 1
            elif old != v + 1:
                raise AsyncDesign(
                    "the excitation of pin %d at %s in state %s with latch pins %s at %s is "
                    "%d once and %d once: not a circuit of registers and latches on those "
                    "pins" % (self.L[j], self._cinputs(i & (N - 1)),
                              _state(self.S[i >> n1 & ((1 << (nn - n1)) - 1)]), self.L,
                              "".join(str(i >> (nn + q) & 1) for q in range(m)), old - 1, v))

        def hold(a, b, lv, moving=()):
            """Neighbouring points at both of which the latches not moving
            are excited to stay as they are."""
            d = a ^ b
            if not d or d & (d - 1):
                return
            var = d.bit_length() - 1
            if n1 <= var < nn:
                return
            lo = a & ~d
            for j in range(m):
                if j not in moving:
                    arr = self.hz[j].get(var)
                    if arr is None:
                        arr = self.hz[j][var] = bytearray(size + 7 >> 3)
                    arr[lo >> 3] |= 1 << (lo & 7)

        multi, late = [], []
        rm = range(m)
        for w in self.lwalks:
            lvl = [_b01(w.la[p], N) for p in self.L]
            und = self._undriven(w, N)        # per vector, the latch pins the GAL lets go
            if w.order is None:
                for a, mask in w.smask.items():
                    seen = _b01(mask, N)
                    for xp in range(0, N, 2):
                        if seen[xp]:
                            lc = 0
                            for j in rm:
                                lc |= lvl[j][xp] << j
                            i = xp & canon | a << n1 | lc << nn
                            u = und[xp] if und else 0
                            for j in rm:
                                if not u >> j & 1:
                                    put(j, i, lc >> j & 1)
                continue
            sid, clean, prev = w.sid, w.clean, None
            for k, xp in enumerate(w.order):
                lc = 0
                for j in rm:
                    lc |= lvl[j][xp] << j
                xx = xp & canon | (sid[xp] - 1) << n1
                i = xx | lc << nn
                u = und[xp] if und else 0
                for j in rm:
                    if not u >> j & 1:
                        put(j, i, lc >> j & 1)
                if prev is not None and clean[k]:
                    pxx, lp, pu, pxp = prev
                    for j, b in bj:
                        if (xp ^ pxp) >> b & 1 and (lc ^ lp) >> j & 1 and not u >> j & 1:
                            raise AsyncDesign(
                                "pin %d follows its drive at %s in state %s, where the "
                                "enable learned for it is on: not one product term of the "
                                "inputs, the registers and the latch pins" % (
                                    self.L[j], self._cinputs(xp), _state(self.S[sid[xp] - 1])))
                    if u & pu:
                        # a pin the GAL drives neither before nor after the
                        # step is an input, at the level it is driven to
                        lp = (lp & ~(u & pu)) | (lc & u & pu)
                    # A pin the GAL lets go in this step moves to its drive
                    # through 1K, after the latches have reacted to the step
                    # with the pin still at its level: lcm is where they
                    # are before it moves.
                    rel = u & ~pu & (lc ^ lp)
                    lcm = lc ^ rel
                    e = xx | lp << nn
                    ch = lcm ^ lp
                    if ch & (ch - 1):
                        multi.append((pxx, xx, lp, lcm, rel, lc))
                    elif rel and ch:
                        # did the latch react before the pin moved, or to
                        # its move?  Decided when every walk is in.
                        late.append((e, xx | (lp ^ rel) << nn, lp, lcm, lc, u))
                    else:
                        for j in rm:
                            if not u >> j & 1:
                                put(j, e, lcm >> j & 1)
                        f1 = (ch.bit_length() - 1,) if ch else ()
                        hold(pxx | lp << nn, e, lp, f1)
                        if ch:
                            hold(e, xx | lcm << nn, lp, f1)
                prev = (xx, lc, u, xp)
        # A step in which the GAL lets go of a pin and a latch flips: the
        # latch reacted to the step with the pin still at its level (at e),
        # or only to the pin's move to its drive (at e2).  Whichever the
        # other samples allow; if both, the first.
        todo = late
        while todo:
            rest = []
            for e, e2, lp, lcm, lc, u in todo:
                drv = [j for j in rm if not u >> j & 1]
                first = all(g[j][e] in (0, (lcm >> j & 1) + 1) for j in drv)
                then = all(g[j][e] in (0, (lp >> j & 1) + 1)
                           and g[j][e2] in (0, (lc >> j & 1) + 1) for j in drv)
                if first and then and todo is late:
                    rest.append((e, e2, lp, lcm, lc, u))       # see what the others say
                elif first:
                    for j in drv:
                        put(j, e, lcm >> j & 1)
                else:
                    for j in drv:
                        put(j, e, lp >> j & 1)
                        put(j, e2, lc >> j & 1)
            todo, late = rest, None
        if multi:
            k = nn + m
            full, v1 = litmasks(k)
            flat = _Flat(nn, self.L)
            proj = [flat._supported(_from_bytes(g[j], _ONES), _from_bytes(g[j], _ZEROS),
                                    k, full, v1) for j in rm]
            for pxx, xx, lp, lcm, rel, lc in multi:
                if not (flat._step(g, proj, xx, lp, lcm, hold, pxx | lp << nn)
                        or rel and flat._step(g, proj, xx, lp ^ rel, lc, hold, 0)):
                    raise AsyncDesign(
                        "latch pins %s change together at %s in state %s, in no order that "
                        "agrees with the other walks" % (
                            [self.L[j] for j in rm if (lp ^ lc) >> j & 1],
                            self._cinputs(xx & (N - 1)), _state(self.S[xx >> n1])))

    def _enable_cube(self, q):
        """The output enable of a latch pin that is also enumerated: one
        product term, as (literals over (pin 1, V), literals over the
        registers and the other latch pins), or None if the GAL never
        drives the pin.
        The GAL drives the pin for sure where it is not at the level
        Brutus drives it to, and does not where the pin follows when the
        walk changes its drive.  The term has to hold all the first: at
        most the literals that are the same in all of them.  Of those it
        keeps as few as keep every read of the second kind out."""
        bq, jq, nreg = self.Vc.index(q), self.L.index(q), len(self.reg)
        m, N = len(self.L), 1 << self.n1
        xs, cs = {}, {}
        free = {}                 # context -> the vectors at which the pin follows
        for w in self.lwalks:
            ctxs = self._contexts(w, self.L)
            for a, lv, mask in ctxs:
                pos = mask & (w.la[q] ^ self.cv1[bq])
                if not pos:
                    continue
                for v in range(self.n1):
                    if v != bq:
                        if pos & self.cv1[v]:
                            xs.setdefault(v, set()).add(1)
                        if pos & ~self.cv1[v]:
                            xs.setdefault(v, set()).add(0)
                cw = self.W[self.S[a]] + tuple(lv >> j & 1 for j in range(m))
                for i, v in enumerate(cw):
                    if i != nreg + jq:
                        cs.setdefault(i, set()).add(v)
            if w.order is None:
                continue
            # steps that change only this pin's drive, with its level following
            lq = _b01(w.la[q], N)
            fol = bytearray(N)
            pxp = None
            for xp in w.order:
                if pxp is not None and xp ^ pxp == 1 << bq and lq[xp] != lq[pxp]:
                    fol[xp] = fol[pxp] = 1
                pxp = xp
            fbits = _from_bytes(fol, bytes.maketrans(b"\x00\x01", b"01"))
            if fbits:
                for a, lv, mask in ctxs:
                    if mask & fbits:
                        free[(a, lv)] = free.get((a, lv), 0) | mask & fbits
        if not xs and not cs:
            return None
        xl = {v: min(vals) for v, vals in xs.items() if len(vals) == 1}
        cl = {i: min(vals) for i, vals in cs.items() if len(vals) == 1}

        def lets_go():
            """Whether the term is true at a read at which the pin follows."""
            bits = self.cfull
            for v, val in xl.items():
                bits &= self.cv1[v] if val else self.cfull ^ self.cv1[v]
            for (a, lv), f in free.items():
                cw = self.W[self.S[a]] + tuple(lv >> j & 1 for j in range(m))
                if f & bits and all(cw[i] == v for i, v in cl.items()):
                    return True
            return False
        if lets_go():
            raise AsyncDesign("pin %d follows its drive where it is driven at other times: "
                              "its enable is not one product term of the inputs, the "
                              "registers and the latch pins" % q)
        for lits in (xl, cl):
            for k in sorted(lits, reverse=True):
                val = lits.pop(k)
                if lets_go():
                    lits[k] = val
        return xl, cl

    def _enabled(self, q, a, lv):
        """The vectors at which the GAL drives the enumerated latch pin q,
        in a state with the latch pins at lv."""
        bits = self._oebits.get((q, a, lv))
        if bits is None:
            cube = self.oecube[q]
            bits = 0
            if cube is not None:
                cw = self.W[self.S[a]] + tuple(lv >> j & 1 for j in range(len(self.L)))
                if all(cw[i] == v for i, v in cube[1].items()):
                    bits = self.cfull
                    for v, val in cube[0].items():
                        bits &= self.cv1[v] if val else self.cfull ^ self.cv1[v]
            self._oebits[(q, a, lv)] = bits
        return bits

    def _undriven(self, w, N):
        """Per vector of the walk, the enumerated latch pins (bit j for
        self.L[j]) that the GAL does not drive; None if there are none."""
        if not self.B:
            return None
        acc = 0
        for a, lv, mask in self._contexts(w, self.L):
            for q in self.B:
                off = mask & ~self._enabled(q, a, lv)
                if off:
                    acc |= int.from_bytes(_b01(off, N), "little") << self.L.index(q)
        return acc.to_bytes(N, "little")

    def _walk_sets(self, w):
        """Per output and context of one walk: where it is driven, where
        not, and where it is driven high and low."""
        vc, cv1, cfull = self.Vc, self.cv1, self.cfull
        for a, lv, m in self._contexts(w, self.L):
            for p in self.opins:
                la = w.la[p]
                if p in self.B:
                    d = m & self._enabled(p, a, lv)
                    und = m ^ d
                elif p in vc:
                    b = vc.index(p)
                    known = m & swap(m, b, cv1) & cfull
                    d = known & ~(la ^ swap(la, b, cv1))
                    und = known ^ d
                else:
                    und = m & (la ^ w.lb[p])
                    d = m ^ und
                yield p, (a, lv), d, und, la & d, d & ~la

    def _collect(self):
        """Every output's value and enable per context, from every walk."""
        self.opins = list(self.O)
        if self.device == "GAL22V10":
            self.opins += [r for r in sorted(self.reg) if r not in self.O]
        self.Fv = {p: {} for p in self.O if p not in self.L}
        self.Foe = {p: {} for p in self.opins}
        for w in self.lwalks:
            for p, ctx, d, und, hi, lo in self._walk_sets(w):
                f = self.Foe[p].setdefault(ctx, [0, 0])
                f[0] |= d
                f[1] |= und
                if p in self.Fv:
                    f = self.Fv[p].setdefault(ctx, [0, 0])
                    f[0] |= hi
                    f[1] |= lo

    def asynchronous(self):
        if not self.mixed:
            return
        if self.clock_in_v:
            raise AsyncDesign("the chip changes state without a clock, with pin 1 held: galre "
                              "walks pin 1 to learn latches in a clocked design")
        self._model()
        if self.n1 < 3:
            raise AsyncDesign("too few inputs to walk in different orders")
        self.lwalks = [self._table_walk(s) for s in self.states]
        for w in self.lwalks:
            self._let_go(w)
        self.log("registers %s; walking pin 1 with the inputs%s" % (
            sorted(self.reg), " for %s" % _pins(sorted(self.hist)) if self.hist else ""))
        for cfg in self._configs():
            self.lwalks.append(self._event(*cfg))
        self._solve()
        good = checks = 0
        need = self._need()
        limit = max(self.max_checks, 3 * need)
        for cfg in self._checks():
            if good >= need or checks >= limit:
                break
            checks += 1
            w = self._event(*cfg)
            miss = self._predict(w)
            if w.surprises:
                miss = "the registers went elsewhere than predicted at %d clock edge%s" % (
                    w.surprises, "" if w.surprises == 1 else "s")
            self.lwalks.append(w)
            if miss is None:
                good += 1
                continue
            good = 0
            self.log("walk from state %s with rot=%d rev=%d start=%x mix=%d: %s; learning "
                     "again from %d walks" % ((_state(self.S[cfg[0]]),) + cfg[1:4] + (cfg[5],)
                                              + (miss, len(self.lwalks) - len(self.S))))
            self._solve()
        if good < need:
            raise AsyncDesign("the model of registers and latches did not predict %d walks in "
                              "a row within %d walks" % (need, limit))
        cut = [w for w in self.lwalks if w.cut]
        latches = [p for p in self.L if p not in self.readpins]
        self.log("latch model: latch pins %s, other outputs %s, from %d walks with pin 1 "
                 "enumerated%s" % (latches, [p for p in self.O if p not in latches],
                                   len(self.lwalks) - len(self.S),
                                   " (%d cut short, the first: %s)" % (len(cut), cut[0].cut)
                                   if cut else ""))
        if self.rise:
            self.log("%d clock edges went another way than in the sweeps, with a latch at "
                     "another level: the register inputs read latch pins" % len(self.rise))

    def _solve(self):
        """Learn the latches and every function from the walks.  The walks
        that took a clock edge on trust are then followed again, with the
        register inputs just learned; if that changes a walk, learn
        again."""
        for _ in range(8):
            self._learn()
            self._collect()
            self.equations()
            self._dtab = {}
            self.lpos = [divmod(self.O.index(q), 8) for q in self.L]
            again = False
            for w in self.lwalks:
                if w.order is not None and w.unsure:
                    old, known = bytes(w.sid), len(self.rise)
                    self._interpret(w)
                    again |= bytes(w.sid) != old or len(self.rise) != known
            if not again:
                return
        raise AsyncDesign("the registers' states in the walks keep changing with what is "
                          "learned from them: not a circuit of registers and latches that "
                          "galre models")

    # 4. equations -------------------------------------------------------------
    def _eq_space(self):
        if not self.mixed:
            return super()._eq_space()
        m = len(self.L)
        self.EV, self.efull, self.ev1 = self.Vc, self.cfull, self.cv1
        self.cvars = sorted(self.reg) + self.L
        self.CW = {(a, lv): self.W[s] + tuple(lv >> j & 1 for j in range(m))
                   for a, s in enumerate(self.S) for lv in range(1 << m)}

    def _dcontexts(self):
        """Clock edges by context: [(state index, latch levels, next state
        index, x)], from the sweeps (with the latch levels each x was
        clocked in with) and from the walks."""
        out = []
        lvs = range(1 << len(self.L))
        for s, m in self.trans.items():
            a, pre = self.sidx[s], self.pre.get(s)
            for s2, xs in m.items():
                if s in self.ar_dc:
                    xs &= ~self.ar_set            # the reset decides there
                if pre is None:
                    groups = [(lv, xs) for lv in lvs]
                else:
                    groups = [(0, xs)]
                    for j, q in enumerate(self.L):
                        ng = []
                        for lv, g in groups:
                            hi = g & pre[q]
                            if g ^ hi:
                                ng.append((lv, g ^ hi))
                            if hi:
                                ng.append((lv | 1 << j, hi))
                        groups = ng
                out += [(a, lv, self.sidx[s2], g) for lv, g in groups if g]
        for (b, x, kb), a2 in self.rise.items():
            lv = 0
            for j, (c, bit) in enumerate(divmod(self.O.index(q), 8) for q in self.L):
                lv |= (kb[c] >> bit & 1) << j
            out.append((b, lv, a2, 1 << x))
        return out

    def functions(self):
        if not self.mixed:
            return super().functions()
        F = {}
        lvs = range(1 << len(self.L))
        edges = self._dcontexts()
        for i, r in enumerate(sorted(self.reg)):
            dd = {}
            for a, lv, a2, xs in edges:
                f = dd.setdefault((a, lv), [0, 0])
                f[0 if self.W[self.S[a2]][i] else 1] |= xs
            # pin 1 is low before the edge
            F[(r, "d")] = {ctx: (_spread(on), _spread(off)) for ctx, (on, off) in dd.items()}
        N = 1 << self.n1
        for p in self.O:
            if p in self.L:
                gj, f = self.g[self.L.index(p)], {}
                for a in range(len(self.S)):
                    for lv in lvs:
                        base = a << self.n1 | lv << self.nn
                        chunk = gj[base:base + N]
                        on, off = _from_bytes(chunk, _ONES), _from_bytes(chunk, _ZEROS)
                        if p in self.B:           # only where the GAL drives it
                            en = self._enabled(p, a, lv)
                            on, off = on & en, off & en
                        if on or off:
                            f[(a, lv)] = (on, off)
                F[(p, "")] = f
            else:
                F[(p, "")] = self.Fv[p]
        for p in self.opins:
            if p in self.B:
                oe = {ctx: (self._enabled(p, *ctx), self.cfull ^ self._enabled(p, *ctx))
                      for ctx in self.CW}
                if any(off for _, off in oe.values()):
                    F[(p, "oe")] = oe
            elif any(und for _, und in self.Foe[p].values()):
                F[(p, "oe")] = self.Foe[p]
        # an enumerated latch pin counts by its level (a context), whoever
        # drives it: nothing depends on Brutus's drive of it as well
        for q in self.B:
            b = self.Vc.index(q)
            for key, f in F.items():
                F[key] = {ctx: (on | swap(on, b, self.cv1) & self.cfull,
                                off | swap(off, b, self.cv1) & self.cfull)
                          for ctx, (on, off) in f.items()}
        return F

    def _levels(self, y):
        if not self.mixed:
            return super()._levels(y)
        return {c: (hi, lo) for c, (hi, lo) in self.Fv.get(y, {}).items()}

    def equations(self):
        if not self.mixed:
            return super().equations()
        try:
            return super().equations()
        except Limit as e:
            raise AsyncDesign("%s, pin 1 and the latch pins %s" % (e, self.L))

    def _hold_pairs(self, j, sup):
        """The observed steps that hold latch j, as (minterm, minterm,
        level) over its support."""
        n1, nn, m = self.n1, self.nn, len(self.L)
        nr = len(self.reg)
        gj = self.g[j]
        pairs = set()
        for var, arr in self.hz[j].items():
            v = var if var < n1 else n1 + nr + var - nn
            if v not in sup:
                continue
            bit = 1 << sup.index(v)
            edges = int.from_bytes(arr, "little")
            for a in range(len(self.S)):
                for lv in range(1 << m):
                    base = a << n1 | lv << nn
                    e = edges >> base & self.cfull
                    if not e:
                        continue
                    w = self.CW[(a, lv)]
                    cell = 0
                    for i, s in enumerate(sup):
                        if s >= n1 and w[s - n1]:
                            cell |= 1 << i
                    chunk = gj[base:base + (1 << n1)]
                    for level, tbl in ((1, _ONES), (0, _ZEROS)):
                        groups = [(cell, e & _from_bytes(chunk, tbl))]
                        for i, s in enumerate(sup):
                            if s >= n1:
                                continue
                            ng = []
                            for c, bits in groups:
                                lo, hi = bits & ~self.cv1[s], bits & self.cv1[s]
                                if lo:
                                    ng.append((c, lo))
                                if hi:
                                    ng.append((c | 1 << i, hi))
                            groups = ng
                        for c, bits in groups:
                            if bits:
                                pairs.add((c & ~bit, c | bit, level))
        return pairs

    def _latch_names(self):
        return [self.name(p) for p in self.L] if self.mixed else []

    def _hazards(self, p, kind, sup, pos, neg, on, off):
        if kind or p not in self.L:
            return pos, neg
        pairs = self._hold_pairs(self.L.index(p), sup)
        k = len(sup)
        full, v1 = litmasks(k)
        hf = LatchAnalysis._hazard_free
        return (hf(pos, [(a, b) for a, b, v in pairs if v], off, k, full, v1),
                hf(neg, [(a, b) for a, b, v in pairs if not v], on, k, full, v1))

    # the learned circuit --------------------------------------------------------
    def _predict(self, w):
        """What the learned circuit gets wrong in walk w, as text (None if
        nothing)."""
        m, N = len(self.L), 1 << self.n1
        if m and w.order is not None and len(w.order):
            lvl = [_b01(w.la[p], N) for p in self.L]
            tabs = {}
            rm = range(m)

            def tab(j, a, lv):
                t = tabs.get((j, a, lv))
                if t is None:
                    t = tabs[(j, a, lv)] = _b01(self._cov((self.L[j], ""), self.CW[(a, lv)]), N)
                return t
            def enabled(q, a, lv):
                t = tabs.get((q, a, lv, "oe"))
                if t is None:
                    t = tabs[(q, a, lv, "oe")] = _b01(self._enabled(q, a, lv), N)
                return t
            bq = [(q, self.L.index(q), self.Vc.index(q)) for q in self.B]
            sid = w.sid
            lv = 0
            for j in rm:
                lv |= lvl[j][w.order[0]] << j
            pxp = w.order[0]
            for xp in w.order:
                a = sid[xp] - 1
                # a pin the GAL does not drive is an input: it follows its drive
                for q, j, b in bq:
                    if (xp ^ pxp) >> b & 1 and (lv >> j ^ xp >> b) & 1 \
                            and not enabled(q, a, lv)[xp]:
                        lv ^= 1 << j
                pxp = xp
                for _ in range(3 * m + 4):
                    free, rel = 0, -1
                    for q, j, b in bq:
                        if not enabled(q, a, lv)[xp]:
                            free |= 1 << j
                            if (lv >> j ^ xp >> b) & 1:
                                rel = j
                    exc = [j for j in rm if not free >> j & 1
                           and tab(j, a, lv)[xp] != lv >> j & 1]
                    if exc:
                        lv ^= 1 << exc[0]         # the latches react first
                    elif rel >= 0:
                        lv ^= 1 << rel            # then a pin let go moves to its drive
                    else:
                        break
                else:
                    return "the latches do not settle at %s" % self._cinputs(xp)
                for j in rm:
                    if lvl[j][xp] != lv >> j & 1:
                        return "pin %d reads %d at %s in state %s, predicted %d" % (
                            self.L[j], lvl[j][xp], self._cinputs(xp), _state(self.S[a]),
                            lv >> j & 1)
        for q in self.B:
            b = self.Vc.index(q)
            for a, lv, mask in self._contexts(w, self.L):
                bad = mask & (w.la[q] ^ self.cv1[b]) & ~self._enabled(q, a, lv)
                if bad:
                    return "pin %d is driven at %s in state %s, outside its enable" % (
                        q, self._cinputs(lowest(bad)), _state(self.S[a]))
        for p, ctx, d, und, hi, lo in self._walk_sets(w):
            cw = self.CW[ctx]
            for kind, on, off in (("oe", d, und), ("", hi, lo)):
                if kind == "" and p in self.L:
                    continue
                if (p, kind) in self.cover:
                    cov = self._cov((p, kind), cw)
                    bad = (on & ~cov) | (off & cov)
                else:
                    bad = und if kind == "oe" else 0       # always driven, as far as seen
                if bad:
                    return "pin %d%s differs at %s in state %s" % (
                        p, " (enable)" if kind else "", self._cinputs(lowest(bad)),
                        _state(self.S[ctx[0]]))
        return None

    def check_equations(self):
        bad = super().check_equations()
        if self.mixed:
            for w in self.lwalks:
                miss = self._predict(w)
                if miss:
                    bad.append(("walk",) + tuple(w.cfg) + (miss,))
        return bad
