"""Check a rebuilt JED against the captures, and compare two JEDs."""

import random

from analysis import Table
from latch import _to_bytes
from galsim import GAL
from rig import SimRig


def check_captures(a, jed, notes=None):
    """Simulate jed through the same walks and transition sweeps as the
    analysis: every reachable state, every input vector.  With an
    asynchronous reset term, the jobs go around it as the analysis did,
    and it must fire in the same states.  A latch design (LatchAnalysis)
    goes through the same walks, compared from the first read where both
    agree (the latches' state at power-up is a race the rebuilt chip may
    settle differently), and a walk that starts differently is added to
    notes.  Returns a list of differences (empty when the rebuilt JED
    matches the captures)."""
    if hasattr(a, "walks"):
        return _check_walks(a, jed, notes)
    rig = SimRig(jed, careful=True)
    bad = []
    hist = sorted(a.hist)             # latches: read in the walks with pin 1 instead
    for s, (path, t) in a.states.items():
        drives = a.drives(path)
        active = s in a.ar_on
        if a.ar and path and a.probe_x:
            _, _, sigs = rig.walk(a.V, a.base, drives, probes=a.probes(), cand=a.cand,
                                  avoid=a.ar, data=False)
            if (sigs[0] != sigs[1]) != active:
                bad.append(("asynchronous reset", s))
        la, lb, _ = rig.walk(a.V, a.base, drives, skip=a.ar if active else None,
                             avoid=a.ar if path else None, **a._wctx())
        t2 = Table(a.V, la, lb, a.full, a.v1, a.full ^ a.ar_set if active else None)
        for p in a.pins:
            if (p == 1 and not a.clock_in_v) or p in hist:
                continue
            if (t.driven[p] != t2.driven[p]
                    or (t.value[p] & t.driven[p]) != (t2.value[p] & t2.driven[p])):
                bad.append(("pin %d" % p, s))
        if s in a.trans and not a.clock_in_v and a.probe_x:
            res = rig.transitions(a.V, a.base, drives, a.probes(), a.cand, avoid=a.ar,
                                  pre=a.mixed, **a._tctx())
            m = {}
            for o, xs in res[0].items():
                s2 = a._reg(a._sig_obs(o))
                m[s2] = m.get(s2, 0) | xs
            if not a.mixed:
                if m != a.trans[s]:
                    bad.append(("transitions", s))
            elif s in a.pre:
                # where a clock edge goes may depend on the latches, and the
                # sweep here need not leave them as the captured one did:
                # compare the x clocked in with the outputs at the same levels
                same = a.full
                for p in a.O:
                    same &= ~(res[2][p] ^ a.pre[s][p])
                if any((m.get(s2, 0) ^ a.trans[s].get(s2, 0)) & same
                       for s2 in set(m) | set(a.trans[s])):
                    bad.append(("transitions", s))
    if a.mixed:
        bad += _check_events(a, rig, notes)
    return bad


def _check_events(a, rig, notes=None):
    """Registers and latches: jed through the analysis's walks with pin 1
    enumerated, from the first read where every pin agrees (the latches
    may come out of power-up and the path differently)."""
    from reglatch import _b01
    bad = []
    N = 1 << a.n1
    for w in a.lwalks:
        if w.order is None:
            continue
        s, rot, rev, start, mode, mix = w.cfg
        path = a.states[a.S[s]][0]
        kw = {}
        if mode == a.AROUND:
            kw = dict(skip=a.ar, avoid=a.ar)
        elif mode == a.AVOID:
            kw = dict(avoid=a.ar)
        kw.update(a._wctx())
        la, lb, _ = rig.walk(a.Vc, a.base, a.drives(path), gray=1, rot=rot, rev=rev,
                             start=start, mix=mix, **kw)
        name = "walk from state %s rot=%d rev=%d start=%x mix=%d" % (
            "".join(map(str, a.S[s])), rot, rev, start, mix)
        diff = 0
        for p in la:
            diff |= (la[p] ^ w.la[p]) | (lb[p] ^ w.lb[p])
        if not diff:
            continue
        db = _b01(diff, N)
        order = [xp for xp in w.order]
        first = next((k for k, xp in enumerate(order) if not db[xp]), None)
        if first is None:
            bad.append((name, "never agrees"))
            continue
        later = [xp for xp in order[first:] if db[xp]]
        if later:
            pins = sorted(p for p in la if ((la[p] ^ w.la[p]) | (lb[p] ^ w.lb[p])) >> later[0] & 1)
            bad.append((name, "pins %s at %s" % (pins, a._cinputs(later[0]))))
        elif notes is not None:
            notes.append("%s: the rebuilt chip agrees from read %d" % (name, first))
    return bad


def _check_walks(a, jed, notes=None):
    """A latch design: jed through the analysis's walks, from the first
    read where all outputs agree."""
    rig = SimRig(jed)
    bad = []
    for w in a.walks:
        rot, rev, start, mix = w.cfg
        la, _, _ = rig.walk(a.V, a.wbase, [], gray=1, rot=rot, rev=rev, start=start, mix=mix)
        got = {p: _to_bytes(la[p], a.n) for p in a.outputs}
        name = "walk rot=%d rev=%d start=%x mix=%d" % w.cfg
        same = [all(got[p][x] == w.levels[p][x] for p in a.outputs) for x in w.xs]
        if True not in same:
            bad.append((name, "never agrees"))
            continue
        k = same.index(True)
        if k and notes is not None:
            notes.append("%s: the rebuilt chip powers up with %s, agrees from read %d" % (
                name, ", ".join("pin %d at %d" % (p, got[p][w.xs[0]]) for p in a.outputs
                                if got[p][w.xs[0]] != w.levels[p][w.xs[0]]), k))
        for p in a.outputs:
            if any(got[p][x] != w.levels[p][x] for x in w.xs[k:]):
                bad.append((name, "pin %d" % p))
    return bad


def compare_jeds(a, b, steps=30000, seed=1, fixed=None, sic=False):
    """Random stimulus on every signal pin (weakly driven, clock edges on
    pin 1); per step, compare pin levels and which pins each chip drives.
    With sic, each step changes one pin, as latches need, and the chips
    are compared from the first step where they agree (latches may power
    up differently).  Returns ({pin: differing steps}, first difference or
    None, oscillating steps of a and b)."""
    ga, gb = GAL(a), GAL(b)
    pins = ga.signal_pins()
    free = [p for p in pins if p not in (fixed or {})]
    rnd = random.Random(seed)
    ext = {p: 0 for p in pins}
    ext.update(fixed or {})
    ga.settle(ext)
    gb.settle(ext)
    diffs, first, osc = {}, None, [0, 0]
    synced = not sic
    for i in range(steps):
        if sic:
            ext = dict(ext)
            p = rnd.choice(free)
            ext[p] = 1 - ext[p]
        elif rnd.random() < 0.5:
            ext = dict(ext)
            ext[1] = 1 - ext[1]
        else:
            ext = {p: rnd.randint(0, 1) for p in pins}
            ext[1] = 0
        ext.update(fixed or {})
        la, da = ga.apply(ext)
        lb, db = gb.apply(ext)
        osc[0] += bool(ga.unstable)
        osc[1] += bool(gb.unstable)
        step = {}
        for p in pins:
            va = "X" if p in ga.unstable else ("Z" if p not in da else la[p])
            vb = "X" if p in gb.unstable else ("Z" if p not in db else lb[p])
            if va != vb:
                step[p] = (va, vb)
        if not synced and not step:
            synced = True
            diffs, first = {}, None
        for p, (va, vb) in step.items():
            diffs[p] = diffs.get(p, 0) + 1
            if first is None:
                first = (i, p, va, vb)
    return diffs, first, osc
