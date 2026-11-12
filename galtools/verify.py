"""Check a rebuilt JED against the captures, and compare two JEDs."""

import random

from analysis import Table
from galsim import GAL
from rig import SimRig


def check_captures(a, jed):
    """Simulate jed through the same walks and transition sweeps as the
    analysis: every reachable state, every input vector.  Returns a list
    of differences (empty when the rebuilt JED matches the captures)."""
    rig = SimRig(jed)
    bad = []
    for s, (path, t) in a.states.items():
        t2 = Table(a.V, *rig.walk(a.V, a.base, a.drives(path)), a.full, a.v1)
        for p in a.pins:
            if p == 1 and not a.clock_in_v:
                continue
            if (t.driven[p] != t2.driven[p]
                    or (t.value[p] & t.driven[p]) != (t2.value[p] & t2.driven[p])):
                bad.append(("pin %d" % p, s))
        if s in a.trans and not a.clock_in_v and a.probe_x:
            obs = rig.transitions(a.V, a.base, a.drives(path), a.probes(), a.cand)
            m = {}
            for o, xs in obs.items():
                s2 = a._sig_obs(o)
                m[s2] = m.get(s2, 0) | xs
            if m != a.trans[s]:
                bad.append(("transitions", s))
    return bad


def compare_jeds(a, b, steps=30000, seed=1, fixed=None):
    """Random stimulus on every signal pin (weakly driven, clock edges on
    pin 1); per step, compare pin levels and which pins each chip drives.
    Returns ({pin: differing steps}, first difference or None, oscillating
    steps of a and b)."""
    ga, gb = GAL(a), GAL(b)
    pins = ga.signal_pins()
    rnd = random.Random(seed)
    ext = {p: 0 for p in pins}
    ga.settle(ext)
    gb.settle(ext)
    diffs, first, osc = {}, None, [0, 0]
    for i in range(steps):
        if rnd.random() < 0.5:
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
        for p in pins:
            va = "X" if p in ga.unstable else ("Z" if p not in da else la[p])
            vb = "X" if p in gb.unstable else ("Z" if p not in db else lb[p])
            if va != vb:
                diffs[p] = diffs.get(p, 0) + 1
                if first is None:
                    first = (i, p, va, vb)
    return diffs, first, osc
