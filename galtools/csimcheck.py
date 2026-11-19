"""A second opinion on a result: CUPL's simulator on the recovered source.

galre checks the JED it built with its own simulator (galsim.py), which
reads the fuse map.  csim, the simulator that comes with CUPL, works from
the compiled source instead (the .abs file), with the device library's
models of the macrocells.  check() writes test vectors for csim (NAME.si)
from what galsim says the JED does under random stimulus, runs csim on
them, and reports where the two disagree.

The stimulus changes one pin per vector, or pulses the clock.  A pin the
GAL drives is expected at its level (H, L); one it lets go is driven (0,
1), or expected open (Z) if nothing reads it.

csim knows three levels: a latch that nothing has set since power-up is
unknown, and so is what depends on it, also a register that loaded from
it.  galsim starts every pin low.  So the vectors come from several
chips whose combinational outputs power up differently; where they
differ, the level is expected unknown (X), and csim's "unstable output"
there is no disagreement.
"""

import os
import random
import re
import subprocess

from galsim import GAL

CHIPS = 10          # power-up states tried for the unknown levels


def vectors(pld, jed, steps=400, seed=3):
    """Write the .si file next to pld; returns the vectors (strings)."""
    src = open(pld).read()
    head = src[:src.index("Device")] + re.search(r"Device\s+\S+;", src).group(0) + "\n"
    pins = {int(m.group(1)): (bool(m.group(2)), m.group(3))
            for m in re.finditer(r"^Pin\s+(\d+)\s*=\s*(!?)(\w+)\s*;", src, re.M)}
    order = sorted(pins)
    clocked = pins.get(1, (False, ""))[1] == "CLK"
    rnd = random.Random(seed)
    chips = []
    for k in range(CHIPS):
        g = GAL(jed)
        sig = g.signal_pins()
        g.power_up()
        if k:
            # the combinational outputs come up at other levels
            outs = [o.pin for o in g.olmcs if o.mode == "comb"]
            g.levels = {p: 0 for p in sig}
            for p in outs:
                g.levels[p] = 1 if k == 1 else rnd.randint(0, 1)
            g.driven = set(outs)
        g.settle({p: 0 for p in sig})
        chips.append(g)
    g = chips[0]
    io = {o.pin for o in g.olmcs}
    read = set()                    # pins that some product term reads
    for o in g.olmcs:
        rows = list(o.live)
        if isinstance(o.oe_row, int) and g.rows[o.oe_row] is not None:
            rows.append(g.rows[o.oe_row])
        for m in rows:
            for k, (kind, n) in enumerate(g.cols):
                if (m >> (2 * k)) & 3 and not (kind == "fb" and g._olmc(n).registered):
                    read.add(n)
    ext = {p: 0 for p in sig}
    free = [p for p in order if not (p == 1 and clocked)]
    lines = []
    for step in range(steps):
        ext = dict(ext)
        clk = "0"
        if step == 0:
            pass        # every pin low, as the chips above were powered up: csim starts unknown
        elif clocked and rnd.random() < 0.35:
            clk = "C"                               # one clock pulse
        else:
            p = rnd.choice(free)
            ext[p] = 1 - ext[p]
        res = []
        for c in chips:
            if clk == "C":
                c.apply({**ext, 1: 1})
            res.append(c.apply({**ext, 1: 0} if clocked else ext) + (set(c.unstable),))
        lv, drv, _ = res[0]
        row = []
        for p in order:
            inv = pins[p][0]
            if p == 1 and clocked:
                row.append(clk)
            elif any(p in r[1] for r in res):
                same = all(r[0][p] == lv[p] and (p in r[1]) == (p in drv) and p not in r[2]
                           for r in res)
                row.append("HL"[(1 - lv[p]) ^ inv] if same else "X")
            elif p in io and p not in read:
                row.append("Z")
            else:
                row.append(str(ext[p] ^ inv))
        lines.append("".join(row))
    with open(os.path.splitext(pld)[0] + ".si", "w") as f:
        f.write("%s\nORDER: %s;\n\nVECTORS:\n%s\n" % (
            head, ", ".join(pins[p][1] for p in order), "\n".join(lines)))
    return lines


def check(csim, pld, jed, steps=400):
    """Run csim on pld's compiled design (pld's .abs must exist) with
    vectors from jed.  Returns (vectors, vectors with an unknown level,
    [(vector number, csim's message)] for the disagreements), or None if
    csim did not run."""
    lines = vectors(pld, jed, steps)
    d, name = os.path.split(os.path.splitext(pld)[0])
    so = os.path.join(d, name + ".so")
    if os.path.exists(so):
        os.remove(so)
    subprocess.run([csim, "-l", name], cwd=d or ".", capture_output=True, text=True)
    if not os.path.exists(so):
        return None
    unknown = {i + 1 for i, v in enumerate(lines) if "X" in v}
    bad, vec, unstable = [], None, set()
    for line in open(so, errors="replace"):
        m = re.match(r"(\d{4}): ", line)
        if m:
            vec = int(m.group(1))
        m = re.match(r"\s*\[(\d+)\w+\]\s*(.*)", line)
        if m and vec is not None:
            if m.group(1) == "0020":
                unstable.add(vec)              # csim: a level is unknown here
            else:
                bad.append((vec, m.group(2).strip()))
    # where csim calls a level unknown, the other complaints follow from it
    bad = [(v, msg) for v, msg in bad if v not in unstable and v not in unknown]
    bad += [(v, "csim: unstable output") for v in sorted(unstable - unknown)]
    return len(lines), len(unknown | unstable), sorted(bad)


def report(csim, pld, jed, steps=400):
    """One line for galre's report."""
    r = check(csim, pld, jed, steps)
    if r is None:
        return "csim did not run on %s" % os.path.basename(pld)
    n, unknown, bad = r
    if not bad:
        return "csim on the source, %d random vectors from the JED: agrees%s" % (
            n, " (in %d of them a level is unknown since power-up)" % unknown if unknown else "")
    return "csim on the source, %d random vectors from the JED: differs in %d, the first " \
           "vector %d (%s)" % (n, len({v for v, _ in bad}), bad[0][0], bad[0][1])
