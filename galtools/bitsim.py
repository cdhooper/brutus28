"""Bit-parallel evaluation of a decoded GAL: all input vectors at once.

A set of input vectors is a Python int used as a bitset, one bit per vector
x.  For n enumerated pins, litmasks(n) gives the bitset of the vectors in
which enumerated pin b is 1.  Bits.settle() evaluates the GAL for a fixed
register state and returns per-pin bitsets.
"""


def litmasks(n):
    """(full, v1): the bitset of all 2^n vectors, and per variable b the
    bitset of the vectors with bit b set."""
    N = 1 << n
    full = (1 << N) - 1
    v1 = []
    for b in range(n):
        size = 1 << b
        block = ((1 << size) - 1) << size
        span = 2 * size
        while span < N:
            block |= block << span
            span *= 2
        v1.append(block & full)
    return full, v1


class Bits:
    """Evaluate a galsim.GAL for every vector of a bitset at once."""

    def __init__(self, g):
        self.g = g
        self.rowcols = []
        for m in g.rows:
            if m is None:
                self.rowcols.append(None)
            else:
                self.rowcols.append([c for c in range(g.ncols) if m >> c & 1])

    def _row(self, r, col, full):
        cs = self.rowcols[r]
        if cs is None:
            return 0
        v = full
        for c in cs:
            v &= col[c]
            if not v:
                break
        return v

    def settle(self, q, ext, full):
        """q: pin -> 0/1 register state; ext: pin -> bitset of the external
        (weak) drive.  Returns (level, driven, nextq, ar) bitsets, where
        nextq is the register state after a rising clock edge."""
        g = self.g
        level = dict(ext)
        qb = {p: (full if v else 0) for p, v in q.items()}
        ar = 0
        for _ in range(60):
            col = []
            for kind, n in g.cols:
                if kind == "fb" and g._olmc(n).registered:
                    v = full ^ qb[n]
                else:
                    v = level[n]
                col.append(v)
                col.append(full ^ v)
            drv, val = {}, {}
            for o in g.olmcs:
                if o.mode == "input":
                    continue
                if o.mode == "reg" and g.kind == "GAL16V8":
                    en = full ^ level[11]
                    v = full ^ qb[o.pin]
                elif o.mode == "reg":
                    en = self._row(o.oe_row, col, full)
                    v = qb[o.pin] if o.active_high else full ^ qb[o.pin]
                else:
                    if o.oe_row == "always":
                        en = full
                    elif o.oe_row == "never":
                        en = 0
                    else:
                        en = self._row(o.oe_row, col, full)
                    s = 0
                    for r in o.rows:
                        s |= self._row(r, col, full)
                    v = s if o.active_high else full ^ s
                drv[o.pin], val[o.pin] = en, v
            if g.ar_row is not None:
                a = self._row(g.ar_row, col, full)
                if a & ~ar:
                    ar |= a
                    for p in qb:
                        qb[p] &= full ^ ar
                    continue
            new = dict(ext)
            for p, en in drv.items():
                new[p] = (en & val[p]) | ((full ^ en) & ext[p])
            if new == level:
                break
            level = new
        else:
            raise RuntimeError("bit-parallel settle does not converge "
                               "(combinational loop)")
        nextq = {}
        sp = self._row(g.sp_row, col, full) if g.sp_row is not None else 0
        for o in g.olmcs:
            if not o.registered:
                continue
            s = 0
            for r in o.rows:
                s |= self._row(r, col, full)
            if g.kind == "GAL16V8":
                nextq[o.pin] = (full ^ s) if o.active_high else s
            else:
                nextq[o.pin] = (sp | ((full ^ sp) & s)) & (full ^ ar)
        driven = {p: drv.get(p, 0) for p in g.signal_pins()}
        return level, driven, nextq, ar
