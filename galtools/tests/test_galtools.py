"""galtools tests: python3 -m unittest discover -s galtools/tests

The firmware tests compile fw/capture.c for the host (needs a C compiler)
and run the analysis through it; the CUPL round trips need a CUPL compiler
($CUPL or cupl on the PATH).  Both are skipped without them.
"""

import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import fwsim                                       # noqa: E402
import galre                                       # noqa: E402
import verify                                      # noqa: E402
from analysis import DEV, Analysis, AsyncDesign, LatchSuspected, Limit   # noqa: E402
from capture import CaptureError                   # noqa: E402
from latch import LatchAnalysis                    # noqa: E402
from galsim import GAL                             # noqa: E402
from reglatch import RegLatchAnalysis               # noqa: E402
from rig import (BrutusRig, RecordingRig, ReplayRig, SimRig, gray_order,  # noqa: E402
                 walk_x)

DESIGNS = os.path.join(HERE, "..", "designs")
CUPL = os.environ.get("CUPL") or shutil.which("cupl")

# design -> (device, expected states, registered pins, pins held for the
# firmware runs so the scalar simulator behind the harness stays quick)
CASES = {
    "cnt16v8": ("GAL16V8", 4, [18, 19], {6: 0, 7: 0, 8: 0, 9: 0, 12: 0, 13: 0, 16: 0}),
    "fsm22v10": ("GAL22V10", 3, [21, 22, 23],
                 {6: 0, 7: 0, 8: 0, 9: 0, 10: 0, 11: 0, 15: 0, 16: 0, 17: 0, 18: 0}),
    "comb16v8": ("GAL16V8", 1, [], {4: 0, 5: 0, 6: 0, 7: 0, 8: 0, 9: 0, 11: 0,
                                    12: 0, 13: 0, 14: 0, 15: 0, 16: 0}),
    # GAL20V8 in its three modes: registered (pin 13 is the enable of the
    # registers), complex and simple
    "cnt20v8": ("GAL20V8", 4, [21, 22], {p: 0 for p in (5, 6, 7, 8, 9, 10, 11, 15, 16, 17, 18)}),
    "comb20v8": ("GAL20V8", 1, [], {p: 0 for p in (6, 7, 8, 9, 10, 11, 16, 17, 18, 19)}),
    "simple20v8": ("GAL20V8", 1, [], {p: 0 for p in (3, 4, 5, 6, 7, 8, 9, 10, 11, 16, 17, 21)}),
}

# designs with state that changes without a clock that galre does not
# model -> (device, pins held, what galre must say when it stops)
ASYNC_CASES = {
    # an output that oscillates and a latch in one chip: where the pins
    # settle, their levels depend on the walk order
    "oscstate16v8": ("GAL16V8", "6-9=0,11-16=0,19=0",
                     "but high in one walk order and low in another: the chip holds state "
                     "as well as oscillating, which galre does not model"),
    "buried22v10": ("GAL22V10", "5-11=0,13-21=0",
                    "led to state 1Z, but the same clocks from power-up lead to 0Z: the chip "
                    "has state that no pin shows, or its readings do not repeat"),
}

# an output that feeds back inverted and oscillates (the simulated chip
# then reads arbitrary levels, as a real one would), and one that follows
# it: the pins held, and what galre must infer
OSC = ("5-9=0,11-16=0,19=0", {
    17: "P4 & P18",
    18: "!P3 & !P18\n    # !P2 & !P3"})

# designs with registers and latches -> (device, pins held, pins that change
# without a clock, latch pins, registers, states, the equations galre must
# find).  reglatch22v10's latch reads pin 1 and the registers, and shows
# only after two clocks; its output pin 20 has an enable; its reset term
# lets go of the latch.  reglatch16v8's latch shows in the power-up walk,
# so galre first tries latches alone.  In latchd16v8 the register input
# reads the latch.  trilatch22v10's latch is on a bidirectional pin (22):
# the GAL drives it only while pin 4 is high, and an output and the
# register input read its level, whoever drives it.  oeread22v10 has no
# latch and no register: pin 23 is an output with an enable, and the
# enable of pin 22 reads its level where the GAL lets it go (the analysis
# keeps pin 23 with the latch pins, a.L, so that the equations read its
# level; it holds no state, so it is not in the fourth column, a.Lh).
MIXED_CASES = {
    "reglatch22v10": ("GAL22V10", "7-11=0,13-19=0", [20, 21], [21], [22, 23], 4, {
        (20, ""): "P6 & P21",
        (20, "oe"): "P4",
        (21, ""): "CLK & !P3 & P4\n    # CLK & P3 & P22\n    # P3 & P22 & P21"
                  "\n    # CLK & P4 & P22",
        (22, "d"): "P23",
        (23, "d"): "P2 & !P22"}),
    "reglatch16v8": ("GAL16V8", "5-9=0,11=0,12-16=0,19=0", [17], [17], [18], 2, {
        (17, ""): "P2 & !P3\n    # !P3 & P17",
        (18, "d"): "P4"}),
    "latchd16v8": ("GAL16V8", "5-9=0,11=0,12-16=0,19=0", [17], [17], [18], 2, {
        (17, ""): "P2 & !P3\n    # !P3 & P17",
        (18, "d"): "P4 & P17"}),
    "trilatch22v10": ("GAL22V10", "7-11=0,13-20=0", [21, 22], [22], [23], 2, {
        (21, ""): "P5 & P22",
        (22, ""): "P2 & !P3\n    # !P3 & P22",
        (22, "oe"): "P4",
        (23, "d"): "P6 & P22"}),
    # three latches in a chain: one input change sets off all three, and
    # the chip is seen only before and after.  What comes out holds each
    # latch by the one before it, which reads the same wherever the chip
    # comes to rest as the original's "until R"
    "chain16v8": ("GAL16V8", "8-9=0,11-14=0,19=0", [15, 16, 17, 18], [16, 17, 18], [], 1, {
        (15, ""): "P7 & P16",
        (15, "oe"): "P6",
        (16, ""): "P5 & P17\n    # P16 & P17",
        (17, ""): "P4 & P18\n    # P17 & P18",
        (18, ""): "P2 & !P3\n    # !P3 & P18"}),
    # no registers: pin 23 is a latch that is open while pin 1 is high.
    # Clocked, it looks like a register; in the walks with pin 1 it does
    # not follow the clock edges, and is taken for a latch that reads
    # pin 1
    "clklatch22v10": ("GAL22V10", "7-11=0,13-20=0", [22, 23], [22, 23], [], 1, {
        (21, ""): "P5 & P23",
        (21, "oe"): "P6",
        (22, ""): "P3 & !P4\n    # !P4 & P22",
        (23, ""): "P1 & P2\n    # !P1 & P23\n    # P2 & P23"}),
    "oeread22v10": ("GAL22V10", "4-11=0,13-20=0", [], [], [], 1, {
        (21, ""): "P3 & P23",
        (22, ""): "'b'0",
        (22, "oe"): "!P23",
        (23, ""): "'b'0",
        (23, "oe"): "P2"}),
}

# latches A and B that set together, and C that follows them through
# A & B: the pins held, and the terms galre must find for each pin
LATCHFOLLOW = ("12-14=0,19=0", {
    (15, ""): {"P16 & P18"},
    (15, "oe"): {"P4"},
    (16, ""): {"P5 & P6", "P5 & P7", "P5 & P8", "P5 & P9", "P17 & P18", "P11 & P16"},
    (17, ""): {"P1 & P2", "P1 & P3", "P8 & P9", "P4 & P17"},
    (18, ""): {"P1 & P2", "P1 & P3", "P6 & P7", "P4 & P18"}})

# an edge detector made from an output's delay: the pins held, and what
# galre must find
EDGELATCH = ("7-9=0,11-14=0,19=0", {
    (15, ""): "P5",
    (15, "oe"): "!P6",
    (16, ""): "P4",
    (17, ""): "P2 & !P3 & !P4 & P16\n    # P2 & !P16 & P17\n    # P2 & P4 & P17",
    (18, ""): "P3 & !P16 & P17\n    # P3 & P4 & P17"})

# the same with the delay of two outputs, which a real GAL16V8 takes (the
# one above it does not): the pins held, and what galre finds, in the
# simulator and on the chip.  Pin 14 copies pin 16 in the original; settled
# levels do not show which of the two is later, and galre writes pin 16,
# which its term reads, as following pin 14.
EDGE2LATCH = ("7-9=0,11-13=0,19=0", {
    (14, ""): "P4",
    (15, ""): "P5",
    (15, "oe"): "!P6",
    (16, ""): "P14",
    (17, ""): "P2 & !P3 & !P4 & P16\n    # P2 & !P16 & P17\n    # P2 & P4 & P17"
              "\n    # P2 & !P3 & P17",
    (18, ""): "P3 & !P16 & P17\n    # P3 & P4 & P17"})

# a register input that reads a latch which must have been set on the way
# to the state: the pins held, and what galre must find
ROUTELATCH = ("5-11=0,13-20=0", {
    (21, ""): "P2 & !P3\n    # !P3 & P21",
    (22, "d"): "!P2 & !P3 & !P4 & P23 & P21\n    # !P3 & P22",
    (23, "d"): "P4"})

# a registered GAL16V8 with a latch that the register input reads: the
# pins held, and what galre must find.  Pin 11 is not held: while it is
# high no register shows on its pin
OELATCH = ("6-9=0,12=0,14=0", {
    (15, "d"): "!'b'0",
    (16, ""): "P18",
    (16, "oe"): "!P2 & P4",
    (17, ""): "P3 & P17\n    # P2",
    (18, "d"): "!P17",
    (19, ""): "P5 & P13"})

# designs of latches built from combinational feedback -> (device, pins
# held, latch pins, other outputs, the equations galre must find).
# latch22v10's latches feed each other, so one input change can flip all
# three in turn, and pin 20 is a function of two of them; dlat22v10's two
# latches flip together; srnor16v8's cross-coupled pair leaves one latch,
# and its state at power-up is a race (the only design whose rebuilt chip
# starts differently).
LATCH_CASES = {
    "latch16v8": ("GAL16V8", "7-9=0,11-15=0,19=0", [17, 18], [16], {
        16: "!P4 & P6",
        17: "P4 & P5\n    # !P5 & P17\n    # P4 & P17",
        18: "P2 & !P3\n    # !P3 & P18"}),
    "latch22v10": ("GAL22V10", "7-11=0,13-19=0", [21, 22, 23], [20], {
        20: "!P22 & P23",
        21: "!P6 & P21\n    # P6 & P22\n    # P21 & P22",
        22: "!P5 & P22\n    # P4 & P23",
        23: "P2 & P3\n    # P2 & P23\n    # P3 & P23"}),
    "dlat22v10": ("GAL22V10", "5-11=0,13=0,17-23=0", [14, 15], [16], {
        14: "!P2 & P4\n    # !P4 & P14\n    # !P2 & P14",
        15: "P3 & P4\n    # !P4 & P15\n    # P3 & P15",
        16: "!P14 & P15"}),
    "quietlatch16v8": ("GAL16V8", "2=0,6-9=0,11-16=0,19=0", [18], [17], {
        17: "!!P3 & !P5",
        18: "!!P3 & !P5 & !P18\n    # !P4"}),
    "srnor16v8": ("GAL16V8", "6-9=0,11-14=0,19=0", [17, 18], [15, 16], {
        15: "P4 & !P17\n    # !P4 & P17",
        16: "!P2 & !P17",
        17: "P2 & !P3\n    # !P3 & P17",
        18: "P4 & P5\n    # P18"}),
}

# GAL22V10 designs with an asynchronous reset term -> (pins held, the term
# galre must find, states in which it fires).  arlow22v10's reset input is
# active low, so it is true in the probe vectors galre would pick, and its
# register outputs are active low (high at power-up and after the reset);
# arw22v10's term also reads a register.
AR_CASES = {
    "ar22v10": ("6-11=0,13-20=0", "P3 & !P4", 3),
    "arlow22v10": ("4=0,6-11=0,13-20=0", "!P3", 3),
    "arw22v10": ("4=0,6-11=0,13-20=0", "P3 & P22", 2),
    # the term reads an I/O pin that is used as an input (16)
    "aropen22v10": ("5-11=0,13-15=0,17-18=0,20-21=0", "!P16", 3),
}


def jed(name):
    return os.path.join(DESIGNS, name + ".jed")


def quiet(*_):
    pass


class OddReads:
    """A rig with the reads of one pin changed.  With where (pin levels),
    the pin is high in share of the reads at those inputs: a pin that
    toggles and is seldom high.  Without, share of all its reads are
    wrong: a bad contact."""

    def __init__(self, rig, pin, share, where=None):
        import random
        self.rig, self.pin, self.share, self.where = rig, pin, share, where
        self.rnd = random.Random(5)
        self.pins, self.device, self.ops = rig.pins, rig.device, rig.ops

    def __getattr__(self, name):
        return getattr(self.rig, name)

    def walk(self, V, base, path, **kw):
        la, lb, sigs = self.rig.walk(V, base, path, **kw)
        if la is None:
            return la, lb, sigs
        la, lb = dict(la), dict(lb)
        for x in range(1 << len(V)):
            if self.where is not None and any(
                    (x >> V.index(p) & 1 if p in V else base.get(p, 0)) != v
                    for p, v in self.where.items()):
                continue
            for reads in (la, lb):
                if self.where is None:
                    reads[self.pin] ^= (self.rnd.random() < self.share) << x
                else:
                    reads[self.pin] = (reads[self.pin] & ~(1 << x)
                                       | (self.rnd.random() < self.share) << x)
        return la, lb, sigs


class SimulatedChip(unittest.TestCase):
    def test_designs(self):
        for name, (device, nstates, regs, _) in CASES.items():
            with self.subTest(design=name):
                rig = SimRig(jed(name))
                a = Analysis(rig, device, log=quiet)
                a.run()
                self.assertEqual(len(a.states), nstates)
                self.assertEqual(sorted(a.reg), regs)
                self.assertEqual(a.check_equations(), [])

    def test_tables_match_chip(self):
        """Every walk table the analysis keeps, against the simulator's own
        levels and driven pins; also with held pins the design uses (pin 5
        is B in fsm22v10, pin 2 is B in comb16v8)."""
        cases = [(name, device, fixed) for name, (device, _, _, fixed) in CASES.items()]
        cases += [("fsm22v10", "GAL22V10", {5: 1}), ("comb16v8", "GAL16V8", {2: 1, 9: 0})]
        for name, device, fixed in cases:
            with self.subTest(design=name, fixed=fixed):
                a = Analysis(SimRig(jed(name)), device, fixed=fixed, log=quiet)
                a.run()
                self.assertEqual(a.check_equations(), [])
                sim = SimRig(jed(name))
                for path, t in a._walks.items():
                    sim._replay(a.drives(path))
                    ext = sim._ext(a.V, a.base, a.full, a.v1)
                    level, driven, _, _ = sim.bits.settle(sim.q, ext, a.full)
                    for p in a.comb + a.reg:
                        self.assertEqual(t.driven[p], driven[p], "pin %d driven" % p)
                        self.assertEqual(t.value[p] & driven[p], level[p] & driven[p],
                                         "pin %d value" % p)

    def test_async_parts(self):
        found = {name: GAL(jed(name)).async_parts()
                 for name in list(CASES) + list(ASYNC_CASES) + list(AR_CASES) +
                 list(LATCH_CASES) + list(MIXED_CASES) + ["osc16v8"]}
        ar = ["an asynchronous reset term"]
        self.assertEqual(found, {
            "cnt16v8": [], "fsm22v10": [], "comb16v8": [],
            "cnt20v8": [], "comb20v8": [], "simple20v8": [],
            "ar22v10": ar, "arlow22v10": ar, "arw22v10": ar, "aropen22v10": ar,
            "latch16v8": ["combinational feedback through pins 17, 18"],
            "latch22v10": ["combinational feedback through pins 21, 22, 23"],
            "dlat22v10": ["combinational feedback through pins 14, 15"],
            "srnor16v8": ["combinational feedback through pins 16, 17, 18"],
            "reglatch16v8": ["combinational feedback through pin 17"],
            "latchd16v8": ["combinational feedback through pin 17"],
            "reglatch22v10": ar + ["combinational feedback through pin 21"],
            "trilatch22v10": ["combinational feedback through pin 22"],
            "chain16v8": ["combinational feedback through pins 16, 17, 18"],
            "clklatch22v10": ["combinational feedback through pins 22, 23"],
            "quietlatch16v8": ["combinational feedback through pin 18"],
            "osc16v8": ["combinational feedback through pin 18"],
            "oscstate16v8": ["combinational feedback through pins 17, 18"],
            "oeread22v10": [],
            "buried22v10": ["combinational feedback through pin 22"]})

    def test_mixed_gray_order(self):
        """A mixed Gray code visits every vector once and changes one bit
        per step.  All but the bits that split the walk first change
        often, and over a few values of mix every bit does."""
        from rig import mix_bits
        for n in (1, 2, 5, 11):
            for mix in (1, 2, 0x1234):
                order = list(gray_order(n, 0, 0, 5 % (1 << n), mix))
                self.assertEqual(sorted(order), list(range(1 << n)))
                self.assertTrue(all(bin(a ^ b).count("1") == 1
                                    for a, b in zip(order, order[1:])))
        total = [0] * 11
        for mix in (1, 2, 7):
            flips = [0] * 11
            for b in mix_bits(11, mix):
                flips[b] += 1
                total[b] += 1
            self.assertGreater(sorted(flips)[3], 2047 // 11 // 2)
            self.assertLess(max(flips), 2047 // 11 * 3)
        self.assertGreater(min(total), 2047 // 11 // 2)
        self.assertNotEqual(list(gray_order(6, mix=1)), list(gray_order(6, mix=2)))
        self.assertNotEqual(list(gray_order(6, mix=1)), list(gray_order(6)))

    def test_latch_that_counting_walks_miss(self):
        """quietlatch16v8's latch holds only while another signal keeps
        still.  Where the pins of a vector change in one write, the chip
        reads as combinational in the counting orders, with a wrong
        equation; the Gray code orders show the latch.  Where one pin
        changes per write, a counting order shows it too."""
        fixed = galre.parse_fixed(LATCH_CASES["quietlatch16v8"][1])
        with mock.patch.object(Analysis, "ORDERS", Analysis.ORDERS[:3]):
            a = Analysis(SimRig(jed("quietlatch16v8"), serial=False), "GAL16V8", fixed=fixed,
                         log=quiet)
            a.run()
        self.assertIn((18, "", False, "P4", 1), a.eqs)        # L = A: wrong
        a = Analysis(SimRig(jed("quietlatch16v8"), serial=False), "GAL16V8", fixed=fixed,
                     log=quiet)
        with self.assertRaisesRegex(LatchSuspected, "pin 18 read differently .* Gray code"):
            a.run()
        a = Analysis(SimRig(jed("quietlatch16v8")), "GAL16V8", fixed=fixed, log=quiet)
        with self.assertRaisesRegex(LatchSuspected, "pin 18 read differently .* descending"):
            a.run()

    def test_reads_another_output(self):
        """comb20v8's W is a parity of P, D and E, where P is a parity
        itself.  Over the inputs W needs 16 product terms; the macrocell
        has 7.  galre writes it with pin 20, as the original does."""
        said = []
        a = Analysis(SimRig(jed("comb20v8")), "GAL20V8", fixed=CASES["comb20v8"][3],
                     log=said.append)
        a.run()
        self.assertTrue(any("pin 21 needs 16 product terms" in t for t in said))
        eq = {(p, kind): (expr, n) for p, kind, _, expr, n in a.eqs}
        self.assertEqual(eq[(21, "")][1], 4)
        self.assertIn("P20", eq[(21, "")][0])
        self.assertEqual(eq[(20, "")][1], 4)
        self.assertEqual(a.check_equations(), [])
        a.scover[(21, "")][0].pop()                 # the check does check these covers
        self.assertEqual([b[:2] for b in a.check_equations()], [(21, "")])

    def test_gal20v8_modes(self):
        self.assertEqual({n: GAL(jed(n)).mode for n in ("cnt20v8", "comb20v8", "simple20v8")},
                         {"cnt20v8": "registered", "comb20v8": "complex",
                          "simple20v8": "simple"})
        with self.assertRaisesRegex(ValueError, "not a valid mode"):
            f = list(GAL(jed("cnt20v8")).fuses)
            f[2705] = 0
            GAL(f)

    def test_latch_held_by_another_output(self):
        """racelatch16v8's latch is held by another output's pin, which is
        late when its input falls, so the latch drops.  No circuit in
        which only the latch has a delay does that; galre then takes every
        output pin for a signal of its own, and the latch's equation reads
        pin 15."""
        said = []
        a = galre.analyse(SimRig(jed("racelatch16v8")), "GAL16V8",
                          fixed=galre.parse_fixed("6-9=0,11-14=0,18-19=0"), log=said.append)
        self.assertTrue(a.deep)
        self.assertTrue(any("every output pin as a signal with a delay" in t for t in said))
        self.assertEqual(a.Lh, [16])
        self.assertEqual(a.L, [15, 16, 17])
        eq = {(p, kind): expr for p, kind, _, expr, _ in a.eqs}
        self.assertIn("P15", eq[(16, "")])
        self.assertEqual(eq[(17, "oe")], "P5")
        self.assertEqual(a.check_equations(), [])
        self.assertEqual(verify.check_captures(a, jed("racelatch16v8")), [])

    def test_gray_order(self):
        for n, rot, rev, start in ((1, 0, 0, 0), (4, 0, 0, 0), (5, 2, 0, 9), (5, 3, 1, 30),
                                   (6, 5, 1, 0)):
            self.assertEqual(list(gray_order(n, rot, rev, start)),
                             [walk_x(i, n, gray=1, rot=rot, rev=rev, start=start)
                              for i in range(1 << n)])

    def test_registers_and_latches(self):
        """Latches in a registered design: the pins that change without a
        clock, the latch pins among them, the registers and every
        equation; the original JED matches every capture, also the walks
        with pin 1 enumerated."""
        for name, (device, fix, hist, latches, regs, nstates, eqs) in MIXED_CASES.items():
            with self.subTest(design=name):
                a = galre.analyse(SimRig(jed(name)), device, fixed=galre.parse_fixed(fix),
                                  log=quiet)
                self.assertIsInstance(a, RegLatchAnalysis)
                self.assertEqual(sorted(a.hist), hist)
                self.assertEqual(a.Lh, latches)
                self.assertFalse(a.deep)
                self.assertEqual(sorted(a.reg), regs)
                self.assertEqual(len(a.states), nstates)
                self.assertEqual({(p, kind): expr for p, kind, _, expr, _ in a.eqs}, eqs)
                self.assertFalse(any(inv for _, _, inv, _, _ in a.eqs))
                self.assertEqual(a.check_equations(), [])
                self.assertEqual(verify.check_captures(a, jed(name)), [])
        a.F[(a.L[0], "")][(0, 0)] = (a.cfull, 0)   # the check does check
        self.assertNotEqual(a.check_equations(), [])

    def test_latches(self):
        """Latches built from combinational feedback: the latch pins, the
        other outputs and their equations; the original JED matches every
        walk."""
        for name, (device, fix, latches, comb, eqs) in LATCH_CASES.items():
            with self.subTest(design=name):
                a = galre.analyse(SimRig(jed(name)), device, fixed=galre.parse_fixed(fix),
                                  log=quiet)
                self.assertIsInstance(a, LatchAnalysis)
                self.assertEqual(a.L, latches)
                self.assertEqual([p for p in a.outputs if p not in a.L], comb)
                self.assertEqual({p: "!" * inv + expr for p, _, inv, expr, _ in a.eqs}, eqs)
                self.assertEqual(a.check_equations(), [])
                self.assertEqual(verify.check_captures(a, jed(name)), [])

    def test_async_reset(self):
        """The asynchronous reset term is found and written; the original
        JED matches the captures, taken around the term (the careful
        SimRig), and fires in the same states."""
        for name, (fix, term, active) in AR_CASES.items():
            with self.subTest(design=name):
                a = Analysis(SimRig(jed(name)), "GAL22V10", fixed=galre.parse_fixed(fix),
                             log=quiet)
                a.run()
                self.assertEqual(a.ar_expr(), term)
                self.assertEqual(len(a.ar_on), active)
                self.assertEqual(len(a.states), 4)
                self.assertEqual(a.check_equations(), [])
                src = a.cupl(name)
                for r in a.reg:
                    self.assertIn("P%d.ar = %s;" % (r, term), src)
                self.assertEqual(verify.check_captures(a, jed(name)), [])

    def test_reset_term_on_an_io_pin(self):
        """The reset term reads an I/O pin that is used as an input: a
        state's signature is read without driving that pin both ways,
        which would fire the term.  Pin 19 is an open-drain copy of a
        register that a register input reads where the chip lets it go:
        a constant output with an enable, not a register."""
        a = Analysis(SimRig(jed("aropen22v10")), "GAL22V10",
                     fixed=galre.parse_fixed(AR_CASES["aropen22v10"][0]), log=quiet)
        a.run()
        self.assertEqual(a.ar, {16: 0})
        self.assertNotIn(16, a.cand)
        self.assertEqual(sorted(a.reg), [22, 23])
        self.assertEqual({(p, kind): expr for p, kind, _, expr, _ in a.eqs}, {
            (19, ""): "'b'0",
            (19, "oe"): "P23",
            (22, "d"): "P3 & P19 & !P23\n    # P4 & P22",
            (23, "d"): "P2"})

    def test_state_reached_only_with_a_latch_set(self):
        """Q1 loads a one only while the latch is set and its set input
        is low again.  Clocked from one path per register state, it never
        does, and passes for a constant.  galre then explores by the
        registers and the latch level together: every clocked step is
        followed by the same reads, so that each of the six states is
        reached by its own path, and the four register states come out
        with the latch as an input of Q1."""
        fix, eqs = ROUTELATCH
        logs = []
        a = galre.analyse(SimRig(jed("routelatch22v10")), "GAL22V10",
                          fixed=galre.parse_fixed(fix), log=logs.append)
        self.assertIsInstance(a, RegLatchAnalysis)
        self.assertTrue(a.composite)
        self.assertEqual(a.lsig, (21,))
        self.assertIn("6 states by registers and latch levels (pin 21), 4 by the registers "
                      "alone", logs)
        self.assertEqual(len(a.states), 4)
        self.assertEqual(sorted(a.reg), [22, 23])
        self.assertEqual(a.Lh, [21])
        self.assertFalse(a.deep)
        self.assertEqual({(p, kind): "!" * inv + expr for p, kind, inv, expr, _ in a.eqs}, eqs)
        self.assertEqual(a.check_equations(), [])
        self.assertEqual(verify.check_captures(a, jed("routelatch22v10")), [])

    def test_latch_taken_to_follow(self):
        """One input change sets latches A and B, and C follows them through
        A & B; from outside the three rise in one step.  galre finds C
        following by itself here.  Where it takes such a latch to move
        with the others, the latch needs their terms too and may not fit
        its macrocell; galre then learns again with it moving after the
        other latches of a step.  Told that pin 16 does not fit, it does
        so, and finds the same design."""
        fix, eqs = LATCHFOLLOW

        def terms(a):
            self.assertFalse(any(inv for _, _, inv, _, _ in a.eqs))
            return {(p, kind): set(expr.split("\n    # ")) for p, kind, _, expr, _ in a.eqs}
        a = galre.analyse(SimRig(jed("latchfollow16v8")), "GAL16V8",
                          fixed=galre.parse_fixed(fix), log=quiet)
        self.assertIsInstance(a, RegLatchAnalysis)
        self.assertEqual(a.follow, set())
        self.assertEqual(a.Lh, [16, 17, 18])
        self.assertEqual(terms(a), eqs)
        real, calls, logs = RegLatchAnalysis._over, [], []

        def over(a):
            calls.append(1)
            return [16] if len(calls) == 1 else real(a)
        with mock.patch.object(RegLatchAnalysis, "_over", over):
            a = galre.analyse(SimRig(jed("latchfollow16v8")), "GAL16V8",
                              fixed=galre.parse_fixed(fix), log=logs.append)
        self.assertEqual(a.follow, {16})
        self.assertIn("pin 16 taken to move after the other latches of a step", logs)
        self.assertEqual(terms(a), eqs)
        self.assertEqual(a.check_equations(), [])
        self.assertEqual(verify.check_captures(a, jed("latchfollow16v8")), [])

    def test_walks_chosen_for_what_the_circuit_guesses(self):
        """A product term that was seen to decide a latch with a pin at one
        level only may lack that pin's literal.  galre lists such cases
        (_open_terms), and chooses walks that the learned circuit takes
        through them.  Here the walks learned from are cut down to those
        from before the confirmation, so that cases are open; a chosen
        walk settles some of them, and comes out as the circuit says."""
        fix, _ = LATCHFOLLOW
        a = galre.analyse(SimRig(jed("latchfollow16v8")), "GAL16V8",
                          fixed=galre.parse_fixed(fix), log=quiet)
        self.assertEqual(a.guided, 0)
        a._open_terms()
        self.assertEqual(a._open, set())          # every term seen at both levels
        a.lwalks = a.lwalks[:len(a.S) + 6]
        a._solve()
        a._open_terms()
        self.assertTrue(a._open)
        j, ci, i, side = sorted(a._open)[0]
        sup, cubes, _ = a.cover[(a.L[j], "")]
        self.assertFalse(cubes[ci][0] >> i & 1)   # a variable the term does not read
        cfg, claims = a._guess_walk(set())
        self.assertTrue(claims and claims <= a._open)
        w = a._event(*cfg)
        self.assertIsNone(a._predict(w))

    def test_output_that_does_not_settle(self):
        """R = !EN & !STOP # !R & !STOP is its own complement while EN is
        high and STOP low, and toggles; a read catches an arbitrary level.
        Where a pin does not settle is what there is to learn: galre
        reads every vector often, takes the pin for its own complement
        where it is unsteady, and the output that is unsteady with it for
        one that follows it.  The rebuilt chip settles at the same
        vectors, at the same levels."""
        from osc import OscAnalysis
        fix, eqs = OSC
        logs = []
        a = galre.analyse(SimRig(jed("osc16v8")), "GAL16V8", fixed=galre.parse_fixed(fix),
                          log=logs.append)
        self.assertIsInstance(a, OscAnalysis)
        self.assertIn("asynchronous behaviour: the same walk read differently twice on pins "
                      "17, 18: a bad contact, a floating pin or an oscillating output", logs)
        self.assertEqual(a.L, [18])
        self.assertEqual(a.follows, {17: 18})
        self.assertEqual(bin(a.un[18]).count("1"), 4)  # EN & !STOP, of 16 vectors
        self.assertEqual(bin(a.un[17]).count("1"), 2)  # and A
        self.assertEqual({p: "!" * inv + expr for p, _, inv, expr, _ in a.eqs}, eqs)
        self.assertEqual(a.check_equations(), [])
        self.assertEqual(verify.check_captures(a, jed("osc16v8")), [])
        src = a.cupl("osc")
        self.assertIn("MIN [P18] = 0;", src)
        self.assertIn("/* P18 does not settle at some inputs:", src)
        self.assertIn("/* P17 is taken to follow P18 where that does not settle;", src)
        # another chip does not settle at the same vectors
        self.assertTrue(verify.check_captures(a, jed("oscstate16v8")))

    def test_pin_seldom_high_while_it_toggles(self):
        """On a real GAL16V8 the output behind the oscillating pin was
        high in a fifth of the reads: twelve reads of a vector are then
        all low now and then, and the vector looks settled in that walk
        order.  galre counts how often, and reads more walk orders."""
        fix, eqs = "11-16=0,19=0", OSC[1]
        logs = []
        rig = OddReads(SimRig(jed("osc16v8")), 17, 0.15, {2: 1, 3: 0, 4: 1})
        a = galre.analyse(rig, "GAL16V8", fixed=galre.parse_fixed(fix), log=logs.append)
        more = [l for l in logs if l.startswith("pin 17 looks settled in one walk order at ")]
        self.assertEqual(len(more), 1, logs)
        self.assertRegex(more[0], r"of the 64 input vectors at which it does not settle \(it is "
                         r"seldom at one of its levels\): reading \d+ walk orders, not 4$")
        self.assertGreater(len(a.walks), 4)
        self.assertEqual(bin(a.un[18]).count("1"), 128)  # EN & !STOP, of 512 vectors
        self.assertEqual(bin(a.un[17]).count("1"), 64)   # and A: none looked settled throughout
        self.assertEqual({p: "!" * inv + expr for p, _, inv, expr, _ in a.eqs}, eqs)
        self.assertEqual(verify.check_captures(a, jed("osc16v8")), [])

    def test_bad_contact_is_not_an_oscillator(self):
        """A pin that reads wrong now and then is unsteady at other
        vectors in every walk order."""
        from analysis import AsyncDesign
        rig = OddReads(SimRig(jed("comb16v8")), 19, 0.01)
        logs = []
        with self.assertRaises(AsyncDesign) as e:
            galre.analyse(rig, "GAL16V8", fixed=galre.parse_fixed("11-16=0"), log=logs.append)
        self.assertRegex(str(e.exception),
                         r"^pin 19 reads unsteadily at \d+ input vectors, and settles at \d+ of "
                         r"them in another walk order \(the first: .*\): a bad contact or a "
                         r"floating pin rather than an oscillating output$")
        self.assertIn("trying an output that feeds back inverted: where does each pin settle?",
                      logs)

    def test_inputs_of_one_step_race(self):
        """On a board every pin has its own 1K, capacitance and threshold:
        where a step changes several inputs, the chip sees them one after
        the other, in an order that differs from time to time.  A real
        GAL16V8 with oelatch16v8 in it (L = S # L & R) kept L in two
        steps of three in which S fell and R rose, so the same walk read
        differently twice.  One input at a time it reads the same every
        time: latches, and no bad contact.  That is firmware which
        changes the pins of a vector in one write (serial=False); with
        one pin per write there is no such race."""
        from analysis import Unsteady
        for name, device, fix, eqs in (
                ("oelatch16v8", "GAL16V8", OELATCH[0], OELATCH[1]),
                ("latch16v8", "GAL16V8", LATCH_CASES["latch16v8"][1], None),
                ("chain16v8", "GAL16V8", MIXED_CASES["chain16v8"][1], None)):
            with self.subTest(design=name):
                fixed = galre.parse_fixed(fix)
                plain = galre.analyse(SimRig(jed(name)), device, fixed=fixed, log=quiet)
                logs = []
                a = galre.analyse(SimRig(jed(name), race=1, serial=False), device, fixed=fixed,
                                  log=logs.append)
                # (a race may look like a clock edge that changes the chip: then
                # the pin that enables a GAL16V8's registers is held low first)
                first = next(line for line in logs if line.startswith("asynchronous behaviour"))
                self.assertRegex(
                    first, r"^asynchronous behaviour: pins? [\d, ]+ read differently when the "
                    r"same walk is repeated \(\d+ of \d+ input vectors, the first .*\), but "
                    r"the same every time when one input changes at a time: ")
                self.assertIs(type(a), type(plain))
                got = {(p, kind): "!" * inv + expr for p, kind, inv, expr, _ in a.eqs}
                self.assertEqual(got, {(p, kind): "!" * inv + expr
                                       for p, kind, inv, expr, _ in plain.eqs})
                if eqs:
                    self.assertEqual(got, eqs)
                self.assertEqual(verify.check_captures(a, jed(name)), [])
                logs = []
                b = galre.analyse(SimRig(jed(name), race=1), device, fixed=fixed,
                                  log=logs.append)
                self.assertFalse(any("same walk is repeated" in line for line in logs))
                self.assertEqual({(p, kind): "!" * inv + expr for p, kind, inv, expr, _ in b.eqs},
                                 got)
        # a pin that reads wrong now and then does so one input at a time too
        rig = OddReads(SimRig(jed("comb16v8"), race=1, serial=False), 19, 0.01)
        with self.assertRaises(Unsteady):
            Analysis(rig, "GAL16V8", fixed=galre.parse_fixed("11-16=0"), log=quiet).run()

    def test_edge_detector_from_an_output_delay(self):
        """Pin 16 is a copy of pin 4, a gate delay late, and pin 17 is set
        by !P4 & P16: a flip-flop clocked by the falling edge of pin 4.
        With every output as a signal of its own galre finds the term,
        and says in the source that it is true only in passing."""
        fix, eqs = EDGELATCH
        a = galre.analyse(SimRig(jed("edgelatch16v8")), "GAL16V8",
                          fixed=galre.parse_fixed(fix), log=quiet)
        self.assertIsInstance(a, RegLatchAnalysis)
        self.assertTrue(a.deep)
        self.assertEqual(a.Lh, [17])
        self.assertEqual({(p, kind): "!" * inv + expr for p, kind, inv, expr, _ in a.eqs}, eqs)
        self.assertEqual(len(a.notes), 1)
        self.assertIn("P17 has a product term that is true only in passing: "
                      "P2 & !P3 & !P4 & P16.  P16 follows P4,", a.notes[0])
        src = a.cupl("edge")
        self.assertIn("/* P17 has a product term that is true only in passing:", src)
        self.assertIn("depends on its timing. */\n\nP15 = P5;", src)
        self.assertEqual(a.check_equations(), [])
        self.assertEqual(verify.check_captures(a, jed("edgelatch16v8")), [])

    def test_edge_detector_with_margin(self):
        """Pin 14 is a copy of pin 16, two gate delays behind pin 4, the
        latch is set by !P4 & P14 and holds itself after one: the circuit
        a real GAL16V8 takes the edge with.  galre finds a latch set in
        passing by pin 16, and pins 14 and 16 as copies of pin 4: one
        delay or two read the same once the pins have settled.  A real
        chip with that source, one delay, did not take the edge.  So the
        copies are written as a chain (P16 = P14), the term's pin last,
        and the source says so."""
        fix, eqs = EDGE2LATCH
        a = galre.analyse(SimRig(jed("edge2latch16v8")), "GAL16V8",
                          fixed=galre.parse_fixed(fix), log=quiet)
        self.assertIsInstance(a, RegLatchAnalysis)
        self.assertEqual(a.Lh, [17])
        self.assertEqual(a.chain, {16: 14})
        self.assertEqual({(p, kind): "!" * inv + expr for p, kind, inv, expr, _ in a.eqs}, eqs)
        note = [n for n in a.notes if n.startswith(
            "P17 has a product term that is true only in passing: P2 & !P3 & !P4 & P16.")]
        self.assertEqual(len(note), 1, a.notes)
        self.assertIn("P14 reads the same as P16 wherever the chip rests, and whether one "
                      "follows the other in the chip does not show.  P16 is written as "
                      "following P14, not the inputs, so that the term is true for 2 gate "
                      "delays: the longer pulse is the safer guess.", note[0])
        self.assertIn("\nP14 = P4;\n", a.cupl("edge2"))
        self.assertIn("\nP16 = P14;\n", a.cupl("edge2"))
        self.assertEqual(a.check_equations(), [])
        self.assertEqual(verify.check_captures(a, jed("edge2latch16v8")), [])
        # without the terms that only hold a latch through a step, too (a
        # retry of the compiler computes the equations again)
        a.hold_terms = False
        self.assertIn((16, "", False, "P14", 1), a.equations())
        # one output only that follows pin 4: nothing to chain
        b = galre.analyse(SimRig(jed("edgelatch16v8")), "GAL16V8",
                          fixed=galre.parse_fixed(EDGELATCH[0]), log=quiet)
        self.assertEqual(b.chain, {})

    def test_guess_taken_back(self):
        """What galre adds to explain a step is a guess, and a later step
        may show it to be wrong: the latch that it moves there, the chip
        left alone.  The guess then gives way.  Here a wrong one is put
        in by hand: pin 17 rising where pin 4 rises."""
        fix, _ = EDGELATCH
        a = galre.analyse(SimRig(jed("edgelatch16v8")), "GAL16V8",
                          fixed=galre.parse_fixed(fix), log=quiet)
        j, b2, b4 = a.L.index(17), a.Vc.index(2), a.Vc.index(4)
        rec = next(r for r in a.steps
                   if r[0][3] != r[0][4] and not (r[0][3] | r[0][4]) >> j & 1
                   and a._base[0][j][r[1]] == 0 and r[0][0] >> b2 & 1 and r[0][0] >> b4 & 1
                   and (r[0][0] ^ r[0][1]) >> b4 & 1)
        e = rec[1]
        a.extra[(j, e)] = 1
        a._apply()
        a.equations()
        self.assertTrue(a._refine())
        self.assertEqual(a.extra[(j, e)], 0)
        a._solve()
        self.assertEqual(a.check_equations(), [])

    def test_skipped_vectors_are_not_driven_pins(self):
        """A walk around the reset term skips vectors, which read as 0.
        An enumerated I/O pin that the design does not use (19) is not
        taken for an output because of them."""
        device, fix, hist, latches, regs, nstates, eqs = MIXED_CASES["reglatch22v10"]
        logs = []
        a = galre.analyse(SimRig(jed("reglatch22v10")), device,
                          fixed=galre.parse_fixed("7-11=0,13-18=0"), log=logs.append)
        self.assertIn(19, a.V)
        self.assertEqual(a.late_comb, {})
        self.assertFalse([l for l in logs if "driven in a walk" in l])
        self.assertEqual({(p, kind): expr for p, kind, _, expr, _ in a.eqs}, eqs)

    def test_registered_gal16v8_with_a_latch(self):
        """Pin 11 of a registered GAL16V8 enables the registers' outputs
        and nothing else.  galre holds it low for the walks with pin 1,
        so that every clock edge shows.  An output that only pin 11
        enables stays a register (one with a constant input); one with
        an enable of its own that shows a register is combinational."""
        fix, eqs = OELATCH
        a = galre.analyse(SimRig(jed("oelatch16v8")), "GAL16V8", fixed=galre.parse_fixed(fix),
                          log=quiet)
        self.assertIsInstance(a, RegLatchAnalysis)
        self.assertEqual(a.oe_held, 11)
        self.assertEqual(a.fixed[11], 0)
        self.assertEqual(sorted(a.reg), [15, 18])
        self.assertEqual(a.Lh, [17])
        self.assertEqual({(p, kind): "!" * inv + expr for p, kind, inv, expr, _ in a.eqs}, eqs)
        self.assertEqual(a.check_equations(), [])
        self.assertEqual(verify.check_captures(a, jed("oelatch16v8")), [])

    def test_register_polarity(self):
        """A GAL22V10 register output that is high at power-up (active low)
        gets a negated .d, which CUPL makes active low."""
        a = Analysis(SimRig(jed("arlow22v10")), "GAL22V10",
                     fixed=galre.parse_fixed(AR_CASES["arlow22v10"][0]), log=quiet)
        a.run()
        self.assertEqual(a.W[a.power_up_state], (1, 1))
        d = {p: inv for p, kind, inv, _, _ in a.eqs if kind == "d"}
        self.assertEqual(d, {22: True, 23: True})

    def test_held_pin_driven_by_the_gal(self):
        with self.assertRaisesRegex(RuntimeError, "pin 19 is held"):
            Analysis(SimRig(jed("comb16v8")), "GAL16V8", fixed={19: 0}, log=quiet).run()

    @unittest.skipUnless(CUPL and shutil.which("csim"), "no CUPL compiler and simulator")
    def test_csim_second_opinion(self):
        """csim, on the source galre wrote, agrees with galsim's reading of
        the JED built from it: registers, a reset term, latches (whose
        level is unknown to csim until something sets them), a latch on a
        bidirectional pin.  It does not agree with another design's JED."""
        import csimcheck
        csim = shutil.which("csim")
        out = tempfile.mkdtemp(prefix="galre")
        cases = [("cnt16v8", "GAL16V8", None), ("fsm22v10", "GAL22V10", None),
                 ("arlow22v10", "GAL22V10", AR_CASES["arlow22v10"][0]),
                 ("latch16v8", "GAL16V8", LATCH_CASES["latch16v8"][1])]
        cases += [(n, MIXED_CASES[n][0], MIXED_CASES[n][1])
                  for n in ("reglatch22v10", "reglatch16v8", "trilatch22v10")]
        try:
            built = {}
            for name, device, fix in cases:
                with self.subTest(design=name):
                    a = galre.analyse(SimRig(jed(name)), device, fixed=galre.parse_fixed(fix),
                                      log=quiet)
                    pld = os.path.join(out, name + ".pld")
                    with open(pld, "w") as f:
                        f.write(a.cupl(name))
                    built[name] = pld, galre.compile_cupl(CUPL, pld, quiet)
                    n, unknown, bad = csimcheck.check(csim, *built[name], steps=300)
                    self.assertEqual((n, bad), (300, []))
                    if name == "latch16v8":
                        self.assertGreater(unknown, 0)      # until its latches are set
            # reglatch16v8's source against latchd16v8's JED: the register differs
            _, _, bad = csimcheck.check(csim, built["reglatch16v8"][0], jed("latchd16v8"), 300)
            self.assertTrue(bad)
            self.assertIn("P18", bad[0][1])
        finally:
            shutil.rmtree(out)

    def test_return_paths_replace_power_cycles(self):
        rig = SimRig(jed("cnt16v8"))
        a = Analysis(rig, "GAL16V8", log=quiet)
        a.run()
        self.assertGreater(rig.ops["ret_ok"], 10 * rig.ops["power_cycles"])
        self.assertEqual(rig.ops["ret_fail"], 0)

    @unittest.skipUnless(CUPL, "no CUPL compiler")
    def test_cupl_round_trip(self):
        out = tempfile.mkdtemp(prefix="galre")
        cases = [(name, device, None) for name, (device, _, _, _) in CASES.items()]
        cases += [(name, "GAL22V10", galre.parse_fixed(fix))
                  for name, (fix, _, _) in AR_CASES.items()]
        cases += [(name, device, galre.parse_fixed(fix))
                  for name, (device, fix, _, _, _) in LATCH_CASES.items()]
        cases += [(name, c[0], galre.parse_fixed(c[1])) for name, c in MIXED_CASES.items()]
        cases += [("racelatch16v8", "GAL16V8", galre.parse_fixed("6-9=0,11-14=0,18-19=0")),
                  ("oelatch16v8", "GAL16V8", galre.parse_fixed(OELATCH[0])),
                  ("routelatch22v10", "GAL22V10", galre.parse_fixed(ROUTELATCH[0])),
                  ("latchfollow16v8", "GAL16V8", galre.parse_fixed(LATCHFOLLOW[0])),
                  ("edgelatch16v8", "GAL16V8", galre.parse_fixed(EDGELATCH[0])),
                  ("edge2latch16v8", "GAL16V8", galre.parse_fixed(EDGE2LATCH[0])),
                  ("osc16v8", "GAL16V8", galre.parse_fixed(OSC[0]))]
        try:
            for name, device, fixed in cases:
                with self.subTest(design=name):
                    a = galre.analyse(SimRig(jed(name)), device, fixed=fixed, log=quiet)
                    pld = os.path.join(out, name + ".pld")
                    with open(pld, "w") as f:
                        f.write(a.cupl(name))
                    latches = name in LATCH_CASES or name in MIXED_CASES or "latch" in name
                    rebuilt = galre.compile_cupl(CUPL, pld, quiet)
                    self.assertEqual(rebuilt, os.path.join(out, name + ".jed"))
                    notes = []
                    self.assertEqual(verify.check_captures(a, rebuilt, notes), [])
                    self.assertEqual(bool(notes), name == "srnor16v8")
                    d, first, _ = verify.compare_jeds(jed(name), rebuilt, 5000, fixed=fixed,
                                                      sic=latches)
                    self.assertEqual(d, {}, first)
            # The source says not to minimise the latch (MIN), so it keeps
            # the product term that only holds it through a step
            # (CLK & P4 & P22); without that statement CUPL removes it.
            pld = os.path.join(out, "reglatch22v10.pld")
            src = open(pld).read()
            self.assertIn("MIN [P21] = 0;", src)
            self.assertIn("Compile with: cupl -jaxm4 reglatch22v10.pld", src)
            self.assertEqual(len(GAL(galre.compile_cupl(CUPL, pld, quiet))._olmc(21).live), 4)
            with open(pld, "w") as f:
                f.write(src.replace("MIN [P21] = 0;", ""))
            self.assertEqual(len(GAL(galre.compile_cupl(CUPL, pld, quiet))._olmc(21).live), 3)
        finally:
            shutil.rmtree(out)

    def test_register_low_after_power_up(self):
        """A 22V10 whose outputs look like those of a GAL20V8 in its
        registered mode is not taken for one: a register that is low after
        power-up is none of a GAL20V8.  galre stops as soon as it has seen
        the register change, and names the device to try."""
        fix = galre.parse_fixed("4-11=0,14-19=0,23=0")
        logs = []
        with self.assertRaises(Limit) as e:
            galre.analyse(SimRig(jed("lowreg22v10")), "GAL20V8", fixed=fix, survey=16,
                          log=logs.append)
        self.assertIn("pin 22 is a register and low after power-up", str(e.exception))
        self.assertIn("-d GAL22V10", str(e.exception))
        self.assertFalse(any("states reachable" in line for line in logs))    # before the sweeps
        a = galre.analyse(SimRig(jed("lowreg22v10")), "GAL22V10", fixed=fix, survey=16, log=quiet)
        self.assertEqual(sorted(a.reg), [20, 21, 22])
        self.assertEqual(verify.check_captures(a, jed("lowreg22v10")), [])
        # a GAL20V8's own registers, of either polarity, are high after power-up
        galre.analyse(SimRig(jed("cnt20v8")), "GAL20V8", survey=16, log=quiet,
                      fixed=galre.parse_fixed("5-11=0,14=0,23=0"))

    def test_enable_pin_read_alone(self):
        """A GAL16V8 or GAL20V8 that clock edges change is first taken for
        one with registers: pin 11 (13) is held low and walked alone.  It
        comes out as with the pin walked with the other inputs, in fewer
        vectors.  A chip without registers, or with latches too, is read
        as before."""
        REGS20 = {p: 0 for p in (5, 6, 7, 8, 9, 10, 11, 14, 15, 23)}
        for name, device, fixed, regs in (
                ("cnt16v8", "GAL16V8", CASES["cnt16v8"][3], [18, 19]),
                ("cnt20v8", "GAL20V8", CASES["cnt20v8"][3], [21, 22]),
                ("regs20v8", "GAL20V8", REGS20, [17, 18, 19, 20, 21, 22])):
            with self.subTest(design=name):
                oe = DEV[device]["oe"]
                logs = []
                rig = SimRig(jed(name))
                a = galre.analyse(rig, device, fixed=fixed, survey=16, log=logs.append)
                self.assertEqual((a.oe_alone, a.oe_off, sorted(a.reg)), (oe, set(regs), regs))
                self.assertIn("pin %d is held low and read alone" % oe, logs[0])
                self.assertIn("pin %d, walked alone, switches off pins %s" % (
                    oe, ", ".join(map(str, regs))), logs)
                self.assertEqual(a.check_equations(), [])
                self.assertEqual(verify.check_captures(a, jed(name)), [])
                src = a.cupl(name)
                self.assertIn("Pin %d = !OE;" % oe, src)
                self.assertIn("Pin %d enables the registers' outputs" % oe, src)
                old = SimRig(jed(name))
                b = galre.analyse(old, device, fixed=fixed, survey=16, log=quiet, oe_first=False)
                self.assertIsNone(b.oe_alone)
                self.assertEqual({(p, kind): "!" * inv + expr for p, kind, inv, expr, _ in a.eqs},
                                 {(p, kind): "!" * inv + expr for p, kind, inv, expr, _ in b.eqs})
                self.assertLess(rig.ops["io"], old.ops["io"])
        # a register that never changes is one all the same: its pin goes with pin 13
        self.assertIn("!P18.d = 'b'0;", src)
        self.assertNotIn("P18.oe", src)
        # an output that reads registers only changes with the clock too, but stays on
        self.assertIn(16, a.comb)
        self.assertIn("\nP16 = !P21 & !P22;\n", src)
        # the rebuilt chip must let go of its outputs as the chip does
        a.oe_walks[0][1][22] ^= 2
        self.assertEqual(verify.check_captures(a, jed("regs20v8")),
                         [("pin 22", "pin 13 walked alone")])
        # no registers: nothing is held, and the report says nothing of it
        for name, device in (("comb16v8", "GAL16V8"), ("comb20v8", "GAL20V8")):
            with self.subTest(design=name):
                logs = []
                a = galre.analyse(SimRig(jed(name)), device, fixed=CASES[name][3], survey=16,
                                  log=logs.append)
                self.assertIsNone(a.oe_alone)
                self.assertFalse(any("held low" in line for line in logs))
        # registers and a latch: the latch shows with the pin held low, and
        # the chip is read again as before
        name, device, fix = "reglatch16v8", "GAL16V8", "5-9=0,12-16=0,19=0"
        logs = []
        a = galre.analyse(SimRig(jed(name)), device, fixed=galre.parse_fixed(fix), log=logs.append)
        self.assertIn("pin 11 is held low and read alone", logs[0])
        self.assertIn("not a GAL16V8 with registers and nothing else, as seen with pin 11 held "
                      "low: ", logs[1])
        self.assertEqual(logs[2], "starting again with pin 11 walked like the other inputs")
        self.assertTrue(a.mixed)
        self.assertIsNone(a.oe_alone)
        self.assertEqual(verify.check_captures(a, jed(name)), [])
        # the pin or the clock held by the caller: as before
        for held in ({13: 0}, {1: 0}):
            a = RegLatchAnalysis(SimRig(jed("cnt20v8")), "GAL20V8",
                                 fixed={**CASES["cnt20v8"][3], **held}, log=quiet, oe_alone=True)
            self.assertIsNone(a.oe_alone)
        # a chip without registers, read as one with: it says so
        from analysis import NotRegistered
        a = RegLatchAnalysis(SimRig(jed("comb20v8")), "GAL20V8", log=quiet, oe_alone=True)
        with self.assertRaises(NotRegistered) as e:
            a.run()
        self.assertIn("clocking does not change the state", str(e.exception))

    def test_combinational_gal22v10(self):
        """A GAL22V10 without registers that reads pin 1.  CUPL leaves its
        unused OLMCs registered, but those registers drive nothing and
        nothing reads them: the rebuilt JED is checked, with pin 1
        walked, without the firmware built for the host."""
        g = GAL(jed("comb22v10"))
        self.assertEqual(sorted(o.pin for o in g.olmcs if o.registered),
                         [15, 16, 17, 18, 19, 20, 21])
        self.assertEqual(g.live_registers(), [])
        fixed = galre.parse_fixed("4-11=0,15-21=0")
        with mock.patch.object(fwsim, "library", side_effect=AssertionError("firmware asked for")):
            rig = SimRig(jed("comb22v10"))
            a = galre.analyse(rig, "GAL22V10", fixed=fixed, log=quiet)
            self.assertTrue(a.clock_in_v)
            self.assertEqual(sorted(a.comb), [22, 23])
            self.assertEqual(verify.check_captures(a, jed("comb22v10")), [])
        self.assertIsNone(rig.fw)
        # registers that show, or that other logic reads without their showing
        self.assertEqual(GAL(jed("fsm22v10")).live_registers(), [22, 23])
        self.assertEqual(GAL(jed("buried22v10")).live_registers(), [23])
        self.assertEqual(GAL(jed("cnt16v8")).live_registers(), [18, 19])
        self.assertEqual(GAL(jed("comb16v8")).live_registers(), [])

    def test_sweep_that_reads_wrongly_now_and_then(self):
        """On a board every sweep of clock edges is run twice, and five
        times where the two differ: an input vector leads where most of
        the sweeps said.  One wrong reading would otherwise pass for the
        chip's logic, or for a latch."""
        fix = galre.parse_fixed("6-11=0,14-19=0,23=0")

        class Flaky:
            """Every third sweep takes one input vector to another state
            than the chip does."""

            def __init__(self, twice):
                self.rig = SimRig(jed("lowreg22v10"))
                self.pins, self.device, self.ops = self.rig.pins, self.rig.device, self.rig.ops
                self.walk = self.rig.walk
                self.twice, self.sweeps = twice, 0

            def transitions(self, *args, **kw):
                res = self.rig.transitions(*args, **kw)
                self.sweeps += 1
                out = dict(res[0])
                if self.sweeps % 3 == 0 and len(out) > 1:
                    (o1, x1), (o2, x2) = list(out.items())[:2]
                    x = x1 & -x1
                    out[o1], out[o2] = x1 ^ x, x2 | x
                    if not out[o1]:
                        del out[o1]
                return (out,) + tuple(res[1:])

        logs = []
        a = galre.analyse(Flaky(True), "GAL22V10", fixed=fix, survey=16, log=logs.append)
        self.assertEqual(sorted(a.reg), [20, 21, 22])
        self.assertEqual(verify.check_captures(a, jed("lowreg22v10")), [])
        self.assertGreater(a.unsteady[0], 0)
        self.assertTrue(any("does not answer the same every time" in line and
                            "in 5 sweeps of the same vectors" in line for line in logs))
        self.assertIn("did not answer every clock edge the same way", a.cupl("t"))
        good = galre.analyse(SimRig(jed("lowreg22v10")), "GAL22V10", fixed=fix, survey=16,
                             log=quiet)
        self.assertEqual(a.trans, good.trans)
        self.assertEqual(good.unsteady, [0, 0])
        self.assertNotIn("did not answer", good.cupl("t"))
        # asked once, the wrong readings are the chip
        try:
            b = galre.analyse(Flaky(False), "GAL22V10", fixed=fix, survey=16, log=quiet)
        except (AsyncDesign, Limit):
            b = None
        self.assertTrue(b is None or b.trans != good.trans)


class Firmware(unittest.TestCase):
    """fw/capture.c on the host, a simulated GAL in the socket."""

    @classmethod
    def setUpClass(cls):
        try:
            cls.lib = fwsim.library()
        except subprocess.CalledProcessError as e:
            raise AssertionError("fw/capture.c does not build for the host: %s" % e)
        if not cls.lib:
            raise unittest.SkipTest("no C compiler")

    def transport(self, name, socket):
        return fwsim.HarnessTransport(self.lib, fwsim.SocketDevice(jed(name), socket))

    def test_jumper_preflight(self):
        t = self.transport("cnt16v8", "dip20")
        dev = t.dev
        vcc, gnd = dev.vcc_bit, dev.gnd_bit          # socket pins 28 and 10

        def jumpers():
            return t.command("pld capture jumpers").splitlines()[-1]
        t.command("pld capture clear")               # the library is shared
        # The supply is switched on to measure it only when the pins match
        # the job's vcc= gnd=
        self.assertEqual(jumpers(), "jumpers vcc=%07x gnd=%07x supply=-" % (vcc, gnd))
        self.assertEqual(dev.supply_measured, 0)
        # base drives GND high and VCC low: the firmware overrides both
        self.assertEqual(t.command("pld capture set pins=6 clk=1 cand=c0000 base=%x "
                                   "vcc=%x gnd=%x supply=5000" % (gnd, vcc, gnd)), "")
        self.assertEqual(jumpers(), "jumpers vcc=%07x gnd=%07x supply=4950" % (vcc, gnd))
        self.assertEqual(dev.supply_measured, 1)
        t.command("pld capture probe 400 0")
        self.assertEqual(t.capture("pld capture state").status, 0)
        self.assertEqual(dev.supply_measured, 2)
        self.assertTrue(dev.writes)
        self.assertTrue(all(w & vcc and not w & gnd for w in dev.writes))

        def refused(msg, measured=0):
            ons, before = dev.power_ons, dev.supply_measured
            with self.assertRaisesRegex(CaptureError, msg):
                t.capture("pld capture state")
            self.assertEqual(dev.power_ons, ons, "powered despite " + msg)
            self.assertEqual(dev.supply_measured - before, measured)

        dev.jumper_gnd = 1 << 11                      # GND jumper on socket pin 12
        refused("preflight: jumpers vcc=8000000 gnd=0000800, job expects "
                "vcc=8000000 gnd=0000200")
        self.assertEqual(jumpers(), "jumpers vcc=8000000 gnd=0000800 supply=-")
        dev.jumper_vcc, dev.jumper_gnd = gnd, vcc     # VCC and GND swapped
        refused("preflight: jumpers vcc=0000200 gnd=8000000")
        dev.jumper_vcc, dev.jumper_gnd = vcc, gnd
        for mv, ok in ((3280, False), (4199, False), (4200, True), (5800, True), (5801, False)):
            dev.supply_mv = mv
            if ok:
                self.assertEqual(t.capture("pld capture state").status, 0)
            else:
                refused("preflight: supply %d mV, job expects 5000 mV" % mv, 1)
        dev.supply_mv = 4950
        dev.jumpers_ok = False
        refused("preflight: could not detect")
        self.assertIn("detection failed", t.command("pld capture jumpers"))
        dev.jumpers_ok = True
        t.command("pld capture set pins=206")         # enumerates the GND pin
        refused("preflight: the job uses a VCC or GND pin")
        t.command("pld capture set pins=6 gnd=0")
        refused("preflight: set both vcc= and gnd=")
        # vcc=0 gnd=0 supply=0 turns the check off
        dev.jumper_gnd = 1 << 11
        t.command("pld capture set vcc=0 supply=0")
        self.assertEqual(t.capture("pld capture state").status, 0)

    def test_rig_preflight(self):
        t = self.transport("fsm22v10", "dip24")
        rig = BrutusRig(t, "GAL22V10", log=quiet)
        self.assertEqual(rig.preflight(), (1 << 27, 1 << 11, 4950))
        meta = rig.metadata()
        self.assertEqual(meta["jumpers"], {"vcc": [28], "gnd": [12], "supply_mv": 4950,
                                           "readings": ["jumpers: (host harness, no readings)"]})
        self.assertEqual(meta["firmware"], "Version host harness")
        t.dev.jumper_vcc = 1 << 23
        with self.assertRaises(CaptureError) as e:
            rig.preflight()
        self.assertEqual(str(e.exception),
                         "the jumpers do not suit a GAL22V10 in the dip24 socket\n"
                         "  expected: VCC on socket pin 28, GND on socket pin 12, supply 5 V\n"
                         "  detected: VCC on socket pin 24, GND on socket pin 12, "
                         "supply not measured (only once VCC and GND are right)\n"
                         "  board: jumpers: (host harness, no readings)")
        self.assertEqual(t.dev.supply_measured, 1)    # only for the first preflight
        t.dev.jumper_vcc, t.dev.supply_mv = 1 << 27, 3300
        with self.assertRaisesRegex(CaptureError, "detected: VCC on socket pin 28, "
                                    "GND on socket pin 12, supply 3.30 V"):
            rig.preflight()
        # The firmware checks too, whatever the host did
        with self.assertRaisesRegex(CaptureError, "preflight"):
            rig.walk([2, 3], {}, [])
        rig.jumper_check = False
        rig.walk([2, 3], {}, [])

    def test_plcc20_socket(self):
        """A PLCC20 GAL16V8 sits on socket pins 1 to 20, so its VCC is on
        socket pin 20.  With the VCC jumper on 28, as for every other
        package, galre stops before anything is powered."""
        t = self.transport("cnt16v8", "plcc20")
        rig = BrutusRig(t, "GAL16V8", "plcc20", log=quiet)
        self.assertEqual(rig.preflight(), (1 << 19, 1 << 9, 4950))
        self.assertEqual(rig.metadata()["jumpers"]["vcc"], [20])
        t.dev.jumper_vcc = 1 << 27
        with self.assertRaisesRegex(
                CaptureError,
                "the jumpers do not suit a GAL16V8 in the plcc20 socket\n"
                "  expected: VCC on socket pin 20, GND on socket pin 10, supply 5 V\n"
                "  detected: VCC on socket pin 28, GND on socket pin 10"):
            rig.preflight()
        with self.assertRaisesRegex(ValueError, "socket plcc20 does not fit a GAL22V10"):
            BrutusRig(t, "GAL22V10", "plcc20", log=quiet)

    def test_rails_on_spare_socket_pins(self):
        """An ATF22V10C in a PLCC28 ties pin 1 to VCC and pins 8, 15 and 22
        to GND inside, where a GAL22V10 has no connection: the board
        finds them as jumpered.  They are no pins of the device, so the
        check passes, and the jobs name the rails as detected.  The same
        on a pin of the device is refused."""
        t = self.transport("fsm22v10", "plcc28")
        logs = []
        rig = BrutusRig(t, "GAL22V10", "plcc28", log=logs.append)
        vcc, gnd = rig.vcc_bit, rig.gnd_bit
        t.dev.jumper_vcc = vcc | 1 << 0
        t.dev.jumper_gnd = gnd | 1 << 7 | 1 << 14 | 1 << 21
        self.assertEqual(rig.preflight(), (vcc | 1, gnd | 0x204080, 4950))
        self.assertEqual(logs, [
            "jumper check: socket pin 1 (VCC) and socket pins 8, 15, 22 (GND): no pin of a "
            "GAL22V10 in the plcc28 socket, and tied to the rail inside the part, as in an "
            "ATF22V10C.  Taken as detected."])
        self.assertEqual(rig.metadata()["jumpers"]["gnd"], [8, 14, 15, 22])
        # Pin 1 is found as VCC one time and not the next: the firmware
        # refuses a job whose rails are not exactly those it finds, and
        # the job is run again with them
        found = t.dev.jumper_pins
        calls = []

        def flaky():
            calls.append(1)
            vcc_now, gnd_now = found()
            return (vcc_now & ~1 if len(calls) % 3 == 0 else vcc_now), gnd_now
        t.dev.jumper_pins = flaky
        a = Analysis(rig, "GAL22V10", fixed=CASES["fsm22v10"][3], log=quiet)
        a.run()
        self.assertEqual(len(a.states), CASES["fsm22v10"][1])
        self.assertGreater(len(calls), rig.ops["runs"])      # some jobs took two tries
        self.assertTrue(all(w & 1 << 27 and not w & 0x204080 for w in t.dev.writes))
        t.dev.jumper_pins = found
        # GND on socket pin 3, a pin of the device
        t.dev.jumper_gnd = gnd | 1 << 2
        rig = BrutusRig(t, "GAL22V10", "plcc28", log=quiet)
        with self.assertRaisesRegex(
                CaptureError, "detected: VCC on socket pins 1, 28, GND on socket pins 3, 14"):
            rig.preflight()
        self.assertEqual((rig.vcc_bit, rig.gnd_bit), (vcc, gnd))
        # the VCC pin itself not found
        t.dev.jumper_vcc, t.dev.jumper_gnd = 1 << 0, gnd
        with self.assertRaisesRegex(CaptureError, "detected: VCC on socket pin 1, GND"):
            BrutusRig(t, "GAL22V10", "plcc28", log=quiet).preflight()

    def test_record_and_replay(self):
        name, (device, nstates, regs, fixed) = "fsm22v10", CASES["fsm22v10"]
        out = tempfile.mkdtemp(prefix="galre")
        try:
            rec = os.path.join(out, "capture.jsonl")
            brutus = BrutusRig(self.transport(name, "dip24"), device, log=quiet)
            brutus.preflight()
            meta = brutus.metadata()
            meta.update(title=name, fixed={str(p): v for p, v in fixed.items()}, survey=16)
            recording = RecordingRig(brutus, rec, meta)
            a = Analysis(recording, device, fixed=fixed, survey=16, log=quiet)
            eqs = a.run()
            recording.close()
            replay = ReplayRig(rec)                    # no device given
            self.assertEqual(replay.device, device)
            self.assertEqual(replay.meta["jumpers"]["vcc"], [28])
            b = Analysis(replay, replay.device, fixed=fixed, survey=16, log=quiet)
            self.assertEqual(b.run(), eqs)
        finally:
            shutil.rmtree(out)

    def test_board_asks_for_the_compiler(self):
        """galre board does not start without a CUPL compiler that builds a
        design for the device: its result would not be checked.  It says so
        before it opens the port; --no-cupl records anyway."""
        out = tempfile.mkdtemp(prefix="galre")

        def opened(port):
            raise AssertionError("the port was opened")

        def board(*more):
            with mock.patch.object(galre, "SerialTransport", opened), \
                    contextlib.redirect_stdout(io.StringIO()):
                r = galre.main(["board", "-p", "-", "-d", "GAL16V8", "-o", out] + list(more))
            with open(os.path.join(out, "report.txt")) as f:
                return r, f.read()
        try:
            # no compiler
            r, report = board("--cupl", "")
            self.assertEqual(r, 2)
            self.assertIn("no CUPL compiler", report)
            self.assertIn("No capture was run", report)
            # one that cannot be run
            r, report = board("--cupl", os.path.join(out, "nocupl"))
            self.assertEqual(r, 2)
            self.assertIn("does not build a design for the GAL16V8", report)
            self.assertIn("Nothing was run", report)
            # one that runs and makes no JED (no device library, say)
            fake = os.path.join(out, "fakecupl")
            with open(fake, "w") as f:
                f.write("#!/bin/sh\necho device not in library\n")
            os.chmod(fake, 0o755)
            r, report = board("--cupl", fake)
            self.assertEqual(r, 2)
            self.assertIn("device not in library", report)
            self.assertIsNotNone(galre.check_cupl(fake, "GAL22V10"))
            # --no-cupl goes on to the port, with or without a compiler
            for more in (["--no-cupl"], ["--no-cupl", "--cupl", fake]):
                with self.assertRaises(AssertionError):
                    board(*more)
        finally:
            shutil.rmtree(out)

    @unittest.skipUnless(CUPL, "no CUPL compiler")
    def test_compiler_check(self):
        """The check galre makes of the compiler passes with a real one, for
        every device."""
        for device in ("GAL16V8", "GAL20V8", "GAL22V10"):
            self.assertIsNone(galre.check_cupl(CUPL, device))

    def test_async_designs_stop(self):
        """galre sim and board stop with exit code 3 on a chip with state
        that changes without a clock, and replay says the same."""
        out = tempfile.mkdtemp(prefix="galre")

        def galre_main(*argv):
            with contextlib.redirect_stdout(io.StringIO()):
                r = galre.main(list(argv) + ["--no-cupl"])
            with open(os.path.join(argv[argv.index("-o") + 1], "report.txt")) as f:
                return r, f.read()
        try:
            for name, (device, fix, says) in ASYNC_CASES.items():
                with self.subTest(design=name):
                    r, report = galre_main("sim", jed(name), "-o", os.path.join(out, "sim"),
                                           "--fix", fix)
                    self.assertEqual(r, 3)
                    self.assertIn(says, report)
                    self.assertIn("no equations written", report)
                    dev = fwsim.SocketDevice(jed(name), "dip20" if device == "GAL16V8" else "dip24")
                    board = os.path.join(out, "board")
                    with mock.patch.object(galre, "SerialTransport",
                                           lambda port: fwsim.HarnessTransport(self.lib, dev)):
                        r, report = galre_main("board", "-p", "-", "-d", device, "-o", board,
                                               "--fix", fix)
                    self.assertEqual(r, 3)
                    self.assertIn(says, report)
                    r, report = galre_main("replay", os.path.join(board, "capture.jsonl"),
                                           "-o", os.path.join(out, "replay"))
                    self.assertEqual(r, 3)
                    self.assertIn(says, report)
        finally:
            shutil.rmtree(out)

    def test_async_reset_board_and_replay(self):
        """galre board finds the asynchronous reset term through the
        host-built firmware, as sim does, and replay from the recording."""
        out = tempfile.mkdtemp(prefix="galre")

        def galre_main(*argv):
            with contextlib.redirect_stdout(io.StringIO()):
                r = galre.main(list(argv) + ["--no-cupl"])
            with open(os.path.join(argv[argv.index("-o") + 1], "report.txt")) as f:
                return r, f.read()
        try:
            for name, (fix, term, _) in AR_CASES.items():
                with self.subTest(design=name):
                    pld = {}
                    r, report = galre_main("sim", jed(name), "-o", os.path.join(out, "sim"),
                                           "--fix", fix, "-t", name)
                    self.assertEqual(r, 0)
                    dev = fwsim.SocketDevice(jed(name), "dip24")
                    board = os.path.join(out, "board")
                    with mock.patch.object(galre, "SerialTransport",
                                           lambda port: fwsim.HarnessTransport(self.lib, dev)):
                        r, report = galre_main("board", "-p", "-", "-d", "GAL22V10", "-o",
                                               board, "--fix", fix, "-t", name)
                    self.assertEqual(r, 0)
                    self.assertIn("asynchronous reset: ", report)
                    r, report = galre_main("replay", os.path.join(board, "capture.jsonl"),
                                           "-o", os.path.join(out, "replay"))
                    self.assertEqual(r, 0)
                    for d in ("sim", "board", "replay"):
                        with open(os.path.join(out, d, name + ".pld")) as f:
                            pld[d] = f.read()
                    self.assertIn(".ar = %s;" % term, pld["sim"])
                    self.assertEqual(pld["board"], pld["sim"])
                    self.assertEqual(pld["replay"], pld["sim"])
        finally:
            shutil.rmtree(out)

    def test_latches_board_and_replay(self):
        """galre board learns latches through the host-built firmware (Gray
        code walks in rotated bit orders), as sim does, and replay from the
        recording."""
        out = tempfile.mkdtemp(prefix="galre")

        def galre_main(*argv):
            with contextlib.redirect_stdout(io.StringIO()):
                r = galre.main(list(argv) + ["--no-cupl"])
            with open(os.path.join(argv[argv.index("-o") + 1], "report.txt")) as f:
                return r, f.read()
        try:
            for name, (device, fix, latches, _, _) in LATCH_CASES.items():
                with self.subTest(design=name):
                    pld = {}
                    r, report = galre_main("sim", jed(name), "-o", os.path.join(out, "sim"),
                                           "--fix", fix, "-t", name)
                    self.assertEqual(r, 0)
                    dev = fwsim.SocketDevice(jed(name),
                                             "dip20" if device == "GAL16V8" else "dip24")
                    board = os.path.join(out, "board")
                    with mock.patch.object(galre, "SerialTransport",
                                           lambda port: fwsim.HarnessTransport(self.lib, dev)):
                        r, report = galre_main("board", "-p", "-", "-d", device, "-o",
                                               board, "--fix", fix, "-t", name)
                    self.assertEqual(r, 0)
                    self.assertIn("latch model: latch pins %s" % latches, report)
                    r, report = galre_main("replay", os.path.join(board, "capture.jsonl"),
                                           "-o", os.path.join(out, "replay"))
                    self.assertEqual(r, 0)
                    for d in ("sim", "board", "replay"):
                        with open(os.path.join(out, d, name + ".pld")) as f:
                            pld[d] = f.read()
                    self.assertEqual(pld["board"], pld["sim"])
                    self.assertEqual(pld["replay"], pld["sim"])
        finally:
            shutil.rmtree(out)

    def test_registers_and_latches_board_and_replay(self):
        """galre board learns latches in a registered design through the
        host-built firmware (walks with pin 1 enumerated), as sim does,
        and replay from the recording."""
        out = tempfile.mkdtemp(prefix="galre")

        def galre_main(*argv):
            with contextlib.redirect_stdout(io.StringIO()):
                r = galre.main(list(argv) + ["--no-cupl"])
            with open(os.path.join(argv[argv.index("-o") + 1], "report.txt")) as f:
                return r, f.read()
        try:
            for name, (device, fix, _, latches, _, _, _) in MIXED_CASES.items():
                with self.subTest(design=name):
                    pld = {}
                    r, report = galre_main("sim", jed(name), "-o", os.path.join(out, "sim"),
                                           "--fix", fix, "-t", name)
                    self.assertEqual(r, 0)
                    dev = fwsim.SocketDevice(jed(name),
                                             "dip20" if device == "GAL16V8" else "dip24")
                    board = os.path.join(out, "board")
                    with mock.patch.object(galre, "SerialTransport",
                                           lambda port: fwsim.HarnessTransport(self.lib, dev)):
                        r, report = galre_main("board", "-p", "-", "-d", device, "-o",
                                               board, "--fix", fix, "-t", name)
                    self.assertEqual(r, 0)
                    self.assertIn("latch model: latch pins", report)
                    r, report = galre_main("replay", os.path.join(board, "capture.jsonl"),
                                           "-o", os.path.join(out, "replay"))
                    self.assertEqual(r, 0)
                    for d in ("sim", "board", "replay"):
                        with open(os.path.join(out, d, name + ".pld")) as f:
                            pld[d] = f.read()
                    self.assertEqual(pld["board"], pld["sim"])
                    self.assertEqual(pld["replay"], pld["sim"])
        finally:
            shutil.rmtree(out)

    def test_oscillator_board_and_replay(self):
        """galre board infers an oscillating output through the host-built
        firmware as sim does (other noise, the same pins unsteady at the
        same vectors), and replay answers the repeated walks in the order
        recorded."""
        out = tempfile.mkdtemp(prefix="galre")
        name, fix = "osc16v8", OSC[0]

        def galre_main(*argv):
            with contextlib.redirect_stdout(io.StringIO()):
                r = galre.main(list(argv) + ["--no-cupl"])
            with open(os.path.join(argv[argv.index("-o") + 1], "report.txt")) as f:
                return r, f.read()
        try:
            pld = {}
            r, report = galre_main("sim", jed(name), "-o", os.path.join(out, "sim"),
                                   "--fix", fix, "-t", name)
            self.assertEqual(r, 0)
            dev = fwsim.SocketDevice(jed(name), "dip20")
            board = os.path.join(out, "board")
            with mock.patch.object(galre, "SerialTransport",
                                   lambda port: fwsim.HarnessTransport(self.lib, dev)):
                r, report = galre_main("board", "-p", "-", "-d", "GAL16V8", "-o", board,
                                       "--fix", fix, "-t", name)
            self.assertEqual(r, 0)
            self.assertIn("pin 18 does not settle at 4 of 16 input vectors, and pin 17 with "
                          "it at some: its own complement there, an oscillator", report)
            r, report = galre_main("replay", os.path.join(board, "capture.jsonl"),
                                   "-o", os.path.join(out, "replay"))
            self.assertEqual(r, 0)
            for d in ("sim", "board", "replay"):
                with open(os.path.join(out, d, name + ".pld")) as f:
                    pld[d] = f.read()
            self.assertEqual(pld["board"], pld["sim"])
            self.assertEqual(pld["replay"], pld["sim"])
        finally:
            shutil.rmtree(out)

    def test_latch_levels_board_and_replay(self):
        """galre board explores by registers and latch levels together
        (psig, also) through the host-built firmware as sim does, and
        replay finds every such job in the recording."""
        out = tempfile.mkdtemp(prefix="galre")
        name, fix = "routelatch22v10", ROUTELATCH[0]

        def galre_main(*argv):
            with contextlib.redirect_stdout(io.StringIO()):
                r = galre.main(list(argv) + ["--no-cupl"])
            with open(os.path.join(argv[argv.index("-o") + 1], "report.txt")) as f:
                return r, f.read()
        try:
            pld = {}
            r, report = galre_main("sim", jed(name), "-o", os.path.join(out, "sim"),
                                   "--fix", fix, "-t", name)
            self.assertEqual(r, 0)
            dev = fwsim.SocketDevice(jed(name), "dip24")
            board = os.path.join(out, "board")
            with mock.patch.object(galre, "SerialTransport",
                                   lambda port: fwsim.HarnessTransport(self.lib, dev)):
                r, report = galre_main("board", "-p", "-", "-d", "GAL22V10", "-o", board,
                                       "--fix", fix, "-t", name)
            self.assertEqual(r, 0)
            self.assertIn("6 states by registers and latch levels (pin 21), 4 by the registers "
                          "alone", report)
            r, report = galre_main("replay", os.path.join(board, "capture.jsonl"),
                                   "-o", os.path.join(out, "replay"))
            self.assertEqual(r, 0)
            for d in ("sim", "board", "replay"):
                with open(os.path.join(out, d, name + ".pld")) as f:
                    pld[d] = f.read()
            self.assertEqual(pld["board"], pld["sim"])
            self.assertEqual(pld["replay"], pld["sim"])
        finally:
            shutil.rmtree(out)

    def test_c_model(self):
        """fwstub/galmodel.c, the chip SimRig puts in the socket, reads the
        same as galsim.py's GAL in fwsim.SocketDevice after every write:
        single pins, clock edges, whole vectors and power cycles."""
        import ctypes
        import glob
        import random
        lib = ctypes.CDLL(self.lib)
        lib.gal_io.restype = ctypes.c_uint32
        lib.gal_io.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        lib.gal_power.argtypes = [ctypes.c_void_p, ctypes.c_int]
        for path in sorted(glob.glob(os.path.join(DESIGNS, "*.jed"))):
            for socket in (["dip20", "plcc20"] if GAL(path).kind == "GAL16V8"
                           else ["dip24", "plcc28"]):
                # also with the pins of one write arriving in any order
                race = 5 if socket in ("dip20", "dip24") else 0
                with self.subTest(design=os.path.basename(path), socket=socket):
                    dev = fwsim.SocketDevice(path, socket, race=race)
                    cdev = fwsim.SocketDevice(path, socket, race=race)
                    c = cdev.cgal(lib)
                    rnd = random.Random(7)
                    bits = [dev.sbit[p] for p in dev.pins]
                    v = 0
                    for i in range(1500):
                        r = rnd.random()
                        if i == 0 or r < 0.004:
                            on = int(i == 0 or r < 0.003)
                            dev.power(on)
                            lib.gal_power(c, on)
                            continue
                        if r < 0.5:
                            v ^= rnd.choice(bits)
                        elif r < 0.6:
                            v ^= dev.sbit[1]
                        else:
                            v = sum(b for b in bits if rnd.random() < 0.5)
                        self.assertEqual(lib.gal_io(c, v), dev.io(v), (i, hex(v)))

    def test_walk_with_pin_1(self):
        """A walk that enumerates pin 1 after a clocked path clocks the
        chip whenever it raises pin 1: cnt16v8 counts."""
        t = self.transport("cnt16v8", "dip20")
        rig = BrutusRig(t, "GAL16V8", log=quiet)
        V, base = [1, 2], {p: 0 for p in rig.pins if p > 2}
        g = GAL(jed("cnt16v8"))
        for path, start in (([], 0), ([{2: 1}], 3)):
            la, _, _ = rig.walk(V, base, path, gray=1, start=start)
            g.power_up()
            ext = {p: 0 for p in g.signal_pins()}
            g.settle(ext)
            for d in path:
                g.apply({**ext, 2: d.get(2, 0), 1: 0})
                g.apply({**ext, 2: d.get(2, 0), 1: 1})
            for x in gray_order(2, 0, 0, start):
                levels, _ = g.apply({**ext, 1: x & 1, 2: x >> 1})
                for p in (18, 19):
                    self.assertEqual(la[p] >> x & 1, levels[p], (path, x, p))

    def test_transition_check(self):
        """trans with chk flags the vectors whose new state the probes
        change: in arlow22v10 a probe with every input low resets it."""
        rig = BrutusRig(self.transport("arlow22v10", "dip24"), "GAL22V10", log=quiet)
        V = [2, 3, 5]
        base = {p: 0 for p in rig.pins if p not in V and p != 1}
        _, flags = rig.transitions(V, base, [], [{}], [22, 23], chk=True)
        # P2 (EN) and P3 (!RST) high; the reset changes pin 23
        self.assertEqual(sorted(flags), [(3, (23,)), (7, (23,))])
        _, flags = rig.transitions(V, base, [], [{}], [22, 23], chk=True, ignore=[23])
        self.assertEqual(flags, [])
        _, flags = rig.transitions(V, base, [], [{3: 1}], [22, 23], chk=True)
        self.assertEqual(flags, [])
        rig = BrutusRig(self.transport("fsm22v10", "dip24"), "GAL22V10", log=quiet)
        _, flags = rig.transitions([2, 3, 4], {}, [{2: 1}], [{}], [21, 22, 23], chk=True)
        self.assertEqual(flags, [])

    def test_reads_after_every_step(self):
        """With psig, power-up and every clocked step are followed by the
        signature reads, so a path leads to the state that a sweep saw,
        latches included: the sweep's observation of a vector is the one
        before a walk from the path with that vector, and it ends with
        the level of the pins given as also."""
        rig = BrutusRig(self.transport("routelatch22v10", "dip24"), "GAL22V10", log=quiet)
        V = [2, 3, 4]
        base = {p: 0 for p in rig.pins if p not in V and p != 1}
        kw = dict(psig=True, also=(21,))
        def drive(x):
            d = dict(base)
            d.update((p, x >> b & 1) for b, p in enumerate(V))
            return d
        out, flags = rig.transitions(V, base, [], [drive(0)], [22, 23], chk=True,
                                     nochk=[21], **kw)
        self.assertEqual(flags, [])
        seen = {}
        for o, xs in out.items():
            for x in range(8):
                if xs >> x & 1:
                    seen[x] = o
        # S and D: Q0 set and the latch set; D alone: Q0 set, the latch not
        self.assertEqual(seen[5][-1], (1,))
        self.assertEqual(seen[4][-1], (0,))
        self.assertEqual(seen[5][:-1], seen[4][:-1])
        for x in (4, 5):
            _, _, sigs = rig.walk(V, base, [drive(x)], probes=[drive(0)], cand=[22, 23],
                                  data=False, **kw)
            self.assertEqual(sigs[0], seen[x])
        # from the state with the latch set, every input low sets Q1; without, not
        q1 = {}
        for x in (4, 5):
            out, _ = rig.transitions(V, base, [drive(x)], [drive(0)], [22, 23], **kw)
            q1[x] = [o[0][0] for o, xs in out.items() if xs & 1][0]
        self.assertEqual(q1, {4: (0, 0), 5: (1, 1)})
        # chkm: reglatch22v10's latch pins change when a state is read
        rig = BrutusRig(self.transport("reglatch22v10", "dip24"), "GAL22V10", log=quiet)
        V = [2, 3, 4, 5, 6]
        base = {p: 0 for p in rig.pins if p not in V and p != 1}
        job = (V, base, [drive(9)], [drive(12)], [20, 21, 22, 23])
        _, flags = rig.transitions(*job, chk=True, avoid={5: 0})
        self.assertIn((10, (21,)), flags)
        self.assertTrue(all(set(pins) <= {20, 21} for _, pins in flags))
        _, flags = rig.transitions(*job, chk=True, avoid={5: 0}, nochk=[20, 21])
        self.assertEqual(flags, [])
        t = self.transport("cnt16v8", "dip20")
        t.command("pld capture clear")
        t.command("pld capture set pins=6 clk=1 psig=1")
        self.assertIn("psig needs probe vectors", self._walk_error(t))

    def test_mixed_gray_walk(self):
        """The firmware walks a mixed Gray code in the order rig.gray_order
        gives: a combinational chip reads the same at every vector as in
        counting order, also rotated, reversed and from another vector."""
        t = self.transport("comb16v8", "dip20")
        rig = BrutusRig(t, "GAL16V8", log=quiet)
        V = [p for p in rig.pins if p < 12]
        base = {}
        la0, lb0, _ = rig.walk(V, base, [])
        for kw in (dict(mix=1), dict(mix=0xbeef, rot=3), dict(mix=2, rev=1, start=0x155)):
            with self.subTest(**kw):
                la, lb, _ = rig.walk(V, base, [], gray=1, **kw)
                self.assertEqual((la, lb), (la0, lb0))
        self.assertIn("mix needs gray", t.command("pld capture set mix=1 gray=0") or
                      self._walk_error(t))

    def _walk_error(self, t):
        try:
            t.capture("pld capture walk")
        except CaptureError as e:
            return str(e)
        return ""

    def test_gray_skip_quiet_walks(self):
        """Gray code walks from a start vector, with skipped vectors, decode
        to the counting walk (skipped vectors read 0); a quiet walk
        returns only the signatures."""
        t = self.transport("fsm22v10", "dip24")
        rig = BrutusRig(t, "GAL22V10", log=quiet)
        sim = SimRig(jed("fsm22v10"))
        V = [2, 3, 4, 5, 13, 14, 20]
        base = {6: 1, 9: 1, 15: 1}
        path = [{2: 1}, {3: 1}]
        probes, cand = [{4: 1}, {}], [21, 22, 23]
        for opts in ({"gray": 1}, {"gray": 1, "start": 0x55}, {"start": 0x0f, "rev": 1},
                     {"gray": 1, "rot": 3}, {"gray": 1, "rot": 6, "rev": 1, "start": 0x2a},
                     {"gray": 1, "start": 3, "skip": {3: 1, 13: 0}, "avoid": {3: 1, 13: 0}}):
            with self.subTest(**{k: str(v) for k, v in opts.items()}):
                self.assertEqual(rig.walk(V, base, path, probes=probes, cand=cand, **opts),
                                 sim.walk(V, base, path, probes=probes, cand=cand, **opts))
        la, lb, sigs = rig.walk(V, base, path, probes=probes, cand=cand, data=False)
        self.assertEqual((la, lb), (None, None))
        self.assertEqual(sigs, sim.walk(V, base, path, probes=probes, cand=cand)[2])

    def test_avoid_writes(self):
        """A walk that skips a cube spanning two port groups still passes
        through it in the momentary mixes of old and new pin values (when a
        write raises P3 and P13 together); with avoid it never does."""
        dev = fwsim.SocketDevice(jed("ar22v10"), "dip24")
        rig = BrutusRig(fwsim.HarnessTransport(self.lib, dev), "GAL22V10", log=quiet)
        V = [2, 3, 4, 5, 13]                  # socket pins 2-5, then 17
        cube = {3: 1, 13: 0}
        m, v = rig._mask(cube), rig._mask([3])
        seen = []
        apply = dev._apply

        def record(value):
            seen.append(value)
            return apply(value)
        dev._apply = record
        for avoid in (None, cube):
            with self.subTest(avoid=avoid):
                seen.clear()
                rig.walk(V, {}, [{2: 1}], mul=0x13, skip=cube, avoid=avoid)
                self.assertEqual(any(w & m == v for w in seen), avoid is None)

    def test_walk_orders(self):
        """Every walk order, with and without numpy, decodes to the counting
        walk of a clocked design, and its signatures before and after
        agree with the simulator's."""
        t = self.transport("fsm22v10", "dip24")
        rig = BrutusRig(t, "GAL22V10", log=quiet)
        sim = SimRig(jed("fsm22v10"))
        V = [2, 3, 4, 5, 13, 14, 20]
        base = {6: 1, 9: 1, 15: 1}
        path = [{2: 1}, {3: 1}]
        probes, cand = [{4: 1}, {}], [21, 22, 23]
        ref = sim.walk(V, base, path, probes=probes, cand=cand)
        self.assertEqual(ref[2][0], ref[2][1])
        for name, kw in Analysis.ORDERS + (("scattered", dict(mul=0x35)),):
            kw = dict(kw)
            if "mul" in kw:
                kw["mul"] &= 0x7f
            for numpy in (True, False):
                with self.subTest(order=name, numpy=numpy):
                    with mock.patch.dict(sys.modules, {} if numpy else {"numpy": None}):
                        got = rig.walk(V, base, path, probes=probes, cand=cand, **kw)
                    self.assertEqual(got, ref)

    def test_walk_without_second_read(self):
        """With every pin outside the enumeration held (as in the power-up
        walk without --fix), there is nothing to invert and the firmware
        reads each vector once."""
        rig = BrutusRig(self.transport("fsm22v10", "dip24"), "GAL22V10", log=quiet)
        sim = SimRig(jed("fsm22v10"))
        V = [2, 3, 4, 13]
        base = {p: 0 for p in rig.pins if p not in V and p != 1}
        got = rig.walk(V, base, [])
        self.assertEqual(got, sim.walk(V, base, []))
        self.assertEqual(got[0], got[1])

    def test_port_write_transients(self):
        """A write that changes pins in more than one port group passes
        through the value with only the first groups written."""
        dev = fwsim.SocketDevice(jed("latch16v8"), "dip20")
        seen = []
        apply = dev._apply

        def record(value):
            seen.append(value)
            return apply(value)
        dev._apply = record
        dev.power(1)
        dev.io(0x1000001)               # socket pins 1 and 25
        dev.io(0x1000001)
        dev.io(0x0010000)               # pins 1 and 25 off, 17 on
        self.assertEqual(seen, [0x0000001, 0x1000001, 0x1000001,
                                0x1000000, 0x1010000, 0x0010000])

    def test_galre_board_and_replay(self):
        """galre board through the host-built firmware, then galre replay
        with nothing but the recording."""
        dev = fwsim.SocketDevice(jed("comb16v8"), "dip20")
        out = tempfile.mkdtemp(prefix="galre")

        def galre_main(*argv):
            with contextlib.redirect_stdout(io.StringIO()):
                r = galre.main(list(argv) + ["--no-cupl"])
            with open(os.path.join(argv[argv.index("-o") + 1], "report.txt")) as f:
                return r, f.read()
        try:
            board = os.path.join(out, "board")
            with mock.patch.object(galre, "SerialTransport",
                                   lambda port: fwsim.HarnessTransport(self.lib, dev)):
                dev.jumper_gnd = 1 << 11
                r, report = galre_main("board", "-p", "-", "-d", "GAL16V8", "-o", board)
                self.assertEqual(r, 2)
                self.assertIn("detected: VCC on socket pin 28, GND on socket pin 12, "
                              "supply not measured", report)
                self.assertEqual((dev.power_ons, dev.supply_measured), (0, 0))
                dev.jumper_gnd = dev.gnd_bit
                r, report = galre_main("board", "-p", "-", "-d", "GAL16V8", "-o", board,
                                       "-t", "comb", "--fix", "4-9=0,11-16=0", "--survey", "8")
                self.assertEqual(r, 0)
                self.assertIn("jumpers: VCC on socket [28], GND on socket [10], "
                              "supply 4950 mV", report)
                self.assertIn("jumper detection: jumpers: (host harness", report)
            replayed = os.path.join(out, "replay")
            r, report = galre_main("replay", os.path.join(board, "capture.jsonl"), "-o", replayed)
            self.assertEqual(r, 0)
            for line in ("title: comb", "device: GAL16V8", "socket: dip20",
                         "firmware: Version host harness", "held pins: 4=0, 5=0", "survey: 8"):
                self.assertIn("recorded " + line, report)
            with open(os.path.join(board, "comb.pld")) as f, \
                    open(os.path.join(replayed, "comb.pld")) as g:
                self.assertEqual(f.read(), g.read())
        finally:
            shutil.rmtree(out)

    def test_commands(self):
        t = self.transport("cnt16v8", "dip20")
        t.command("pld capture clear")          # the library is shared between tests
        self.assertEqual(t.command("pld capture set pins=3 base=0x8000000 settle=2"), "")
        self.assertIn("Invalid", t.command("pld capture set pins=xyz"))
        self.assertIn("Invalid", t.command("pld capture set foo=1"))
        self.assertIn("overlap", t.command("pld capture set inv=1 clk=1"))
        self.assertEqual(t.command("pld capture set inv=0 pins=1 clk=1"), "")   # for a walk
        t.command("pld capture probe 0")
        with self.assertRaisesRegex(CaptureError, "trans: pins and clk must not overlap"):
            t.capture("pld capture trans")
        t.command("pld capture set pins=3 clk=0")
        self.assertIn("At most", t.command("pld capture set cand=fff1fff"))
        self.assertEqual(t.command("pld capture probe 1 2"), "")
        self.assertEqual(t.command("pld capture probe + 3"), "")
        self.assertIn("probe (3): 0000001 0000002 0000003", t.command("pld capture show"))
        self.assertEqual(t.command("pld capture ret 1234abcd 5 6"), "")
        self.assertIn("ret 1234abcd", t.command("pld capture show"))
        t.command("pld capture clear")
        with self.assertRaises(CaptureError):
            t.capture("pld capture trans")         # no probes: refused

    def test_walk_matches_simulation(self):
        t = self.transport("fsm22v10", "dip24")
        rig = BrutusRig(t, "GAL22V10", log=quiet)
        sim = SimRig(jed("fsm22v10"))
        V = [2, 3, 4, 5, 13, 14, 20]
        base = {6: 1, 9: 1, 15: 1}
        path = [{2: 1}, {3: 1}]
        self.assertEqual(rig.walk(V, base, path), sim.walk(V, base, path))

    def test_state(self):
        t = self.transport("cnt16v8", "dip20")
        t.command("pld capture set pins=6 clk=1 cand=c0000")
        t.command("pld capture probe 400 0")
        cap = t.capture("pld capture state")
        self.assertEqual((cap.kind, cap.status, cap.done), ("S", 0, 1))
        self.assertEqual(len(cap.sigs), 1)
        self.assertEqual(len(cap.sigs[0]), 2 * 3)

    def _edges(self, follow, dclk=False):
        """fsm22v10 explored through the firmware: the analysis, the pins
        driven against the GAL at a rising clock edge, the pins driven
        directly."""
        t = self.transport("fsm22v10", "dip24")
        dev, io = t.dev, t.dev.io
        log = []

        def logged(value):
            r = io(value)
            if dev.powered:
                log.append((value, r))
            return r
        dev.io = logged
        t.lib.harness_direct()
        rig = BrutusRig(t, "GAL22V10", log=quiet, follow=follow, dclk=dclk)
        a = Analysis(rig, "GAL22V10", fixed=CASES["fsm22v10"][3], log=quiet)
        a.run()
        direct = t.lib.harness_direct()
        clk, mask = rig.sbit[1], rig.readmask
        fought = 0
        for (_, r), (w, _) in zip(log, log[1:]):
            if w & clk and not r & clk:
                fought |= (w ^ r) & mask & ~clk
        return a, fought, direct

    def test_follow_and_dclk(self):
        plain, fought, direct = self._edges(follow=False)
        self.assertNotEqual(fought, 0)      # the test sees a fight
        self.assertEqual(direct, 0)
        a, fought, direct = self._edges(follow=True, dclk=True)
        self.assertEqual(fought, 0)
        self.assertEqual(direct, 1 << 0)    # socket pin 1, the clock
        self.assertEqual(a.equations(), plain.equations())
        self.assertEqual(a.trans, plain.trans)

    def _most_pins_per_write(self, serial):
        """fsm22v10 explored through the firmware: the analysis, and per
        kind of job the most device pins that one write changed."""
        t = self.transport("fsm22v10", "dip24")
        dev, io, capture = t.dev, t.dev.io, t.capture
        pins = sum(dev.sbit[p] for p in dev.pins)
        most, job = {}, [None]

        def logged(value):
            if dev.powered:
                n = bin((value ^ dev.last) & pins).count("1")
                most[job[0]] = max(most.get(job[0], 0), n)
            return io(value)

        def run(line):
            job[0] = line.split()[-1]
            return capture(line)
        dev.io, t.capture = logged, run
        rig = BrutusRig(t, "GAL22V10", log=quiet, serial=serial)
        a = Analysis(rig, "GAL22V10", fixed=CASES["fsm22v10"][3], log=quiet)
        a.run()
        return a, most

    def test_one_pin_per_write(self):
        """With ser set, no write changes more than one pin: the vectors
        of a walk, clocked steps, the clock edge, the reads of a state,
        the way back."""
        a, most = self._most_pins_per_write(serial=True)
        self.assertEqual((most["walk"], most["trans"]), (1, 1))
        b, most = self._most_pins_per_write(serial=False)
        self.assertGreater(most["walk"], 1)
        self.assertGreater(most["trans"], 1)
        self.assertEqual(a.equations(), b.equations())
        self.assertEqual(a.trans, b.trans)

    def test_firmware_without_ser(self):
        """A firmware that does not know ser= is used as it is, and the
        run says so once."""
        t = self.transport("fsm22v10", "dip24")
        command = t.command
        t.command = lambda line: ("Unknown setting ser=1\n" if "ser=" in line
                                  else command(line))
        logs = []
        rig = BrutusRig(t, "GAL22V10", log=logs.append)
        self.assertFalse(rig.metadata()["serial"])
        a = Analysis(rig, "GAL22V10", fixed=CASES["fsm22v10"][3], log=quiet)
        a.run()
        self.assertIsNone(rig.serial)
        self.assertEqual(sum("it has no ser=" in line for line in logs), 1)
        b, _ = self._most_pins_per_write(serial=True)
        self.assertEqual(a.equations(), b.equations())
        self.assertTrue(BrutusRig(self.transport("fsm22v10", "dip24"), "GAL22V10",
                                  log=quiet).metadata()["serial"])

    def test_job_that_lost_its_state_runs_again(self):
        """The firmware gives a job up when two power cycles and the same
        clocks do not lead back to its state.  A chip that reads wrongly
        now and then does that: the job is run again, twice at most."""
        t = self.transport("fsm22v10", "dip24")
        capture, lost = t.capture, [1]

        def run(line):
            cap = capture(line)
            if lost[0]:
                lost[0] -= 1
                cap.status = 3
            return cap
        t.capture = run
        logs = []
        rig = BrutusRig(t, "GAL22V10", log=logs.append)
        first = rig.walk([2, 3], {}, [])
        self.assertEqual(sum("running the job again" in line for line in logs), 1)
        self.assertEqual(rig.walk([2, 3], {}, []), first)
        lost[0] = 3
        with self.assertRaises(CaptureError) as e:
            rig.walk([2, 3], {}, [])
        self.assertIn("could not get back to the path state", str(e.exception))
        self.assertEqual(lost[0], 0)

    def test_inputs_that_fall_with_the_enable(self):
        """A chip that now and then loses the state of its registers when
        the pin that enables its outputs falls in one write with other
        inputs (fwsim.SocketDevice, glitch), as a part did on a board.
        With one pin per write it never does; with all pins of a vector
        in one write, galre gets a chip that it is not."""
        fix = galre.parse_fixed("6-11=0,14-19=0,23=0")
        for seed in (1, 2):
            a = galre.analyse(SimRig(jed("lowreg22v10"), glitch=seed), "GAL22V10", fixed=fix,
                              survey=16, log=quiet)
            self.assertEqual(sorted(a.reg), [20, 21, 22])
            self.assertEqual(a.unsteady, [0, 0])
            self.assertEqual(verify.check_captures(a, jed("lowreg22v10")), [])
        good = a.trans
        try:
            b = galre.analyse(SimRig(jed("lowreg22v10"), glitch=1, serial=False, twice=False),
                              "GAL22V10", fixed=fix, survey=16, log=quiet)
        except (AsyncDesign, Limit):
            b = None
        self.assertTrue(b is None or b.trans != good)

    def test_power_cycle_that_does_not_reset(self):
        """A chip that keeps its charge while the supply is off is not
        reset when it comes back soon (fwsim.SocketDevice, hold_ms): it
        comes up in another state, and every job would start from that.
        The rig finds a power-off time that resets it."""
        io, fix = DEV["GAL22V10"]["io"], galre.parse_fixed("6-11=0,14-19=0,23=0")

        def rig(hold, logs, **kw):
            t = fwsim.HarnessTransport(self.lib, fwsim.SocketDevice(jed("lowreg22v10"), "dip24",
                                                                 hold_ms=hold))
            r = BrutusRig(t, "GAL22V10", log=logs.append, **kw)
            if r.jumper_check:
                r.preflight()
            return r
        for hold, off in ((0, 20), (30, 60), (70, 120), (300, 480)):
            logs = []
            r = rig(hold, logs)
            self.assertEqual(r.check_power_cycle(io, fix), off)
            self.assertEqual(r.metadata()["off_ms"], off)
            said = [line for line in logs if "does not reset this chip" in line]
            self.assertEqual(len(said), int(hold > 0))
            if hold:
                self.assertIn("power is now off for %d ms" % off, said[0])
        # with it, the chip comes out as it is
        r = rig(30, [])
        r.check_power_cycle(io, fix)
        a = galre.analyse(r, "GAL22V10", fixed=fix, survey=16, log=quiet)
        self.assertEqual(verify.check_captures(a, jed("lowreg22v10")), [])
        # without it, a power cycle leads back to the state of a job only if
        # one clock with every input low leads to the same state from anywhere
        with self.assertRaises(CaptureError) as e:
            galre.analyse(rig(30, []), "GAL22V10", fixed=fix, survey=16, log=quiet)
        self.assertIn("could not get back to the path state", str(e.exception))
        self.assertIn("give a longer --off", str(e.exception))
        # no jumper check: nothing is powered before a job, nothing to find
        r = rig(30, [], jumper_check=False)
        self.assertEqual(r.check_power_cycle(io, fix), 20)

    def test_outputs_switched_on_together(self):
        """A chip that takes it for a clock edge when six of its outputs
        are switched on at once, each against the level its pin is held
        at (fwsim.SocketDevice, bounce), as a PALCE20V8 did on a board
        whenever the pin that enables its registers fell while the board
        held six of their pins low.  With that pin held low and walked
        alone, no output is switched on against the board.  Walked with
        the other inputs, the registers change in the walks, and galre
        takes them for latches."""
        fix = "5-11=0,14-15=0,23=0"
        fixed = galre.parse_fixed(fix)
        rig = SimRig(jed("regs20v8"), bounce=6)
        a = galre.analyse(rig, "GAL20V8", fixed=fixed, survey=16, log=quiet)
        self.assertEqual(rig.socket_device.bounced, 0)
        self.assertEqual((len(a.states), sorted(a.reg)), (32, [17, 18, 19, 20, 21, 22]))
        self.assertEqual(verify.check_captures(a, jed("regs20v8")), [])
        rig = SimRig(jed("regs20v8"), bounce=6)
        logs = []
        try:
            b = galre.analyse(rig, "GAL20V8", fixed=fixed, survey=16, log=logs.append,
                              oe_first=False)
        except (AsyncDesign, Limit):
            b = None
        self.assertGreater(rig.socket_device.bounced, 0)
        self.assertIn("the chip holds state without a clock", logs[0])
        self.assertTrue(b is None or getattr(b, "trans", None) != a.trans)

        # on the board, and from its recording
        def galre_main(*argv):
            with contextlib.redirect_stdout(io.StringIO()):
                r = galre.main(list(argv) + ["--no-cupl"])
            with open(os.path.join(argv[argv.index("-o") + 1], "report.txt")) as f:
                return r, f.read()

        def source(d):
            with open(os.path.join(d, "regs.pld")) as f:
                return f.read()
        out = tempfile.mkdtemp(prefix="galre")
        try:
            board, old = os.path.join(out, "board"), os.path.join(out, "old")
            dev = fwsim.SocketDevice(jed("regs20v8"), "dip24", bounce=6)
            with mock.patch.object(galre, "SerialTransport",
                                   lambda port: fwsim.HarnessTransport(self.lib, dev)):
                r, report = galre_main("board", "-p", "-", "-d", "GAL20V8", "-o", board,
                                       "-t", "regs", "--fix", fix, "--survey", "16")
            self.assertEqual((r, dev.bounced), (0, 0))
            self.assertIn("\npin 13: held low and read alone, if clock edges change the chip\n",
                          report)
            self.assertIn("\npin 13, walked alone, switches off pins 17, 18, 19, 20, 21, 22\n",
                          report)
            self.assertIn("\nPin 13 = !OE;\n", report)
            r, report = galre_main("replay", os.path.join(board, "capture.jsonl"),
                                   "-o", os.path.join(out, "replay"))
            self.assertEqual(r, 0)
            self.assertIn("recorded pin 13: held low and read alone", report)
            self.assertEqual(source(os.path.join(out, "replay")), source(board))
            # a recording of a run that walked the pin with the others (as every
            # recording before this one was) is replayed the same way
            dev = fwsim.SocketDevice(jed("regs20v8"), "dip24")
            with mock.patch.object(galre, "SerialTransport",
                                   lambda port: fwsim.HarnessTransport(self.lib, dev)):
                r, report = galre_main("board", "-p", "-", "-d", "GAL20V8", "-o", old,
                                       "-t", "regs", "--fix", fix, "--survey", "16",
                                       "--no-oe-first")
            self.assertEqual(r, 0)
            self.assertIn("\npin 13: walked with the other inputs\n", report)
            rec = os.path.join(old, "capture.jsonl")
            with open(rec) as f:
                lines = f.read().split("\n", 1)
            self.assertIn('"oe_first": false', lines[0])
            with open(rec, "w") as f:
                f.write(lines[0].replace(', "oe_first": false', "") + "\n" + lines[1])
            r, report = galre_main("replay", rec, "-o", os.path.join(out, "replay2"))
            self.assertEqual(r, 0)
            self.assertNotIn("held low and read alone", report)
            self.assertEqual(source(os.path.join(out, "replay2")), source(old))
        finally:
            shutil.rmtree(out)

    def test_check_that_cannot_run(self):
        """The check of the rebuilt JED takes the firmware built for the
        host where the design has latches.  Where that cannot be built,
        galre says so and ends with exit code 1, the equations written;
        it used to end in a traceback, after the captures."""
        if not CUPL:
            self.skipTest("no CUPL compiler")
        name, (device, fix, _, _, _) = "latch16v8", LATCH_CASES["latch16v8"]
        out = tempfile.mkdtemp(prefix="galre")

        def run(*argv):
            with contextlib.redirect_stdout(io.StringIO()):
                r = galre.main(list(argv) + ["--cupl", CUPL])
            with open(os.path.join(argv[argv.index("-o") + 1], "report.txt")) as f:
                return r, f.read()
        try:
            board, rec = os.path.join(out, "board"), os.path.join(out, "board", "capture.jsonl")
            dev = fwsim.SocketDevice(jed(name), "dip20")
            with mock.patch.object(galre, "SerialTransport",
                                   lambda port: fwsim.HarnessTransport(self.lib, dev)):
                r, report = run("board", "-p", "-", "-d", device, "-o", board, "--fix", fix,
                                "-t", name)
            self.assertEqual(r, 0)
            self.assertIn(name + ".jed against the captures", report)
            # no C compiler
            with mock.patch.object(fwsim, "library", return_value=None):
                r, report = run("replay", rec, "-o", os.path.join(out, "nocc"))
            self.assertEqual(r, 1)
            self.assertIn("The rebuilt JED could not be checked against the captures: ", report)
            self.assertIn("needs a C compiler", report)
            self.assertIn(name + ".pld is written, but nothing says yet that it is the chip.",
                          report)
            self.assertTrue(os.path.exists(os.path.join(out, "nocc", name + ".pld")))
            # galtools copied without the firmware's sources
            with mock.patch.object(fwsim, "FW", os.path.join(out, "nowhere")), \
                    mock.patch.object(fwsim, "_library", []):
                r, report = run("replay", rec, "-o", os.path.join(out, "nofw"))
            self.assertEqual(r, 1)
            self.assertIn("could not be checked against the captures: the firmware's capture.c "
                          "is not beside galtools", report)
        finally:
            shutil.rmtree(out)

    def test_dclk_refuses_supply_pin(self):
        t = self.transport("fsm22v10", "dip24")
        self.assertIn("dclk: a clock pin is a VCC or GND pin",
                      t.command("pld capture set clk=8000000 vcc=8000000 gnd=800 dclk=1"))

    def test_analysis_through_firmware(self):
        for name, (device, nstates, regs, fixed) in CASES.items():
            for socket in (["dip20", "plcc20"] if device == "GAL16V8" else ["dip24", "plcc28"]):
                with self.subTest(design=name, socket=socket):
                    rig = BrutusRig(self.transport(name, socket), device, socket, log=quiet)
                    a = Analysis(rig, device, fixed=fixed, log=quiet)
                    eqs = a.run()
                    sim = SimRig(jed(name))
                    b = Analysis(sim, device, fixed=fixed, log=quiet)
                    self.assertEqual(eqs, b.run())
                    self.assertEqual(a.states.keys(), b.states.keys())
                    self.assertEqual(a.trans, b.trans)
                    self.assertEqual(sorted(a.reg), regs)
                    self.assertEqual(len(a.states), nstates)
                    if nstates > 1:
                        self.assertGreater(rig.ops["ret_ok"], 0)
                        self.assertEqual(rig.ops["ret_fail"], 0)


if __name__ == "__main__":
    unittest.main()
