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
from analysis import Analysis, LatchSuspected      # noqa: E402
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
}

# designs with state that changes without a clock that galre does not
# model -> (device, pins held, what galre must say when it stops)
ASYNC_CASES = {
    "buried22v10": ("GAL22V10", "5-11=0,13-21=0",
                    "led to state 1Z, but the same clocks from power-up lead to 0Z: the chip "
                    "has state that no pin shows, or its readings do not repeat"),
}

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
    "oeread22v10": ("GAL22V10", "4-11=0,13-20=0", [], [], [], 1, {
        (21, ""): "P3 & P23",
        (22, ""): "'b'0",
        (22, "oe"): "!P23",
        (23, ""): "'b'0",
        (23, "oe"): "P2"}),
}

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
}


def jed(name):
    return os.path.join(DESIGNS, name + ".jed")


def quiet(*_):
    pass


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
                 list(LATCH_CASES) + list(MIXED_CASES)}
        ar = ["an asynchronous reset term"]
        self.assertEqual(found, {
            "cnt16v8": [], "fsm22v10": [], "comb16v8": [],
            "ar22v10": ar, "arlow22v10": ar, "arw22v10": ar,
            "latch16v8": ["combinational feedback through pins 17, 18"],
            "latch22v10": ["combinational feedback through pins 21, 22, 23"],
            "dlat22v10": ["combinational feedback through pins 14, 15"],
            "srnor16v8": ["combinational feedback through pins 16, 17, 18"],
            "reglatch16v8": ["combinational feedback through pin 17"],
            "latchd16v8": ["combinational feedback through pin 17"],
            "reglatch22v10": ar + ["combinational feedback through pin 21"],
            "trilatch22v10": ["combinational feedback through pin 22"],
            "quietlatch16v8": ["combinational feedback through pin 18"],
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
        still.  In the counting orders the chip reads as combinational,
        with a wrong equation; the Gray code orders show the latch."""
        fixed = galre.parse_fixed(LATCH_CASES["quietlatch16v8"][1])
        with mock.patch.object(Analysis, "ORDERS", Analysis.ORDERS[:3]):
            a = Analysis(SimRig(jed("quietlatch16v8")), "GAL16V8", fixed=fixed, log=quiet)
            a.run()
        self.assertIn((18, "", False, "P4", 1), a.eqs)        # L = A: wrong
        a = Analysis(SimRig(jed("quietlatch16v8")), "GAL16V8", fixed=fixed, log=quiet)
        with self.assertRaisesRegex(LatchSuspected, "pin 18 read differently .* Gray code"):
            a.run()

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
        cases += [("racelatch16v8", "GAL16V8", galre.parse_fixed("6-9=0,11-14=0,18-19=0"))]
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

    def test_async_designs_stop(self):
        """galre sim and board stop with exit code 3 on a chip with state
        that changes without a clock, and replay says the same."""
        out = tempfile.mkdtemp(prefix="galre")

        def galre_main(*argv):
            with contextlib.redirect_stdout(io.StringIO()):
                r = galre.main(list(argv) + ["--cupl", ""])
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
                    dev = fwsim.SocketDevice(jed(name), "dip24" if device == "GAL22V10" else "dip20")
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
                r = galre.main(list(argv) + ["--cupl", ""])
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
                r = galre.main(list(argv) + ["--cupl", ""])
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
                                             "dip24" if device == "GAL22V10" else "dip20")
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
                r = galre.main(list(argv) + ["--cupl", ""])
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
                                             "dip24" if device == "GAL22V10" else "dip20")
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
            for socket in (["dip24", "plcc28"] if GAL(path).kind == "GAL22V10" else ["dip20"]):
                with self.subTest(design=os.path.basename(path), socket=socket):
                    dev = fwsim.SocketDevice(path, socket)
                    cdev = fwsim.SocketDevice(path, socket)
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
                r = galre.main(list(argv) + ["--cupl", ""])
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

    def test_dclk_refuses_supply_pin(self):
        t = self.transport("fsm22v10", "dip24")
        self.assertIn("dclk: a clock pin is a VCC or GND pin",
                      t.command("pld capture set clk=8000000 vcc=8000000 gnd=800 dclk=1"))

    def test_analysis_through_firmware(self):
        for name, (device, nstates, regs, fixed) in CASES.items():
            for socket in (["dip24", "plcc28"] if device == "GAL22V10" else ["dip20"]):
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
