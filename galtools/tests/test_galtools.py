"""galtools tests: python3 -m unittest discover -s galtools/tests

The firmware tests compile fw/capture.c for the host (needs a C compiler)
and run the analysis through it; the CUPL round trips need a CUPL compiler
($CUPL or cupl on the PATH).  Both are skipped without them.
"""

import os
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import fwharness                                   # noqa: E402
import verify                                      # noqa: E402
from analysis import Analysis                      # noqa: E402
from capture import CaptureError                   # noqa: E402
from rig import BrutusRig, SimRig                  # noqa: E402

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

    def test_return_paths_replace_power_cycles(self):
        rig = SimRig(jed("cnt16v8"))
        a = Analysis(rig, "GAL16V8", log=quiet)
        a.run()
        self.assertGreater(rig.ops["ret_ok"], 10 * rig.ops["power_cycles"])
        self.assertEqual(rig.ops["ret_fail"], 0)

    @unittest.skipUnless(CUPL, "no CUPL compiler")
    def test_cupl_round_trip(self):
        out = tempfile.mkdtemp(prefix="galre")
        try:
            for name, (device, _, _, _) in CASES.items():
                with self.subTest(design=name):
                    a = Analysis(SimRig(jed(name)), device, log=quiet)
                    a.run()
                    pld = os.path.join(out, name + ".pld")
                    with open(pld, "w") as f:
                        f.write(a.cupl(name))
                    subprocess.run([CUPL, "-jaxm4", name + ".pld"], cwd=out,
                                   capture_output=True, check=True)
                    rebuilt = os.path.join(out, name + ".jed")
                    self.assertEqual(verify.check_captures(a, rebuilt), [])
                    d, first, _ = verify.compare_jeds(jed(name), rebuilt, 5000)
                    self.assertEqual(d, {}, first)
        finally:
            shutil.rmtree(out)


class Firmware(unittest.TestCase):
    """fw/capture.c on the host, a simulated GAL in the socket."""

    @classmethod
    def setUpClass(cls):
        try:
            cls.lib = fwharness.build()
        except subprocess.CalledProcessError as e:
            raise AssertionError("fw/capture.c does not build for the host: %s" % e)
        if not cls.lib:
            raise unittest.SkipTest("no C compiler")

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(os.path.dirname(cls.lib), ignore_errors=True)

    def transport(self, name, socket):
        return fwharness.HarnessTransport(self.lib, fwharness.SocketDevice(jed(name), socket))

    def test_commands(self):
        t = self.transport("cnt16v8", "dip20")
        self.assertEqual(t.command("pld capture set pins=3 base=0x8000000 settle=2"), "")
        self.assertIn("Invalid", t.command("pld capture set pins=xyz"))
        self.assertIn("Invalid", t.command("pld capture set foo=1"))
        self.assertIn("overlap", t.command("pld capture set pins=1 clk=1"))
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
