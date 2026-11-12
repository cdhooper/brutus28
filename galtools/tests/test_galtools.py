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

import fwharness                                   # noqa: E402
import galre                                       # noqa: E402
import verify                                      # noqa: E402
from analysis import Analysis                      # noqa: E402
from capture import CaptureError                   # noqa: E402
from rig import BrutusRig, RecordingRig, ReplayRig, SimRig  # noqa: E402

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

    def test_jumper_preflight(self):
        t = self.transport("cnt16v8", "dip20")
        dev = t.dev
        vcc, gnd = dev.vcc_bit, dev.gnd_bit          # socket pins 28 and 10
        self.assertEqual(t.command("pld capture jumpers"),
                         "jumpers vcc=%07x gnd=%07x supply=5000\n" % (vcc, gnd))
        # base drives GND high and VCC low: the firmware overrides both
        self.assertEqual(t.command("pld capture set pins=6 clk=1 cand=c0000 base=%x "
                                   "vcc=%x gnd=%x supply=5000" % (gnd, vcc, gnd)), "")
        t.command("pld capture probe 400 0")
        self.assertEqual(t.capture("pld capture state").status, 0)
        self.assertTrue(dev.writes)
        self.assertTrue(all(w & vcc and not w & gnd for w in dev.writes))

        def refused(msg):
            ons = dev.power_ons
            with self.assertRaisesRegex(CaptureError, msg):
                t.capture("pld capture state")
            self.assertEqual(dev.power_ons, ons, "powered despite " + msg)

        dev.jumper_gnd = 1 << 11                      # GND jumper on socket pin 12
        refused("preflight: jumpers vcc=8000000 gnd=0000800, job expects "
                "vcc=8000000 gnd=0000200")
        dev.jumper_gnd = gnd
        dev.supply_mv = 3300
        refused("preflight: supply=3300 mV, job expects 5000 mV")
        dev.supply_mv = 5000
        dev.jumpers_ok = False
        refused("preflight: could not detect")
        self.assertIn("detection failed", t.command("pld capture jumpers"))
        dev.jumpers_ok = True
        t.command("pld capture set pins=206")         # enumerates the GND pin
        refused("preflight: the job uses a VCC or GND pin")
        # vcc=0 gnd=0 supply=0 turns the check off
        dev.jumper_gnd = 1 << 11
        t.command("pld capture set pins=6 vcc=0 gnd=0 supply=0")
        self.assertEqual(t.capture("pld capture state").status, 0)

    def test_rig_preflight(self):
        t = self.transport("fsm22v10", "dip24")
        rig = BrutusRig(t, "GAL22V10", log=quiet)
        self.assertEqual(rig.preflight(), (1 << 27, 1 << 11, 5000))
        self.assertEqual(rig.metadata()["jumpers"], {"vcc": [28], "gnd": [12], "supply_mv": 5000})
        self.assertEqual(rig.metadata()["firmware"], "Version host harness")
        t.dev.jumper_vcc, t.dev.supply_mv = 1 << 23, 3300
        with self.assertRaises(CaptureError) as e:
            rig.preflight()
        self.assertEqual(str(e.exception),
                         "the jumpers do not suit a GAL22V10 in the dip24 socket\n"
                         "  expected: VCC on socket pin 28, GND on socket pin 12, 5 V\n"
                         "  detected: VCC on socket pin 24, GND on socket pin 12, 3.3 V")
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

    def test_galre_board_and_replay(self):
        """galre board through the host-built firmware, then galre replay
        with nothing but the recording."""
        dev = fwharness.SocketDevice(jed("comb16v8"), "dip20")
        out = tempfile.mkdtemp(prefix="galre")

        def galre_main(*argv):
            with contextlib.redirect_stdout(io.StringIO()):
                r = galre.main(list(argv) + ["--cupl", ""])
            with open(os.path.join(argv[argv.index("-o") + 1], "report.txt")) as f:
                return r, f.read()
        try:
            board = os.path.join(out, "board")
            with mock.patch.object(galre, "SerialTransport",
                                   lambda port: fwharness.HarnessTransport(self.lib, dev)):
                dev.jumper_gnd = 1 << 11
                r, report = galre_main("board", "-p", "-", "-d", "GAL16V8", "-o", board)
                self.assertEqual(r, 2)
                self.assertIn("detected: VCC on socket pin 28, GND on socket pin 12", report)
                self.assertEqual(dev.power_ons, 0)
                dev.jumper_gnd = dev.gnd_bit
                r, report = galre_main("board", "-p", "-", "-d", "GAL16V8", "-o", board,
                                       "-t", "comb", "--fix", "4-9=0,11-16=0", "--survey", "8")
                self.assertEqual(r, 0)
                self.assertIn("jumpers: VCC on socket [28], GND on socket [10]", report)
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
