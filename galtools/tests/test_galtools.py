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
from analysis import Analysis                      # noqa: E402
from capture import CaptureError                   # noqa: E402
from galsim import GAL                             # noqa: E402
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

# designs with state that changes without a clock -> (device, pins held,
# what galre must say when it stops)
ASYNC_CASES = {
    "ar22v10": ("GAL22V10", "6-11=0,13-20=0",
                "changed it to 000, the power-up state, as the GAL22V10's asynchronous "
                "reset term does"),
    "latch16v8": ("GAL16V8", "7-9=0,11-15=0,19=0",
                  "pins 17, 18 read differently when the inputs are walked in descending "
                  "order"),
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
                 for name in list(CASES) + list(ASYNC_CASES)}
        self.assertEqual(found, {"cnt16v8": [], "fsm22v10": [], "comb16v8": [],
                                 "ar22v10": ["an asynchronous reset term"],
                                 "latch16v8": ["combinational feedback through pins 17, 18"]})

    def test_held_pin_driven_by_the_gal(self):
        with self.assertRaisesRegex(RuntimeError, "pin 19 is held"):
            Analysis(SimRig(jed("comb16v8")), "GAL16V8", fixed={19: 0}, log=quiet).run()

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
        for name, mul, rev in Analysis.ORDERS + (("scattered", 0x35, 0),):
            for numpy in (True, False):
                with self.subTest(order=name, numpy=numpy):
                    with mock.patch.dict(sys.modules, {} if numpy else {"numpy": None}):
                        got = rig.walk(V, base, path, mul=mul & 0x7f, rev=rev,
                                       probes=probes, cand=cand)
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
