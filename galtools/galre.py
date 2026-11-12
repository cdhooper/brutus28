#!/usr/bin/env python3
"""Reverse-engineer a GAL16V8 / GAL22V10 as a black box with a Brutus-28.

  galre.py board -p /dev/cu.usbmodemXXXX -d GAL22V10 -o out [--title NAME]
      run on the board (firmware with "pld capture"); checks the VCC, GND
      and supply jumpers first, and records the setup and every capture in
      out/capture.jsonl
  galre.py replay out/capture.jsonl -o out2
      run the analysis again from a recording (device, held pins and
      survey come from the recording unless given)
  galre.py sim ORIGINAL.jed -o out
      run against a simulated chip, and compare the result with the original

Writes out/NAME.pld (CUPL) and out/report.txt; with a CUPL compiler (--cupl,
$CUPL, or cupl on the PATH) also out/NAME.jed, checked against
every captured state and input vector.
"""

import argparse
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from analysis import Analysis                              # noqa: E402
from capture import CaptureError, SerialTransport          # noqa: E402
from rig import BrutusRig, RecordingRig, ReplayRig, SimRig  # noqa: E402
import verify                                              # noqa: E402


def parse_fixed(text):
    fixed = {}
    for item in (text or "").split(","):
        if not item:
            continue
        pins, _, val = item.partition("=")
        lo, _, hi = pins.partition("-")
        for p in range(int(lo), int(hi or lo) + 1):
            fixed[p] = int(val or "0")
    return fixed


def describe(meta):
    """The recorded setup, one item per line."""
    lines = []
    for k in ("title", "date", "device", "socket", "firmware"):
        if meta.get(k):
            lines.append("%s: %s" % (k, meta[k]))
    if "settle_us" in meta:
        lines.append("timing: settle %s us, power off %s ms, on %s ms" % (
            meta["settle_us"], meta["off_ms"], meta["on_ms"]))
    j = meta.get("jumpers")
    if j:
        lines.append("jumpers: VCC on socket %s, GND on socket %s, supply %s mV" % (
            j["vcc"], j["gnd"], j["supply_mv"]))
        lines += ["jumper detection: " + r for r in j.get("readings", [])]
    elif "jumper_check" in meta and not meta["jumper_check"]:
        lines.append("jumpers: not checked")
    if meta.get("fixed"):
        lines.append("held pins: %s" % ", ".join(
            "%s=%s" % kv for kv in sorted(meta["fixed"].items(), key=lambda kv: int(kv[0]))))
    if "survey" in meta:
        lines.append("survey: %s" % meta["survey"])
    return lines


def compile_cupl(cupl, pld, log):
    out = os.path.dirname(pld)
    base = os.path.splitext(os.path.basename(pld))[0]
    r = subprocess.run([cupl, "-jaxm4", os.path.basename(pld)], cwd=out,
                       capture_output=True, text=True)
    jed = os.path.join(out, base + ".jed")
    if not os.path.exists(jed):
        log("CUPL did not produce %s:\n%s%s" % (jed, r.stdout, r.stderr))
        return None
    return jed


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["board", "replay", "sim"])
    ap.add_argument("source", nargs="?", help="JED (sim) or recording (replay)")
    ap.add_argument("-o", "--out", required=True, help="output directory")
    ap.add_argument("-d", "--device", choices=["GAL16V8", "GAL22V10"])
    ap.add_argument("-p", "--port", help="Brutus-28 serial device (board)")
    ap.add_argument("-s", "--socket", choices=["dip20", "dip24", "plcc28"])
    ap.add_argument("-t", "--title", help="design name (default galre)")
    ap.add_argument("--fix", help="pins held and not enumerated, e.g. 6-9=0,13=1")
    ap.add_argument("--survey", type=int,
                    help="inputs sampled per state before the full sweeps (default 64)")
    ap.add_argument("--settle", type=int, default=1, help="usec after each write")
    ap.add_argument("--off", type=int, default=20, help="msec power off per power cycle")
    ap.add_argument("--on", type=int, default=2, help="msec after power on")
    ap.add_argument("--no-jumper-check", action="store_true",
                    help="power the socket without checking the VCC, GND and supply "
                         "jumpers (board)")
    ap.add_argument("--cupl", default=os.environ.get("CUPL") or shutil.which("cupl"))
    ap.add_argument("--steps", type=int, default=30000,
                    help="random steps to compare with the original (sim)")
    args = ap.parse_args(argv)

    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "report.txt"), "w") as report:

        def log(msg):
            print(msg)
            report.write(msg + "\n")
            report.flush()

        fixed = parse_fixed(args.fix)
        survey = 64 if args.survey is None else args.survey
        if args.mode == "sim":
            rig = SimRig(args.source, args.off, args.on)
            device = rig.device
        elif args.mode == "replay":
            if not args.source:
                ap.error("replay needs the recording")
            try:
                rig = ReplayRig(args.source, args.device)
            except CaptureError as e:
                ap.error("%s (--device)" % e)
            meta = rig.meta
            device = rig.device
            if args.fix is None and "fixed" in meta:
                fixed = {int(p): v for p, v in meta["fixed"].items()}
            if args.survey is None and "survey" in meta:
                survey = meta["survey"]
            args.title = args.title or meta.get("title")
            for line in describe(meta):
                log("recorded " + line)
        else:
            if not (args.device and args.port):
                ap.error("board needs --device and --port")
            t = SerialTransport(args.port)
            brutus = BrutusRig(t, args.device, args.socket, args.settle, args.off, args.on,
                               log, jumper_check=not args.no_jumper_check)
            if brutus.jumper_check:
                try:
                    brutus.preflight()
                except CaptureError as e:
                    log("jumper check: %s" % e)
                    log("No capture was run.  Fix the jumpers, or pass --no-jumper-check "
                        "if the detection is wrong.")
                    return 2
            meta = brutus.metadata()
            meta.update(title=args.title or "galre", date=time.strftime("%Y-%m-%d %H:%M:%S"),
                        fixed={str(p): v for p, v in sorted(fixed.items())}, survey=survey)
            for line in describe(meta):
                log(line)
            rig = RecordingRig(brutus, os.path.join(args.out, "capture.jsonl"), meta)
            device = args.device
        args.title = args.title or "galre"

        t0 = time.time()
        a = Analysis(rig, device, fixed=fixed, survey=survey, log=log)
        try:
            a.run()
        finally:
            if isinstance(rig, RecordingRig):
                rig.close()
        bad = a.check_equations()
        if bad:
            log("equations disagree with the captures: %s" % bad[:6])
        log("")
        for s, (path, _) in a.states.items():
            log("state %s  W=%s  path %s" % ("".join(map(str, s)), a.W.get(s), list(path)))
        src = a.cupl(args.title)
        pld = os.path.join(args.out, args.title + ".pld")
        with open(pld, "w") as f:
            f.write(src)
        log("")
        log(src)
        ops = rig.ops
        if args.mode != "replay":
            log("board work: %s; about %.0f s on a Brutus-28" % (
                dict(ops), ops.seconds(args.off, args.on)))
        log("analysis %.1f s" % (time.time() - t0))

        if not args.cupl:
            log("no CUPL compiler (--cupl or $CUPL): %s not compiled" % pld)
            return 0 if not bad else 1
        jed = compile_cupl(args.cupl, pld, log)
        if not jed:
            return 1
        diff = verify.check_captures(a, jed)
        log("%s against the captures (%d states x %d input vectors): %s" % (
            os.path.basename(jed), len(a.states), 1 << len(a.V),
            "identical" if not diff else diff[:6]))
        if args.mode == "sim":
            d, first, osc = verify.compare_jeds(args.source, jed, args.steps, fixed=a.fixed)
            log("random %d-step run, original vs rebuilt: %s" % (
                args.steps, "no difference" if not d else "%s, first %s" % (d, first)))
            diff = diff or d
        return 0 if not (bad or diff) else 1


if __name__ == "__main__":
    sys.exit(main())
