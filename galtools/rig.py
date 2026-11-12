"""Rigs: what the analysis may do with a GAL in the Brutus-28 socket.

Pins are numbered as on the DIP package (GAL16V8 1..20, GAL22V10 1..24).
A drive is a dict pin -> 0/1 over the signal pins (missing pins are 0).
A path is a list of drives; each is clocked in with one rising edge on
pin 1, starting from power-up.  Every signal pin is driven weakly (1K), so
a pin the GAL does not drive reads back its drive.

    walk(V, base, path)
        From the path state, for every assignment x of the enumerated pins V
        (bit b of x = pin V[b]; the other pins at base, pin 1 low) read every
        pin, and again with every pin outside V except pin 1 inverted.
        Returns (la, lb): pin -> bitset over x.
    transitions(V, base, path, probes, cand, count=None, mul=1, returns=())
        For each x (all, or (i * mul) mod 2^n for i < count): from the path
        state, clock x in and observe the new state: for each probe drive
        (pin 1 low), per candidate pin c, (level of c, level of c with only
        c's drive inverted).  Returns {observation: bitset of x}.  returns
        lists (observation, [drives]): clocked paths from the state with
        that observation back to the path state, which save power cycles.

SimRig simulates a JED; BrutusRig drives a real Brutus-28.  Both count
the board work in .ops.
"""

import json
import re
import struct
import zlib
from array import array
from functools import reduce

from bitsim import Bits, litmasks
from galsim import GAL
from capture import STATUS, CaptureError

CLOCK = 1

# Rough firmware timing for the estimate: one write + 1 us settle + read
IO_SEC = 2.5e-6
USB_BYTES_PER_SEC = 600e3

# Socket pin of each package pin on the Brutus-28 rev 2 28-pin socket (DIP
# parts sit at the top of the socket) or its PLCC28 socket.
SOCKETS = {
    "dip20": (20, 20, 10, lambda d: d if d <= 10 else d + 8),
    "dip24": (24, 24, 12, lambda d: d if d <= 12 else d + 4),
    "plcc28": (24, 24, 12, lambda d: d + 1 + (d >= 7) + (d >= 13) + (d >= 19)),
}
DEFAULT_SOCKET = {"GAL16V8": "dip20", "GAL22V10": "dip24"}
SUPPLY_MV = 5000                    # GAL16V8 and GAL22V10 are 5 V parts
SUPPLY_TOLERANCE = 16               # percent, as fw/capture.c

# Firmware limits (fw/capture.c)
MAX_PATH, MAX_PROBES, MAX_CAND, MAX_RET, MAX_RET_LEN = 48, 8, 12, 32, 16


def popcount(x):
    return bin(x).count("1")


def x_selection(n, count, mul):
    """Bitset of the x a transitions() call with count/mul covers."""
    N = 1 << n
    if count is None or count >= N:
        return (1 << N) - 1
    bm = bytearray((N + 7) // 8)
    for i in range(count):
        x = (i * mul) & (N - 1)
        bm[x >> 3] |= 1 << (x & 7)
    return int.from_bytes(bm, "little")


def survey_mul(n):
    """An odd multiplier that spreads a partial sweep over all inputs."""
    return (0x9E3779B1 & ((1 << n) - 1)) | 1


class Ops(dict):
    KEYS = ("io", "power_cycles", "clocks", "ret_ok", "ret_fail", "bytes", "runs")

    def __init__(self):
        super().__init__({k: 0 for k in self.KEYS})

    def seconds(self, off_ms, on_ms):
        return (self["io"] * IO_SEC + self["power_cycles"] * (off_ms + on_ms) / 1000.0
                + self["bytes"] / USB_BYTES_PER_SEC)


class SimRig:
    """A GAL simulated from its JED, with the firmware's semantics."""

    def __init__(self, jed, off_ms=20, on_ms=2):
        self.g = GAL(jed)
        self.device = self.g.kind
        self.bits = Bits(self.g)
        self.pins = self.g.signal_pins()
        self.off_ms, self.on_ms = off_ms, on_ms
        self.ops = Ops()
        self.q = None

    def _replay(self, path):
        self.q = {o.pin: 0 for o in self.g.olmcs if o.registered}
        for drive in path:
            self.q = self._step(self.q, drive)

    def _step(self, q, drive):
        e = {p: 1 if drive.get(p, 0) else 0 for p in self.pins}
        e[CLOCK] = 0
        _, _, nq, _ = self.bits.settle(q, e, 1)
        q = dict(q)
        q.update(nq)
        return q

    def _observe(self, q, probes, cand):
        obs = []
        for drive in probes:
            e = {p: 1 if drive.get(p, 0) else 0 for p in self.pins}
            e[CLOCK] = 0
            la, _, _, _ = self.bits.settle(q, e, 1)
            one = []
            for c in cand:
                e2 = dict(e)
                e2[c] ^= 1
                lb, _, _, _ = self.bits.settle(q, e2, 1)
                one.append((la[c], lb[c]))
            obs.append(tuple(one))
        return tuple(obs)

    def _ext(self, V, base, full, v1):
        ext = {p: (v1[V.index(p)] if p in V else (full if base.get(p, 0) else 0))
               for p in self.pins}
        if CLOCK not in V:
            ext[CLOCK] = 0
        return ext

    def walk(self, V, base, path):
        self._replay(path)
        n = len(V)
        full, v1 = litmasks(n)
        ext = self._ext(V, base, full, v1)
        la, _, _, _ = self.bits.settle(self.q, ext, full)
        ext_b = {p: (e if p in V or p == CLOCK else full ^ e) for p, e in ext.items()}
        lb, _, _, _ = self.bits.settle(self.q, ext_b, full)
        self.ops["runs"] += 1
        self.ops["power_cycles"] += 1
        self.ops["clocks"] += len(path)
        self.ops["io"] += 2 * len(path) + (2 << n)
        self.ops["bytes"] += 8 << n
        return la, lb

    def transitions(self, V, base, path, probes, cand, count=None, mul=1, returns=()):
        self._replay(path)
        q0 = dict(self.q)
        n = len(V)
        full, v1 = litmasks(n)
        sel = x_selection(n, count, mul)
        _, _, nextq, _ = self.bits.settle(q0, self._ext(V, base, full, v1), full)
        groups = [((), sel)]
        regs = sorted(nextq)
        for p in regs:
            ng = []
            for key, xs in groups:
                for v, part in ((0, xs & ~nextq[p]), (1, xs & nextq[p])):
                    if part:
                        ng.append((key + (v,), part))
            groups = ng
        ref = self._observe(q0, probes, cand)
        nsig = len(probes) * (1 + len(cand))
        out = {}
        ops = self.ops
        ops["runs"] += 1
        ops["power_cycles"] += 1
        for key, xs in groups:
            q = dict(zip(regs, key))
            o = self._observe(q, probes, cand)
            out[o] = out.get(o, 0) | xs
            k = popcount(xs)
            ops["io"] += k * (2 + nsig)
            ops["clocks"] += k
            if o == ref:
                continue
            back = [r for ob, r in returns if ob == o]
            if back:
                qr = q
                for drive in back[0]:
                    qr = self._step(qr, drive)
                ops["io"] += k * (2 * len(back[0]) + nsig)
                ops["clocks"] += k * len(back[0])
                if self._observe(qr, probes, cand) == ref and qr == q0:
                    ops["ret_ok"] += k
                    continue
                ops["ret_fail"] += k
            ops["power_cycles"] += k
            ops["clocks"] += k * len(path)
            ops["io"] += k * (2 * len(path) + nsig)
        ops["bytes"] += popcount(sel) + 4 * nsig * (len(out) + 1)
        return out


def socket_pins(mask):
    """'socket pin 28', 'socket pins 12, 28' or 'no socket pin'."""
    pins = [str(b + 1) for b in range(28) if mask >> b & 1]
    if not pins:
        return "no socket pin"
    return "socket pin%s %s" % ("s" if len(pins) > 1 else "", ", ".join(pins))


def supply_ok(mv):
    return mv is not None and abs(mv - SUPPLY_MV) * 100 <= SUPPLY_MV * SUPPLY_TOLERANCE


class BrutusRig:
    """A GAL in a real Brutus-28, through the "pld capture" commands.

    With jumper_check, every run asks the firmware to check that the VCC,
    GND and supply jumpers suit the device before it powers the socket;
    preflight() checks the same from the host, with a readable message.
    The firmware finds the VCC and GND pins without connecting the supply,
    and switches the supply on to measure it only when those pins are
    right.
    """

    def __init__(self, transport, device, socket=None, settle_us=1, off_ms=20,
                 on_ms=2, log=print, jumper_check=True):
        self.socket = socket or DEFAULT_SOCKET[device]
        npins, vcc, gnd, smap = SOCKETS[self.socket]
        if npins != {"GAL16V8": 20, "GAL22V10": 24}[device]:
            raise ValueError("socket %s does not fit a %s" % (socket, device))
        self.t, self.device, self.log = transport, device, log
        self.settle_us, self.off_ms, self.on_ms = settle_us, off_ms, on_ms
        self.jumper_check = jumper_check
        self.pins = [p for p in range(1, npins + 1) if p not in (vcc, gnd)]
        self.sbit = {p: 1 << (smap(p) - 1) for p in range(1, npins + 1)}
        self.vcc_bit, self.gnd_bit = self.sbit[vcc], self.sbit[gnd]
        self.readmask = self._mask(self.pins)
        self.crcs = {}               # observation -> signature CRCs seen
        self.detected = None         # (vcc mask, gnd mask, supply mV or None)
        self.jumper_readings = []    # the firmware's detection readings
        self.ops = Ops()

    @staticmethod
    def _parse_jumpers(out):
        """(VCC pin mask, GND pin mask, supply mV or None if not measured)
        from "pld capture jumpers"; socket pin n = bit n-1."""
        m = re.search(r"^jumpers vcc=([0-9a-fA-F]+) gnd=([0-9a-fA-F]+) supply=(\d+|-)\s*$",
                      out, re.M)
        if not m:
            raise CaptureError("pld capture jumpers: %s" % (
                out.strip() or "no answer") + "\n(does the firmware have pld capture jumpers?)")
        mv = None if m.group(3) == "-" else int(m.group(3))
        return int(m.group(1), 16), int(m.group(2), 16), mv

    def preflight(self):
        """Check the jumpers before anything powers the chip."""
        self.detected = None
        self._cmd("pld capture clear")
        self._cmd("pld capture set vcc=%x gnd=%x supply=%d" % (
            self.vcc_bit, self.gnd_bit, SUPPLY_MV))
        out = self.t.command("pld capture jumpers")
        self.jumper_readings = [l for l in out.strip().splitlines()
                                if not l.startswith("jumpers vcc=")]
        self.detected = vcc, gnd, mv = self._parse_jumpers(out)
        if (vcc, gnd) != (self.vcc_bit, self.gnd_bit) or not supply_ok(mv):
            raise CaptureError(
                "the jumpers do not suit a %s in the %s socket\n"
                "  expected: VCC on %s, GND on %s, supply %g V\n"
                "  detected: VCC on %s, GND on %s, %s%s" % (
                    self.device, self.socket,
                    socket_pins(self.vcc_bit), socket_pins(self.gnd_bit), SUPPLY_MV / 1000,
                    socket_pins(vcc), socket_pins(gnd),
                    "supply not measured (only once VCC and GND are right)" if mv is None
                    else "supply %.2f V" % (mv / 1000),
                    "".join("\n  board: " + l for l in self.jumper_readings)))
        return self.detected

    def metadata(self):
        """The setup, for the recording."""
        meta = {"device": self.device, "socket": self.socket, "settle_us": self.settle_us,
                "off_ms": self.off_ms, "on_ms": self.on_ms, "jumper_check": self.jumper_check,
                "firmware": self.t.command("version").strip()}
        if self.detected:
            vcc, gnd, mv = self.detected
            meta["jumpers"] = {"vcc": [b + 1 for b in range(28) if vcc >> b & 1],
                               "gnd": [b + 1 for b in range(28) if gnd >> b & 1],
                               "supply_mv": mv, "readings": self.jumper_readings}
        return meta

    def _mask(self, pins):
        return reduce(lambda a, p: a | self.sbit[p], pins, 0)

    def _vec(self, drive):
        return self._mask([p for p, v in drive.items() if v and p in self.pins]) | self.vcc_bit

    def _cmd(self, line):
        out = self.t.command(line).strip()
        if out:
            raise CaptureError("%s: %s" % (line, out))

    def _list(self, name, vecs, limit):
        if len(vecs) > limit:
            raise CaptureError("%d %s vectors, the firmware takes %d" % (len(vecs), name, limit))
        self._cmd("pld capture %s" % name)
        for i in range(0, len(vecs), 32):
            self._cmd("pld capture %s + %s" % (name, " ".join("%x" % v for v in vecs[i:i + 32])))

    def _job(self, V, base, path, probes, cand, inv=0, count=None, mul=1, returns=()):
        if list(V) != sorted(V):
            raise ValueError("enumerated pins must be in ascending order")
        if len(cand) > MAX_CAND:
            raise CaptureError("%d candidate pins, the firmware takes %d" % (len(cand), MAX_CAND))
        self._cmd("pld capture clear")
        self._cmd("pld capture set pins=%x base=%x inv=%x clk=%x cand=%x mask=%x "
                  "count=%x mul=%x settle=%d off=%d on=%d%s" % (
                      self._mask(V), self._vec(base), inv, self.sbit[CLOCK] if CLOCK not in V else 0,
                      self._mask(cand), self.readmask, count or 0, mul,
                      self.settle_us, self.off_ms, self.on_ms,
                      " vcc=%x gnd=%x supply=%d" % (self.vcc_bit, self.gnd_bit, SUPPLY_MV)
                      if self.jumper_check else ""))
        self._list("path", [self._vec(d) for d in path], MAX_PATH)
        self._list("probe", [self._vec(d) for d in probes], MAX_PROBES)
        nret = 0
        for o, drives in returns:
            if len(drives) > MAX_RET_LEN:
                continue
            for crc in sorted(self.crcs.get(o, ())):
                if nret == MAX_RET:
                    break
                self._cmd("pld capture ret %08x %s" % (crc, " ".join("%x" % self._vec(d) for d in drives)))
                nret += 1

    def _run(self, what):
        cap = self.t.capture("pld capture " + what)
        self.ops["runs"] += 1
        self.ops["power_cycles"] += 1 + cap.power_cycles
        self.ops["ret_ok"] += cap.ret_ok
        self.ops["ret_fail"] += cap.ret_fail
        self.ops["bytes"] += len(cap.data)
        if cap.status != 0:
            raise CaptureError("pld capture %s: %s" % (what, STATUS.get(cap.status, cap.status)))
        if cap.done != cap.count:
            raise CaptureError("pld capture %s: %d of %d vectors" % (what, cap.done, cap.count))
        return cap

    def _bitsets(self, data, step, offset, n):
        """pin -> bitset over x of the reads data[offset::step]."""
        out = {}
        try:
            import numpy as np
            a = np.frombuffer(bytes(data), dtype="<u4")[offset::step]
            for p in self.pins:
                b = ((a & self.sbit[p]) != 0).astype(np.uint8)
                out[p] = int.from_bytes(np.packbits(b, bitorder="little").tobytes(), "little")
        except ImportError:
            reads = array("I")
            reads.frombytes(bytes(data))
            reads = reads[offset::step]
            table = bytes.maketrans(b"\x00\x01", b"01")
            for p in self.pins:
                m = self.sbit[p]
                s = bytes(1 if r & m else 0 for r in reads).translate(table)
                out[p] = int(s[::-1], 2) if s else 0
        return out

    def _obs(self, reads, probes, cand):
        """The observation of a signature's reads; remembers its CRC."""
        per = 1 + len(cand)
        o = tuple(tuple((int((reads[i * per] & self.sbit[c]) != 0),
                         int((reads[i * per + 1 + j] & self.sbit[c]) != 0))
                        for j, c in enumerate(cand))
                  for i in range(len(probes)))
        self.crcs.setdefault(o, set()).add(zlib.crc32(struct.pack("<%dI" % len(reads), *reads)))
        return o

    def walk(self, V, base, path):
        inv = self._mask([p for p in self.pins if p not in V and p != CLOCK])
        self._job(V, base, path, (), (), inv=inv)
        cap = self._run("walk")
        n = len(V)
        if cap.count != 1 << n or len(cap.data) != 8 << n:
            raise CaptureError("walk returned %d bytes for %d vectors" % (len(cap.data), cap.count))
        self.ops["io"] += 2 * len(path) + (2 << n)
        self.ops["clocks"] += len(path)
        return self._bitsets(cap.data, 2, 0, n), self._bitsets(cap.data, 2, 1, n)

    def transitions(self, V, base, path, probes, cand, count=None, mul=1, returns=()):
        n = len(V)
        self._job(V, base, path, probes, cand, count=count, mul=mul, returns=returns)
        cap = self._run("trans")
        nsig = cap.sig_reads
        self._obs(cap.sigs[0], probes, cand)          # the path state itself
        data = cap.data
        N = 1 << n
        defs, maps = [], []
        pos = 0
        for i in range(cap.count):
            k = data[pos]
            pos += 1
            if k == len(defs):
                reads = struct.unpack_from("<%dI" % nsig, data, pos)
                pos += 4 * nsig
                defs.append(self._obs(list(reads), probes, cand))
                maps.append(bytearray((N + 7) // 8))
            x = (i * mul) & (N - 1)
            maps[k][x >> 3] |= 1 << (x & 7)
        out = {}
        for o, bm in zip(defs, maps):
            out[o] = out.get(o, 0) | int.from_bytes(bm, "little")
        self.ops["io"] += cap.count * (2 + nsig)
        self.ops["clocks"] += cap.count
        return out


# -- recording and replay ----------------------------------------------------

def _enc(v):
    if isinstance(v, int):
        return "%x" % v
    if isinstance(v, dict):
        return {str(k): _enc(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_enc(x) for x in v]
    return v


def _key(op, V, base, path, probes=(), cand=(), count=None, mul=1):
    return json.dumps([op, list(V), sorted(base.items()), [sorted(d.items()) for d in path],
                       [sorted(d.items()) for d in probes], list(cand), count, mul])


class RecordingRig:
    """Wraps a rig and writes every answer to a JSON-lines file.  meta
    (the setup: device, socket, timing, jumpers, analysis options) goes on
    the first line, {"meta": {...}}."""

    def __init__(self, rig, path, meta=None):
        self.rig, self.f = rig, open(path, "w")
        self.pins, self.device, self.ops = rig.pins, rig.device, rig.ops
        if meta:
            self.f.write(json.dumps({"meta": meta}) + "\n")
            self.f.flush()

    def close(self):
        self.f.close()

    def walk(self, V, base, path):
        la, lb = self.rig.walk(V, base, path)
        self.f.write(json.dumps({"key": _key("walk", V, base, path), "la": _enc(la), "lb": _enc(lb)}) + "\n")
        self.f.flush()
        return la, lb

    def transitions(self, V, base, path, probes, cand, count=None, mul=1, returns=()):
        out = self.rig.transitions(V, base, path, probes, cand, count, mul, returns)
        self.f.write(json.dumps({"key": _key("trans", V, base, path, probes, cand, count, mul),
                                 "out": [[_enc(o), "%x" % xs] for o, xs in out.items()]}) + "\n")
        self.f.flush()
        return out


class ReplayRig:
    """Answers from a RecordingRig file; for re-running the analysis.
    device defaults to the one in the recording's setup (.meta)."""

    def __init__(self, path, device=None):
        self.meta = {}
        self.db = {}
        with open(path) as f:
            for line in f:
                rec = json.loads(line)
                if "meta" in rec:
                    self.meta.update(rec["meta"])
                else:
                    self.db[rec["key"]] = rec
        self.device = device or self.meta.get("device")
        if self.device not in ("GAL16V8", "GAL22V10"):
            raise CaptureError("%s does not name the device; give it" % path)
        self.pins = [p for p in range(1, {"GAL16V8": 20, "GAL22V10": 24}[self.device] + 1)
                     if p not in {"GAL16V8": (10, 20), "GAL22V10": (12, 24)}[self.device]]
        self.ops = Ops()

    def walk(self, V, base, path):
        rec = self.db.get(_key("walk", V, base, path))
        if rec is None:
            raise CaptureError("walk not in the recording; rerun on the board")
        return ({int(p): int(v, 16) for p, v in rec["la"].items()},
                {int(p): int(v, 16) for p, v in rec["lb"].items()})

    def transitions(self, V, base, path, probes, cand, count=None, mul=1, returns=()):
        rec = self.db.get(_key("trans", V, base, path, probes, cand, count, mul))
        if rec is None:
            raise CaptureError("transitions not in the recording; rerun on the board")
        return {tuple(tuple((int(a, 16), int(b, 16)) for a, b in pr) for pr in o): int(xs, 16)
                for o, xs in rec["out"]}
