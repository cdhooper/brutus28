"""Rigs: what the analysis may do with a GAL in the Brutus-28 socket.

Pins are numbered as on the DIP package (GAL16V8 1..20, GAL20V8 and
GAL22V10 1..24).
A drive is a dict pin -> 0/1 over the signal pins (missing pins are 0).
A path is a list of drives; each is clocked in with one rising edge on
pin 1, starting from power-up.  Every signal pin is driven weakly (1K), so
a pin the GAL does not drive reads back its drive.

    walk(V, base, path, mul=1, rev=0, probes=(), cand=(), gray=0, start=0,
         skip=None, avoid=None, data=True, rot=0, mix=0, psig=False, also=())
        From the path state, for every assignment x of the enumerated pins V
        (bit b of x = pin V[b]; the other pins at base, pin 1 low) read every
        pin, and again with every pin outside V inverted, except pin 1 and
        the pins given in base (held inputs keep their value).  The i-th
        vector is x = (i * mul) mod 2^n (mul odd), or the i-th Gray code
        with gray (one pin changes per vector; with mix not 0, of another
        Gray code that mix selects, in which most pins change about
        equally often: mix_bits), rotated left by rot bits (bit
        0 then drives V[rot]), bit-reversed with rev (V[-1] then changes
        fastest), then XORed with start.  Vectors in the skip
        cube (a drive dict, pin -> 0/1) are not driven and read as 0.
        Returns (la, lb, sigs): la, lb pin -> bitset over x (None without
        data: only the signatures are read back); with probes, sigs is
        (observation before the walk, observation after it), as
        transitions() observes a state, else None.  For a clocked design
        the order and sigs make no difference; a chip with asynchronous
        state shows them.
        With a path, V may include pin 1: the path clocks as usual, then
        the walk drives pin 1 like any other enumerated pin (bit of start:
        the level it begins at), and the chip is clocked whenever the walk
        raises it.
    transitions(V, base, path, probes, cand, count=None, mul=1, returns=(),
                avoid=None, chk=False, ignore=(), pre=False, psig=False,
                also=(), nochk=())
        For each x (all, or (i * mul) mod 2^n for i < count): from the path
        state, clock x in and observe the new state: for each probe drive
        (pin 1 low), per candidate pin c, (level of c, level of c with only
        c's drive inverted).  Returns ({observation: bitset of x}, flags).
        returns lists (observation, [drives]): clocked paths from the state
        with that observation back to the path state, which save power
        cycles.  With chk, flags lists (x, pins) for the x whose state the
        observation changed (read at x after the clock edge and again
        after the probes; pins read differently), at most 16 of them; else
        it is empty.  The pins in ignore are left out of every read: pins
        known to change without a clock (latches) then neither flag a
        vector nor tell two states apart.  With pre, a third value is
        returned: pin -> bitset over x of the pin's level with x driven,
        just before the clock edge (every pin, also those in ignore).

avoid (a cube, like skip) makes every write go around it: no momentary mix
of old and new pin values on the way to a vector lies in it (fw/capture.c,
Avoid).  The analysis passes the asynchronous reset term's inputs.

psig: where a clock edge leads may depend on a latch, and the latch on the
writes since the edge before, which the reads of a state's observation are
part of.  With psig (and probes), power-up and every clocked step, of a
path as of a sweep, are followed by the same reads (fw/capture.c, psig),
so that a path leads to the state that the sweep saw.  also: pins whose
levels at the end of those reads are added to every observation, as one
more tuple (the latches: they tell states with the same registers apart).
nochk: pins that do not flag a vector, though they are read.

SimRig simulates a JED; BrutusRig drives a real Brutus-28.  Both count
the board work in .ops.
"""

import json
import re
import struct
import sys
import zlib
from array import array
from functools import reduce
from operator import itemgetter

from bitsim import Bits, litmasks
from galsim import GAL
from capture import STATUS, CaptureError

CLOCK = 1

# Rough firmware timing for the estimate: one write + 1 us settle + read
IO_SEC = 2.5e-6
USB_BYTES_PER_SEC = 600e3

# Socket pin of each package pin on the Brutus-28 rev 2 28-pin socket (DIP
# parts sit at the top of the socket), its PLCC28 socket, or its PLCC20
# socket, which is wired pin for pin to socket pins 1 to 20: VCC is then on
# socket pin 20, not 28.
SOCKETS = {
    "dip20": (20, 20, 10, lambda d: d if d <= 10 else d + 8),
    "dip24": (24, 24, 12, lambda d: d if d <= 12 else d + 4),
    "plcc20": (20, 20, 10, lambda d: d),
    "plcc28": (24, 24, 12, lambda d: d + 1 + (d >= 7) + (d >= 13) + (d >= 19)),
}
DEFAULT_SOCKET = {"GAL16V8": "dip20", "GAL20V8": "dip24", "GAL22V10": "dip24"}
PACKAGE = {"GAL16V8": (20, 20, 10), "GAL20V8": (24, 24, 12), "GAL22V10": (24, 24, 12)}  # pins, VCC, GND
SUPPLY_MV = 5000                    # these GALs are 5 V parts
SUPPLY_TOLERANCE = 16               # percent, as fw/capture.c

# Firmware limits (fw/capture.c)
MAX_PATH, MAX_PROBES, MAX_CAND, MAX_RET, MAX_RET_LEN = 512, 8, 12, 32, 16


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


def bitrev(x, n):
    return int(format(x, "0%db" % n)[::-1], 2) if n else 0


def walk_x(i, n, mul=1, rev=0, gray=0, start=0, rot=0):
    """x of the i-th vector of a walk (as fw/capture.c orders them)."""
    m = (1 << n) - 1
    x = (i ^ (i >> 1)) if gray else (i * mul) & m
    if rot:
        x = ((x << rot) | (x >> (n - rot))) & m
    return (bitrev(x, n) if rev else x) ^ start


def _mix_hash(seed, level, part):
    """cap_mix_hash in fw/capture.c."""
    h = (seed ^ (part * 0x9e3779b1) ^ ((level + 1) * 0x85ebca6b)) & 0xffffffff
    h ^= h >> 15
    h = (h * 0x2c1b3c6d) & 0xffffffff
    h ^= h >> 12
    h = (h * 0x297a2d39) & 0xffffffff
    return h ^ (h >> 15)


def mix_bits(n, mix):
    """The bit that changes at each step of the mixed Gray code that mix
    selects (fw/capture.c, cap_mix_next): the cube is walked as one half,
    one bit's change, the other half, and each half again so, with the
    bit that splits a part chosen by a hash of mix and the part's place.
    Every vector comes once.  The bit that splits the whole walk changes
    once, and the next one or two rarely; the others change about equally
    often.  Another mix makes other bits the rare ones."""
    rem = [None] * n
    split = [0] * n

    def enter(level, i):
        for k in range(level - 1, -1, -1):
            if k + 1 < n:
                rem[k] = [b for b in rem[k + 1] if b != split[k + 1]]
            split[k] = rem[k][((_mix_hash(mix, k, i >> (k + 1)) >> 16) * (k + 1)) >> 16]
    if n:
        rem[n - 1] = list(range(n))
    enter(n, 0)
    for i in range(1, 1 << n):
        level = (i & -i).bit_length() - 1
        yield split[level]
        enter(level, i)


def gray_order(n, rot=0, rev=0, start=0, mix=0):
    """The vectors of a Gray code walk in the order written: walk_x(i, n,
    gray=1, ...) for i = 0, 1, ...; with mix, of the mixed Gray code."""
    pos = [1 << (n - 1 - (g + rot) % n if rev else (g + rot) % n) for g in range(n)]
    x = start
    yield x
    if mix:
        for b in mix_bits(n, mix):
            x ^= pos[b]
            yield x
        return
    for i in range(1, 1 << n):
        x ^= pos[(i & -i).bit_length() - 1]
        yield x


_orders = {}


def gray_arrays(n, rot=0, rev=0, start=0, mix=0):
    """(order, where): gray_order as an array, and for each vector the
    step at which it is written.  The last few are kept: a walk is
    decoded, followed and checked in the same order."""
    key = (n, rot, rev, start, mix)
    hit = _orders.get(key)
    if hit is None:
        order = array("I", gray_order(n, rot, rev, start, mix))
        where = array("I", bytes(4 << n))
        for i, x in enumerate(order):
            where[x] = i
        if len(_orders) >= 24:
            del _orders[next(iter(_orders))]
        hit = _orders[key] = (order, where)
    return hit


# per bit of a byte: the byte values as '1' (bit set) or '0'
_BIT = [bytes(49 if v >> k & 1 else 48 for v in range(256)) for k in range(8)]


def pin_bitsets(raw, sbit):
    """pin -> bitset over x, from 4 bytes per x (a 28-bit socket read, low
    byte first) and each pin's socket bit."""
    out = {}
    for p, m in sbit.items():
        b = m.bit_length() - 1
        s = raw[b >> 3::4].translate(_BIT[b & 7])
        out[p] = int(s[::-1], 2) if s else 0
    return out


def cube_set(cube, V, base, full, v1):
    """Bitset of the x whose drive lies in cube (pin -> 0/1); pins outside
    V are at base."""
    s = full
    for p, v in cube.items():
        if p in V:
            b = V.index(p)
            s &= v1[b] if v else full ^ v1[b]
        elif (1 if base.get(p, 0) else 0) != v:
            return 0
    return s


class Ops(dict):
    KEYS = ("io", "power_cycles", "clocks", "ret_ok", "ret_fail", "bytes", "runs")

    def __init__(self):
        super().__init__({k: 0 for k in self.KEYS})

    def seconds(self, off_ms, on_ms):
        return (self["io"] * IO_SEC + self["power_cycles"] * (off_ms + on_ms) / 1000.0
                + self["bytes"] / USB_BYTES_PER_SEC)


class SimRig:
    """A GAL simulated from its JED, with the firmware's semantics.

    A clocked design is evaluated for all input vectors at once (bitsim).
    A design with asynchronous parts (the 22V10's asynchronous reset term,
    or combinational outputs fed back in a loop, which can latch) depends
    on the exact order of the firmware's writes, so it runs through the
    real firmware built for the host (fwsim; needs a C compiler), with the
    scalar simulator in the socket.

    careful: the caller's jobs go around the asynchronous reset term (they
    avoid its inputs, and skip them in the states where it can fire).  A
    design whose only asynchronous part is that term is then evaluated
    per input vector, which is much faster than the firmware; a walk that
    does trigger the term reads as if it had not, except for the
    signature after the walk.

    race (a seed, not 0): the pins that one port write changes reach the
    chip one after the other, in an order that differs from write to
    write, as on a board (fwsim.SocketDevice).  Only a design with
    asynchronous parts can tell.

    glitch (a seed, not 0): the chip now and then loses the state of its
    registers when the pin that enables its outputs falls in one write
    with other inputs (fwsim.SocketDevice).  serial: the firmware changes
    one pin per write (BrutusRig), so that this cannot happen.  twice: the
    analysis runs every sweep twice, as on a board (Analysis._trans); by
    default it does for a chip with a glitch.
    """

    def __init__(self, jed, off_ms=20, on_ms=2, socket=None, careful=False, race=0,
                 glitch=0, serial=True, twice=None):
        self.g = GAL(jed)
        self.race = race
        self.glitch, self.serial = glitch, serial
        self.twice = bool(glitch) if twice is None else bool(twice)
        self.device = self.g.kind
        self.bits = Bits(self.g)
        self.pins = self.g.signal_pins()
        self.off_ms, self.on_ms = off_ms, on_ms
        self.ops = Ops()
        self.q = None
        self.q0 = {o.pin: 0 for o in self.g.olmcs if o.registered}
        self.async_parts = self.g.async_parts()
        self.fw = None
        self._fw_args = (jed, socket)
        loops = [a for a in self.async_parts if a.startswith("combinational")]
        if loops or (self.async_parts and not careful) or glitch:
            self._firmware()

    def _firmware(self):
        """From now on, run every job through the firmware built for the
        host."""
        import fwsim
        jed, socket = self._fw_args
        lib = fwsim.library()
        if not lib:
            raise RuntimeError("%s has %s: simulating it needs a C compiler" % (
                jed, " and ".join(self.async_parts) or "pin 1 walked in a registered design"))
        socket = socket or DEFAULT_SOCKET[self.device]
        self.socket_device = fwsim.SocketDevice(jed, socket, fast=True, race=self.race,
                                                glitch=self.glitch)
        self.fw = BrutusRig(fwsim.HarnessTransport(lib, self.socket_device), self.device,
                            socket, off_ms=self.off_ms, on_ms=self.on_ms, log=lambda *a: None,
                            jumper_check=False, serial=self.serial)
        for k, v in self.ops.items():
            self.fw.ops[k] += v
        self.ops = self.fw.ops

    def _replay(self, path):
        self.q = dict(self.q0)
        for drive in path:
            self.q = self._step(self.q, drive)

    def _step(self, q, drive):
        """The registers after drive (pin 1 low), then a rising edge.  An
        asynchronous reset the drive triggers clears them before the
        edge, which then loads from the cleared state; one that the new
        state triggers clears them again."""
        e = {p: 1 if drive.get(p, 0) else 0 for p in self.pins}
        e[CLOCK] = 0
        _, _, nq, ar = self.bits.settle(q, e, 1)
        if ar:
            _, _, nq, _ = self.bits.settle(self.q0, e, 1)
        q = dict(q)
        q.update(nq)
        if self.g.ar_row is not None and self.bits.settle(q, e, 1)[3]:
            q = dict(self.q0)
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

    def walk(self, V, base, path, mul=1, rev=0, probes=(), cand=(), gray=0, start=0,
             skip=None, avoid=None, data=True, rot=0, mix=0, psig=False, also=()):
        if not self.fw and (psig or CLOCK in V and self.q0):
            self._firmware()          # the walk clocks the registers
        if self.fw:
            return self.fw.walk(V, base, path, mul=mul, rev=rev, probes=probes, cand=cand,
                                gray=gray, start=start, skip=skip, avoid=avoid, data=data,
                                rot=rot, mix=mix, psig=psig, also=also)
        self._replay(path)
        n = len(V)
        full, v1 = litmasks(n)
        ext = self._ext(V, base, full, v1)
        la, _, _, ar = self.bits.settle(self.q, ext, full)
        inv = [p for p in self.pins if p not in V and p != CLOCK and p not in base]
        if inv and data:
            ext_b = {p: (full ^ e if p in inv else e) for p, e in ext.items()}
            lb, _, _, _ = self.bits.settle(self.q, ext_b, full)
        else:
            lb = dict(la)
        walked = full
        if skip:
            walked ^= cube_set(skip, V, base, full, v1)
            la = {p: v & walked for p, v in la.items()}
            lb = {p: v & walked for p, v in lb.items()}
        sigs = None
        nsig = len(probes) * (1 + len(cand))
        if probes:
            o = self._observe(self.q, probes, cand)
            # clocked, the walk leaves the state alone; a reset it triggers
            # leaves the power-up state
            sigs = (o, self._observe(self.q0, probes, cand) if ar & walked else o)
        reads = 2 if inv and data else 1
        self.ops["runs"] += 1
        self.ops["power_cycles"] += 1
        self.ops["clocks"] += len(path)
        self.ops["io"] += 2 * len(path) + reads * popcount(walked) + 2 * nsig
        self.ops["bytes"] += (4 * reads << n if data else 0) + 8 * nsig
        if not data:
            return None, None, sigs
        return la, lb, sigs

    def transitions(self, V, base, path, probes, cand, count=None, mul=1, returns=(),
                    avoid=None, chk=False, ignore=(), pre=False, psig=False, also=(),
                    nochk=()):
        if psig and not self.fw:
            self._firmware()
        if self.fw:
            return self.fw.transitions(V, base, path, probes, cand, count=count, mul=mul,
                                       returns=returns, avoid=avoid, chk=chk, ignore=ignore,
                                       pre=pre, psig=psig, also=also, nochk=nochk)
        self._replay(path)
        q0 = dict(self.q)
        n = len(V)
        full, v1 = litmasks(n)
        sel = x_selection(n, count, mul)
        ext = self._ext(V, base, full, v1)
        before, _, nextq, ar = self.bits.settle(q0, ext, full)
        if ar:
            # cleared before the edge, which loads from the cleared state
            b0, _, nq0, _ = self.bits.settle(self.q0, ext, full)
            nextq = {p: (v & ~ar) | (nq0[p] & ar) for p, v in nextq.items()}
            before = {p: (v & ~ar) | (b0[p] & ar) for p, v in before.items()}
        groups = [((), sel)]
        regs = sorted(nextq)
        for p in regs:
            ng = []
            for key, xs in groups:
                for v, part in ((0, xs & ~nextq[p]), (1, xs & nextq[p])):
                    if part:
                        ng.append((key + (v,), part))
            groups = ng
        if self.g.ar_row is not None:
            # a state the edge loads may clear itself at once
            zero, ng = 0, []
            for key, xs in groups:
                ar2 = self.bits.settle(dict(zip(regs, key)), ext, full)[3] & xs
                if xs & ~ar2:
                    ng.append((key, xs & ~ar2))
                zero |= ar2
            if zero:
                z = tuple(0 for _ in regs)
                ng = [(k, xs | zero) if k == z else (k, xs) for k, xs in ng]
                if not any(k == z for k, _ in ng):
                    ng.append((z, zero))
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
        ops["bytes"] += popcount(sel) * (6 if pre else 2) + 4 * nsig * (len(out) + 1)
        if pre:
            return out, [], {p: before[p] & sel for p in self.pins}
        return out, []


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

    dclk: drive the clock pin directly, without its 1K.  Through the 1K
    the edge is slow, and a GAL22V10 then takes some transitions wrongly:
    its registers do not all switch at the same moment, and one can catch
    another's new value through the feedback.  follow: before each clock
    edge, drive every pin at the level it reads, so no pin the GAL drives
    is fought through its 1K at the edge.  Both need firmware that knows
    "set follow= dclk=".
    """

    def __init__(self, transport, device, socket=None, settle_us=1, off_ms=20,
                 on_ms=2, log=print, jumper_check=True, follow=True, dclk=True, serial=True,
                 twice=True):
        self.socket = socket or DEFAULT_SOCKET[device]
        npins, vcc, gnd, smap = SOCKETS[self.socket]
        if npins != PACKAGE[device][0]:
            raise ValueError("socket %s does not fit a %s" % (socket, device))
        self.t, self.device, self.log = transport, device, log
        self.settle_us, self.off_ms, self.on_ms = settle_us, off_ms, on_ms
        self.jumper_check = jumper_check
        self.follow, self.dclk = follow, dclk
        # every write changes one pin (firmware with ser=); None once the
        # firmware turned out to be older
        self.serial = serial
        self.twice = twice           # the analysis runs every sweep twice (Analysis._trans)
        self.pins = [p for p in range(1, npins + 1) if p not in (vcc, gnd)]
        self.sbit = {p: 1 << (smap(p) - 1) for p in range(1, npins + 1)}
        # the device's VCC and GND pins, and the rails as the jobs name them
        self.vcc_pin, self.gnd_pin = self.sbit[vcc], self.sbit[gnd]
        self.vcc_bit, self.gnd_bit = self.vcc_pin, self.gnd_pin
        # socket pins that are no pin of the device in this socket
        self.spare = 0x0fffffff & ~reduce(lambda a, b: a | b, self.sbit.values())
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

    def _detect(self):
        self._cmd("pld capture clear")
        self._cmd("pld capture set vcc=%x gnd=%x supply=%d" % (
            self.vcc_bit, self.gnd_bit, SUPPLY_MV))
        out = self.t.command("pld capture jumpers")
        self.jumper_readings = [l for l in out.strip().splitlines()
                                if not l.startswith("jumpers vcc=")]
        return self._parse_jumpers(out)

    def _spare_rails(self, vcc, gnd):
        """The device's VCC and GND pins are among those detected, and
        every other detected pin is no pin of the device."""
        extra = (vcc & ~self.vcc_pin) | (gnd & ~self.gnd_pin)
        return bool(vcc & self.vcc_pin and gnd & self.gnd_pin and not vcc & gnd
                    and not extra & ~self.spare)

    def preflight(self):
        """Check the jumpers before anything powers the chip.

        A part may tie socket pins that are none of its signal pins to
        its rails inside: an ATF22V10C in a PLCC28 has VCC on pin 1 too
        and GND on pins 8, 15 and 22, where a GAL22V10 has no
        connection.  They then read as jumpered.  With the VCC and GND
        pins themselves as expected, and nothing else on a pin of the
        device, the jobs name the rails as detected (the firmware wants
        them exactly), and the supply is measured.  Such a pin may read
        as a rail one time and not the next (pin 1 of that part came
        within 40 mV of the VCC pin's reading, and once not within 250):
        a job that the firmware refuses for that alone is run again with
        the rails as it found them (_run)."""
        self.detected = None
        want = self.vcc_bit, self.gnd_bit = self.vcc_pin, self.gnd_pin
        self.detected = vcc, gnd, mv = self._detect()
        for _ in range(3):
            if (vcc, gnd) == (self.vcc_bit, self.gnd_bit) or not self._spare_rails(vcc, gnd):
                break
            if (self.vcc_bit, self.gnd_bit) == want:
                both = [("%s (VCC)" % socket_pins(vcc & ~want[0])) if vcc & ~want[0] else "",
                        ("%s (GND)" % socket_pins(gnd & ~want[1])) if gnd & ~want[1] else ""]
                self.log("jumper check: %s: no pin of a %s in the %s socket, and tied to the "
                         "rail inside the part, as in an ATF22V10C.  Taken as detected." % (
                             " and ".join(b for b in both if b), self.device, self.socket))
            self.vcc_bit, self.gnd_bit = vcc, gnd
            self.detected = vcc, gnd, mv = self._detect()
        if (vcc, gnd) != (self.vcc_bit, self.gnd_bit) or not supply_ok(mv):
            self.vcc_bit, self.gnd_bit = want
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

    LONG_OFF_MS = 1000           # without power for so long, a chip is taken to be reset

    def _power_up_reads(self, base, probes, cand, off_ms):
        """What the pins read after a power cycle of off_ms."""
        keep, self.off_ms = self.off_ms, off_ms
        try:
            self._job([], base, [], probes, cand)
            return self._run("state").sigs[0]
        finally:
            self.off_ms = keep

    def check_power_cycle(self, io, fixed=None):
        """Does a power cycle of off_ms reset the chip?

        A chip that keeps its charge while the supply is off is not reset
        when it comes back: it comes up in some other state, the same one
        every time, and every job then starts from that state as if it
        were the power-up state.  (The supply is measured just before
        each job, so the chip has been powered then.)  Seen with a part
        in a PLCC28: after 20 ms it came up in the state that one clock
        with every input low leads to, after 50 ms and more in its
        power-up state, every registered output high.

        Reads the pins after a power cycle of off_ms and after one of
        LONG_OFF_MS, with every input low and with every input high (pin 1
        low), each of the pins io driven both ways.  If they differ,
        off_ms is doubled until they agree, and raised by half again.
        Returns off_ms.  Without the jumper check nothing is powered
        before a job, and the firmware itself goes on to ten times off_ms
        where a power cycle does not lead back to the state of a job."""
        if not self.jumper_check:
            return self.off_ms
        fixed = dict(fixed or {})
        base = {p: fixed.get(p, 0) for p in self.pins}
        high = {p: fixed.get(p, 0 if p == CLOCK else 1) for p in self.pins}
        cand = [p for p in io if p not in fixed][:MAX_CAND]
        reads = lambda ms: self._power_up_reads(base, [base, high], cand, ms)  # noqa: E731
        cold = reads(self.LONG_OFF_MS)
        if reads(self.LONG_OFF_MS) != cold:
            self.log("The chip does not come up the same way twice after %d ms without "
                     "power: the power-off time stays at %d ms." % (self.LONG_OFF_MS, self.off_ms))
            return self.off_ms
        first = off = self.off_ms
        while off < self.LONG_OFF_MS and any(reads(off) != cold for _ in range(3)):
            off *= 2
        if off != first:
            self.off_ms = min(self.LONG_OFF_MS, off + off // 2)
            self.log("A power cycle of %d ms does not reset this chip: it comes up in another "
                     "state than after %d ms without power (it keeps its charge), and every "
                     "job would start from that state.  It does after %d ms: power is now off "
                     "for %d ms in every power cycle, and the captures take longer." % (
                         first, self.LONG_OFF_MS, min(off, self.LONG_OFF_MS), self.off_ms))
        return self.off_ms

    def _serial(self):
        """Ask for one pin per write (after "pld capture clear", which turns
        it off).  A firmware that does not know the setting goes on as
        before, and the run says so once."""
        if not self.serial:
            return
        try:
            self._cmd("pld capture set ser=1")
        except CaptureError:
            self.serial = None
            self.log("This firmware changes several pins in one write (it has no ser=).  "
                     "They reach the chip in no certain order, and a chip may now and then "
                     "take that for a clock edge: install the firmware from the same commit.")

    def metadata(self):
        """The setup, for the recording."""
        self._cmd("pld capture clear")
        self._serial()
        meta = {"device": self.device, "socket": self.socket, "settle_us": self.settle_us,
                "off_ms": self.off_ms, "on_ms": self.on_ms, "jumper_check": self.jumper_check,
                "follow": self.follow, "dclk": self.dclk, "serial": bool(self.serial),
                "twice": bool(self.twice),
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

    def _job(self, V, base, path, probes, cand, inv=0, count=None, mul=1, returns=(), rev=0,
             gray=0, start=0, skip=None, avoid=None, quiet=False, chk=False, rot=0,
             ignore=(), pre=False, mix=0, psig=False, nochk=()):
        if list(V) != sorted(V):
            raise ValueError("enumerated pins must be in ascending order")
        if len(cand) > MAX_CAND:
            raise CaptureError("%d candidate pins, the firmware takes %d" % (len(cand), MAX_CAND))
        self._cmd("pld capture clear")
        self._cmd("pld capture set pins=%x base=%x inv=%x clk=%x cand=%x mask=%x "
                  "count=%x mul=%x rev=%x settle=%d off=%d on=%d%s" % (
                      self._mask(V), self._vec(base), inv, self.sbit[CLOCK],
                      self._mask(cand), self.readmask & ~self._mask(ignore), count or 0, mul, rev,
                      self.settle_us, self.off_ms, self.on_ms,
                      " vcc=%x gnd=%x supply=%d" % (self.vcc_bit, self.gnd_bit, SUPPLY_MV)
                      if self.jumper_check else ""))
        extra = []
        if gray or start or rot:
            extra.append("gray=%x rot=%x start=%x" % (gray, rot, self._mask(
                [p for b, p in enumerate(V) if start >> b & 1])))
        if mix:
            extra.append("mix=%x" % mix)
        for name, cube in (("sk", skip), ("av", avoid)):
            if cube:
                extra.append("%sm=%x %sv=%x" % (name, self._mask(cube), name,
                                                self._mask([p for p, v in cube.items() if v])))
        if self.follow:
            extra.append("follow=1")
        if self.dclk:
            extra.append("dclk=1")
        if quiet:
            extra.append("quiet=1")
        if chk:
            extra.append("chk=1")
        if pre:
            extra.append("pre=1")
        if extra:
            self._cmd("pld capture set " + " ".join(extra))
        self._serial()
        new = []
        if psig:
            new.append("psig=1")
        if nochk:
            new.append("chkm=%x" % (self.readmask & ~self._mask(ignore) & ~self._mask(nochk)))
        if new:
            try:
                self._cmd("pld capture set " + " ".join(new))
            except CaptureError as e:
                raise CaptureError("%s\n(this firmware does not read a state after every "
                                   "clocked step: install the one from the same commit)" % e)
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

    LOST = 3                     # status: could not get back to the path state
    RERUNS = 2                   # times a job with that status is run again

    def _run(self, what):
        for again in range(self.RERUNS + 1):
            for attempt in range(4):
                try:
                    cap = self.t.capture("pld capture " + what)
                    break
                except CaptureError as e:
                    # rails on pins that are no pins of the device: see preflight
                    m = re.search(r"preflight: jumpers vcc=([0-9a-fA-F]+) gnd=([0-9a-fA-F]+), "
                                  r"job expects", str(e))
                    if not m or attempt == 3 or not self.jumper_check:
                        raise
                    vcc, gnd = int(m.group(1), 16), int(m.group(2), 16)
                    if not self._spare_rails(vcc, gnd):
                        raise
                    self.vcc_bit, self.gnd_bit = vcc, gnd
                    self._cmd("pld capture set vcc=%x gnd=%x" % (vcc, gnd))
            self.ops["runs"] += 1
            self.ops["power_cycles"] += 1 + cap.power_cycles
            self.ops["ret_ok"] += cap.ret_ok
            self.ops["ret_fail"] += cap.ret_fail
            self.ops["bytes"] += len(cap.data)
            if cap.status != self.LOST or again == self.RERUNS:
                break
            # The same clocks from power-up led to another state than at the
            # start of the job, twice.  A chip that reads wrongly now and
            # then does that; one that has state no pin shows does it every
            # time, and the job fails as before.
            self.log("%s: two power cycles and the same clocks did not lead back to the "
                     "state the job started from (vector %d of %d); running the job again" % (
                         what, cap.done, cap.count))
        if cap.status == self.LOST:
            raise CaptureError("pld capture %s: %s (a chip that keeps its charge through a power "
                               "cycle of %d ms and of ten times that does this: give a longer "
                               "--off)" % (what, STATUS[cap.status], self.off_ms))
        if cap.status != 0:
            raise CaptureError("pld capture %s: %s" % (what, STATUS.get(cap.status, cap.status)))
        if cap.done != cap.count:
            raise CaptureError("pld capture %s: %d of %d vectors" % (what, cap.done, cap.count))
        return cap

    def _bitsets(self, data, step, offset, n, mul=1, rev=0, gray=0, start=0, rot=0, mix=0):
        """pin -> bitset over x of the reads data[offset::step], where the
        i-th read is of vector walk_x(i, n, mul, rev, gray, start, rot), as
        walk() orders them."""
        out = {}
        N = 1 << n
        try:
            import numpy as np
            a = np.frombuffer(bytes(data), dtype="<u4")[offset::step]
            if gray and mix:
                idx = np.frombuffer(gray_arrays(n, rot, rev, start, mix)[0], dtype=np.uint32)
                ordered = np.empty_like(a)
                ordered[idx] = a
                a = ordered
            elif mul != 1 or rev or gray or start or rot:
                u = np.uint64
                idx = np.arange(N, dtype=u)
                idx = idx ^ (idx >> u(1)) if gray else (idx * u(mul)) & u(N - 1)
                if rot:
                    idx = ((idx << u(rot)) | (idx >> u(n - rot))) & u(N - 1)
                if rev:
                    r = np.zeros_like(idx)
                    for b in range(n):
                        r |= ((idx >> u(b)) & u(1)) << u(n - 1 - b)
                    idx = r
                idx ^= u(start)
                ordered = np.empty_like(a)
                ordered[idx] = a
                a = ordered
            for p in self.pins:
                b = ((a & self.sbit[p]) != 0).astype(np.uint8)
                out[p] = int.from_bytes(np.packbits(b, bitorder="little").tobytes(), "little")
        except ImportError:
            reads = array("I")
            reads.frombytes(bytes(data))
            if sys.byteorder != "little":
                reads.byteswap()
            reads = reads[offset::step]
            if mul != 1 or rev or gray or start or rot:
                if gray and N > 1:
                    # the read of vector x is the where[x]-th
                    reads = array("I", itemgetter(*gray_arrays(n, rot, rev, start, mix)[1])(reads))
                else:
                    ordered = array("I", bytes(4 * N))
                    for i, r in enumerate(reads):
                        ordered[walk_x(i, n, mul, rev, gray, start, rot)] = r
                    reads = ordered
            if sys.byteorder != "little":
                reads.byteswap()
            raw = reads.tobytes()                # 4 bytes per read, low byte first
            for p in self.pins:
                b = self.sbit[p].bit_length() - 1
                s = raw[b >> 3::4].translate(_BIT[b & 7])
                out[p] = int(s[::-1], 2) if s else 0
        return out

    def _obs(self, reads, probes, cand, also=()):
        """The observation of a signature's reads; remembers its CRC.
        also: pins whose levels at the last read are one more tuple."""
        per = 1 + len(cand)
        o = tuple(tuple((int((reads[i * per] & self.sbit[c]) != 0),
                         int((reads[i * per + 1 + j] & self.sbit[c]) != 0))
                        for j, c in enumerate(cand))
                  for i in range(len(probes)))
        if also:
            o += (tuple(int((reads[-1] & self.sbit[c]) != 0) for c in also),)
        self.crcs.setdefault(o, set()).add(zlib.crc32(struct.pack("<%dI" % len(reads), *reads)))
        return o

    def walk(self, V, base, path, mul=1, rev=0, probes=(), cand=(), gray=0, start=0,
             skip=None, avoid=None, data=True, rot=0, mix=0, psig=False, also=()):
        inv = self._mask([p for p in self.pins
                          if p not in V and p != CLOCK and p not in base]) if data else 0
        self._job(V, base, path, probes, cand, inv=inv, mul=mul, rev=rev, gray=gray,
                  start=start, skip=skip, avoid=avoid, quiet=not data, rot=rot, mix=mix,
                  psig=psig)
        cap = self._run("walk")
        n = len(V)
        reads = 2 if inv else 1             # no second read when nothing is inverted
        if cap.count != 1 << n or len(cap.data) != ((4 * reads) << n if data else 0):
            raise CaptureError("walk returned %d bytes for %d vectors" % (len(cap.data), cap.count))
        sigs = None
        if probes:
            if len(cap.sigs) != 2:
                raise CaptureError("walk returned %d signatures, not 2" % len(cap.sigs))
            sigs = tuple(self._obs(list(r), probes, cand, also) for r in cap.sigs)
        self.ops["io"] += 2 * len(path) + (reads << n) + 2 * cap.sig_reads
        if psig:
            self.ops["io"] += (1 + len(path)) * (2 + 2 * cap.sig_reads) - cap.sig_reads
        self.ops["clocks"] += len(path)
        if not data:
            return None, None, sigs
        la = self._bitsets(cap.data, reads, 0, n, mul, rev, gray, start, rot, mix)
        lb = self._bitsets(cap.data, 2, 1, n, mul, rev, gray, start, rot, mix) if inv else dict(la)
        return la, lb, sigs

    def transitions(self, V, base, path, probes, cand, count=None, mul=1, returns=(),
                    avoid=None, chk=False, ignore=(), pre=False, psig=False, also=(),
                    nochk=()):
        n = len(V)
        self._job(V, base, path, probes, cand, count=count, mul=mul, returns=returns,
                  avoid=avoid, chk=chk, ignore=ignore, pre=pre, psig=psig, nochk=nochk)
        cap = self._run("trans")
        nsig = cap.sig_reads
        self._obs(cap.sigs[0], probes, cand, also)    # the path state itself
        data = cap.data
        N = 1 << n
        defs, maps = [], []
        before = bytearray(4 * N) if pre else None
        pos = 0
        for i in range(cap.count):
            k = data[pos] | data[pos + 1] << 8
            pos += 2
            if k == len(defs):
                reads = struct.unpack_from("<%dI" % nsig, data, pos)
                pos += 4 * nsig
                defs.append(self._obs(list(reads), probes, cand, also))
                maps.append(bytearray((N + 7) // 8))
            x = (i * mul) & (N - 1)
            maps[k][x >> 3] |= 1 << (x & 7)
            if pre:
                before[4 * x:4 * x + 4] = data[pos:pos + 4]
                pos += 4
        out = {}
        for o, bm in zip(defs, maps):
            out[o] = out.get(o, 0) | int.from_bytes(bm, "little")
        self.ops["io"] += cap.count * (2 + nsig + (2 + nsig if psig else 2 if chk else 0))
        self.ops["clocks"] += cap.count
        flags = [(x, tuple(p for p in self.pins if d & self.sbit[p])) for x, d in cap.flags]
        if pre:
            return out, flags, pin_bitsets(bytes(before), {p: self.sbit[p] for p in self.pins})
        return out, flags


# -- recording and replay ----------------------------------------------------

def _enc(v):
    if isinstance(v, int):
        return "%x" % v
    if isinstance(v, dict):
        return {str(k): _enc(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_enc(x) for x in v]
    return v


def _dec_obs(o):
    """An observation: per probe the candidates' (level, level with the
    drive inverted); with also, one more tuple of plain levels."""
    return tuple(tuple(int(a, 16) for a in pr) if pr and isinstance(pr[0], str)
                 else tuple((int(a, 16), int(b, 16)) for a, b in pr) for pr in o)


def _key(op, V, base, path, probes=(), cand=(), count=None, mul=1, **opts):
    """The recording key of a job; options at their defaults are left out
    (so older recordings still match)."""
    k = [op, list(V), sorted(base.items()), [sorted(d.items()) for d in path],
         [sorted(d.items()) for d in probes], list(cand), count, mul]
    extra = {}
    for name, v in sorted(opts.items()):
        if name == "data":
            if not v:
                extra[name] = False
        elif isinstance(v, dict):
            extra[name] = sorted(v.items())
        elif v:
            extra[name] = v
    if extra:
        k.append(extra)
    return json.dumps(k)


class RecordingRig:
    """Wraps a rig and writes every answer to a JSON-lines file.  meta
    (the setup: device, socket, timing, jumpers, analysis options) goes on
    the first line, {"meta": {...}}."""

    def __init__(self, rig, path, meta=None):
        self.rig, self.f = rig, open(path, "w")
        self.pins, self.device, self.ops = rig.pins, rig.device, rig.ops
        self.twice = getattr(rig, "twice", False)
        if meta:
            self.f.write(json.dumps({"meta": meta}) + "\n")
            self.f.flush()

    def close(self):
        self.f.close()

    def walk(self, V, base, path, mul=1, rev=0, probes=(), cand=(), gray=0, start=0,
             skip=None, avoid=None, data=True, rot=0, mix=0, psig=False, also=()):
        la, lb, sigs = self.rig.walk(V, base, path, mul=mul, rev=rev, probes=probes, cand=cand,
                                     gray=gray, start=start, skip=skip, avoid=avoid, data=data,
                                     rot=rot, mix=mix, psig=psig, also=also)
        rec = {"key": _key("walk", V, base, path, probes, cand, None, mul, rev=rev, gray=gray,
                           start=start, skip=skip, avoid=avoid, data=data, rot=rot, mix=mix,
                           psig=psig, also=list(also))}
        if data:
            rec["la"], rec["lb"] = _enc(la), _enc(lb)
        if sigs is not None:
            rec["sigs"] = _enc(sigs)
        self.f.write(json.dumps(rec) + "\n")
        self.f.flush()
        return la, lb, sigs

    def transitions(self, V, base, path, probes, cand, count=None, mul=1, returns=(),
                    avoid=None, chk=False, ignore=(), pre=False, psig=False, also=(),
                    nochk=()):
        res = self.rig.transitions(V, base, path, probes, cand, count=count, mul=mul,
                                   returns=returns, avoid=avoid, chk=chk, ignore=ignore, pre=pre,
                                   psig=psig, also=also, nochk=nochk)
        out, flags = res[0], res[1]
        rec = {"key": _key("trans", V, base, path, probes, cand, count, mul, avoid=avoid,
                           chk=chk, ignore=list(ignore), pre=pre, psig=psig, also=list(also),
                           nochk=list(nochk)),
               "out": [[_enc(o), "%x" % xs] for o, xs in out.items()]}
        if flags:
            rec["flags"] = _enc(flags)
        if pre:
            rec["pre"] = _enc(res[2])
        self.f.write(json.dumps(rec) + "\n")
        self.f.flush()
        return res


class ReplayRig:
    """Answers from a RecordingRig file; for re-running the analysis.
    device defaults to the one in the recording's setup (.meta)."""

    def __init__(self, path, device=None):
        self.meta = {}
        self.db = {}                 # key -> the answers to that job, in the order recorded
        self.used = {}
        with open(path) as f:
            for line in f:
                rec = json.loads(line)
                if "meta" in rec:
                    self.meta.update(rec["meta"])
                else:
                    self.db.setdefault(rec["key"], []).append(rec)
        self.device = device or self.meta.get("device")
        # the analysis asks as it did when the recording was made
        self.twice = bool(self.meta.get("twice"))
        if self.device not in PACKAGE:
            raise CaptureError("%s does not name the device; give it" % path)
        self.pins = [p for p in range(1, PACKAGE[self.device][0] + 1)
                     if p not in PACKAGE[self.device][1:]]
        self.ops = Ops()

    def _next(self, key):
        """The answer to this job: when the analysis ran the same job
        again (to see whether the chip reads the same twice), the next
        one recorded; the last one once they are used up."""
        recs = self.db.get(key)
        if not recs:
            return None
        k = self.used.get(key, 0)
        self.used[key] = k + 1
        return recs[min(k, len(recs) - 1)]

    def walk(self, V, base, path, mul=1, rev=0, probes=(), cand=(), gray=0, start=0,
             skip=None, avoid=None, data=True, rot=0, mix=0, psig=False, also=()):
        rec = self._next(_key("walk", V, base, path, probes, cand, None, mul, rev=rev,
                               gray=gray, start=start, skip=skip, avoid=avoid, data=data,
                               rot=rot, mix=mix, psig=psig, also=list(also)))
        if rec is None:
            raise CaptureError("walk not in the recording; rerun on the board")
        sigs = tuple(_dec_obs(o) for o in rec["sigs"]) if "sigs" in rec else None
        if not data:
            return None, None, sigs
        return ({int(p): int(v, 16) for p, v in rec["la"].items()},
                {int(p): int(v, 16) for p, v in rec["lb"].items()}, sigs)

    def transitions(self, V, base, path, probes, cand, count=None, mul=1, returns=(),
                    avoid=None, chk=False, ignore=(), pre=False, psig=False, also=(),
                    nochk=()):
        rec = self._next(_key("trans", V, base, path, probes, cand, count, mul, avoid=avoid,
                               chk=chk, ignore=list(ignore), pre=pre, psig=psig,
                               also=list(also), nochk=list(nochk)))
        if rec is None:
            raise CaptureError("transitions not in the recording; rerun on the board")
        flags = [(int(f, 16), ()) if isinstance(f, str)       # recorded before the pins
                 else (int(f[0], 16), tuple(int(p, 16) for p in f[1]))
                 for f in rec.get("flags", ())]
        out = {_dec_obs(o): int(xs, 16) for o, xs in rec["out"]}
        if pre:
            return out, flags, {int(p): int(v, 16) for p, v in rec["pre"].items()}
        return out, flags
