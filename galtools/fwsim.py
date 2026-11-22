"""Run the real firmware capture engine (fw/capture.c) on the host.

build() compiles fw/capture.c with stand-in headers and fwstub/harness.c
into a shared library; HarnessTransport then answers "pld capture"
commands like the Brutus-28 console, with a galsim GAL in the socket.
SimRig uses this for designs with asynchronous parts, whose results depend
on the exact order of the firmware's writes; the tests use it to check the
firmware itself.
"""

import atexit
import ctypes
import os
import shutil
import subprocess
import tempfile

from capture import MARKER, CaptureError, read_capture
from galsim import GAL
from rig import SOCKETS

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.join(HERE, "..", "fw")

# The firmware drives the socket pins with three port writes (pldd_output
# in fw/pld.c): socket pins 1-16, then 17-24, then 25-28.  In between, the
# chip sees the new value on the pins written so far and the old value on
# the rest, for a few tens of ns: long enough for a GAL.  Clocked logic
# does not care (clock edges are writes of their own), but a latch or the
# 22V10's asynchronous reset can.
PORT_GROUPS = (0x000ffff, 0x0ff0000, 0xf000000)      # CAP_PORT_GROUPS in fw/capture.h


def build(cc=None):
    """Compile the harness; returns the library path, or None without a
    C compiler."""
    cc = cc or os.environ.get("CC") or shutil.which("cc") or shutil.which("gcc")
    if not cc:
        return None
    d = tempfile.mkdtemp(prefix="capharness")
    for f in ("capture.c", "capture.h"):
        shutil.copy(os.path.join(FW, f), d)
    for f in ("main.h", "cmdline.h", "printf.h", "harness.c", "galmodel.c", "galmodel.h"):
        shutil.copy(os.path.join(HERE, "fwstub", f), d)
    lib = os.path.join(d, "libcapture.so")
    subprocess.run([cc, "-shared", "-fPIC", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Wno-unused-parameter", "-o", lib, "capture.c", "harness.c",
                    "galmodel.c"], cwd=d, check=True)
    return lib


_library = []


def library():
    """The harness, built once per process and removed at exit; None
    without a C compiler."""
    if not _library:
        lib = build()
        if lib:
            atexit.register(shutil.rmtree, os.path.dirname(lib), True)
        _library.append(lib)
    return _library[0]


class SocketDevice:
    """A GAL in the Brutus-28 socket, as the firmware sees it.

    With fast, the harness simulates the chip itself, in C (fwstub/
    galmodel.c, the same model as galsim.py's GAL and this class): about
    a hundred times faster, but power() and io() are then not called, so
    the counters and writes below stay as they are."""

    def __init__(self, jed, socket, fast=False, race=0, glitch=0, glitch_pin=13, hold_ms=0):
        self.fast = fast and not glitch and not hold_ms
        self.hold_ms = hold_ms       # not 0: the chip keeps its charge for so long without
                                     # power.  Powered again sooner, it is not reset: it comes
                                     # up with its registers as they were, and as after one
                                     # more clock edge with every input low.  A part in a
                                     # PLCC28 did so on a board: after 20 ms without power it
                                     # came up in the state that such a clock leads to from
                                     # any state, after 50 ms in its power-up state.
        self.clock = None            # () -> msec: the firmware's delays so far (the transport)
        self.wire_ms = 0             # and the time its commands took on the wire
        self.off_at = None           # when the supply went off; None: the rail was drained
        self.glitch = glitch         # not 0: a chip that now and then (about one time in
                                     # eight, by this seed) loses the state of its registers
                                     # when glitch_pin falls in one write with two or more
                                     # other input pins, and no input pin rises.  A part in a
                                     # PLCC28 did so on a board when the pin that enables its
                                     # outputs fell together with four inputs: where a state
                                     # is read after a clock, and at a carry of a walk that
                                     # counts.  Never when the pins fell in two writes.  The
                                     # seed runs on over power cycles.
        self.glitch_pin = glitch_pin
        self.race = race             # not 0: the pins that one port write changes arrive one
                                     # after the other, in an order that this seed decides and
                                     # that differs from write to write, as on a board, where
                                     # every pin has its own 1K, capacitance and threshold.
                                     # It runs on over power cycles.
        self._cgal = None
        self.g = GAL(jed)
        npins, vcc, gnd, smap = SOCKETS[socket]
        self.pins = self.g.signal_pins()
        self.sbit = {p: 1 << (smap(p) - 1) for p in range(1, npins + 1)}
        half = self.g.npins // 2     # the pins that are inputs only: 1 to 9 and 11, or 1 to
        self.glitch_mask = sum(      # 11 and 13
            self.sbit[p] for p in self.pins if p < half or p == half + 1)
        self.vcc_bit = self.sbit[vcc]
        self.gnd_bit = self.sbit[gnd]
        # What the board's jumper detection reports; tests change these.
        # supply_mv is the VCC rail reading when the supply is switched on.
        self.jumper_vcc, self.jumper_gnd, self.supply_mv = self.vcc_bit, self.gnd_bit, 4950
        self.jumpers_ok = True
        self.powered = False
        self.power_ons = 0
        self.supply_measured = 0     # times the supply was switched on to measure it
        self.io_count = 0
        self.writes = []             # every value written while powered
        self.transients = True       # model the port writes (PORT_GROUPS)
        self.last = 0                # the socket drive the chip sees
        self.noise = 1               # for the level read of a pin that oscillates; it runs on
                                     # over power cycles, so that the same walk reads differently

    def _now(self):
        return (self.clock() if self.clock else 0) + self.wire_ms

    def _come_up(self):
        low = {p: 0 for p in self.pins}                # drivers come up low
        if (self.hold_ms and self.off_at is not None
                and self._now() - self.off_at < self.hold_ms):
            self.g.apply(low)                          # not reset
            self.g.apply({**low, 1: 1})
            self.g.apply(low)
        else:
            self.g.power_up()
            self.g.settle(low)
        self.last = 0

    def jumper_pins(self):
        self.off_at = None           # the detection drains the VCC rail through the pins
        if not self.jumpers_ok:
            return None
        return self.jumper_vcc, self.jumper_gnd

    def supply(self):
        self.supply_measured += 1
        if self.hold_ms:
            # the supply is on while it is measured, and off from now
            self._come_up()
            self.off_at = self._now()
        return self.supply_mv

    def power(self, on):
        if self.powered and not on:
            self.off_at = self._now()
        self.powered = bool(on)
        if on:
            self.power_ons += 1
            self._come_up()

    def io(self, value):
        self.io_count += 1
        if not self.powered:
            return 0
        self.writes.append(value)
        if self.glitch:
            moved = (value ^ self.last) & self.glitch_mask
            if (not (moved & value) and moved & self.sbit[self.glitch_pin]
                    and bin(moved).count("1") >= 3):
                self.glitch = (self.glitch * 1664525 + 1013904223) & 0xffffffff
                if self.glitch >> 29 == 0:
                    self.g.q = {p: 0 for p in self.g.q}
        if self.transients:
            done = 0
            for g in PORT_GROUPS:
                done |= g
                mid = (value & done) | (self.last & ~done)
                self._arrive(mid)
                if mid != self.last and mid != value:
                    self._apply(mid)
        levels = self._apply(value)
        out = self.vcc_bit
        for p in self.pins:
            if p in self.g.unstable:
                # The chip does not come to rest (an output that feeds back
                # inverted): the pin oscillates, and what a read catches is
                # arbitrary.  The simulator's level is what it stopped at,
                # the same every time, which would pass for logic.
                self.noise = (self.noise * 1664525 + 1013904223) & 0xffffffff
                if self.noise >> 31:
                    out |= self.sbit[p]
            elif levels[p]:
                out |= self.sbit[p]
        return out


    def _arrive(self, mid):
        """Go from the value last applied to mid, which differs in the pins
        of one port write: all but the last pin one at a time, if the pins
        race."""
        diff = mid ^ self.last
        while self.race and diff & (diff - 1):
            self.race = (self.race * 1664525 + 1013904223) & 0xffffffff
            d = diff
            for _ in range((self.race >> 16) % bin(diff).count("1")):
                d &= d - 1
            d &= -d
            diff ^= d
            self._apply(self.last ^ d)

    def _apply(self, value):
        self.last = value
        levels, _ = self.g.apply({p: 1 if value & self.sbit[p] else 0 for p in self.pins})
        return levels

    def cgal(self, lib):
        """This chip as a galmodel.c GAL of the harness library lib."""
        if self._cgal is None:
            g = self.g
            u8, u32, u64 = ctypes.c_uint8, ctypes.c_uint32, ctypes.c_uint64
            ncols = len(g.cols)
            regfb = [kind == "fb" and g._olmc(n).registered for kind, n in g.cols]
            # a row mask has its true and complement fuses side by side, as galmodel.c
            rows = [0 if m is None else m for m in g.rows]
            sbit = [self.sbit.get(p, 0) for p in range(32)]
            pins = sum(1 << p for p in self.pins)
            lib.gal_new.restype = ctypes.c_void_p
            lib.gal_new.argtypes = [ctypes.c_int] * 5 + [ctypes.c_void_p] * 2 + [ctypes.c_int] \
                + [ctypes.c_void_p] * 2 + [u32, ctypes.c_void_p, u32, ctypes.c_int]
            lib.gal_add_olmc.argtypes = [ctypes.c_void_p] + [ctypes.c_int] * 6 + [ctypes.c_void_p]
            c = lib.gal_new(
                g.kind != "GAL22V10", g.clock_pin or 0,
                -1 if g.ar_row is None else g.ar_row, -1 if g.sp_row is None else g.sp_row,
                ncols, (u8 * ncols)(*[n for _, n in g.cols]), (u8 * ncols)(*regfb),
                len(rows), (u64 * len(rows))(*rows),
                (u8 * len(rows))(*[m is not None for m in g.rows]),
                pins, (u32 * 32)(*sbit), self.vcc_bit, self.transients)
            if not c:
                raise RuntimeError("galmodel.c does not take this GAL")
            for o in g.olmcs:
                mode = {"input": 0, "comb": 1, "reg": 2}[o.mode]
                if o.mode == "reg" and g.oe_pin:
                    oe, row = 3, g.oe_pin             # the OE pin
                elif o.oe_row == "always":
                    oe, row = 0, 0
                elif o.oe_row == "never":
                    oe, row = 1, 0
                else:
                    oe, row = 2, o.oe_row
                if lib.gal_add_olmc(c, o.pin, mode, o.active_high, oe, row, len(o.live),
                                    (u64 * max(1, len(o.live)))(*o.live)):
                    raise RuntimeError("galmodel.c does not take this GAL")
            lib.gal_free.argtypes = [ctypes.c_void_p]
            self._cgal = ctypes.c_void_p(c)
            self._lib = lib
            lib.gal_set_race.argtypes = [ctypes.c_void_p, u32]
            lib.gal_set_race(self._cgal, self.race)
        return self._cgal

    def __del__(self):
        if getattr(self, "_cgal", None) is not None:
            self._lib.gal_free(self._cgal)


class HarnessTransport:
    """The harness library is shared by every transport in the process, so
    each run registers this transport's callbacks first."""

    def __init__(self, lib, device):
        self.lib = ctypes.CDLL(lib)
        self.dev = device
        IO = ctypes.CFUNCTYPE(ctypes.c_uint32, ctypes.c_uint32)
        PWR = ctypes.CFUNCTYPE(None, ctypes.c_int)
        PINS = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_uint32),
                                ctypes.POINTER(ctypes.c_uint32))
        SUPPLY = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_uint))
        self._io = IO(lambda v: device.io(v))
        self._pwr = PWR(lambda on: device.power(on))
        self._pins = PINS(self._jumper_pins)
        self._supply = SUPPLY(self._measure_supply)
        self.lib.harness_run.restype = ctypes.c_size_t
        self.lib.harness_run.argtypes = [ctypes.c_char_p]
        self.lib.harness_output.restype = ctypes.POINTER(ctypes.c_ubyte)
        self.lib.harness_gal.argtypes = [ctypes.c_void_p]
        self.lib.harness_msec.restype = ctypes.c_ulong
        device.clock = self.lib.harness_msec

    def _jumper_pins(self, vcc, gnd):
        j = self.dev.jumper_pins()
        if j is None:
            return -1
        vcc[0], gnd[0] = j
        return 0

    def _measure_supply(self, rail):
        rail[0] = self.dev.supply()
        return 0

    WIRE_MS = 10                 # a command and its answer on the wire

    def _run(self, line):
        self.dev.wire_ms += self.WIRE_MS
        self.lib.harness_init(self._io, self._pwr, self._pins, self._supply)
        self.lib.harness_gal(self.dev.cgal(self.lib) if self.dev.fast else None)
        n = self.lib.harness_run(line.encode())
        return bytes(ctypes.cast(self.lib.harness_output(),
                                 ctypes.POINTER(ctypes.c_ubyte * n)).contents) if n else b""

    def command(self, line):
        return self._run(line).decode(errors="replace").replace("\r", "")

    def capture(self, line):
        raw = self._run(line)
        pos = raw.find(MARKER)
        if pos < 0:
            raise CaptureError("%s: %s" % (line, raw.decode(errors="replace").strip()))
        buf = memoryview(raw)[pos + len(MARKER):]
        state = {"pos": 0}

        def read_exact(n):
            p = state["pos"]
            if p + n > len(buf):
                raise CaptureError("capture output ends early")
            state["pos"] = p + n
            return bytes(buf[p:p + n])
        cap = read_capture(read_exact)
        if state["pos"] != len(buf):
            raise CaptureError("%d bytes after the end chunk" % (len(buf) - state["pos"]))
        return cap
