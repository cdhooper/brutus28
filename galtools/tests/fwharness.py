"""Run the real firmware capture engine (fw/capture.c) on the host.

build() compiles fw/capture.c with stand-in headers and harness.c into a
shared library; HarnessTransport then answers "pld capture" commands like
the Brutus-28 console, with a galsim GAL in the socket.
"""

import ctypes
import os
import shutil
import subprocess
import tempfile

from capture import MARKER, CaptureError, read_capture
from galsim import GAL
from rig import SOCKETS

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.join(HERE, "..", "..", "fw")


def build(cc=None):
    """Compile the harness; returns the library path, or None without a
    C compiler."""
    cc = cc or os.environ.get("CC") or shutil.which("cc") or shutil.which("gcc")
    if not cc:
        return None
    d = tempfile.mkdtemp(prefix="capharness")
    for f in ("capture.c", "capture.h"):
        shutil.copy(os.path.join(FW, f), d)
    for f in ("main.h", "cmdline.h", "printf.h", "harness.c"):
        shutil.copy(os.path.join(HERE, "fwstub", f), d)
    lib = os.path.join(d, "libcapture.so")
    subprocess.run([cc, "-shared", "-fPIC", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-Wno-unused-parameter", "-o", lib, "capture.c", "harness.c"],
                   cwd=d, check=True)
    return lib


class SocketDevice:
    """A GAL in the Brutus-28 socket, as the firmware sees it."""

    def __init__(self, jed, socket):
        self.g = GAL(jed)
        npins, vcc, gnd, smap = SOCKETS[socket]
        self.pins = self.g.signal_pins()
        self.sbit = {p: 1 << (smap(p) - 1) for p in range(1, npins + 1)}
        self.vcc_bit = self.sbit[vcc]
        self.gnd_bit = self.sbit[gnd]
        # What the board's jumper detection reports; tests change these
        self.jumper_vcc, self.jumper_gnd, self.supply_mv = self.vcc_bit, self.gnd_bit, 5000
        self.jumpers_ok = True
        self.powered = False
        self.power_ons = 0
        self.io_count = 0
        self.writes = []             # every value written while powered

    def jumpers(self):
        if not self.jumpers_ok:
            return None
        return self.jumper_vcc, self.jumper_gnd, self.supply_mv

    def power(self, on):
        self.powered = bool(on)
        if on:
            self.power_ons += 1
            self.g.power_up()
            self.g.settle({p: 0 for p in self.pins})   # drivers come up low

    def io(self, value):
        self.io_count += 1
        if not self.powered:
            return 0
        self.writes.append(value)
        ext = {p: 1 if value & self.sbit[p] else 0 for p in self.pins}
        levels, _ = self.g.apply(ext)
        out = self.vcc_bit
        for p in self.pins:
            if levels[p]:
                out |= self.sbit[p]
        return out


class HarnessTransport:
    def __init__(self, lib, device):
        self.lib = ctypes.CDLL(lib)
        self.dev = device
        IO = ctypes.CFUNCTYPE(ctypes.c_uint32, ctypes.c_uint32)
        PWR = ctypes.CFUNCTYPE(None, ctypes.c_int)
        JMP = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.POINTER(ctypes.c_uint32),
                               ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint))
        self._io = IO(lambda v: device.io(v))
        self._pwr = PWR(lambda on: device.power(on))
        self._jmp = JMP(self._jumpers)
        self.lib.harness_init(self._io, self._pwr, self._jmp)
        self.lib.harness_run.restype = ctypes.c_size_t
        self.lib.harness_run.argtypes = [ctypes.c_char_p]
        self.lib.harness_output.restype = ctypes.POINTER(ctypes.c_ubyte)

    def _jumpers(self, vcc, gnd, supply):
        j = self.dev.jumpers()
        if j is None:
            return -1
        vcc[0], gnd[0], supply[0] = j
        return 0

    def _run(self, line):
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
