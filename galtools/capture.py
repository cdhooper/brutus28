"""Host side of the Brutus-28 "pld capture" firmware commands (fw/capture.c).

The firmware answers a run ("pld capture walk / trans / state") with the
line "---- CAPTURE ----" and binary chunks <type:1> <length:2 LE> <payload>:
'H' header, 'P' signature reads, 'D' data, 'E' end with status, counters and
a CRC-32 over all chunk bytes before it.  Configuration commands ("pld
capture set / path / probe / ret") print nothing unless they fail.
"""

import os
import select
import struct
import termios
import time
import zlib

MARKER = b"---- CAPTURE ----\r\n"
PROMPT = b"CMD> "
STATUS = {0: "ok", 1: "aborted", 2: "more than 255 different signatures",
          3: "could not get back to the path state"}


class CaptureError(Exception):
    pass


class Capture:
    """One decoded run."""

    def __init__(self):
        self.sigs = []          # 'P' sections, each a list of reads
        self.data = bytearray()  # all 'D' payloads
        self.status = None

    def _header(self, p):
        (self.version, kind, self.rec_reads, self.sig_reads, self.nbits,
         self.count, self.pins, self.base, self.inv, self.clk, self.cand,
         self.mask, self.mul) = struct.unpack("<BcBBIIIIIIIII", p)
        self.kind = kind.decode()

    def _end(self, p):
        (self.status, self.done, self.power_cycles, self.ret_ok,
         self.ret_fail, crc) = struct.unpack("<B3xIIIII", p)
        return crc


def read_capture(read_exact):
    """Decode chunks from read_exact(n) up to and including the 'E' chunk."""
    cap = Capture()
    crc = 0
    last = None
    while True:
        head = read_exact(3)
        kind, n = chr(head[0]), head[1] | head[2] << 8
        payload = read_exact(n)
        if kind == "E":
            if cap._end(payload) != crc:
                raise CaptureError("capture CRC mismatch")
            return cap
        crc = zlib.crc32(head + payload, crc)
        if kind == "H":
            cap._header(payload)
        elif kind == "P":
            reads = list(struct.unpack("<%dI" % (n // 4), payload))
            if last == "P":
                cap.sigs[-1].extend(reads)
            else:
                cap.sigs.append(reads)
        elif kind == "D":
            cap.data += payload
        else:
            raise CaptureError("unknown chunk type %r" % kind)
        last = kind


class SerialTransport:
    """The Brutus-28 console on a serial device (USB CDC or UART)."""

    def __init__(self, path, timeout=60.0):
        self.timeout = timeout
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attr = termios.tcgetattr(self.fd)
        attr[0] = 0                                     # iflag
        attr[1] = 0                                     # oflag
        attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attr[3] = 0                                     # lflag
        attr[4] = attr[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        self.buf = bytearray()
        self.sync()

    def close(self):
        os.close(self.fd)

    def _fill(self, timeout):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            raise CaptureError("no answer from the Brutus-28 for %.0f s" % timeout)
        try:
            data = os.read(self.fd, 65536)
        except BlockingIOError:
            data = b""
        self.buf += data

    def _write(self, data):
        while data:
            try:
                n = os.write(self.fd, data)
            except BlockingIOError:
                select.select([], [self.fd], [], 1.0)
                continue
            data = data[n:]

    def _until(self, *tokens, timeout=None):
        """Bytes up to and including the first of tokens; (token, bytes)."""
        while True:
            hits = [(self.buf.find(t), t) for t in tokens if t in self.buf]
            if hits:
                pos, tok = min(hits)
                end = pos + len(tok)
                out = bytes(self.buf[:end])
                del self.buf[:end]
                return tok, out
            self._fill(timeout or self.timeout)

    def read_exact(self, n):
        while len(self.buf) < n:
            self._fill(self.timeout)
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out

    def sync(self):
        for _ in range(3):
            self._write(b"\r")
            try:
                time.sleep(0.2)
                self._until(PROMPT, timeout=2.0)
                break
            except CaptureError:
                continue
        else:
            raise CaptureError("no CMD> prompt from the Brutus-28")
        time.sleep(0.1)
        while select.select([self.fd], [], [], 0.1)[0]:
            self._fill(0.1)
        self.buf.clear()

    @staticmethod
    def _text(raw):
        text = raw.decode(errors="replace").replace("\r", "")
        if "\n" in text:
            text = text.split("\n", 1)[1]            # drop the echo
        else:
            text = ""
        if text.endswith(PROMPT.decode()):
            text = text[:-len(PROMPT)]
        return text

    def command(self, line):
        self._write(line.encode() + b"\r")
        _, raw = self._until(PROMPT)
        return self._text(raw)

    def capture(self, line):
        self._write(line.encode() + b"\r")
        tok, raw = self._until(MARKER, PROMPT)
        if tok == PROMPT:
            raise CaptureError("%s: %s" % (line, self._text(raw).strip()))
        cap = read_capture(self.read_exact)
        self._until(PROMPT)
        return cap
