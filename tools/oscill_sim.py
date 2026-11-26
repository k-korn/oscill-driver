#!/usr/bin/env python3
"""
Oscill DSO1 probe simulator on a pseudo-terminal, for testing the ngscopeclient plugin without hardware.

    python3 tools/oscill_sim.py            # prints the pty path, e.g. /dev/pts/5
    ngscopeclient  ->  driver "oscill", transport "uart", path "/dev/pts/5:9600"

Implements the frames documented in docs/Protocol.md: CONNECT, Abort, GET 'D' (digitize), PUT 'C' (calibrate),
'p' property reads, 'q' register reads and writes, speed change. Like the real probe it answers 'D' one record
duration after the request. It does not simulate the hardware trigger.
"""

import argparse
import math
import os
import pty
import select
import struct
import sys
import termios
import time
import tty

# Identity registers: 4 ASCII characters packed in an F1 value (as a real fw 1.26 probe answers)
STRING_REGS = {"VNM": "Uosc", "VSN": "3404", "VHW": "1.25", "VSW": "1.26"}
# MC* are in 10 ps units; TS = clocks per sample * 256, limited to [TMl, TMh]; V1 = mV/div
# Power-up values of a real fw 1.26 probe (MC drifts on real hardware; fixed here)
NUM_REGS = {"MCl": 800, "MCh": 5425, "MCd": 1248, "MC": 1991, "TMl": 25, "TMh": 256, "TCh": 252, "QSh": 254,
            "O1": 0, "M1": 0, "T1": 0x30, "S1": 127, "V1": 10000, "RT": 0, "TA": 65280, "TW": 65280,
            "TC": 0, "TD": 0, "TS": 512, "QS": 254, "AP": 0, "AR": 1, "RS": 0}


# Register widths recovered from oscill.exe (which of PutGetOne/Two/FourbyteRegister each name is passed to)
REG_WIDTH = {"O1": 1, "M1": 1, "T1": 1, "S1": 1, "RT": 1, "AP": 1, "AR": 1, "RS": 1,
             "MC": 2, "V1": 2, "QS": 2, "TC": 2, "TS": 4, "TD": 4, "TA": 4, "TW": 4}


def frame(op, body=b""):
    return bytes([op]) + struct.pack(">H", 3 + len(body)) + body


class Probe:
    def __init__(self, fd, args):
        self.fd = fd
        self.args = args
        self.rx = b""
        self.regs = dict(NUM_REGS)
        self.pending_d_at = None
        self.phase = 0.0
        self.log = args.verbose

    def send(self, data):
        if self.log:
            print("TX", data[:24].hex(" "), "..." if len(data) > 24 else "", flush=True)
        os.write(self.fd, data)

    def record_seconds(self):
        # a real probe answers GET 'D' only after acquiring a fresh record: QS samples of TS/256 clocks each
        return self.regs["QS"] * self.regs["TS"] / 256 * self.regs["MC"] * 1e-11

    def waveform(self):
        # real layout: 'r' echo, End-of-Body header, 4 metadata bytes, QS samples, the first two of them garbage
        n = self.regs["QS"]
        self.phase += 0.3
        samples = bytes([250, 66]) + bytes(
            max(0, min(255, int(128 + 100 * math.sin(2 * math.pi * 3 * i / n + self.phase)))) for i in range(2, n))
        body = bytes([self.regs["T1"] & 0x30, 0, 0, 0]) + samples
        return frame(0xA0, b"\x72\x00\x04D" + b"\x49" + struct.pack(">H", 3 + len(body)) + body)

    def value_header(self, name, default_width=4):
        w = REG_WIDTH.get(name, default_width)
        v = self.regs[name]
        if w == 1:
            return bytes([0xB1, v & 0xff])
        if w == 2:
            return b"\xF0" + struct.pack(">I", v & 0xffff)
        return b"\xF1" + struct.pack(">I", v & 0xffffffff)

    def handle(self, pkt):
        if self.log:
            print("RX", pkt.hex(" "), flush=True)
        op = pkt[0]
        if self.args.silent:
            return
        if op == 0x80:
            if self.args.boot:
                # bootloader reply as captured from a real probe
                self.send(bytes.fromhex("a0 00 14 10 00 04 80 05 00 0d 00 42 00 4f 00 4f 00 54 00 00"))
            else:
                self.send(bytes.fromhex("A0 00 07 10 00 00 18"))
            return
        if self.args.boot:
            self.send(frame(0xD0))
            return
        if len(pkt) >= 7 and pkt[3] == 0x72:
            cmd = chr(pkt[6])
            if cmd == "C":
                # calibration
                self.pending_d_at = None
                self.send(bytes.fromhex("A0 00 03"))
            elif cmd == "D":
                self.pending_d_at = time.time() + self.record_seconds()
            else:
                self.send(frame(0xD0))
            return
        if self.args.silent:
            return
        if op == 0xFF:
            # Abort: stop a pending digitization
            self.pending_d_at = None
            self.send(frame(0xA0))
            return
        if op in (0x82, 0x83) and self.args.reject_writes and len(pkt) > 6 and pkt[3] == 0x71 and \
                len(pkt) > 3 + struct.unpack(">H", pkt[4:6])[0]:
            self.send(frame(0xD0))
            return
        if op in (0x82, 0x83) and len(pkt) >= 6 and pkt[3] == 0x71:
            # 71 LL LL name, then value header: B1 v | F0 00 00 hi lo | F1 b3..b0
            hl = struct.unpack(">H", pkt[4:6])[0]
            name = pkt[6:3 + hl].decode(errors="replace")
            vh = pkt[3 + hl:]
            width = REG_WIDTH.get(name)
            if name in self.regs and not vh:
                # name-only 'q': read a settings register; reply = echoed name header + value header
                self.send(frame(0xA0, pkt[3:3 + hl] + self.value_header(name)))
                return
            if name in self.regs and vh:
                marker, val = vh[0], vh[1:]
                if marker == 0xB1 and len(val) == 1 and width == 1:
                    self.regs[name] = val[0]
                elif marker == 0xF0 and len(val) == 4 and width == 2:
                    self.regs[name] = struct.unpack(">I", val)[0]
                elif marker == 0xF1 and len(val) == 4 and width == 4:
                    self.regs[name] = struct.unpack(">i", val)[0]
                else:
                    print(f"bad write to {name}: {vh.hex(' ')} (register is {width} bytes)", flush=True)
                    self.send(frame(0xD0))
                    return
                if name == "TS" and self.regs["TS"] < 65536:
                    self.regs["TS"] -= self.regs["TS"] % 256     # real probe: whole clocks at fast rates
                print(f"set {name} = {self.regs[name]}", flush=True)
                if op == 0x82:
                    self.send(frame(0xA0))      # real probe: PUT write -> bare A0 00 03
                else:
                    self.send(frame(0xA0, pkt[3:3 + hl] + self.value_header(name)))   # GET write: echo
                return
            self.send(frame(0xD0))
            return
        if op == 0x91 and len(pkt) == 4:
            print(f"baud change to {1843200 // pkt[3]}", flush=True)
            self.send(frame(0xA0))
            return
        if op in (0x82, 0x83) and len(pkt) >= 6 and pkt[3] == 0x70:
            name = pkt[6:].decode(errors="replace")
            hdr_body = pkt[6:]
            if name in STRING_REGS:
                self.send(frame(0xA0, b"\x70" + struct.pack(">H", 3 + len(hdr_body)) + hdr_body +
                                b"\xF1" + STRING_REGS[name].encode()[:4]))
                return
            elif name in self.regs and len(name) >= 3:
                # value header follows the echoed name header, like a real probe (MCl: F0 00 00 03 20)
                self.send(frame(0xA0, b"\x70" + struct.pack(">H", 3 + len(hdr_body)) + hdr_body +
                                self.value_header(name, 2)))
                return
            else:
                self.send(frame(0xD0))
                return
        self.send(frame(0xD0))

    def run(self):
        while True:
            timeout = 0.05
            r, _, _ = select.select([self.fd], [], [], timeout)
            if r:
                try:
                    self.rx += os.read(self.fd, 4096)
                except OSError:
                    time.sleep(0.05)
                    continue
                while len(self.rx) >= 3:
                    n = struct.unpack(">H", self.rx[1:3])[0]
                    if n < 3 or n > 4096:
                        self.rx = self.rx[1:]
                        continue
                    if len(self.rx) < n:
                        break
                    pkt, self.rx = self.rx[:n], self.rx[n:]
                    self.handle(pkt)
            if self.pending_d_at is not None and time.time() >= self.pending_d_at:
                self.pending_d_at = None
                self.send(self.waveform())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", help="also create a symlink to the pty at this path")
    ap.add_argument("--reject-writes", action="store_true", help="answer every register PUT with 0xD0 (error)")
    ap.add_argument("--boot", action="store_true", help="behave like a probe stuck in its bootloader")
    ap.add_argument("--silent", action="store_true", help="never answer anything (dead probe)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    master, slave = pty.openpty()
    tty.setraw(master)
    attrs = termios.tcgetattr(slave)
    tty.setraw(slave)
    name = os.ttyname(slave)
    if args.link:
        if os.path.lexists(args.link):
            os.unlink(args.link)
        os.symlink(name, args.link)
    print(name, flush=True)
    try:
        Probe(master, args).run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    sys.exit(main())
