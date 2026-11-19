#!/usr/bin/env python3
"""
Oscill DSO1 probe simulator on a pseudo-terminal, for testing the ngscopeclient plugin without hardware.

    python3 tools/oscill_sim.py            # prints the pty path, e.g. /dev/pts/5
    ngscopeclient  ->  driver "oscill", transport "uart", path "/dev/pts/5:9600"

Implements the frames documented in docs/Protocol.md: CONNECT, GET 'D' / 'F' / 'C', register PUT 'q' and
GET 'p'. In Normal trigger mode (RT=2) it holds the 'D' reply for --trigger-delay seconds, like a real probe
waiting for a trigger, so cancel handling gets exercised.
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
NUM_REGS = {"MCl": 800, "MCh": 5425, "MCd": 1248, "MC": 1250, "TMl": 256, "TMh": 256 * 65536, "TOv": 1000,
            "O1": 0x02, "M1": 0, "T1": 0x20, "S1": 128, "TCh": 2000, "V1": 1000, "RT": 0, "TA": 128, "TC": 0, "TD": 172, "TS": 20480}


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

    def waveform(self):
        n = 344
        self.phase += 0.3
        samples = bytes(
            max(0, min(255, int(128 + 100 * math.sin(2 * math.pi * 3 * i / n + self.phase)))) for i in range(n))
        meta = bytes([self.regs["TS"] & 0xff, 0, 0, 0, 0, 172 >> 8 & 0xff, 172 & 0xff])
        return frame(0xA0, b"\x72\x00\x04D" + meta + samples)

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
            if cmd == "F":
                # the real oscilloscope firmware (1.26) rejects GET 'F'
                self.send(frame(0xD0))
            elif cmd == "C":
                self.pending_d_at = None
                self.send(bytes.fromhex("A0 00 03"))
            elif cmd == "D":
                delay = self.args.trigger_delay if self.regs["RT"] == 2 else 0.0
                self.pending_d_at = time.time() + delay
            else:
                self.send(frame(0xD0))
            return
        if self.args.silent:
            return
        if op == 0x83 and self.args.reject_writes:
            self.send(frame(0xD0))
            return
        if op == 0x83 and len(pkt) >= 6 and pkt[3] == 0x71:
            # 71 LL LL name, then value header: B1 v | F0 00 00 hi lo | F1 b3..b0
            hl = struct.unpack(">H", pkt[4:6])[0]
            name = pkt[6:3 + hl].decode(errors="replace")
            vh = pkt[3 + hl:]
            width = REG_WIDTH.get(name)
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
                print(f"set {name} = {self.regs[name]}", flush=True)
                self.send(frame(0xA0))
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
            elif name in self.regs:
                # value header follows the echoed name header, like a real probe (MCl: F0 00 00 03 20)
                w = REG_WIDTH.get(name, 4)
                value = {1: bytes([0xB1, self.regs[name] & 0xff]),
                         2: b"\xF0" + struct.pack(">I", self.regs[name] & 0xffff)}.get(
                    w, b"\xF1" + struct.pack(">I", self.regs[name] & 0xffffffff))
                self.send(frame(0xA0, b"\x70" + struct.pack(">H", 3 + len(hdr_body)) + hdr_body + value))
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
    ap.add_argument("--trigger-delay", type=float, default=0.2, help="seconds to hold 'D' in Normal mode")
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
