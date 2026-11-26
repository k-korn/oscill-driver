#!/usr/bin/env python3
"""
Oscill DSO1 protocol poking tool - use this on real hardware to confirm the parts of the protocol the driver
had to guess (register read/write encoding, V1 / TS index tables, trigger flags) before trusting the plugin.

Requires pyserial (apt install python3-serial / pip install pyserial).

Examples:
    oscill_tool.py -p /dev/ttyUSB0 id
    oscill_tool.py -p /dev/ttyUSB0 read VNM VSN VHW VSW MCl MCh MCd MC O1 M1 V1 TS RT
    oscill_tool.py -p /dev/ttyUSB0 write V1 auto 3         # types: u8 | u16 | u32 | auto (known width)
    oscill_tool.py -p /dev/ttyUSB0 capture --csv out.csv
    oscill_tool.py -p /dev/ttyUSB0 sweep TS 0 20           # capture once per value, print metadata + stats
    oscill_tool.py -p /dev/ttyUSB0 speed 115200            # test the 0x91 baud change
    oscill_tool.py -p /dev/ttyUSB0 raw 82 00 07 72 00 04 46
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: apt install python3-serial  (or pip install pyserial)")

# Value headers recovered from oscilink.dll: 1 byte -> B1 v, 2 bytes -> F0 00 00 hi lo, 4 bytes -> F1 b3..b0
MARKERS = {"u8": 0xB1, "u16": 0xF0, "u32": 0xF1}
# Register widths recovered from oscill.exe
REG_TYPE = {"O1": "u8", "M1": "u8", "T1": "u8", "S1": "u8", "RT": "u8", "AP": "u8", "AR": "u8", "RS": "u8",
            "MC": "u16", "V1": "u16", "QS": "u16", "TC": "u16", "TS": "u32", "TD": "u32", "TA": "u32", "TW": "u32"}


def hexs(b, limit=48):
    s = b[:limit].hex(" ")
    return s + (" ..." if len(b) > limit else "")


def header(header_id, body):
    return bytes([header_id]) + struct.pack(">H", 3 + len(body)) + body


def packet(op, body):
    return bytes([op]) + struct.pack(">H", 3 + len(body)) + body


class Link:
    def __init__(self, port, baud, verbose, dtr=None, rts=None):
        # Configure DTR/RTS *before* opening so the lines never glitch: on some adapters they are wired to the
        # probe's reset / boot-select pins
        self.s = serial.Serial()
        self.s.port = port
        self.s.baudrate = baud
        self.s.timeout = 0.1
        if dtr is not None:
            self.s.dtr = dtr
        if rts is not None:
            self.s.rts = rts
        self.s.open()
        self.verbose = verbose

    def send(self, data):
        if self.verbose:
            print("TX", hexs(data), file=sys.stderr)
        self.s.reset_input_buffer()
        self.s.write(data)

    def recv(self, timeout=1.5):
        deadline = time.time() + timeout
        buf = b""
        while time.time() < deadline:
            buf += self.s.read(max(1, (3 if len(buf) < 3 else struct.unpack(">H", buf[1:3])[0]) - len(buf)))
            if len(buf) >= 3:
                n = struct.unpack(">H", buf[1:3])[0]
                if len(buf) >= n:
                    if self.verbose:
                        print("RX", hexs(buf), file=sys.stderr)
                    return buf[:n]
        if buf and self.verbose:
            print("RX (partial)", hexs(buf), file=sys.stderr)
        return None

    def xfer(self, data, timeout=1.5):
        self.send(data)
        return self.recv(timeout)

    def abort(self):
        # OBEX Abort: stops whatever a previous session left running (e.g. a roll-mode stream)
        self.send(bytes.fromhex("ff 00 03"))
        time.sleep(0.1)
        self.s.reset_input_buffer()

    def connect(self):
        self.abort()
        r = self.xfer(bytes.fromhex("80 00 07 10 00 10 00"))
        if not r or r[0] != 0xA0:
            # opening the port can glitch the modem lines and briefly restart the probe: retry once
            time.sleep(0.5)
            self.s.reset_input_buffer()
            r = self.xfer(bytes.fromhex("80 00 07 10 00 10 00"))
        if not r or r[0] != 0xA0:
            sys.exit(f"no CONNECT response (got {hexs(r) if r else 'nothing'})")
        if "BOOT".encode("utf-16-be") in r:
            print("WARNING: probe reports BOOT state (bootloader, not the oscilloscope firmware). "
                  "All other commands will fail. Unplug and replug the probe.", file=sys.stderr)
        return r

    def cmd(self, c, timeout=1.5):
        # vendor document: 'D' (digitize) is a GET (0x83), 'C' (calibrate) a PUT (0x82).
        # 'F' is the firmware-load command: never send it.
        assert c in ("D", "C")
        op = 0x82 if c == "C" else 0x83
        return self.xfer(packet(op, header(0x72, c.encode())), timeout)

    def read_reg(self, name):
        # 3-letter names are read-only constants ('p' read); 2-letter names are settings, read with a name-only 'q'
        hid = 0x70 if len(name) >= 3 else 0x71
        return self.xfer(packet(0x83, header(hid, name.encode())))

    def calibrate(self):
        # PUT 'C': self-calibration of zero and trigger level. Without it a freshly connected probe returned a
        # stale all-255 buffer.
        return self.cmd("C")

    def write_reg(self, name, typ, value):
        if typ == "u8":
            v = bytes([value & 0xff])
        elif typ == "u16":
            v = struct.pack(">I", value & 0xffff)
        else:
            v = struct.pack(">I", value & 0xffffffff)
        return self.xfer(packet(0x82, header(0x71, name.encode()) + bytes([MARKERS[typ]]) + v))

    def set_speed(self, baud):
        """Switch the link speed. The probe ACKs at the old rate (0f 00 04 <coef>) and then switches; the ACK
        is discarded. Verify with verify_link() afterwards."""
        self.send(packet(0x91, bytes([1843200 // baud])))
        self.s.flush()
        time.sleep(0.05)
        self.s.baudrate = baud
        self.s.reset_input_buffer()

    def verify_link(self):
        """CONNECT is always answered."""
        r = self.xfer(bytes.fromhex("80 00 07 10 00 10 00"), 0.5)
        return bool(r and r[0] == 0xA0)

    def capture(self, timeout=5.0):
        r = self.cmd("D", timeout)
        if not r:
            self.abort()
            return None
        if len(r) < 7 or r[0] != 0xA0:
            print("unexpected:", hexs(r))
            return None
        hl = struct.unpack(">H", r[4:6])[0]
        body = r[3 + hl:]
        if body == bytes.fromhex("49 00 03"):
            return "empty"      # Wait mode (RT=1) timed out without a trigger
        if len(body) <= 7:
            print("unexpected:", hexs(r))
            return None
        return body[:7], body[7:]


def decode_value(name, r):
    if not r:
        return "no reply"
    if r[0] != 0xA0:
        return f"error {r[0]:#04x}: {hexs(r)}"
    p = r[3:]
    if p and p[0] in (0x70, 0x71) and len(p) >= 3:
        p = p[3:]
    if p.startswith(name.encode()):
        p = p[len(name):]
    if len(p) >= 2 and p[0] == 0xB1:
        return f"u8 {p[1]}"
    if len(p) >= 5 and p[0] in (0xF0, 0xF1):
        v = struct.unpack(">I", p[1:5])[0]
        text = ""
        if p[0] == 0xF1 and all(0x20 <= c < 0x7f for c in p[1:5]):
            text = f"  ({p[1:5].decode()!r})"     # identity registers pack 4 ASCII characters
        return f"{'u16' if p[0] == 0xF0 else 'u32'} {v}{text}"
    text = p.split(b"\x00")[0]
    return f"str {text.decode(errors='replace')!r}  [{hexs(r)}]"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=9600)
    ap.add_argument("-v", "--verbose", action="store_true", help="hex dump all traffic to stderr")
    onoff = {"on": True, "off": False}
    ap.add_argument("--dtr", choices=onoff, help="hold DTR at this level from the moment the port opens")
    ap.add_argument("--rts", choices=onoff, help="hold RTS at this level from the moment the port opens")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("id")
    r = sub.add_parser("read")
    r.add_argument("regs", nargs="+")
    w = sub.add_parser("write")
    w.add_argument("reg")
    w.add_argument("type", choices=list(MARKERS) + ["auto"], help="auto = width recovered from oscill.exe")
    w.add_argument("value", type=lambda x: int(x, 0))
    c = sub.add_parser("capture")
    c.add_argument("--count", type=int, default=1)
    c.add_argument("--csv")
    c.add_argument("--timeout", type=float, default=5.0)
    c.add_argument("--no-calibrate", action="store_true", help="don't send PUT 'C' (calibrate) before the first capture")
    sw = sub.add_parser("sweep")
    sw.add_argument("reg")
    sw.add_argument("start", type=int)
    sw.add_argument("stop", type=int)
    sw.add_argument("--type", choices=list(MARKERS) + ["auto"], default="auto")
    sp = sub.add_parser("speed", help="switch link speed (1843200/n), verify with CONNECT, switch back")
    sp.add_argument("new_baud", type=int, metavar="baud")
    raw = sub.add_parser("raw")
    raw.add_argument("bytes", nargs="+")
    args = ap.parse_args()

    link = Link(args.port, args.baud, args.verbose,
                {"on": True, "off": False}.get(args.dtr), {"on": True, "off": False}.get(args.rts))
    link.connect()

    if args.cmd == "id":
        for name in ("VNM", "VSN", "VHW", "VSW", "MCl", "MCh", "MCd"):
            print(f"{name}: {decode_value(name, link.read_reg(name))}")
    elif args.cmd == "read":
        for name in args.regs:
            print(f"{name}: {decode_value(name, link.read_reg(name))}")
    elif args.cmd == "write":
        typ = REG_TYPE.get(args.reg, "u32") if args.type == "auto" else args.type
        r = link.write_reg(args.reg, typ, args.value)
        print("reply:", hexs(r) if r else "none", "(OK)" if r and r[0] == 0xA0 else "")
    elif args.cmd == "capture":
        rows = []
        if not args.no_calibrate:
            link.calibrate()
        for i in range(args.count):
            deadline = time.time() + args.timeout
            res = link.capture(args.timeout)
            while res == "empty" and time.time() < deadline:
                time.sleep(0.05)
                res = link.capture(args.timeout)
            if res == "empty":
                print(f"#{i}: no trigger within {args.timeout} s")
                continue
            if not res:
                print(f"#{i}: no waveform (timeout, sent abort)")
                continue
            meta, samples = res
            print(f"#{i}: meta [{meta.hex(' ')}]  n={len(samples)} min={min(samples)} max={max(samples)} "
                  f"mean={sum(samples) / len(samples):.1f}")
            rows.append(samples)
        if args.csv and rows:
            with open(args.csv, "w") as f:
                f.write("index," + ",".join(f"cap{i}" for i in range(len(rows))) + "\n")
                for k in range(max(len(r) for r in rows)):
                    f.write(f"{k}," + ",".join(str(r[k]) if k < len(r) else "" for r in rows) + "\n")
            print("wrote", args.csv)
    elif args.cmd == "sweep":
        link.calibrate()
        for v in range(args.start, args.stop + 1):
            typ = REG_TYPE.get(args.reg, "u32") if args.type == "auto" else args.type
            wr = link.write_reg(args.reg, typ, v)
            ok = "ok " if wr and wr[0] == 0xA0 else "ERR"
            res = link.capture()
            if res and res != "empty":
                meta, s = res
                print(f"{args.reg}={v:4d} {ok} meta [{meta.hex(' ')}] min={min(s):3d} max={max(s):3d}")
            else:
                print(f"{args.reg}={v:4d} {ok} no waveform")
    elif args.cmd == "speed":
        if 1843200 % args.new_baud:
            sys.exit("baud must be 1843200/n")
        link.set_speed(args.new_baud)
        if link.verify_link():
            print(f"link works at {args.new_baud} baud")
            link.set_speed(9600)
            print("reverted to 9600:", "ok" if link.verify_link() else "NO ANSWER")
        else:
            print(f"no answer at {args.new_baud}; sending revert at that rate and returning to 9600")
            link.set_speed(9600)
            print("at 9600:", "ok" if link.verify_link() else "NO ANSWER - replug the probe")
    elif args.cmd == "raw":
        data = bytes(int(x, 16) for x in " ".join(args.bytes).split())
        r = link.xfer(data, 3.0)
        print(hexs(r, 4096) if r else "no reply")


if __name__ == "__main__":
    main()
