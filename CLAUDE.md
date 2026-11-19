# Oscill DSO1 ngscopeclient plugin — working notes

libscopehal driver plugin (driver `oscill`, transport `uart`) for the OKB ROSA Oscill DSO1 pen oscilloscope.
Background: `docs/oscill-proto.md` (original, partly wrong spec), `docs/reverse-engineering.md` (findings from the
vendor's `oscill.exe` / `oscilink.dll`, with function addresses), `README.md` (build, status, test plan).

## Layout
- `src/OscillProtocol.*` — OBEX framing, no scopehal dependency, unit-tested by `tests/test_protocol.cpp`
- `src/OscillSerial.*` — poll()-based I/O on the fd borrowed from libscopehal's `uart` transport (its own reads have
  no timeout on Linux); clears DTR/RTS, disables HUPCL
- `src/OscillOscilloscope.*` — the driver; all guessed hardware mappings are at the top of the .cpp
- `tools/oscill_tool.py` — pyserial poking tool for real hardware; `tools/oscill_sim.py` — pty probe simulator
- `tests/smoke_test.cpp` — loads the plugin via libscopehal's plugin loader and captures from a port

## Real hardware findings so far (probe S/N 3404, hw 1.25, fw 1.26, /dev/ttyUSB1)
- Toggling DTR/RTS on port open/close resets the probe into its **bootloader**: CONNECT reply then contains an OBEX
  Description header "BOOT" (`a0 00 14 10 00 04 80 05 00 0d 00 42 00 4f 00 4f 00 54 00 00`) and every command
  gets `d0 00 03`. Recovery: unplug/replug. Always run the tool with `--dtr off --rts off` and do
  `stty -F /dev/ttyUSB1 -hupcl` once after plugging in.
- Normal CONNECT reply: `a0 00 07 10 00 00 18`.
- GET 'F' is rejected by the oscilloscope firmware (it answered once, apparently from the bootloader, with the string
  inside an End-of-Body 0x49 header: "Uosc1.2534041.21").
- `83 LL LL 70 00 LL <name>` ('p' read) works **only for the 3-letter read-only registers**. Reply = echoed name
  header + separate value header, e.g. `a0 00 0e 70 00 06 'MCl' f0 00 00 03 20`. Values read:
  VNM "Uosc", VSN "3404", VHW "1.25", VSW "1.26" (4 ASCII chars in an F1 value), MCl 800, MCh 5425, MCd 1248,
  TMl 25, TMh 256, TCh 252, QSh 254.
- 'p' reads of the 2-letter settings registers (MC QS TS V1 O1 T1 S1 M1 RT TD TA TW) are rejected (`d0`).
- **Settings registers are read with PUT + a name-only 'q' header**: `83 00 08 71 00 05 'RT'` →
  `a0 00 0a 71 00 05 'RT' b1 00`. GET (0x82) with the same header just returns `a0 00 03`. Widths match the vendor
  code exactly. Power-up values read: MC 1991, QS 254, TS 512, V1 10000 (mV/div), O1 0, T1 0x30, S1 127, M1 0,
  RT 0, TD 0, TA 65280, TW 65280, AP 0, AR 1, RS 0. P1h/P1l rejected.
- Register writes work; the reply echoes name + value (`a0 00 0a 71 00 05 'O1' b1 01`), not a bare `a0 00 03`.
- GET 'D' reply: `a0 01 0c | 72 00 04 44 | 49 01 05 | 00 00 04 00 | 254 samples`. The "7 metadata bytes" of
  the spec are the 3-byte End-of-Body header + 4 bytes (`00 00 04 00` so far). Sample count = QS (254).
- **GET 'C' once after connecting is required**: before it, 'D' returned a stale all-255 buffer that ignored every
  setting; after it the samples follow the input (floating tip at 0.1 V/div: noise 119..143 around 128).
  The first two samples of a record are junk (≈243-255, ≈66).
- In Auto mode with no trigger, 'D' answers at once with an empty body `a0 00 0a 72 00 04 44 49 00 03`.
- **TS confirmed**: sample period = TS/256 × MC × 10 ps. TS 2030000, MC 1982 → 157.2 µs/sample; 50 Hz mains hum
  showed a 126-sample period (50.5 Hz). TMl/TMh (25/256) are NOT TS limits (meaning unknown).
- MC drifts by itself (1991 at power-up, 1982 after writing 1991, later 1993): read it, don't assume it.

## Open questions / next tests (in this order)
1. ~~Register writes~~, ~~settings reads~~, ~~waveform layout~~, ~~stale data~~ (GET 'C'), ~~TS formula~~ — done.
2. Build the plugin against an ngscopeclient build (none installed on this PC yet) and try it end to end.
3. Junk first two samples: drop them in the driver? Check what the 4 metadata bytes mean (byte 0 varies 00/10/20/30).
4. Volt scale with a known DC source (battery), O1 bits (ground made no visible change at 10 V/div — retest at
   0.1 V/div with a signal), trigger S1/T1/TC with mains hum.
5. Then the README's volt-scale, O1-bit and trigger tests.

## Building
Needs a scopehal-apps checkout + build (same one ngscopeclient was built from):
`cmake -B build -DSCOPEHAL_APPS_DIR=... -DSCOPEHAL_BUILD_DIR=... && cmake --build build -j`, then
`cmake --install build` → `~/.scopehal/plugins/liboscill.so`. `build/test_protocol` needs no scopehal.
