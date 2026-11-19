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

## Open questions / next tests (in this order)
1. Does a register write work now that the probe is out of BOOT? `tools/oscill_tool.py -p /dev/ttyUSB1 --dtr off --rts off -v write RT auto 0` (expect `a0 00 03`). The earlier failed write was sent while in BOOT.
2. Read settings registers with a name-only 'q' header (oscilink.dll routine 0x11039690): `... -v raw 83 00 08 71 00 05 52 54` (RT), `... raw 83 00 08 71 00 05 54 53` (TS).
3. First waveform with the tip shorted to ground: `... -v capture`. Check the reply layout (0x49 wrapper? 7 metadata bytes?) and sample count (QSh=254 suggests ≤254, not the spec's 344).
4. Timebase: TMl/TMh = 25..256 contradicts "TS = clocks per sample × 256" for slow rates. Measure: write a few TS values, capture 50 Hz mains hum (touch the tip), count samples per period. Then fix `RateToTS`/`GetSampleRatesNonInterleaved` in the driver — **do not trust the current sample-rate list until this is done**.
5. Then the README's volt-scale, O1-bit and trigger tests.

## Building
Needs a scopehal-apps checkout + build (same one ngscopeclient was built from):
`cmake -B build -DSCOPEHAL_APPS_DIR=... -DSCOPEHAL_BUILD_DIR=... && cmake --build build -j`, then
`cmake --install build` → `~/.scopehal/plugins/liboscill.so`. `build/test_protocol` needs no scopehal.
