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
- **Problem:** in free-run mode the samples are all 255 except the first two (≈240, ≈67) and do not change with
  O1 = 0/1/2/3 (ground made no difference). The ADC data isn't tracking the input — likely a start/arm step the
  vendor app performs is missing. Unresolved.

## Open questions / next tests (in this order)
1. ~~Register writes~~ — work. 2. ~~Reading settings registers~~ — PUT + name-only 'q'. 3. ~~Waveform layout~~ — known.
3b. Why the samples don't follow the input (see Problem above). Ideas: other 'r' commands the vendor app sends
   before 'D'; the ADC needs RT/MC/TS rewritten after power-up; physical setup (battery, tip connection).
4. Timebase (note: TS = 512 at power-up, i.e. above TMh = 256): TMl/TMh = 25..256 contradicts "TS = clocks per sample × 256" for slow rates. Measure: write a few TS values, capture 50 Hz mains hum (touch the tip), count samples per period. Then fix `RateToTS`/`GetSampleRatesNonInterleaved` in the driver — **do not trust the current sample-rate list until this is done**.
5. Then the README's volt-scale, O1-bit and trigger tests.

## Building
Needs a scopehal-apps checkout + build (same one ngscopeclient was built from):
`cmake -B build -DSCOPEHAL_APPS_DIR=... -DSCOPEHAL_BUILD_DIR=... && cmake --build build -j`, then
`cmake --install build` → `~/.scopehal/plugins/liboscill.so`. `build/test_protocol` needs no scopehal.
