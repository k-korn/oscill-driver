# Oscill DSO1 driver: implementation

A libscopehal driver plugin (driver name `oscill`) that lets ngscopeclient use the Oscill DSO1. Protocol facts it
relies on are in [Protocol.md](Protocol.md); this document covers how the driver uses them.

## Files

| Path | Contents |
|---|---|
| `src/OscillPlugin.cpp` | `PluginInit()`: registers the driver with libscopehal |
| `src/OscillOscilloscope.{h,cpp}` | the driver (`OscillOscilloscope`, derived from `SCPIOscilloscope`) |
| `src/OscillSerial.{h,cpp}` | timed serial I/O on the file descriptor of libscopehal's `uart` transport |
| `src/OscillProtocol.{h,cpp}` | packet building / parsing and the software edge finder; no libscopehal dependency |
| `tests/test_protocol.cpp` | unit tests for `OscillProtocol`, many using bytes captured from the real probe |
| `tests/smoke_test.cpp` | end-to-end test through libscopehal's real plugin loader |
| `tools/oscill_tool.py` | command-line tool for poking a real probe (pyserial) |
| `tools/oscill_sim.py` | probe simulator on a pseudo-terminal |

## Connection and serial layer

* ngscopeclient connects with driver **`oscill`**, transport **`uart`**, path **`/dev/ttyUSB0:921600`** (the Add
  Instrument dialog suggests this path).
* libscopehal's `uart` transport opens the port, but its reads block forever on Linux (`VMIN=1`, `SetTimeouts()` is a
  no-op). A silent probe would hang ngscopeclient's instrument thread. `OscillSerial` therefore takes the transport's
  file descriptor (through a pointer-to-member, without modifying libscopehal) and does its own `poll()`-based reads
  with timeouts and its own termios setup (raw 8N1, no flow control).
* Right after attaching it clears **DTR and RTS** and disables **HUPCL**. Modem-line activity resets the probe into
  its bootloader.

### Connect sequence (`Connect()`)

1. CONNECT at the baud rate from the path; if there's no answer, at 9600. (Trying the requested rate first recovers a
   probe left at a high rate by an earlier session.)
2. If the reply says `BOOT`, give up with a clear error (the probe needs a replug).
3. If the requested rate isn't the current one, switch with `0x91` (`ChangeBaudRate()`):
   * send the command, wait 50 ms, switch the host side, discard the garbled acknowledgement;
   * verify with CONNECT (up to 3 tries);
   * on failure, send "back to 9600" **at the new rate** before switching the host back, so the two sides can't end
     up mismatched.
4. Send GET `C` (required for live data).
5. Read identity (`VNM`, `VSN`, `VHW`, `VSW`, 4 ASCII chars each) and limits (`MCl`, `MCh`, `MCd`, `TCh`), plus the
   current `MC`, `QS`, `TS`, `O1` and `T1`.
6. Centre the trigger position in the record, create the edge trigger object, and push the full configuration
   (`PushAll()`).

`IDPing()` reads `VNM`, `VSN` and `VSW` fresh (GET `F` is rejected by the firmware). On disconnect the destructor
cancels any pending request and switches the probe back to 9600, as the vendor app does.

Register access: 3-letter names use the `p` read, 2-letter names the name-only `q` query; writes use `q` with a
`B1`/`F0`/`F1` value by register width. A write counts as successful on any `A0` reply. If the probe rejects writes,
the driver logs the first five failures and keeps capturing with the probe's own settings.

## Acquisition

The probe always runs in free run (`RT` = 0): the hardware trigger has no effect on firmware 1.26, so triggering is
done in software.

### Polling (`PollTrigger()` / `AcquireData()`)

ngscopeclient's instrument thread calls `PollTrigger()` in a loop while armed:

1. If no request is outstanding, send GET `D`.
2. Wait up to 50 ms for a reply (`ReadFrame()` keeps partial data across calls).
3. Empty record (`49 00 03`): clear the request and ask again next poll.
4. Full record: run the software trigger. In Normal / Single mode a record without the selected edge is dropped and
   the next poll asks again. Otherwise the frame is kept and `TRIGGER_MODE_TRIGGERED` returned.
5. Watchdog: no reply within 5 s → cancel with `C` and re-arm.

`AcquireData()` then converts the record: drops the first 2 (garbage) samples, scales codes to volts, sets the sample
period and the trigger phase, and queues the waveform. Records are `QS` − 2 samples (252 with the power-up `QS` of 254; `QS` has also been seen at 172).

Any configuration change first cancels an outstanding `D` with `C` (`CancelPendingAcquisition()`), writes the
register, and lets the next poll re-arm. A waveform captured with old settings is discarded.

### Software trigger (`ApplySoftwareTrigger()`, `FindEdge()`)

* Level in ADC codes: `128 + level_volts / volts_per_code`.
* Edge search on the raw codes (after the 2 garbage samples) with **3 codes of hysteresis**: a rising edge needs the
  signal at or below `level − 3` before it reaches the level (falling: mirror image). This rejects noise of about ±2–3
  codes. There is **no filtering**: a single-sample pulse is a legitimate trigger.
* The crossing is interpolated between the two samples around it, and of all matching crossings the one nearest the
  configured trigger position is chosen.
* The waveform's `m_triggerPhase` is set so that crossing lands exactly on ngscopeclient's trigger marker (libscopehal
  draws sample *i* at `i × timescale + m_triggerPhase`). The whole record is kept.

| Mode | Record without an edge |
|---|---|
| Normal (Run) / Single | dropped; keep polling |
| Auto | shown unaligned |
| Force | shown as is (one record) |

At 921600 baud a record arrives every ~0.04 s, so the driver checks ~25 records per second for an edge. Rare events
between records are missed (the probe's dead time between records is unknown).

### Timeouts written at connect

`TA` (Auto timeout) is set to 100 ms. Its power-up value (~15.6 s) made Auto mode look dead in an earlier version.
`TC` / `TD` / `T1` / `S1` are still written from the trigger settings (harmless; the probe reports them in metadata).

## Mapping to ngscopeclient controls

| ngscopeclient | Driver | Registers |
|---|---|---|
| Volts/div (range = 8 div) | 20 mV … 10 V per div in 1-2-5 steps; smallest range that fits the request | `V1` = mV/div |
| Offset | display only (the probe has no hardware offset in use) | — |
| Coupling | DC / AC (1 MΩ) / GND | `O1` bits 0–1 |
| ADC mode | "Full bandwidth", "Low-pass ~3 MHz (Filt L)", "High-pass ~3 kHz (Filt H)", "Band-pass (Filt L + Filt H)" | `O1` bits 2–3 |
| Bandwidth limit | none / 3 MHz: switches the low-pass only (the API takes whole MHz) | `O1` bit 2 |
| Probe attenuation | software multiplier on the volt scale | — |
| Sample rate | 1-2-5 steps from ~1 S/s up to one sample per MCU clock (~50 MHz), plus that maximum | `TS` = round(256 × Tsample / Tclock) |
| Sample depth | not settable: whatever `QS` delivers, minus 2 garbage samples (read at connect) | — |
| Trigger | edge: rising / falling / either, level; position = trigger offset | software (see above) |
| Run / Single / Stop / Force / Auto | as in the table above | `RT` = 0 always |

Values not exposed: `M1` acquisition modes (Average / HiRes / Peak), `QS`, RIS, `TD` delay mode.

Scaling uses the probe's own `MC`, read at connect (it drifts by itself; see Protocol.md):

```
volts         = (code − 128) × V1/1000 × 8/256
sample period = TS/256 × MC × 10 ps
```

## Building

The plugin runs inside ngscopeclient's process, so it must be compiled against **the same libscopehal version and
compile definitions** as the ngscopeclient you run. `CMakeLists.txt` takes `SCOPEHAL_APPS_DIR` (source tree) and
`SCOPEHAL_BUILD_DIR` (a configured build of it) and copies every `-D` the `scopehal` target uses, from either the
Makefile or the Ninja generator's output. Symbols are resolved at load time from ngscopeclient's libscopehal.

### Against a distribution package (e.g. AUR `scopehal-apps` 0.3)

The package ships no headers, so recreate its source tree and only *configure* it:

```bash
git clone https://github.com/ngscopeclient/scopehal-apps ~/.cache/oscill/scopehal-apps-0.3
```

```bash
cd ~/.cache/oscill/scopehal-apps-0.3 && git checkout v0.3 && git submodule update --init --recursive
```

```bash
cmake -S ~/.cache/oscill/scopehal-apps-0.3 -B ~/.cache/oscill/scopehal-apps-0.3/build -DCMAKE_BUILD_TYPE=Release -GNinja
```

(On the development PC this was done offline from paru's git mirrors in `~/.cache/paru/clone/scopehal-apps`, which
pin the same submodule commits.) Then:

```bash
cmake -B build -G Ninja -DSCOPEHAL_APPS_DIR=$HOME/.cache/oscill/scopehal-apps-0.3 -DSCOPEHAL_BUILD_DIR=$HOME/.cache/oscill/scopehal-apps-0.3/build -DOSCILL_BUILD_SMOKE_TEST=ON
```

```bash
cmake --build build && cmake --install build
```

`cmake --install` copies `liboscill.so` to `~/.scopehal/plugins/`. Restart ngscopeclient to load it. Without
`SCOPEHAL_APPS_DIR` only the unit tests are built.

## Testing

### Unit tests

`build/test_protocol`: framing, register encodings, reply parsing (including bytes captured from the real probe) and
the edge finder. Needs no libscopehal or hardware.

### Simulator

`tools/oscill_sim.py --link /tmp/ttyOSC` creates a pseudo-terminal that answers like a fw 1.26 probe (`q` reads,
`F` rejected, 4-char identity registers, register widths). Options: `--reject-writes`, `--silent` (dead probe),
`--boot` (stuck in bootloader). It produces a moving sine, not the real probe's quirks (no stale buffer before `C`,
no garbage samples).

### Smoke test (`-DOSCILL_BUILD_SMOKE_TEST=ON`)

```bash
build/smoke_test /dev/ttyUSB1:921600 3            # Auto, range/rate/trigger changes, Normal, Single, Force
```

```bash
build/smoke_test /dev/ttyUSB1:921600 3 trigger    # software trigger alignment, needs a periodic signal
```

```bash
build/smoke_test /dev/ttyUSB1:921600 3 filters    # spike height/width per ADC mode, needs a spiky signal
```

* The default run's Normal / Single steps need a signal crossing 0 V on the tip; with a flat input only Auto and Force
  produce waveforms (correct behaviour).
* The smoke test loads plugins from `build/` **and** `~/.scopehal/plugins/`; the one loaded last wins. Run
  `cmake --install build` after every rebuild, or you test a stale plugin.
* It can link against an installed libscopehal; it then compiles liblog's sink classes itself and exits with
  `_exit()` to avoid freeing the shared log-sink list twice.

### Hardware tool

`tools/oscill_tool.py` (pyserial). Always pass `--dtr off --rts off`.

```bash
python3 tools/oscill_tool.py -p /dev/ttyUSB1 --dtr off --rts off read MCd TS V1 O1 RT
```

Commands: `id`, `read REG…` (picks `p` or `q` by name length), `write REG auto|u8|u16|u32 VALUE`,
`capture [--count N] [--csv FILE]` (sends `C` first, waits through empty records), `sweep REG START STOP`,
`speed BAUD` (switch, verify with CONNECT, switch back), `raw HEX…`; `-v` dumps all traffic.

## Known limitations

* The hardware trigger isn't used (it doesn't work on fw 1.26); events that fall between records are missed.
* Sample rates above a few MS/s and the ~3 MHz low-pass are untested on hardware; RIS is not supported.
* `M1` acquisition modes, `QS` record length, `TD` delay mode and the probe's "Memo" / ROLL features aren't exposed.
* Single channel, no external trigger.
