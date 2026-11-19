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
* **Only one program may use the port.** With the vendor app (e.g. `oscill.exe` under Wine) polling the same probe,
  requests interleave, replies get restarted mid-frame, and the vendor DLL may even reset the USB device.
* If the port fails (hangup, I/O error: probe unplugged or USB reset), `OscillSerial` logs once and marks the port
  dead; the driver then reports the instrument as stopped instead of retrying. Reconnect it in ngscopeclient.

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
5. Read identity (`VNM`, `VSN`, `VHW`, `VSW`, 4 ASCII chars each) and log the limits (`MCl`, `MCh`, `MCd`, `TMl`,
   `TMh`, `TCh`, `QSh`).
6. **Start from the probe's state**: `V1` → volts/div, `O1` → coupling and filters, `TS` → sample rate, plus `MC`
   and `QS`.
7. Centre the trigger position in the record, create the edge trigger object, and push the configuration
   (`PushAll()`): a neutral acquisition state (`RT` = 0 free run, `M1` = 0 plain sampling, `TC` = 0, `TD` = 0;
   the vendor app may leave Average / Peak or a delay set), then `O1`, `V1` and `TS`.

`IDPing()` reads `VNM`, `VSN` and `VSW` fresh (GET `F` is rejected by the firmware). On disconnect the destructor
cancels any pending request and switches the probe back to 9600, as the vendor app does.

Register access: 3-letter names use the `p` read, 2-letter names the name-only `q` query; writes use `q` with a
`B1`/`F0`/`F1` value by register width. A write counts as successful on any `A0` reply, which echoes the value the
probe applied: for `TS` the driver uses that echo (the probe rounds fast rates to whole clocks), then re-reads `MC`
(it drifts) and `QS`. If the probe rejects writes, the driver logs the first five failures and keeps capturing with
the probe's own settings.

The hardware trigger registers (`S1`, `T1`, `TC`, `TD`, `TA`, `TW`) are not written after connect: they have no
effect on firmware 1.26, and every write costs a round trip that has to wait for the current record.

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
5. Watchdog: no reply within 2 × record duration + 5 s → cancel and re-arm. (The reply normally comes one record
   duration after the request: 2.5 s at the slowest offered rate.)

`AcquireData()` then converts the record: drops the first 2 (garbage) samples, scales codes to volts, sets the sample
period and the trigger phase, and queues the waveform. Records are `QS` − 2 samples (252 with the power-up `QS`).

Any configuration change (and Stop) first ends an outstanding `D` (`CancelPendingAcquisition()`). The probe neither
accepts commands while acquiring nor tolerates them while transmitting, so the driver **waits for the reply**, until
its expected time (request time + record duration + 1 s), and discards it. Only if it never comes is `C` sent. Then
it writes the register and lets the next poll re-arm. At slow rates a setting change can therefore take up to one
record (2.5 s at 100 S/s), which is why slower rates aren't offered. A waveform captured with old settings is
discarded. Any frame that doesn't parse triggers `Resync()`: drop the buffer and flush the input.

All methods that touch the port hold the oscilloscope's `m_mutex`; ngscopeclient calls them from both its GUI and its
instrument thread.

### Software trigger (`ApplySoftwareTrigger()`, `FindEdge()`)

* Level and edge are **copied from the trigger object under `m_mutex`** whenever ngscopeclient pushes or pulls the
  trigger (`CacheTriggerSettings()`). The instrument thread never touches the trigger object: ngscopeclient's
  `SetTrigger()` replaces and deletes it from the GUI thread without the driver's lock.
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

## Mapping to ngscopeclient controls

| ngscopeclient | Driver | Registers |
|---|---|---|
| Volts/div (range = 8 div) | 20 mV … 10 V per div in 1-2-5 steps; smallest range that fits the request | `V1` = mV/div |
| Offset | display only (the probe has no hardware offset in use) | — |
| Coupling | DC / AC (1 MΩ) / GND | `O1` bits 0–1 |
| ADC mode | "Full bandwidth", "Low-pass ~3 MHz (Filt L)", "High-pass ~3 kHz (Filt H)", "Band-pass (Filt L + Filt H)" | `O1` bits 2–3 |
| Bandwidth limit | none / 3 MHz: switches the low-pass only (the API takes whole MHz) | `O1` bit 2 |
| Probe attenuation | software multiplier on the volt scale | — |
| Sample rate | 1-2-5 steps from 100 S/s to the fastest tested rate (`TS` = 512, ~25 MS/s), plus that maximum; shown as the list entry nearest the rate the probe applied | `TS` = round(256 × Tsample / Tclock) |
| Sample depth | not settable: whatever `QS` delivers, minus 2 garbage samples (re-read after rate changes) | — |
| Trigger | edge: rising / falling / either, level; position = trigger offset | software (see above) |
| Run / Single / Stop / Force / Auto | as in the table above | `RT` = 0 always |

Values not exposed: `M1` acquisition modes (Average / HiRes / Peak), `QS`, RIS, `TD` delay mode.

Scaling uses the probe's own `MC`, read at connect and after every rate change (it drifts by itself; see
Protocol.md):

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
`--boot` (stuck in bootloader). Like the real probe it answers `D` one record duration later, sends 4 metadata bytes
and `QS` samples (the first two garbage) and echoes applied values (rounding fast `TS` to whole clocks). It doesn't
reproduce the stale buffer before `C` or the probe ignoring commands while acquiring.

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

```bash
build/smoke_test /dev/ttyUSB1:921600 3 stress     # 30 s: acquisition thread vs a GUI-like settings thread
```

```bash
build/smoke_test /dev/ttyUSB1:921600 3 slow       # 100 S/s: capture arrives, Stop mid-record keeps the link in sync
```

The stress mode replaces and deletes trigger objects, moves the level, and changes range and sample rate every
~20 ms while another thread acquires. A clean run shows no warnings (e.g. 1938 waveforms during 906 changes).

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
* Sample rates: 100 S/s to ~25 MS/s. Slower rates would block settings changes for a whole record (the probe ignores
  commands while acquiring); one sample per clock (~50 MS/s) and RIS are untested / unsupported.
* The ~3 MHz low-pass is unconfirmed on hardware.
* `M1` acquisition modes, `QS` record length, `TD` delay mode and the probe's "Memo" / ROLL features aren't exposed.
* Single channel, no external trigger.
