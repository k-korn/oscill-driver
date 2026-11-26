# Oscill DSO1 driver: implementation

A libscopehal driver plugin (driver name `oscill`) that lets ngscopeclient use the Oscill DSO1. Protocol facts it
relies on are in [Protocol.md](Protocol.md); this document covers how the driver uses them.

## Files

| Path | Contents |
|---|---|
| `src/OscillPlugin.cpp` | `PluginInit()`: registers the driver with libscopehal |
| `src/OscillOscilloscope.{h,cpp}` | the driver (`OscillOscilloscope`, derived from `SCPIOscilloscope`) |
| `src/OscillSerial.{h,cpp}` | timed serial I/O on the file descriptor of libscopehal's `uart` transport |
| `src/OscillProtocol.{h,cpp}` | packet building / parsing and the edge finder; no libscopehal dependency |
| `tests/test_protocol.cpp` | unit tests for `OscillProtocol`, many using bytes captured from the real probe |
| `tests/smoke_test.cpp` | end-to-end test through libscopehal's real plugin loader |
| `tools/oscill_tool.py` | command-line tool for poking a real probe (pyserial) |
| `tools/oscill_sim.py` | probe simulator on a pseudo-terminal |

## Connection and serial layer

* ngscopeclient connects with driver **`oscill`**, transport **`uart`**, path **`/dev/ttyUSB0:921600`**. USB serial
  numbering changes with plug order (and after suspend / resume), so prefer the stable name, e.g.
  `/dev/serial/by-id/usb-Silicon_Labs_Oscill_USB_interface_<serial>-if00-port0:921600`.
* libscopehal's `uart` transport opens the port, but its reads block forever on Linux (`VMIN=1`, `SetTimeouts()` is a
  no-op). `OscillSerial` therefore takes the transport's file descriptor (through a pointer-to-member, without
  modifying libscopehal) and does its own `poll()`-based reads with timeouts and its own termios setup (raw 8N1, no
  flow control, `HUPCL` off).
* **DTR and RTS are cleared after the baud rate is set.** libscopehal leaves the port at `B0`, and leaving `B0` makes
  the cp210x driver raise both lines; clearing them earlier (as before commit `d43132f`) left them high for the whole
  session.
* **Only one program may use the port.** With other software polling the same probe (e.g. the vendor app under Wine),
  requests interleave and replies get restarted mid-frame. A background service that opens `/dev/ttyUSB0` by number
  can grab the probe when the numbering changes.
* If the port fails (hangup, I/O error: probe unplugged or USB reset), `OscillSerial` logs once and marks the port
  dead; the driver then reports the instrument as stopped. Reconnect it in ngscopeclient.

### Connect sequence (`Connect()`)

1. Abort + CONNECT at the rate from the path, twice; if there's no answer, the same at 9600. Trying the requested rate
   first recovers a probe left at a high rate by a crashed session. (After a clean session the probe is at 9600, so
   "no answer at 921600 baud, retrying at 9600" in the debug log is normal.)
2. **If the probe is silent at both rates**: log the host-side port state (modem lines, bytes stuck in the output
   queue, input queue, termios, other processes holding the tty), then try in turn, logging which one worked:
   CONNECT repeated for ~5 s (drains a probe stuck mid-frame), DTR/RTS high, DTR/RTS low. This exists because
   ngscopeclient occasionally got no answer at all while a test program on the same tty connected fine; it hasn't
   recurred since DTR/RTS are really held low.
3. If the reply says `BOOT`, give up with a clear error (the probe needs a replug).
4. Switch to the requested rate with `0x91` (`ChangeBaudRate()`): send, wait 50 ms, switch the host, verify with
   CONNECT (3 tries). On failure send "back to 9600" at the new rate, then switch the host back.
5. Calibrate (PUT `C`): required for live data.
6. Read identity (`VNM`, `VSN`, `VHW`, `VSW`) and log the limits (`MCl`, `MCh`, `MCd`, `TMl`, `TMh`, `TCh`, `QSh`).
7. **Start from the probe's state**: `V1` → volts/div, `O1` → coupling and filter, `TS` → sample rate, plus `MC`,
   `QS` and `TCh`.
8. Centre the trigger position, create the edge trigger object, push everything (`PushAll()`): `RT` = 2 (free run
   until armed), `M1` = 4 (normal sampling; the power-up 0 is averaging), `TD` = 0, then `O1`, `V1`, `TS` and the
   trigger registers.

`IDPing()` reads `VNM`, `VSN` and `VSW` fresh. The destructor waits out any pending request and switches the probe
back to 9600, as the vendor app does.

Register access: 3-letter names use the `p` read, 2-letter names the name-only `q` query; writes are PUT with `q` and
a `B1`/`F0`/`F1` value by register width. A PUT write is acknowledged with a bare `a0 00 03`; where the applied value
matters (`TS`) the driver reads it back. After a rate change it re-reads `MC` (it drifts), `QS` and `TCh`. If the
probe rejects writes, the first five failures are logged and capturing continues with the probe's own settings.

## Acquisition

### Run modes

| ngscopeclient | `RT` | Timeout | Record without a trigger |
|---|---|---|---|
| Normal (Run) / Single | 1 (Wait) | `TW` = 0.25 s | empty reply; the driver asks again |
| Auto | 0 (Auto) | `TA` = 0.1 s | delivered, aligned on an edge if it has one |
| Force | 2 (free run) | — | delivered |

`RT` is written only when it changes, before the next `D` request. `TW` is kept short because a request can't be
interrupted: a settings change or Stop waits for the outstanding reply.

### Polling (`PollTrigger()` / `AcquireData()`)

ngscopeclient's instrument thread calls `PollTrigger()` in a loop while armed:

1. If no request is outstanding: set `RT` for the current mode, send GET `D`.
2. Wait up to 50 ms for a reply (`ReadFrame()` keeps partial data across calls).
3. Empty reply (Wait timeout): ask again next poll.
4. Record: align the trigger (below), keep the frame, return `TRIGGER_MODE_TRIGGERED`.
5. Watchdog: no reply within 2 × (2 × record + trigger wait) + 5 s → cancel and re-arm.

`AcquireData()` drops the 2-byte size field, scales codes to volts, sets the sample period and trigger phase, and
queues the waveform. Records are `QS` − 2 = 252 samples with the power-up `QS`.

Any configuration change (and Stop) first ends an outstanding `D` (`CancelPendingAcquisition()`) by **waiting for its
reply**, until request time + 2 × record + trigger wait + 1 s, and discarding it; only if it never comes is Abort
sent. At slow rates a change can therefore take a whole record (2.5 s at 100 S/s, the slowest rate offered). Frames
that don't parse trigger `Resync()`: drop the buffer and flush the input.

All methods that touch the port hold `m_mutex`; ngscopeclient calls them from both its GUI and its instrument thread.

### Trigger

* Level and edge are **copied from the trigger object under `m_mutex`** whenever ngscopeclient pushes or pulls the
  trigger (`CacheTriggerSettings()`). The instrument thread never touches the trigger object: ngscopeclient's
  `SetTrigger()` replaces and deletes it from the GUI thread without the driver's lock.
* Registers (`PushTriggerRegisters()`):
  * `T1`: rising `0x20` / falling `0x10` / either `0x30`, plus hysteresis on (`0x0F`), HF + LF path;
  * `S1` = `128 + level / volts_per_code`, clamped to 1…254; rewritten when the range or attenuation changes;
  * `TC` = trigger offset in samples + 2.5 (the measured lag, Protocol.md), clamped to `TCh` and the record;
    rewritten when the offset or sample rate changes, together with `TA` / `TW` (they're in clock units).
* Alignment (`AlignTrigger()`): for a record the hardware triggered on (source bits ≠ `00`) the crossing is expected
  2.5 samples before `TC`. `FindEdge()` looks for it within 4 samples of there and interpolates it; if it isn't found
  (seen in ~1 in 10 records at 6.25 MS/s with a heavily clipped signal) the expected position is used. Records that
  weren't hardware-triggered (Auto timeout, Force) are aligned on the edge nearest the trigger offset if there is one.
* `FindEdge()` works on the raw codes with 3 codes of hysteresis and no filtering (a one-sample spike is a
  legitimate trigger); the waveform's `m_triggerPhase` puts the crossing exactly on ngscopeclient's trigger marker.

Measured with a ±2.3 V signal (`smoke_test … trigger <rate>`): all edges aligned at 6.4 kS/s and 1 MS/s, both
directions.

## Mapping to ngscopeclient controls

| ngscopeclient | Driver | Registers |
|---|---|---|
| Volts/div (range = 8 div) | 20 mV … 10 V per div in 1-2-5 steps; smallest range that fits the request | `V1` = mV/div |
| Offset | display only | — |
| Coupling | DC / AC (1 MΩ) / GND | `O1` bits 0–1 |
| ADC mode | "Full bandwidth", "Low-pass ~3 MHz", "Low-pass ~3 kHz" | `O1` bits 2–3 |
| Bandwidth limit | none / 3 MHz (the API takes whole MHz, so 3 kHz is an ADC mode only) | `O1` bit 2 |
| Probe attenuation | software multiplier on the volt scale (trigger level follows) | `S1` |
| Sample rate | 1-2-5 steps from 100 S/s to ~25 MS/s (`TS` = 512); shown as the list entry nearest the applied rate | `TS` |
| Sample depth | not settable: `QS` − 2 | — |
| Trigger | edge: rising / falling / either, level, position | `T1`, `S1`, `TC` |
| Run / Single / Stop / Force / Auto | see [Run modes](#run-modes) | `RT`, `TA`, `TW` |

Not exposed: `M1` averaging / peak modes, `QS`, RIS, ROLL, `TD`, `P1`.

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

The package ships no headers, so recreate its source tree and only *configure* it (needs the Vulkan headers,
`vulkan-headers` on Arch):

```bash
git clone https://github.com/ngscopeclient/scopehal-apps ~/.cache/oscill/scopehal-apps-0.3
```

```bash
cd ~/.cache/oscill/scopehal-apps-0.3 && git checkout v0.3 && git submodule update --init --recursive
```

```bash
cmake -S ~/.cache/oscill/scopehal-apps-0.3 -B ~/.cache/oscill/scopehal-apps-0.3/build -DCMAKE_BUILD_TYPE=Release -GNinja
```

Then:

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

`tools/oscill_sim.py --link /tmp/ttyOSC` creates a pseudo-terminal that answers like a fw 1.26 probe: power-up
register values, PUT writes acknowledged with a bare `a0 00 03`, GET writes echoed, `C` and Abort answered, `D`
answered one record duration later with the real layout. Options: `--reject-writes`, `--silent` (dead probe),
`--boot` (stuck in bootloader). It doesn't simulate triggering, the stale buffer before `C`, or the probe ignoring
commands while digitizing.

### Smoke test (`-DOSCILL_BUILD_SMOKE_TEST=ON`)

Each mode is one run of `build/smoke_test <port>:921600 <count> [mode]`:

| Mode | What it checks | Needs |
|---|---|---|
| (none) | Auto, range / rate / trigger changes while armed, Normal, Single, Force | a signal crossing 0 V for Normal / Single |
| `connect` | connect and identify only; for repeated-connect tests | — |
| `trigger [rate]` | trigger alignment on rising and falling edges (default 6400 S/s) | a periodic signal |
| `filters` | spike height / width per ADC mode | a spiky signal |
| `stress` | 30 s: acquisition thread vs a GUI-like settings thread replacing trigger objects | — |
| `slow` | 100 S/s: capture arrives, Stop mid-record keeps the link in sync | — |

* The smoke test loads plugins from `build/` **and** `~/.scopehal/plugins/`; the one loaded last wins. Run
  `cmake --install build` after every rebuild, or you test a stale plugin.
* It can link against an installed libscopehal; it then compiles liblog's sink classes itself and exits with
  `_exit()` to avoid freeing the shared log-sink list twice.
* For ngscopeclient itself: `ngscopeclient --debug --trace OscillSerial --trace OscillOscilloscope` logs every byte.

### Hardware tool

`tools/oscill_tool.py` (pyserial). Always pass `--dtr off --rts off`.

```bash
python3 tools/oscill_tool.py -p /dev/ttyUSB1 --dtr off --rts off read MCd TS V1 O1 RT
```

Commands: `id`, `read REG…` (`p` or `q` by name length), `write REG auto|u8|u16|u32 VALUE`,
`capture [--count N] [--csv FILE] [--no-calibrate]`, `sweep REG START STOP`, `speed BAUD` (switch, verify with
CONNECT), `raw HEX…`; `-v` dumps all traffic. `capture` reports a Wait-mode timeout as an empty record.

## Known limitations

* Sample rates 100 S/s to ~25 MS/s. Slower rates would block settings changes for a whole record; one sample per clock
  (~50 MS/s) and RIS are untested.
* Records are 252 samples: longer records (`QS` up to `QSh`) aren't used.
* `M1` averaging / peak modes, `TD` delay, `P1` hardware offset and ROLL aren't exposed.
* Single channel, no external trigger.
