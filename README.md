# Oscill DSO1 plugin for ngscopeclient (Linux)

A libscopehal driver plugin for the OKB ROSA **Oscill DSO1** pen oscilloscope (CP210x USB-UART, OBEX-style framing).
It is based on the protocol notes in `docs/oscill-proto.md`, corrected against the vendor's Windows software
(see `docs/reverse-engineering.md`).

## Status

| Part | State |
|---|---|
| CONNECT handshake, GET `D` (waveform), `F` (ID), `C` (cancel) | Implemented as documented; tested against the simulator |
| Acquisition state machine (auto / normal / single / force / stop, cancel-on-reconfigure) | Implemented; tested against the simulator |
| Register read and write framing, and the width of each register | **Confirmed** by disassembling `oscilink.dll` and `oscill.exe` |
| Baud-rate change (opcode `0x91`) | **Confirmed** packet layout. Use it by putting a higher rate (1843200/n) in the connection string; falls back to 9600 |
| Register read reply layout | Not confirmed; the parser accepts several layouts |
| `V1` = mV/div; `TS` = clocks per sample × 256 within `TMl`..`TMh`; `MC` in 10 ps units | **Confirmed** from `oscill.exe` code; the driver reads `MC`, `TMl` and `TMh` at connect |
| ADC code → volts (code 128 = 0 V, 256 codes = 8 divisions) | Inferred from the vendor's display limits, not confirmed |
| Trigger: level = `S1`, edge = `T1` bits 4–5, position = `TC` (pre-trigger samples), negative offset = `TD` delay | **Confirmed** from `oscill.exe`. That `S1` uses the ADC code scale is very likely but unconfirmed |
| Coupling: `O1` bit 0 = ground (confirmed), bit 1 = DC (guess); filter bits are preserved | Partly confirmed. `M1` is the acquisition mode, so the driver no longer writes it |
| Bandwidth filters (`O1` bits 2–3), record length (`QS`), auto/wait timeouts (`TA`/`TW`), RIS, ROLL | Not implemented |

If the probe rejects register writes, the driver logs a few warnings and keeps capturing with the probe's own
settings, but the volt and time scales may then be wrong.

## Build

The plugin is loaded into ngscopeclient's process, so it **must be built against the same scopehal-apps checkout
and build directory as the ngscopeclient binary you run.**

```bash
cmake -B build -DSCOPEHAL_APPS_DIR=$HOME/scopehal-apps -DSCOPEHAL_BUILD_DIR=$HOME/scopehal-apps/build
```

```bash
cmake --build build -j
```

```bash
cmake --install build      # copies liboscill.so to ~/.scopehal/plugins/
```

To use it, add an oscilloscope in ngscopeclient with driver **`oscill`**, transport **`uart`** and path
**`/dev/ttyUSB0:921600`** (the Add Instrument dialog suggests this). Your user needs access to the port (`dialout` or
`uucp`, depending on the distribution). The driver does the handshake at 9600, switches the probe to the requested rate
with opcode `0x91`, checks it, and puts the probe back to 9600 on disconnect, as the vendor software does. 921600 is
confirmed on hardware: a record takes ~0.04 s to transfer instead of ~0.33 s at 9600, which matters because triggering
is done in software on each record. Any rate of the form 1843200/n works; use `:9600` to stay at the power-up rate.

If no `/dev/ttyUSB*` appears, check `lsusb`. For a `10c4:840e` adapter, bind the cp210x driver by hand:

```bash
echo 10c4 840e | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id
```

## Tests

* `build/test_protocol` runs unit tests of the framing code. It doesn't need scopehal.
* `tools/oscill_sim.py` simulates the probe on a pty (`--reject-writes` and `--silent` simulate failures).
* `build/smoke_test PATH:9600` is an end-to-end test that loads the plugin through libscopehal's real plugin loader.
  Configure with `-DOSCILL_BUILD_SMOKE_TEST=ON` to build it.

## Verifying on real hardware

`tools/oscill_tool.py` needs pyserial. Use it to confirm the guessed parts before trusting the plugin's scaling:

```bash
tools/oscill_tool.py -p /dev/ttyUSB0 -v id
```

```bash
tools/oscill_tool.py -p /dev/ttyUSB0 read MC V1 TS RT M1
```

```bash
tools/oscill_tool.py -p /dev/ttyUSB0 sweep TS 0 20
```

```bash
tools/oscill_tool.py -p /dev/ttyUSB0 speed 115200
```

```bash
tools/oscill_tool.py -p /dev/ttyUSB0 capture --count 3 --csv cap.csv
```

## Design notes

* libscopehal's `uart` transport does fully blocking reads on Linux (`SetTimeouts()` is a no-op). Used directly, a
  silent probe would hang ngscopeclient's instrument thread. The plugin therefore takes the transport's file
  descriptor and does its own `poll()`-based reads with timeouts and its own termios setup.
* In Normal mode the probe holds back the `D` reply until it triggers. `PollTrigger()` blocks for at most 50 ms at a
  time. Any configuration change first cancels the pending request with `C`, and the next poll re-arms.
