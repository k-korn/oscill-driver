# Oscill DSO1 protocol

The OKB ROSA **Oscill DSO1** is a single-channel pen oscilloscope that talks OBEX-style binary packets over a USB-UART
bridge. This document is everything known about that protocol. Driver-specific choices are in [Driver.md](Driver.md).

## Sources and confidence

Each finding below is tagged with where it comes from:

* **[HW]**: verified on a real probe (S/N 3404, hardware 1.25, firmware 1.26), with `tools/oscill_tool.py` and the
  driver's smoke test.
* **[RE]**: read from the vendor's Windows software (static analysis only, nothing executed): `oscill.exe` v1.4.7 and
  `oscilink.dll` v1.04.0003 (both 2013-09-07, Visual Basic 5 native code, UPX-packed). `RE` addresses refer to the
  unpacked files (`oscilink.dll` image base `0x11000000`, `oscill.exe` `0x400000`).
* **[presumed]**: consistent with everything seen, but not directly proven.

The protocol description that this work started from was largely wrong (register meanings, value encodings, units);
nothing in this document relies on it.

## Physical link

* **USB-UART bridge.** The tested probe enumerates as Silicon Labs CP210x `10c4:ea61` ("Oscill USB interface") and
  appears as `/dev/ttyUSB*` via the kernel `cp210x` driver. [HW] The vendor software also looks for `10c4:840e`
  (via Silabs USBXpress) and Prolific `067b` VCP adapters. [RE] For `840e`, bind the driver by hand:
  `echo 10c4 840e | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id`.
* **8N1, no flow control, 9600 baud at power-up.** [HW] Faster rates are switched in-band (see [Link speed](#link-speed)).
* **Keep DTR and RTS low, and never toggle them.** If the modem lines change when the port opens or closes, the probe
  resets into its **bootloader** (see [Bootloader](#bootloader-boot-state)). Clear `HUPCL` so Linux doesn't drop DTR
  on close (`stty -F /dev/ttyUSB1 -hupcl`), and set DTR/RTS low immediately after opening. [HW]

## Packet framing

Requests and replies are OBEX packets:

```
+------------+------------------+----------------------------+
| opcode (1) | total length (2) | headers ...                |
|            | big-endian, incl.|                            |
|            | these 3 bytes    |                            |
+------------+------------------+----------------------------+
```

Headers follow OBEX rules. The top two bits of the header ID give its encoding [RE, HW]:

| ID range | Encoding |
|---|---|
| `0x00`–`0x3F` | Unicode text, 2-byte length (including the 3-byte header) |
| `0x40`–`0x7F` | byte sequence, 2-byte length (including the 3-byte header) |
| `0x80`–`0xBF` | 1-byte value |
| `0xC0`–`0xFF` | 4-byte big-endian value |

### Opcodes

| Opcode | Use | Status |
|---|---|---|
| `0x80` | CONNECT | [HW] |
| `0x81` | DISCONNECT | [RE] `0x11037a50`; not needed |
| `0x82` | GET: single commands (`r` header) | [HW] |
| `0x83` | PUT: register reads and writes | [HW] |
| `0x91` | link speed change | [HW] |
| `0x9b` | Bluetooth module setup | [RE] `0x1103e2f0`; not needed over USB |
| `0xA0` | reply: success | [HW] |
| `0xD0` | reply: error / not supported | [HW] |

### Header IDs

| ID | ASCII | Meaning |
|---|---|---|
| `0x70` | `p` | read-only register name (read request) |
| `0x71` | `q` | settings register name (read or write) |
| `0x72` | `r` | single command |
| `0x49` | | OBEX End-of-Body: carries waveform data and the `F` string |
| `0x05` | | OBEX Description: only seen in the bootloader's CONNECT reply |
| `0xB1` | | 1-byte value |
| `0xF0` | | 2-byte register value, zero-extended into a 4-byte quantity |
| `0xF1` | | 4-byte value |

`B1`/`F0`/`F1` are OBEX 1-byte and 4-byte quantity headers. `F0` vs `F1` distinguishes 2-byte from 4-byte registers;
it says nothing about signedness. [RE, HW]

The vendor DLL also has an alternative framing mode (register names in a standard OBEX Name header `01 LL LL …` and
values wrapped in `49` End-of-Body headers), and optional checksum / repeat / re-request modes. [RE] None of these are
used by the probe's default configuration and they aren't needed.

## Session

### CONNECT

```
TX  80 00 07 10 00 10 00         CONNECT, OBEX 1.0, flags 0, max packet 0x1000
RX  a0 00 07 10 00 00 18         normal reply
```
[HW]

### Bootloader (BOOT state)

If the probe was reset by DTR/RTS activity it answers CONNECT from its bootloader:

```
RX  a0 00 14 10 00 04 80 05 00 0d 00 42 00 4f 00 4f 00 54 00 00
                         `-- OBEX Description header: UTF-16 "BOOT"
```

Every other request then gets `d0 00 03`. Unplug and replug to recover. The vendor app treats this state as "need to
upload firmware" — don't. [HW, RE `0x4fad60`]

### Recommended start-up sequence

1. CONNECT at 9600 (or at the last-used rate first, in case the probe was left there).
2. Optionally switch link speed (`0x91`), then verify with CONNECT.
3. **Send GET `C` once.** Without it, GET `D` returns a stale buffer (all 255) that ignores every setting. After one
   `C`, the samples follow the input. [HW]
4. Read and write registers as needed.

## Single commands (`r` header)

```
TX  82 00 07 72 00 04 <cmd>
```

| Command | Meaning | Reply |
|---|---|---|
| `D` | fetch a waveform record | see below [HW] |
| `C` | cancel / reset acquisition | `a0 00 03` [HW] |
| `F` | firmware ID string | **rejected (`d0 00 03`) by the oscilloscope firmware 1.26.** It answered once, apparently from the bootloader: `a0 00 1a 72 00 04 46 49 00 13 "Uosc1.2534041.21"` (the string inside a `49` header). [HW] Use the `V*` registers instead. |

### Waveform record (`D`)

```
a0 01 0c | 72 00 04 44 | 49 01 05 | m0 m1 m2 m3 | 254 samples
reply      echoed 'D'    End-of-   metadata      8-bit unsigned
                         Body
```

* **Samples**: one byte each, count = `QS` (254 at power-up, 172 seen later). [HW]
* **The first two samples of every record are garbage** (typically ≈ 240–255, then ≈ 66). Discard them. [HW]
* **Metadata**: byte 0 echoes the `T1` edge bits (`0x20` rising, `0x10` falling) in `RT` modes 1 and 3, and is `0x00`
  in `RT` 2; values `0x10`–`0x30` also seen in free run. Byte 2 was `0x04` at fast `TS` and `0x00` at slow `TS`.
  Otherwise unknown. [HW]
* **Empty record**: `a0 00 0a 72 00 04 44 49 00 03`, i.e. a body with no data. Seen in `RT` = 1 shortly after
  arming, at fast `TS` with a floating input; it means "nothing yet, ask again". [HW]
* Transfer time per record: ~0.33 s at 9600 baud, ~0.04 s at 921600. [HW]
* **The reply comes when a fresh record has been acquired**, i.e. one record duration (`QS` × sample period) after
  the request, plus the transfer: 0.04 s at 6.4 kS/s, 0.26 s at 1 kS/s, 1.28 s at 200 S/s, 5.1 s at 50 S/s. [HW]
* **While acquiring, the probe does not process commands.** A GET `C` sent during a 5 s record got no ACK, and the
  requests after it went unanswered or came back mixed up. The only safe way to end an outstanding `D` is to wait
  for its reply. Likewise, never send while a reply is being transmitted: the probe restarts the reply. [HW]

## Registers

There are two kinds, accessed differently. Using the wrong method gives `d0 00 03`. [HW]

### Read-only constants (3-letter names): `p` read

```
TX  83 00 09 70 00 06 'M' 'C' 'l'
RX  a0 00 0e 70 00 06 'M' 'C' 'l' f0 00 00 03 20        MCl = 800
```

The reply echoes the name header, then carries the value as a separate header. [HW] (The vendor DLL also sends
these with opcode `0x83`, not GET. [RE `0x11039770`])

### Settings (2-letter names): `q` query and write

```
read:   TX  83 00 08 71 00 05 'R' 'T'
        RX  a0 00 0a 71 00 05 'R' 'T' b1 00                 RT = 0

write:  TX  83 00 0a 71 00 05 'O' '1' b1 01                 O1 := 1
        RX  a0 00 0a 71 00 05 'O' '1' b1 01                 echo of the value now in effect
```

* The `q` header's length covers **only the name**; the value follows as its own header. [RE, HW]
* Write values by width: 1 byte `B1 vv`, 2 bytes `F0 00 00 hi lo`, 4 bytes `F1 b3 b2 b1 b0`. [RE `0x1103f720`, HW]
* **The write reply echoes the value actually applied**, which can differ from the one sent (writing `MC` = 1991
  echoed 1982). [HW]
* GET (`0x82`) with a name-only `q` header just returns `a0 00 03` and reads nothing. [HW]

### Register reference

Widths come from which `PutGet{One,Two,Four}byteRegister` method the vendor app uses for each name [RE] and match
the probe's replies [HW].

#### Read-only

| Reg | Width | Meaning | Value read on S/N 3404 |
|---|---|---|---|
| `VNM` | 4 | model, 4 ASCII chars | `Uosc` [HW] |
| `VSN` | 4 | serial number, 4 ASCII chars | `3404` [HW] |
| `VHW` | 4 | hardware version, 4 ASCII chars | `1.25` [HW] |
| `VSW` | 4 | firmware version, 4 ASCII chars | `1.26` [HW] |
| `MCl` | 2 | minimum `MC` (10 ps units) | 800 = 8 ns, 125 MHz [HW] |
| `MCh` | 2 | maximum `MC` | 5425 = 54.25 ns, 18.4 MHz [HW] |
| `MCd` | 2 | default `MC` | 1248 [HW] |
| `TMl`, `TMh` | 2 | unknown. **Not `TS` limits** (the probe accepts `TS` far above `TMh`) | 25, 256 [HW] |
| `TCh` | 2 | maximum `TC` | 252 [HW] |
| `QSh` | 2 | maximum `QS` (record length); **depends on the sample rate** | 254 at `TS` = 512 (25 MS/s), 1788 at ≤ 1 MS/s [HW] |
| `TOv`, `TOl`, `TPl`, `Fbw` | | read by the vendor app; meaning unknown [RE] | |
| `P1h`, `P1l` | | listed by the vendor app; rejected by fw 1.26 [HW] | |

#### Settings

| Reg | Width | Meaning | Status | Power-up |
|---|---|---|---|---|
| `V1` | 2 | vertical scale in **mV/div** | [HW] battery test | 10000 |
| `TS` | 4 | sample period in MCU clocks × 256 | [HW] 50 Hz test | 512 |
| `MC` | 2 | MCU clock period, **10 ps units** | [HW] | 1991 |
| `QS` | 2 | record length in samples; doesn't change with `TS` (the 172 once seen was left by the vendor app) | [RE `0x4c0f38`], matches record size [HW] | 254 |
| `O1` | 1 | input options (bit map, below) | [HW] | 0 |
| `M1` | 1 | acquisition mode: Sample / Average / HiRes / Peak | [RE `0x4b5fc0`] | 0 |
| `RT` | 1 | run / trigger mode | see [Trigger](#trigger) | 0 |
| `S1` | 1 | trigger level, ADC code scale [presumed] | [RE `0x4b9580`] | 127 |
| `T1` | 1 | trigger configuration (bit map, below) | [RE `0x4b7770`] | 0x30 |
| `TC` | 2 | pre-trigger sample count, ≤ min(`TCh`, `QS`) | [RE `0x4c3040`] | — |
| `TD` | 4 | trigger delay, timer units (below) | [RE `0x4c79e0`] | 0 |
| `TA` | 4 | Auto-mode timeout, timer units | [RE `0x4c7b80`] | 65280 |
| `TW` | 4 | Wait-mode timeout, timer units | [RE `0x4c7d20`] | 65280 |
| `AP`, `AR`, `RS` | 1 | unknown (`RS` probably RIS) | | 0, 1, 0 |

**`O1`, input options** [HW] (battery and 50 Hz tests; names from the vendor app's buttons):

| Bit | Vendor button | Effect |
|---|---|---|
| 0 | **Gnd** | input grounded: a noiseless 128 regardless of input |
| 1 | **DC** (released) | **AC coupling** when set; clear = DC |
| 2 | **Filt L** | low-pass, ~3 MHz [presumed: no visible effect at ≤ 200 kS/s] |
| 3 | **Filt H** | **high-pass, ~3 kHz**: a 500 mV 50 Hz signal drops to ~5 mV; fast edges pass and decay |

**`T1`, trigger configuration** [RE]: bits 0–1 falling hysteresis, bits 2–3 rising hysteresis, bit 4 falling edge,
bit 5 rising edge (both = either edge), bits 6–7 trigger "speed" (vendor combo, e.g. "Fastest"). Read back
correctly, but see [Trigger](#trigger).

## Scaling

### Volts [HW]

```
volts = (code − 128) × (V1 / 1000) × 8 / 256
```

Code 128 is 0 V, and the 256 codes span 8 divisions. Measured with a 1.361 V battery: 0.5 V/div gave 128.2 (shorted),
215.0 (+), 42.2 (reversed); reversed at 1 V/div 85.0 and at 5 V/div 119.4. All within 1% of the formula. Noise is
about ±2–3 codes. The vendor app offers 20 mV/div to 10 V/div in 1-2-5 steps.

### Time [HW]

```
sample period = TS / 256 × MC × 10 ps
```

Confirmed with 50 Hz mains: `TS` = 2030000, `MC` = 1982 → 157.2 µs/sample, and the hum repeated every 126 samples
(50.5 Hz). `TS` is a fixed-point clock divider: 256 = one sample per clock.

**At fast rates the probe uses whole clocks**: writing `TS` = 514 applies 512, 12845 applies 12800; 128450 and above
were applied unchanged. Use the value echoed by the write. [HW]

`MC` (the clock period) **drifts by itself**: 1991 at power-up, 1982 after writing 1991, later 1993, 2000. Read it
before converting. The vendor app retunes `MC` so the sample period comes out exact:
`MC = round(Tsample × 1e11 × 256 / TS)` [RE `0x4c0e8a`]. Its `TS` choice is
`TS = max(TMl, (2·TMh) div max(2, ceil(2·TMh / (x + 1))))` with `x = 256 × Tsample / Tclock` [RE `0x4c0a30`];
on hardware a plain `round(256 × Tsample / Tclock)` works.

### Timer units (`TD`, `TA`, `TW`) [RE `0x4c7ea0`]

The vendor app stores these settings in milliseconds and converts with `value = ms × 8333.33 / MC`, i.e. **one count
per 12,000 MCU clocks** (150 µs at 80 MHz, ~240 µs at 50 MHz). The power-up 65280 is therefore ~15.6 s at 50 MHz.

## Link speed

```
TX  91 00 04 <1843200 / baud>          9600 → c0, 115200 → 10, 921600 → 02
```

* **The probe acknowledges at the new rate**, so the reply is garbage at the old one; don't read it. Switch the host,
  then verify with CONNECT. [HW]
* Don't verify with GET `F`: firmware 1.26 rejects it, which can make a working switch look failed. [HW]
* If the probe is stuck at a high rate: connect at that rate and send `91 00 04 c0`. [HW]
* The vendor app switches back to 9600 before closing. [RE `0x110372e0`]
* 921600 confirmed: ~0.04 s per record instead of ~0.33 s. [HW]

## Trigger

**On firmware 1.26 the hardware trigger does not gate or align captures.** [HW] Tested over 9600 and 921600 with
50 Hz mains pickup and a clean spiky 50 Hz signal:

* In `RT` = 0, 1, 2 and 3 alike, GET `D` returns the next record as soon as it's acquired, even with the level
  (`S1`) set where the signal never reaches, and with `TA`/`TW` at 65280 or 300.
* Signal features drift by a constant amount from one record to the next (e.g. spikes every 127 samples moving
  ~26 samples per capture), exactly as in free run, independent of `TC` (50 / 100 / 200), `S1` and the edge bits.

What the vendor app does around it [RE]:

* `RT` is written from its mode drop-down's index. The tooltips are ordered Auto, Normal, Wait (infinite), Free, and
  the screen shows buttons F / A / N / W, so the mapping is unconfirmed (the original guess 0 = Free, 1 = Auto,
  2 = Normal, 3 = Wait may be wrong).
* It has a "NO TRIGGER!" indicator and an "If no trigger occurs: clear / keep" setting, so the probe may always
  return a record and report trigger status somewhere (metadata?). The `D` handler is `0x4f7aa0` →
  `0x4ff440` / `0x55ead0`.
* Unexplained: once, at `TS` = 512 with `RT` = 1 and a floating input, `D` returned empty bodies for 20 s. Not
  reproduced at slow `TS`.

Practical conclusion: run the probe in free run and trigger in software (see [Driver.md](Driver.md)).

## Unknowns

* How the hardware trigger is meant to work, if at all, on this firmware; the `RT` value mapping.
* Metadata bytes 1–3; full meaning of byte 0.
* `TMl`, `TMh`, `TOv`, `TOl`, `TPl`, `Fbw`, `AP`, `AR`, `RS`, `D1m`, `G1`, `G2`, `Gs`, `Lt`, `L1`, `L2`.
* Writing `QS` (longer records, up to `QSh`) hasn't been tested.
* `M1` acquisition modes (Average / HiRes / Peak): not tested.
* Exact filter cut-offs; the 3 MHz low-pass is unconfirmed.
* RIS (equivalent-time sampling below one clock per sample; `TS` < 256), ROLL mode.

## Vendor software notes [RE]

* Unpack with `upx -d`; then `objdump -d -M intel` and the VB5 control table (`oscill.exe` `0x42cbb4`, 0x28 bytes per
  control, name pointer + event table) map UI controls to code. Form methods are at vtable `0x6f8 + 4 × index` into
  the method array at `0x42eee4`.
* `oscilink.dll` exposes `PutGetOnebyteRegister`, `PutGetTwobyteRegister`, `PutGetFourbyteRegister` and `SingleCMD`
  (vtable `+0x94`, `+0x98`, `+0x9c`, `+0x84` from `oscill.exe`).
* Packet builders: CONNECT `0x110378e5`, single command `0x110392c0`, register read `0x11039770`, writes
  `0x11039850` / `0x110399c0` / `0x11039b10`, value encoder `0x1103f720`, name header `0x1103f390`, speed `0x1103e1e0`.
* Other features in the app not used here: memory depth / pre-samples settings, ROLL mode, RIS, hardware offset limits,
  "Aoffs" / "Asens" (auto offset / auto sensitivity), a "Delay" mode.
