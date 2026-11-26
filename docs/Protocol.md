# Oscill DSO1 protocol

The OKB ROSA **Oscill DSO1** is a single-channel pen oscilloscope that talks OBEX-style binary packets over a USB-UART
bridge. This document is everything known about that protocol. Driver-specific choices are in [Driver.md](Driver.md).

## Sources and confidence

Each finding is tagged with where it comes from:

* **[VD]**: the vendor's protocol documents (Russian), `oscilobex.pdf` (framing) and `oscill.pdf` (properties,
  registers, commands, data format). Copies are published in the
  [yavfast/Oscill2](https://github.com/yavfast/Oscill2) repository under `docs/protocol/`; they're not reproduced here.
* **[HW]**: verified on a real probe (S/N 3404, hardware 1.25, firmware 1.26) with `tools/oscill_tool.py` and the
  driver's smoke test.
* **[RE]**: read from the vendor's Windows software (static analysis only): `oscill.exe` v1.4.7 and `oscilink.dll`
  v1.04.0003 (both 2013-09-07, Visual Basic 5 native code, UPX-packed). Addresses refer to the unpacked files
  (`oscilink.dll` image base `0x11000000`, `oscill.exe` `0x400000`).
* **[presumed]**: consistent with everything seen, not directly proven.

Where the hardware disagrees with the vendor document, this document says so and follows the hardware.

## Physical link

* **USB-UART bridge.** The tested probe enumerates as Silicon Labs CP210x `10c4:ea61` ("Oscill USB interface") and
  appears as `/dev/ttyUSB*` via the kernel `cp210x` driver. [HW] The vendor software also looks for `10c4:840e`
  (via Silabs USBXpress) and Prolific `067b` adapters. [RE] For `840e`, bind the driver by hand:
  `echo 10c4 840e | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id`.
* **8N1, no flow control. A session always starts at 9600 baud** [VD, HW]; faster rates are switched in-band
  ([Link speed](#link-speed)).
* **Keep DTR and RTS low.** Toggling them reset the probe into its bootloader ([BOOT state](#bootloader-boot-state)).
  [HW] Note for Linux:
  * opening a tty that no one else has open raises DTR and RTS;
  * setting baud rate 0 (`B0`) drops them, and leaving `B0` (setting any real rate) raises them again: the cp210x
    driver does this. libscopehal's `uart` transport sets `B0` when it opens the port (its termios has no baud bits),
    so lines must be cleared *after* the real baud rate is set;
  * clear `HUPCL` so closing the port leaves the lines alone.

## Packet framing

Requests and replies are OBEX packets [VD, HW]:

```
+------------+------------------+----------------------------+
| opcode (1) | total length (2) | headers ...                |
|            | big-endian, incl.|                            |
|            | these 3 bytes    |                            |
+------------+------------------+----------------------------+
```

The top two bits of a header ID give its encoding [VD, HW]:

| ID range | Encoding |
|---|---|
| `0x00`–`0x3F` | null-terminated Unicode text, 2-byte length (including the 3-byte header) |
| `0x40`–`0x7F` | byte sequence, 2-byte length (including the 3-byte header) |
| `0x80`–`0xBF` | 1-byte value |
| `0xC0`–`0xFF` | 4-byte big-endian value |

### Opcodes [VD]

| Opcode | Meaning | Use |
|---|---|---|
| `0x80` | CONNECT | start a session [HW] |
| `0x81` | DISCONNECT | end a session; not needed |
| `0x82` | PUT | register writes, calibrate command [HW] |
| `0x83` | GET | property / register reads, digitize command [HW] |
| `0xFF` | Abort | stop the current operation (multi-packet transfer, ROLL stream) |
| `0x91` | link speed change | [HW] |
| `0x92` | repeat the last response | for corrupted replies; not used |
| `0xA0` | reply: Success | [HW] |
| `0x90` | reply: Continue | ROLL-mode data packets |
| `0xA4`, `0xC0`, `0xD0`, `0xD1` | replies: No Content, Bad Request, Internal Server Error, Not Implemented | the probe answers unsupported requests with `d0 00 03` [HW] |

### Header IDs [VD]

| ID | Meaning |
|---|---|
| `0x70` (`p`) | property name (read-only, 3 ASCII chars: `70 00 06 ...`) |
| `0x71` (`q`) | register name (2 ASCII chars: `71 00 05 ...`) |
| `0x72` (`r`) | command (1 ASCII char: `72 00 04 ...`) |
| `0x49` | End-of-Body: carries waveform data |
| `0x48` | Body (a non-final part of a multi-packet object) |
| `0x01` | Name (Unicode): alternative way to name a register; not needed |
| `0x05` | Description: only seen in the bootloader's CONNECT reply [HW] |
| `0xB1` | 1-byte value |
| `0xF0` | 2-byte value, sent as 4 bytes whose first two are `00 00` |
| `0xF1` | 4-byte value |
| `0xB0` | optional checksum: makes the byte sum of the whole packet 0 mod 256; not used |

The probe supports headers `01 70 71 72 F0 F1 49 B1`. [VD]

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

Every other request then gets `d0 00 03`. Unplug and replug to recover. The vendor app treats this state as "upload
firmware" (command `F`); don't. [HW, RE `0x4fad60`]

### Start-up sequence

1. CONNECT at 9600 (or first at the last-used rate, in case the probe was left there).
2. Optionally switch the link speed (`0x91`), then verify with CONNECT.
3. **Calibrate: PUT `C`.** Without it, GET `D` returned a stale buffer (all 255) that ignored every setting. [VD, HW]
4. Read and write registers as needed. The vendor document recommends reading a register back after writing it, and
   re-reading the registers that depend on it. [VD]

An Abort (`ff 00 03`) before CONNECT is harmless; at idle firmware 1.26 sent no reply to it. [HW]

## Commands (`r` header)

| Command | Packet | Meaning | Reply |
|---|---|---|---|
| `C` | `82 00 07 72 00 04 43` (PUT) | self-calibration (zero, offset and trigger level scaling) | `a0 00 03` [VD, HW] |
| `D` | `83 00 07 72 00 04 44` (GET) | wait for the trigger, digitize, return the record | see below [VD, HW] |
| `F` | | **firmware load. Never send it.** | |

### Digitize (`D`)

```
a0 01 0c | 72 00 04 44 | 49 01 05 | a0 a1 | c0 c1 | n0 n1 | samples
reply      echoed 'D'    End-of-   digit.  channel  size     8-bit unsigned
                         Body      attrs   attrs    field
```

* **Digitization attributes, byte `a0`** [VD, HW]:
  * bit 0: RIS; bit 1: parallel (sent while digitizing); bit 2: endless (ROLL);
  * **bits 5–4: trigger source**: `00` timeout (Auto mode), `01` falling edge, `10` rising edge. `11` is undefined in
    the document but appeared in about half of the triggered records with a fast signal [HW];
  * bits 7–6: number of channels − 1.
* `a1`, `c1`: reserved. **`c0` bits 2–0: sample format**, `100` = one byte per sample (`M1` = 4). [VD, HW]
* **Size field** `n0 n1`: the channel's array size in bytes per the document. On firmware 1.26 it holds garbage
  (≈ 240–255, then ≈ 66), so take the sample count from the header length. [VD, HW]
* **Samples**: `QS` bytes, one per sample (in normal mode). [HW]
* **Wait-mode timeout**: `a0 00 0a 72 00 04 44 49 00 03`, an empty End-of-Body: no trigger within `TW`. [VD, HW]
* **Timing**: the reply comes after the trigger (or timeout), plus the rest of the record (`QS` × sample period),
  plus the transfer: ~0.33 s per 254-sample record at 9600 baud, ~0.04 s at 921600. [HW]
* **Digitizing can't be interrupted.** A command sent during a 5 s record got no ACK, and the requests after it went
  unanswered or came back mixed up. Sending while a reply is being transmitted restarts the reply. The only safe way
  to end an outstanding `D` is to wait for it: allow for the longest trigger wait plus the record. [VD, HW]

Other modes (RIS, parallel transfer, ROLL with `0x90` Continue packets ended by Abort) are described in the vendor
document; none of them were tested.

## Properties and registers

* **Properties** (read-only, 3-letter names): GET with a `p` header. The reply echoes the name, then the value. [VD, HW]

  ```
  TX  83 00 09 70 00 06 'M' 'C' 'l'
  RX  a0 00 0e 70 00 06 'M' 'C' 'l' f0 00 00 03 20        MCl = 800
  ```

* **Registers** (2-letter names): read with GET and a name-only `q` header; write with PUT, `q` header and a value
  header (`B1 vv`, `F0 00 00 hi lo`, `F1 b3 b2 b1 b0`, by register width). [VD, HW]

  ```
  read:   TX  83 00 08 71 00 05 'R' 'T'
          RX  a0 00 0a 71 00 05 'R' 'T' b1 00                 RT = 0
  write:  TX  82 00 0a 71 00 05 'O' '1' b1 01                 O1 := 1
          RX  a0 00 03                                         plain Success
  ```

  * The `q` header's length covers only the name; the value is its own header. [VD, HW]
  * **The probe may apply a different value than written** (`TS` 514 → 512, `MC` 1991 → 1982), so read it back. [VD, HW]
  * A GET with a value header also writes, and echoes the applied value. [HW]
  * The wrong access method gets `d0 00 03`. [HW]

### Properties

| Name | Width | Meaning | S/N 3404 |
|---|---|---|---|
| `VNM`, `VSN`, `VHW`, `VSW` | 4 | model, serial number, hardware and firmware version, 4 ASCII chars each | `Uosc`, `3404`, `1.25`, `1.26` [HW] |
| `VHD`, `VHA`, `VSD`, `VSI`, `VSC`, `VSO` | 4 | versions of the digital / analog boards and firmware parts [VD] | |
| `MCd` | 2 | default `MC` (10 ps units) | 1248 [HW] (document example: 2000) |
| `MCl`, `MCh` | 2 | minimum / maximum `MC` | 800, 5425 [HW] |
| `TOl` | 2 | fastest sample period (clocks × 256) at which any `TS` is allowed; faster only the values in `TOv` [VD] | |
| `TOv` | 4 | bit map of the allowed fast clocks-per-sample values (½, 1, 2 … 30) [VD] | |
| `TMl`, `TMh` | 2 | RIS sample period range (clocks × 256) [VD] | 25, 256 [HW] |
| `TPl` | 4 | fastest sample period for parallel / ROLL acquisition [VD] | |
| `TCh` | 2 | maximum pre-trigger samples; depends on `TS`, `RS`, `M1`, `AP` [VD] | 252 at `TS` = 512, 1787 at ≤ 1 MS/s [HW] |
| `QSh` | 2 | maximum record length; same dependencies [VD] | 254 at `TS` = 512, 1788 at ≤ 1 MS/s [HW] |
| `V1h`, `V1l` | 2 | volts/div limits (mV) [VD] | |
| `P1h`, `P1l` | 2 | offset limits, signed, 1/256 of the ADC range; depend on `V1` [VD] | |
| `D1m` | 2 | trigger comparator delay (10 ps units) [VD] | |

### Registers

| Reg | Width | Meaning | Status | Power-up |
|---|---|---|---|---|
| `MC` | 2 | machine cycle (clock period), **10 ps units**; drifts by itself (1991, 1982, 1993, 2000 seen) | [VD, HW] | 1991 |
| `TS` | 4 | sample period, **clocks × 256**; highest priority, set it first | [VD, HW] | 512 |
| `RS` | 1 | acquisition method: bit 0 RIS, bit 1 parallel transfer, bit 2 ROLL | [VD] | 0 |
| `AP` | 1 | averaging / peak passes − 1 (a power of 2) | [VD] | 0 |
| `QS` | 2 | record length in samples, ≤ `QSh` | [VD, HW] | 254 |
| `TD` | 4 | delay from trigger to the start of digitizing, **units of 12 clocks** | [VD] | 0 |
| `TC` | 2 | pre-trigger samples (trigger position in the record), ≤ `TCh` | [VD, HW] | |
| `RT` | 1 | run mode, below | [VD, HW] | 0 |
| `TA` | 4 | Auto-mode trigger timeout, **units of 12 clocks** | [VD, HW] | 65280 |
| `TW` | 4 | Wait-mode trigger timeout, **units of 12 clocks** | [VD, HW] | 65280 |
| `AR` | 1 | minimum passes per delay in RIS mode | [VD] | 1 |
| `O1` | 1 | input options, below | [VD, HW] | 0 |
| `V1` | 2 | sensitivity, **mV/div** | [VD, HW] | 10000 |
| `P1` | 2 | offset, signed, 1/256 of the ADC range, within `P1l`…`P1h` | [VD]; not used | |
| `M1` | 1 | sample processing, below | [VD, HW] | 0 |
| `T1` | 1 | trigger configuration, below | [VD, HW] | 0x30 |
| `S1` | 1 | trigger level, 1/256 of the ADC range (= ADC code) | [VD, HW] | 127 |

**`RT` run mode** [VD, HW]:

| Value | Mode | Behaviour of `D` |
|---|---|---|
| 0 | Auto | waits up to `TA` for a trigger, then digitizes anyway (source bits `00`) |
| 1 | Wait | waits up to `TW`; on timeout returns the empty End-of-Body |
| 2 | Free run | digitizes immediately (firmware ≥ 1.25) |
| 3 | Wait forever | waits until a trigger (firmware ≥ 1.25); the request can then only end with a trigger |

**`O1` input options** [VD, HW]:

| Bit | Vendor button | Meaning |
|---|---|---|
| 0 | Gnd | input grounded (reads a noiseless 128) |
| 1 | DC (released) | AC coupling when set |
| 2 | | **~3 MHz low-pass** |
| 3 | | **~3 kHz low-pass** ("up to 200 mV/div" per the document) |

Hardware tests of the filters, with a 1.348 V battery:

* Neither filter blocks DC: 1.33–1.37 V with each bit and both. Bit 2 adds about +40 mV of offset at 0.5 V/div and
  lowers the noise slightly.
* Step response at 1 MS/s, 200 mV/div: with bit 2 a step still rises within 1–2 samples; with bit 3 (alone or with
  bit 2) it ramps at ~1 code/µs, 10–90 % in 40–90 µs, i.e. a corner somewhere around 0.5–3 kHz.
* With a high-impedance source (a wire picking up mains hum), bit 3 cut 500 mV of 50 Hz down to ~5 mV: the filter's
  input capacitance divides such a source down. This is not a high-pass.

**`M1` sample processing**, bits 2–0 [VD]:

| Value | Mode | Data |
|---|---|---|
| 0 | averaging, low bits dropped (**power-up value**) | 1 byte per sample |
| 1 | averaging, higher resolution | 2 bytes per sample |
| 2 | peak, alternating min / max | 1 byte per sample |
| 3 | peak, min and max per sample | 2 bytes per sample |
| 4 | **normal: one ADC sample per sample** | 1 byte per sample [HW] |

Averaging and peak modes aren't available with RIS / parallel acquisition or at short sample periods. Only 4 was
tested.

**`T1` trigger configuration** [VD, HW]:

| Bits | Meaning |
|---|---|
| 5 | trigger on rising edge |
| 4 | trigger on falling edge (both bits: either edge, verified) |
| 3–2 | rising-edge hysteresis: `11` on, `00` off |
| 1–0 | falling-edge hysteresis: `11` on, `00` off |
| 7–6 | `00` HF + LF trigger path, `11` LF only |

## Scaling

### Volts [HW]

```
volts = (code − 128) × (V1 / 1000) × 8 / 256
```

Code 128 is 0 V and the 256 codes span 8 divisions. Measured with a 1.361 V battery: 0.5 V/div gave 128.2 (shorted),
215.0 (+), 42.2 (reversed); reversed at 1 V/div 85.0 and at 5 V/div 119.4. All within 1 % of the formula. Noise is
about ±2–3 codes. The vendor app offers 20 mV/div to 10 V/div in 1-2-5 steps.

The vendor document instead puts 240 levels on the 8 divisions ("8.53 mV per ADC range unit"), which would be a
6.7 % larger scale; the battery measurements don't support that.

### Time [VD, HW]

```
sample period = TS / 256 × MC × 10 ps
```

Confirmed with 50 Hz mains: `TS` = 2030000, `MC` = 1982 → 157.2 µs/sample, and the hum repeated every 126 samples.

* **At fast rates only whole or listed clock counts are allowed** (`TOl` / `TOv`): writing `TS` = 514 applied 512,
  12845 applied 12800; 128450 and above were applied unchanged. Read `TS` back. [VD, HW]
* **`MC` drifts**: read it before converting. The vendor app retunes `MC` so the sample period comes out exact:
  `MC = round(Tsample × 1e11 × 256 / TS)` [RE `0x4c0e8a`]. Plain `TS = round(256 × Tsample / Tclock)` with the probe's
  `MC` works too.

### Timer units (`TD`, `TA`, `TW`) [VD, HW]

**One count = 12 machine cycles** (≈ 240 ns at 50 MHz). `TW` = 4166667 timed out after 1.00 s; the power-up 65280 is
~15.7 ms.

### Trigger position [HW]

With `RT` = 0 or 1 and a hardware trigger, the trigger crossing lands just before sample `TC` of the record (counting
the samples after the size field): the first sample past the level was 0–4 samples before `TC`, 2 on average (rising
1.4, falling 2.6), over 48 captures at `TC` 50, 127 and 200. The interpolated crossing is about 2.5 samples before
`TC`. The trigger fires on any crossing of `S1`, including narrow spikes.

## Link speed

```
TX  91 00 04 <divisor>        rate = 1843200 / divisor: 9600 → c0, 115200 → 10, 921600 → 02
```

* The document writes 1842000; 1843200 is what works. [VD, HW]
* The probe acknowledges **at the old rate** and then switches; the driver discards the acknowledgement, switches the
  host and verifies with CONNECT. [HW]
* If the probe is stuck at a high rate: connect at that rate and send `91 00 04 c0`. The vendor app switches back to
  9600 before closing. [HW, RE `0x110372e0`]

## Error recovery [VD]

* A corrupted request gets `0xD0`; repeat it once.
* For a corrupted or truncated response, request a repeat with `92 00 03`.
* With no response at all, repeat the request after a guard time that covers the longest possible digitization.

## Unknowns

* Metadata `11` trigger source; the reserved attribute bytes.
* `QS` above the power-up 254 (up to `QSh`): accepted by the probe, not used by the driver.
* `M1` averaging / peak modes, `AP`, RIS, parallel transfer, ROLL, `TD` delay, `P1` offset: untested.
* The exact filter corners.
* Why a freshly opened port sometimes got no answer at all (seen in ngscopeclient only, before DTR/RTS were really
  held low; see Driver.md).

## Vendor software notes [RE]

* Unpack with `upx -d`; then `objdump -d -M intel` and the VB5 control table (`oscill.exe` `0x42cbb4`, 0x28 bytes per
  control, name pointer + event table) map UI controls to code. Form methods are at vtable `0x6f8 + 4 × index` into
  the method array at `0x42eee4`.
* `oscilink.dll` exposes `PutGetOnebyteRegister`, `PutGetTwobyteRegister`, `PutGetFourbyteRegister` and `SingleCMD`
  (vtable `+0x94`, `+0x98`, `+0x9c`, `+0x84` from `oscill.exe`).
* Packet builders: CONNECT `0x110378e5`, single command `0x110392c0`, register read `0x11039770`, writes
  `0x11039850` / `0x110399c0` / `0x11039b10`, value encoder `0x1103f720`, name header `0x1103f390`, speed `0x1103e1e0`.
* The app's buttons: Peak, DC, Avg, Gnd, HiRes, Filt H, Filt L. Other features not used here: memory depth /
  pre-samples, ROLL, RIS, auto offset / sensitivity, delay mode.
