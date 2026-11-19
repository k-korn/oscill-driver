# Notes from the vendor Windows software

These notes come from static analysis of the vendor package `oscill_new` (`oscill.exe` and `oscilink.dll`
v1.04.0003, dated 2013-09-07). Both are Visual Basic 5 native-code binaries packed with UPX. Nothing was executed.
I unpacked them with `upx -d`, then used `objdump` and `radare2` to read string tables, imports and the
packet-building code. Addresses refer to the unpacked `oscilink.dll`, image base `0x11000000`.

## Packet layouts (confirmed in code)

| Request | Bytes | Where |
|---|---|---|
| CONNECT | `80 00 07 10 00 10 00` | `0x110378e5` |
| Single command | `82 00 07 72 00 04 <cmd>` (`'D'`, `'F'`, `'C'`, ...). Opcode `0x83` is also possible, chosen per command | `0x1103040a`, `0x110392c0` |
| Register read | `83 LL LL 70 00 (3+n) <name>` **(PUT opcode, not GET)** | `0x11039770` |
| Register write, 1 byte | `83 LL LL 71 00 (3+n) <name> B1 vv` | `0x11039850` |
| Register write, 2 bytes | `83 LL LL 71 00 (3+n) <name> F0 00 00 hi lo` | `0x110399c0` |
| Register write, 4 bytes | `83 LL LL 71 00 (3+n) <name> F1 b3 b2 b1 b0` | `0x11039b10` |
| Baud rate change | `91 00 04 <1843200 / baud>` (9600 → `C0`, 115200 → `10`, 921600 → `02`) | `0x1103e1e0` |
| Revert to 9600 on close | `91 00 04 C0`, sent when the link isn't already at 9600 | `0x110372e0` |
| DISCONNECT | opcode `0x81` | `0x11037a50` |
| Bluetooth module setup | opcode `0x9b` (not needed over USB) | `0x1103e2f0` |

What this changes in the original spec:

* The `p`/`q` header length covers **only the register name**. The value comes after it as a separate OBEX header,
  so the spec's example `71 00 05 'O1' B1 00` was correct (5 = 3 + 2).
* `B1`, `F0` and `F1` are not "u8 / u32 / signed" markers. They are OBEX header IDs for a 1-byte quantity, a 2-byte
  register zero-extended into a 4-byte quantity, and a 4-byte register.
* The name helper (`0x1103f390`) has a second mode, selected by a global setting (probably the "packet type" menu
  item `OBEXtype`). In that mode it uses a standard OBEX Name header (`01 LL LL ...`). The value encoder
  (`0x1103f720`) likewise has a mode that wraps raw value bytes in an End-of-Body header (`49 00 (n+3) ...`). The
  driver uses the compact default, which matches the captures in the spec.

## Register widths (from which `PutGet*Register` method `oscill.exe` passes each name to)

| Width | Registers |
|---|---|
| 1 byte | `O1 M1 T1 S1 RT AP AR RS` |
| 2 bytes | `MC V1 QS TC` |
| 4 bytes | `TS TD TA TW` |
| read only | `MCl MCh MCd TOv TOl TMl TMh TPl Fbw QSh TCh P1h P1l VNM VSN VHW VSW` |
| unclear | `D1m G1 G2 Gs Lt L1 L2` (look like calibration or settings data) |

## Other observations

* **USB adapters.** The DLL looks for Silabs `10C4:EA61` and `10C4:840E` through the USBXpress driver (`SIUSBXP`,
  `SI_Open`, `SI_SetBaudRate`, ...), and also for Silabs VCP and Prolific (`067B`) COM ports. In every case the chip
  is a UART bridge, so on Linux the probe should show up as `/dev/ttyUSB*`. The upstream `cp210x` driver claims
  `EA60` and `EA61`. For `840E` you will probably need to bind it by hand:
  `echo 10c4 840e | sudo tee /sys/bus/usb-serial/drivers/cp210x/new_id`
* `oscill.exe` uses configurable memory depth and pre-samples (`MemoDepth`, `PreSamples`, `KeepPropPresmp`), ROLL
  mode, RIS, and hardware offset limits ("Hardware offset: from … to"). The driver doesn't use these yet.
* The link layer has optional "checksum", "repeat packet" and "request repeat" modes (`CheckSymOBEX`, `RepeateOBEX`,
  `ReRequestOBEX`). I didn't decode them; the defaults appear to be off.
* Trigger mode descriptions from the UI: FREE = sample without waiting; AUTO = sample even without a trigger;
  NORM = only on trigger, short wait; WAIT = only on trigger, long or unbounded wait.

## Vertical scale, timebase and clock (from `oscill.exe`)

None of these are table indices. They are physical quantities.

* **`V1` = volts per division in millivolts.** Code at `0x4b1a52` computes `V1 = round(range × 1000 / 8)`, where
  `range` is the full-screen span in volts. The same routine stores `range / 8` as V/div and sets the display limits
  to ±range/2 (or 0 to range in one mode). The UI offers 1-2-5 steps ("10 20 50 100 200 500" mV, V).
* **`MC*` registers are in units of 10 ps.** The MCU clock period is `P = 1e-6 / MCUclock_MHz`, clamped to
  `[MCl / 1e11, MCh / 1e11]` seconds (`0x4bfc58`). So `MC` = 1250 means 12.5 ns (80 MHz), and `MCl`..`MCh` = 800..5425
  means 125..18.4 MHz. The spec's "picoseconds" is off by 10×.
* **`TS` = sample period in MCU clock cycles × 256** (8.8 fixed point), limited to `[TMl, TMh]`. These come from the
  globals `0x5cd17c` (`TMl`) and `0x5cd180` (`TMh`), filled at `0x4f8e90` and `0x4f8feb`. The real-time branch of
  `0x4c0a30` rounds it like this:

  ```
  x  = 256 × Tsample / Tclock          (Tsample = time/div ÷ samples per division)
  k  = max(2, ceil(2·TMh / (x + 1)))   (ceil with a 0.01 tolerance)
  TS = max(TMl, (2·TMh) div k)
  ```

  Typing a value into the manual entry box sends `TS = round(value × 256)` (`0x50a866`). Another path, probably
  ROLL mode, sends whole-clock values `n × 256` and uses `0x80` (half a clock) for n = 0 (`0x4c18fe`).
* There is a separate RIS (equivalent-time) branch with the same structure but a different clock argument. The
  driver doesn't use it.

## Trigger, timing and input registers (from `oscill.exe`)

The spec got most of these wrong. Control names are from the VB form's control table (`0x42cbb4`); the handlers
were found through each control's event table.

| Register | Real meaning | Evidence |
|---|---|---|
| `S1` (1 B) | **Trigger level.** Value of the `ScrollTrigCH1` slider ("Trigger level"), very likely on the ADC code scale (128 = 0 V). The auto-level code compares it with the sample centre value | writer `0x4b9580`, slider handler `0x4b80e0` |
| `T1` (1 B) | **Trigger configuration bits.** Bits 0–1 = falling hysteresis, bits 2–3 = rising hysteresis, bit 4 = falling edge (`\`), bit 5 = rising edge (`/`), both = either edge (`X`), neither = off. Bits 6–7 come from another control (probably `TrigFilter1`, "trigger speed") | `0x4b7770` |
| `TC` (2 B) | **Pre-trigger sample count** (horizontal trigger position). Value of the `HScrollCenter` scrollbar, shown as "N presamples" / percent, limited to `[0, min(TCh, QS)]` | `0x4c3040`, `0x4c28e0` |
| `QS` (2 B) | **Record length** (samples to acquire): `min(samples_needed + 12, limit)`, where the limit comes from a table with a floor of 250 | `0x4c0f38` |
| `TD` (4 B) | **Trigger delay** (setting `TrigDelay`, entered in seconds, stored in ms) | `0x4c79e0`, `0x51d17d` |
| `TA` (4 B) | **Auto-mode timeout** (setting `TimeoutAuto`, ms). Not the trigger level | `0x4c7b80` |
| `TW` (4 B) | **Wait-mode timeout** (setting `TimeoutWait`, ms) | `0x4c7d20` |
| `O1` (1 B) | **Input options bitmask.** Bit 0 = ground (`Ground1`, confirmed); bit 1 = an inverted checkbox, most likely open/closed input, i.e. DC/AC; bits 2–3 = the two input filters (`FilterL1`, `FilterH1`; which bit is which isn't confirmed). Not an offset | `0x4b4f50`, handler `0x4b4d80` |
| `M1` (1 B) | **Acquisition mode** (Sample / Average / HiRes / Peak option buttons). Not coupling | `0x4b5fc0` |

`TD`, `TA` and `TW` share one conversion (`0x4c7ea0`): `value = ms × 8333.33 / MC`, falling back to `MCd`, then 2000.
With `MC` in 10 ps units, that is **one count per 12,000 MCU clocks** (150 µs at 80 MHz), clamped to 2³¹−1. The
reverse conversion when saving is at `0x4c85af`.

After choosing `TS`, the app also sets `MC = round(Tsample × 1e11 × 256 / TS)`, tuning the clock so the sample period
comes out exact (`0x4c0e8a`).

## Still unknown
* The volts/div range the hardware accepts, and how `T1` (attenuator), `S1` and `P1h`/`P1l` (probe calibration)
  relate to `V1`.
* The layout of the 7 metadata bytes in a `D` reply, and the reply format of register reads.
