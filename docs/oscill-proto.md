# Oscill USB/Bluetooth Digital Oscilloscope Protocol Specification

**Document Version:** 1.0  
**Target Firmware:** Oscill DSO1 (Firmware v1.2x - v1.4x, tested on v1.25 / S/N 3404)  
**Host Binaries Referenced:** `oscill.exe` (v1.4.7), `oscilink.dll` (v1.4.3)  
**Author/Reverse-Engineered by:** Antigravity AI & User Session

---

## 1. Overview & Physical Hardware Architecture

The **Oscill DSO1** is a compact pen-style USB/Bluetooth digital storage oscilloscope probe manufactured by OKB ROSA.

### Hardware Details
- **USB Interface:** Silicon Labs CP2102/CP2103 USB-to-UART Bridge (`VID: 0x10c4`, `PID: 0xea60` or `0xea61`).
- **MCU / Core:** High-speed microcontroller with internal analog front-end (AFE), hardware comparator for triggering, and programmable clock synthesizer (PLL).
- **Analog Input:** Single channel (CH1), DC/AC coupled, hardware selectable attenuators and low-pass filters (~3 kHz and ~3 MHz).
- **Communication Topology:** Bidirectional serial stream encapsulated inside an IrDA-like **OBEX (Object Exchange)** binary packet protocol.

---

## 2. Physical & Transport Layer

### 2.1 Serial Line Settings
- **Data Bits:** 8
- **Parity:** None (N)
- **Stop Bits:** 1
- **Flow Control:** None (RTS/CTS can be tracked, but transmission is request-response packetized).
- **Initial Sync Baud Rate:** **9600 baud** (`Start on 9600`).
- **Supported Operating Baud Rates:**
  - Standard COM: `9600`, `19200`, `38400`, `57600`, `115200`, `230400`, `460800`, `921600` baud.
  - Bluetooth: Typically negotiated via RFCOMM over Bluetooth modules (e.g., LMX or HC-0x).
  - PCMCIA 8x high-speed interface mode.

### 2.2 Connection Handshake Procedure
1. Host opens the serial port at **9600 baud**.
2. Host issues an **OBEX CONNECT** packet (`0x80 0x00 0x07 0x10 0x00 0x10 0x00`).
3. Probe responds with an **OBEX SUCCESS** packet (`0xA0 0x00 0x07 0x10 0x00 0x00 0x18`).
4. If a higher operating baud rate is desired, host issues a baud change command and switches host UART baud rate.
5. Host queries device identity registers (`VNM`, `VSN`, `VHW`, `VSW`) and hardware clock limits (`MCl`, `MCh`, `MCd`).
6. Host configures AFE, trigger, and timebase parameters before requesting waveform data.

---

## 3. OBEX Framing & Encapsulation Layer

Packets follow binary IrDA OBEX encapsulation with 16-bit big-endian length prefixes.

### 3.1 Packet Structure
```
+--------------+------------------+------------------------------------+
| Opcode (1 B) | Total Length (2B)| Headers / Payload Data (Variable)  |
| 0x80 / 0x82..| Big-Endian 16-bit| OBEX Header TLVs                   |
+--------------+------------------+------------------------------------+
```

### 3.2 Opcode Definitions
| Opcode (Hex) | OBEX Name | Function in Oscill Protocol |
|--------------|-----------|-----------------------------|
| `0x80`       | `CONNECT` | Link initialization (Final bit set) |
| `0xA0`       | `SUCCESS` | Success / Acknowledgement from probe |
| `0x82`       | `GET`     | Fetch data / trigger single request (Final bit set) |
| `0x02`       | `GET_CONT`| Multi-packet GET continuation |
| `0x83`       | `PUT`     | Write / Set register or parameter |
| `0x92`       | `ABORT`   | Terminate ongoing multi-packet transfer |
| `0xD0`       | `ERROR`   | Command error / invalid parameter |

---

## 4. OBEX Header Types & Data Encoding

Headers follow standard OBEX Type-Length-Value encoding where the top 2 bits specify the data representation:
- `0x00`..`0x3F`: Null-terminated Unicode/ASCII strings with 16-bit length prefix.
- `0x40`..`0x7F`: Byte sequences / binary buffers with 16-bit length prefix.
- `0x80`..`0xBF`: 1-byte integer values.
- `0xC0`..`0xFF`: 4-byte big-endian integer values.

### 4.1 Oscill-Specific Header Identifiers
| Header ID (Hex) | ASCII Char | Meaning |
|-----------------|------------|---------|
| `0x70`          | `'p'`      | Register Query (Read property request) |
| `0x71`          | `'q'`      | Register Write / Set property |
| `0x72`          | `'r'`      | Core Command Selector (`'D'`, `'F'`, `'C'`) |

### 4.2 Payload Type Markers
When reading or writing registers in header `'p'` or `'q'`:
- `\xB1` (`0xB1`): 1-byte unsigned integer (`0..255`).
  - Example: `\x71\x00\x05 O1 \xB1 \x00` (Set Offset 1 to 0).
- `\xF0` (`0xF0`): 4-byte big-endian unsigned integer.
  - Example: `\x71\x00\x05 MC \xF0 \x00\x00\x04\xE2` (Set Master Clock to 1250 ps).
- `\xF1` (`0xF1`): 4-byte big-endian signed/offset integer (used for trigger delay/thresholds).

---

## 5. Core Operational Commands

### 5.1 Link Connect
- **Host Send:** `\x80\x00\x07\x10\x00\x10\x00`
  - `0x80`: CONNECT opcode
  - `0x00\x07`: 7 bytes total length
  - `0x10`: OBEX version 1.0
  - `0x00`: Flags
  - `0x10\x00`: Maximum packet receive buffer size (4096 bytes)
- **Probe Response:** `\xA0\x00\x07\x10\x00\x00\x18`
  - `0xA0`: SUCCESS
  - `0x00\x07`: 7 bytes
  - `0x10\x00\x00\x18`: Probe max packet buffer size negotiation

### 5.2 Waveform Data Acquisition: `GET 'D'`
- **Host Send:** `\x82\x00\x07\x72\x00\x04\x44`
  - `0x82`: OBEX GET (Final)
  - `0x00\x07`: 7 bytes length
  - `\x72\x00\x04\x44`: Header `0x72` ('r'), Length 4, Value `0x44` ('D')
- **Probe Response (Waveform Payload):**
  - Length: Typically **358 bytes**
  - Header: `\xA0\x01\x66\x72\x00\x04\x44` (Opcode 0xA0, Length 358, Header 'D')
  - Followed by sample metadata (trigger point, actual timebase index)
  - Followed by **344 raw 8-bit ADC samples** (`0x00..0xFF`).

### 5.3 Firmware & Device ID Query: `GET 'F'`
- **Host Send:** `\x82\x00\x07\x72\x00\x04\x46` ('F')
- **Probe Response:** ASCII firmware string
  - Example: `Uosc1.25340 41.21` (Model: Oscill, Firmware: v1.25, Hardware SN: 3404, Cal date: 41.21).

### 5.4 Sweep Interrupt / Clear Wait: `GET 'C'`
- **Host Send:** `\x82\x00\x07\x72\x00\x04\x43` ('C')
- **Probe Response:** `\xA0\x00\x03` (Empty success / Cancel ACK).
  - *Note:* Used when waiting for an infinite trigger (`FreeAutoNorm.3` / `'W'`) to cancel sweep without receiving sample data.

---

## 6. Register Dictionary

### 6.1 Hardware Identification & Versioning
| Register | Access | Type | Description |
|----------|--------|------|-------------|
| `VNM`    | R      | Str  | Vendor Name (e.g., "OKB ROSA") |
| `VSN`    | R      | Str  | Device Serial Number |
| `VHW`    | R      | Str  | Hardware PCB Revision |
| `VSW`    | R      | Str  | Software / Firmware Build Version |

### 6.2 Master Clock & Synthesizer (MCU PLL)
All period values are in **picoseconds ($10^{-12}$ s)**.
| Register | Access | Type | Example Value | Description |
|----------|--------|------|---------------|-------------|
| `MCl`    | R      | 4B   | 800 ps        | Minimum supported clock period |
| `MCh`    | R      | 4B   | 5425 ps       | Maximum supported clock period |
| `MCd`    | R      | 4B   | 1248 ps       | Recommended default clock period |
| `MC`     | R/W    | 4B   | 1250 ps       | Target master clock period |

### 6.3 Analog Front-End (Channel 1)
| Register | Access | Type | Values | Description |
|----------|--------|------|--------|-------------|
| `O1`     | R/W    | 1B   | `0..255` | DC Offset zero adjustment DAC |
| `M1`     | R/W    | 1B   | Bitmask | Input Coupling / Mode (`0`=DC, `1`=AC, `2`=GND) |
| `T1`     | R/W    | 1B   | Index  | Hardware Attenuator Selection |
| `V1`     | R/W    | 4B   | Index  | Vertical Volts/Division setting |
| `P1h`    | R/W    | 4B   | Value  | Probe calibration scale factor high |
| `P1l`    | R/W    | 4B   | Value  | Probe calibration scale factor low |
| `S1`     | R/W    | 1B   | Scale  | Sensitivity multiplier |

### 6.4 Trigger & Acquisition Control
| Register | Access | Type | Description |
|----------|--------|------|-------------|
| `RT`     | R/W    | 1B   | Run / Trigger Mode (`0`=Free, `1`=Auto, `2`=Norm, `3`=Wait) |
| `AP`     | R/W    | 1B   | Auto-trigger timeout parameter |
| `AR`     | R/W    | 1B   | Auto-rearm register |
| `TA`     | R/W    | 4B   | Trigger Level threshold |
| `TW`     | R/W    | 4B   | Trigger Hysteresis window width |
| `TD`     | R/W    | 4B   | Trigger Delay offset |
| `TC`     | R/W    | 4B   | Trigger Condition control flags (Slope rise/fall, source) |
| `TS`     | R/W    | 4B   | Timebase (Time/Division) |
| `QS`     | R/W    | 4B   | Quantization / Sampling decimation setting |
| `RS`     | R/W    | 1B   | RIS (Random Interleaved Sampling) strobe mode |
| `TOv`    | R      | 4B   | Communication / sweep timeout value |

---

## 7. Waveform Sample Decoding

When `GET 'D'` returns 358 bytes:
1. **Byte 0..6:** OBEX response header `\xA0\x01\x66\x72\x00\x04\x44`
2. **Byte 7..13:** Acquisition metadata (Sampling frequency index, trigger offset position).
3. **Byte 14..357:** **344 ADC Samples** (8-bit unsigned values, `0` to `255`).
   - `128` represents zero volts (with calibrated offset `O1`).
   - Peak-to-peak voltage mapping:
     $$V_{\text{actual}} = (V_{\text{sample}} - V_{\text{zero\_offset}}) \times \text{ScaleFactor}(V1)$$

---

## 8. Software Architecture & Known Gotchas

### 8.1 Software Modules
- `oscill.exe`: Visual Basic 5 Native GUI application.
- `oscilink.dll`: ActiveX COM component (`_ClassOBEX`) handling background serial I/O, multi-threading timers, and OBEX packet parsing.
- Inter-process COM events:
  - `StartSessionEvent`: Probe successfully initialized, sweep can start.
  - `HeaderEvent`: Register or data packet received from probe.
  - `EndSessionEvent`: Link closed or probe disconnected.
  - `NoResponseEvent`: Request timeout / packet drop.
