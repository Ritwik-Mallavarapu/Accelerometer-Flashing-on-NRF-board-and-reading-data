# Tyre Vibration Sensor — nRF52810 + MMA845X

A wireless accelerometer for measuring vibration on a rotating wheel or tyre.

An MMA845X accelerometer is sampled at 200 Hz by an nRF52810 and streamed over
Bluetooth LE to a laptop, where a Python script decodes the packets and writes
them to a timestamped CSV file.

---

## Contents

- [System overview](#system-overview)
- [Hardware required](#hardware-required)
- [Wiring](#wiring)
- [Software setup](#software-setup)
- [Building and flashing the firmware](#building-and-flashing-the-firmware)
- [Recording data](#recording-data)
- [File reference](#file-reference)
- [BLE protocol](#ble-protocol)
- [Data format](#data-format)
- [Diagnostics](#diagnostics)
- [Operating notes](#operating-notes)

---

## System overview

```
   MMA845X                 nRF52810                  Laptop
  accelerometer              module                      |
       |                       |                        |
       | I2C (SDA/SCL)         | BLE                    | Python
       +---------------------->+----------------------->+---> CSV
```

The nRF52810 performs both tasks: it reads the accelerometer over I2C and
transmits the measurements over Bluetooth LE. No separate microcontroller is
needed.

```
   Development (flashing)                    Operation (measurement)

        Laptop                                    Laptop
          |                                          ^
          | USB                                      | BLE
          v                                          |
      J-Link EDU                                 nRF52810
          |                                          |
          | SWD                                      | I2C
          v                                          v
      nRF52810  <-- USB Type-C power             MMA845X
          |
          | I2C
          v
      MMA845X
```

The J-Link is used only to program the chip. It is not part of the rotating
assembly and is disconnected during operation.

---

## Hardware required

| Item | Specification |
|---|---|
| Accelerometer | MMA845X — used here on a GY-45 breakout module |
| Controller | IndieSemiC **EVK-ISC-nRF52810-A** (Nordic nRF52810 SoC) |
| Programmer | **SEGGER J-Link EDU** |
| Power during development | USB Type-C supply to the nRF board |

### Sensor settings used

| Parameter | Value |
|---|---|
| I2C address | `0x1C` |
| `WHO_AM_I` register | `0x0D`, expected `0x2A` |
| Measurement range | ±8 g |
| Sensitivity | 256 counts per g |
| Output data rate | 200 Hz |
| Resolution | 12-bit |

### nRF52810

| Parameter | Value |
|---|---|
| Core | ARM Cortex-M4, 64 MHz |
| Flash | 192 KB |
| RAM | 24 KB |
| Radio | 2.4 GHz (Bluetooth LE) |

---

## Wiring

### 1. J-Link EDU to the nRF52810 board

The J-Link EDU uses a 20-pin JTAG/SWD header. This project programs over
**SWD**, which is a two-wire protocol that reuses the JTAG pins — so the
signals are named `TMS` and `TCK` on the header.

| J-Link 20-pin | Header label | Signal | nRF board pin |
|---|---|---|---|
| 1 | VTref | Voltage reference | 3V3 |
| 7 | TMS | **SWDIO** | SWDIO |
| 9 | TCK | **SWDCLK** | SWDCLK |
| 15 | RESET | Reset (optional) | RESET |
| 4 | GND | Ground | GND |

```
     J-Link EDU 20-pin              nRF52810 board
     ──────────────────             ──────────────
      1   VTref  ─────────────────  3V3
      7   TMS    ─────────────────  SWDIO
      9   TCK    ─────────────────  SWDCLK
     15   RESET  ─────────────────  RESET
      4   GND    ─────────────────  GND
```

**Notes**

- **VTref is a voltage reference, not a power supply.** It lets the J-Link
  sense the board's logic level. The board must be powered separately over USB
  Type-C.
- Any of the GND pins (4, 6, 8, 10, 12) can be used.
- Do not connect pin 19 (5V-Supply).
- Connect the J-Link to the laptop with its own USB cable.

### 2. MMA845X (GY-45) to the nRF52810 board

| GY-45 pin | nRF board pin |
|---|---|
| VCC-3.3V | 3V3 |
| GND | GND |
| SDA | **P0.10** |
| SCL | **P0.12** |

```
     GY-45 MMA845X                  nRF52810 board
     ──────────────                 ──────────────
      VCC-3.3V  ──────────────────  3V3
      GND       ──────────────────  GND
      SDA       ──────────────────  P0.10
      SCL       ──────────────────  P0.12
      INT1, INT2                    (not connected)
```

**Notes**

- Power the module from its **VCC-3.3V** pin. The module has an on-board
  regulator, and this is the input to it.
- Use **3.3 V only**. Both the sensor and the nRF52810 operate in the 3.3 V
  domain, so no level shifting is required.
- The GY-45 module already includes I2C pull-up resistors. **Do not add
  external pull-ups.**
- Keep the SDA and SCL leads short — long wires cause I2C errors.
- The GY-45 pin header lists the pins as `INT1, INT2, SDA, SCL, GND, 3.3v,
  VCC-3.3V`. Count carefully when connecting — putting power onto an SDA line
  can damage the module.
- Solder the pin header to the module before wiring.

### Complete connection list

| From | To | Purpose |
|---|---|---|
| J-Link 1 (VTref) | nRF 3V3 | Logic level reference |
| J-Link 7 (TMS) | nRF SWDIO | Programming data |
| J-Link 9 (TCK) | nRF SWDCLK | Programming clock |
| J-Link 15 (RESET) | nRF RESET | Reset |
| J-Link 4 (GND) | nRF GND | Common ground |
| GY-45 VCC-3.3V | nRF 3V3 | Sensor power |
| GY-45 GND | nRF GND | Common ground |
| GY-45 SDA | nRF **P0.10** | I2C data |
| GY-45 SCL | nRF **P0.12** | I2C clock |

---

## Software setup

### Firmware toolchain (one-time)

Nordic **nRF Connect SDK v3.3.4** with its matching toolchain, managed by
`nrfutil`. No IDE is required — everything is command line.

1. Download the **Windows version of nRF Util** from Nordic's website and place
   the executable in `C:\nrf-tools\`.

2. Install the SDK manager and the SDK:

```bat
C:\nrf-tools\nrfutil.exe install sdk-manager
C:\nrf-tools\nrfutil.exe sdk-manager install v3.3.4
```

### Laptop (one-time)

- Python 3.10 or newer
- The Bluetooth LE library:

```bat
pip install bleak
```

---

## Building and flashing the firmware

### Step 1 — Open the toolchain environment

In a Command Prompt:

```bat
C:\nrf-tools\nrfutil.exe sdk-manager toolchain launch --ncs-version v3.3.4 --terminal
```

A new window opens with the toolchain active. All build commands go in that
window.

### Step 2 — Build

```bat
cd /d C:\ncs\projects\tyre_sensor
set ZEPHYR_BASE=C:\ncs\v3.3.4\zephyr
west build -b nrf52dk/nrf52810 --build-dir build
```

The board target `nrf52dk/nrf52810` selects the nRF52810 variant of the nRF52
DK definition, which matches this chip. This compile step creates the `build`
folder.

Expect a summary at the end:

```
Memory region         Used Size  Region Size  %age Used
           FLASH:      108664 B       192 KB     55.27%
             RAM:       20544 B        24 KB     83.59%
```

### Step 3 — Flash

With the J-Link connected and the board powered:

```bat
west flash
```

Expected output:

```
-- runners.nrfutil: Board(s) with serial number(s) ... flashed successfully.
```

### Step 4 — Disconnect the programmer

**Unplug the J-Link's USB cable from the laptop.**

An attached, powered J-Link holds the nRF52810 halted over SWD. In that state
the firmware does not run and the device does not advertise.

### Step 5 — Power-cycle the board

Unplug the USB Type-C cable, wait a few seconds, plug it back in. The board now
runs standalone and begins advertising.

---

## Recording data

With the board powered over USB Type-C and the J-Link **unplugged**, open a
Command Prompt:

```bat
cd /d C:\ncs\projects\tyre_sensor\laptop
python receive_v10.py --seconds 20
```

The script scans for the device named **`TYRE_SENSOR`**, connects, subscribes
to the data characteristic and records for the requested duration.

### Options

| Option | Default | Purpose |
|---|---|---|
| `--seconds` | `20` | Recording duration |
| `--out` | `tyre_data` | Base name for the output file |
| `--dir` | `data` | Folder for recordings |

Examples:

```bat
python receive_v10.py --seconds 60
python receive_v10.py --seconds 30 --out bench_test
python receive_v10.py --seconds 30 --dir C:\tyre_logs
```

### Output

Every run creates a **new, timestamped file**, so nothing is overwritten:

```
data/tyre_data_2026-09-23_18-42-07.csv
data/tyre_data_2026-09-23_18-42-07_REJECTED.csv    (only if needed)
```

The `_REJECTED` file appears only when a value falls outside the ±8 g range,
and keeps such values separate from the main dataset.

### Example run

```
Scanning for 'TYRE_SENSOR' ...
Found C8:96:74:9C:70:96
Connected.
Device report: V10 who=2A err=0 i2c=1 rd=1 cptr=1 pkt=199 smp=3980 avg=398 nfail=0 nrc=00
Recording 20 s ...
Disconnected.

Saved 398 values -> data/tyre_data_2026-09-23_18-42-07.csv
Packets          : 199
Packet lengths   : {16: 199}
Packet gaps      : 0
Duration         : 19.90 s
Reported rate    : 20.0 Hz  (target 20)

Range over the recording:
  X   +0.02 .. +0.06 g
  Y   -0.03 .. +0.03 g
  Z   +0.97 .. +1.02 g
  |A| 0.98 .. 1.01 g  (mean 1.00 g)

All values within the physical +/-8 g range. Decode verified.
```

---

## File reference

| File | Purpose |
|---|---|
| `CMakeLists.txt` | Build definition for the firmware |
| `prj.conf` | Firmware configuration: Bluetooth, I2C, memory settings |
| `boards/nrf52dk_nrf52810.overlay` | Sets the I2C pins to SDA = P0.10, SCL = P0.12 |
| `src/main.c` | Firmware: sensor driver, sampling, averaging, BLE service |
| `laptop/receive_v10.py` | Records data over BLE and writes a timestamped CSV |
| `laptop/read_diag.py` | Reads the device's diagnostic status over BLE |
| `laptop/data/` | Recordings, created automatically |

### About the `build` folder

`build/` is created by `west build`. It contains compiled object files, the
linked `zephyr.elf` and the `merged.hex` image that gets flashed. It is
generated output and can be deleted at any time — rebuilding recreates it. It
is normally excluded from version control.

---

## BLE protocol

### Service and characteristics

| | UUID |
|---|---|
| Service | `6b1f0001-7a3c-4d2e-9f10-8a5c1e200001` |
| Data (notify) | `6b1f0002-7a3c-4d2e-9f10-8a5c1e200002` |
| Diagnostics (read) | `6b1f0003-7a3c-4d2e-9f10-8a5c1e200003` |

Device name: **`TYRE_SENSOR`**

### Packet layout

All fields are little-endian. One notification carries one packet.

| Offset | Size | Type | Field |
|---|---|---|---|
| 0 | 2 | uint16 | Sequence number (`0` marks the handshake packet) |
| 2 | 2 | uint16 | Number of values in this packet |
| 4 | 6 × N | int16 × 3 | Averaged X, Y, Z |

With 2 values per packet, a notification is **16 bytes**.

### Timing

| Parameter | Value |
|---|---|
| Sensor sampling | 200 Hz (one raw sample every 5 ms) |
| Averaging | 10 raw samples → 1 value |
| Reported rate | 20 Hz |
| Values per packet | 2 |
| Packet rate | 10 per second |
| Packet size | 16 bytes |

---

## Data format

Each CSV row:

```csv
time_s,seq,n,x_raw,y_raw,z_raw,x_g,y_g,z_g
0.000000,21,0,251,0,254,0.9805,0.0000,0.9922
```

| Column | Meaning |
|---|---|
| `time_s` | Seconds since the first packet arrived |
| `seq` | Packet sequence number — gaps indicate dropped packets |
| `n` | Value index within the packet (0 or 1) |
| `x_raw`, `y_raw`, `z_raw` | Signed 12-bit counts, range ±2048 at ±8 g |
| `x_g`, `y_g`, `z_g` | Acceleration in g |

### Raw counts to g

At ±8 g the sensitivity is **256 counts per g**:

```
acceleration_g = raw_counts / 256
```

Values are already sign-corrected, so negative accelerations appear as
negative numbers.

### Verifying the sensor is reading correctly

Place the board **flat and still**:

- One axis reads approximately **±1.0 g**
- The other two read near **0 g**
- The magnitude `sqrt(x² + y² + z²)` is approximately **1.0 g**

Tilt the board: the axis pointing downward should approach **+1.0 g** and the
axis pointing upward should approach **−1.0 g**.

---

## Diagnostics

The diagnostics characteristic can be read over Bluetooth at any time, without
attaching a programmer:

```bat
cd /d C:\ncs\projects\tyre_sensor\laptop
python read_diag.py
```

It returns a single status line:

```
V10 who=2A err=0 i2c=1 rd=1 cptr=1 pkt=199 smp=3980 avg=398 nfail=0 nrc=00
```

| Field | Meaning | Expected |
|---|---|---|
| `who` | `WHO_AM_I` value read from the sensor | `2A` |
| `err` | Last error code | `0` |
| `i2c` | I2C driver ready | `1` |
| `rd` | Last sensor read succeeded | `1` |
| `cptr` | Connection object held | `1` while connected |
| `pkt` | Notifications successfully sent | Rising |
| `smp` | Raw samples captured since boot | Rising |
| `avg` | Averaged values produced | Roughly `smp / 10` |
| `nfail` | Notification calls that failed | `0` |
| `nrc` | Last `bt_gatt_notify` return code, hex | `00` |

### Error codes

| Code | Meaning |
|---|---|
| 0 | No error |
| 1 | I2C device not ready |
| 2 | `WHO_AM_I` read failed — check wiring and power |
| 3 | `WHO_AM_I` returned an unexpected value |
| 4 | Standby write failed |
| 5 | Range register write failed |
| 6 | Output data rate write failed |
| 7 | Sensor activation failed |
| 8 | X/Y/Z register read failed |

---

## Operating notes

### Averaging limits the bandwidth

Averaging 10 raw samples at 200 Hz acts as a low-pass filter: signal content
above roughly **10 Hz is attenuated**. This is a deliberate trade to keep the
Bluetooth link clean on this chip.

Surface comparisons, peak values and RMS analysis are unaffected. Frequency
analysis targeting higher frequencies is not supported by this configuration.

### Sampling rate versus reported rate

The sensor genuinely samples at **200 Hz**. The reported rate is **20 Hz**
because of averaging. The CSV contains 20 values per second, each the mean of
10 raw readings.

### Timestamps are reconstructed on the laptop

The firmware does not timestamp individual samples. The receiver spaces each
packet's values at 50 ms intervals, which gives correct relative spacing and a
correct average rate. This is suitable for peak, RMS and waveform inspection.

### Measurement range

The sensor measures up to **±8 g** per axis. Readings beyond this clip and
cannot be recovered afterwards.

A sensor at radius `r` rotating with angular velocity `ω` experiences
centripetal acceleration:

```
a_c = ω² r
```

Calculate this for your mounting radius and maximum wheel speed before running
a rotating test, and confirm it stays within range.

### Mounting

The J-Link and laptop are not part of the rotating assembly. The assembly
mounted on the wheel is:

```
      MMA845X
          |
        I2C
          |
      nRF52810
          |
        power
          |
       Battery
          |
          |  BLE
          v
       Laptop
```

For rotating tests the assembly must be rigidly secured and balanced. Do not
spin a wheel with loose boards, jumper wires, breadboards or an unsecured
battery.

### Memory

A Zephyr Bluetooth build on the nRF52810 sits close to the 24 KB RAM limit.
Current usage is approximately **84% RAM** and **55% flash**. Logging is
disabled in `prj.conf` to free memory, which is why the device reports its
status through the diagnostics characteristic instead of a console log.

### After flashing

An attached, powered J-Link holds the nRF52810 halted over SWD, so the firmware
does not run and the device will not advertise. Unplug the J-Link's USB cable
before testing, and reconnect it only when you need to flash again.

---

## Project status

| Component | Status |
|---|---|
| Toolchain, build, flash | Working |
| I2C connection to MMA845X on P0.10 / P0.12 | Working — `WHO_AM_I` = `0x2A` |
| Bluetooth LE link | Working — 16-byte packets, no gaps, ~20 Hz |
| Accelerometer decoding | Working — verified against recorded data |
| Laptop receiver and CSV output | Working |
| Live plotting | Not yet implemented |
| Rotating test | Not yet performed |

### Planned

- Bench validation across all six orientations
- Analysis tooling: magnitude, RMS, peak, peak-to-peak, spectrum
- Live plotting of the accelerometer axes during capture

### Note on TPMS

A tyre pressure monitoring system (TPMS) built into a wheel transmits on
**315 MHz or 433 MHz**, which is outside the 2.4 GHz radio range of the
nRF52810. Capturing TPMS data alongside the accelerometer therefore requires a
separate sub-GHz receiver on the laptop side.

---

## Licence and acknowledgements

The firmware is built on the **nRF Connect SDK** (Nordic Semiconductor) and the
**Zephyr RTOS**. The Bluetooth service structure follows the patterns used in
Nordic's own sample applications, which are Apache-2.0 licensed.

The MMA845X is an NXP part; its datasheet is the reference for register
behaviour and data decoding.
