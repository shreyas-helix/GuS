# Gesture Unlock

Embedded gesture-based authentication system for the **STM32F429I-Discovery** board.

Wave or rotate the onboard gyroscope to enroll a secret gesture, then repeat it later to unlock the device. The LCD touchscreen drives setup and unlock flows; LEDs provide immediate pass/fail feedback.

## Hardware

| Component | Details |
|-----------|---------|
| **MCU / board** | STM32F429ZI on [DISCO_F429ZI](https://www.st.com/en/evaluation-tools/32f429idiscovery.html) |
| **Gyroscope** | L3GD20 (SPI), data-ready on `PA_2` (INT2) |
| **Display** | ILI9341 240×320 LCD |
| **Touchscreen** | STMPE811 capacitive touch controller |
| **User button** | Blue button on `PC_13` — erases stored gesture data |
| **LEDs** | Green (`LED1`) and red (`LED2`) status indicators |

## How It Works

### Architecture

The firmware runs on **Mbed OS** with three concurrent execution contexts:

1. **Main thread** — initializes the LCD, draws the UI, and starts worker threads.
2. **Gyroscope thread** — handles gesture recording, comparison, and erase logic.
3. **Touchscreen thread** — polls touch input and sets event flags for Setup / Unlock.

Threads synchronize through Mbed `EventFlags` (`KEY_FLAG`, `UNLOCK_FLAG`, `ERASE_FLAG`, `DATA_READY_FLAG`).

### Gesture pipeline

1. **Calibration** — 128 samples establish per-axis zero-rate offsets and vibration thresholds.
2. **Recording** — gyro data is sampled at ~20 Hz for **5 seconds** after a 3-second countdown.
3. **Trimming** — leading and trailing near-zero samples are removed (`trim_gyro_data`).
4. **Matching** — Pearson correlation is computed independently on the X, Y, and Z axes between the stored key and the unlock attempt (`calculateCorrelationVectors`).
5. **Decision** — unlock succeeds when all three axis correlations exceed `CORRELATION_THRESHOLD` (default `0.005`).

A Dynamic Time Warping (`dtw`) implementation is also present for sequence-distance measurement but is not used in the current unlock path.

### Data persistence

Gesture keys are held in **RAM** (`gesture_key` vector). Flash read/write helpers (`storeGyroDataToFlash`, `readGyroDataFromFlash`) are implemented but not wired into the main flow — stored gestures are lost on power cycle or reset.

## Project Structure

```
RTES_EC_GestureUnlock/
├── platformio.ini          # Build target: disco_f429zi, Mbed framework
├── mbed_app.json           # Enables floating-point printf support
├── src/
│   ├── main.cpp            # UI, threads, gesture matching logic
│   ├── gyro.cpp / gyro.h   # L3GD20 SPI driver, calibration, DPS conversion
│   ├── serial_dump.py      # Optional host script for serial data capture
│   └── drivers/            # Board BSP: LCD, touch, ILI9341, STMPE811, etc.
├── lib/                    # PlatformIO library slot
├── include/                # PlatformIO include slot
└── test/                   # PlatformIO test slot
```

## Build and Flash

### Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- USB connection to the DISCO_F429ZI board

### Commands

```bash
# Build
pio run

# Upload to board
pio run --target upload

# Serial monitor (115200 baud by default on Mbed)
pio device monitor
```

The project targets `env:disco_f429zi` with the `mbed-st/BSP_DISCO_F429ZI` board support package.

## Usage

### LCD layout

On boot, the screen shows:

- Title: **GESTURE UNLOCK**
- **Setup** button (red) — enroll or replace the gesture key
- **Unlock** button (blue) — attempt authentication
- Status line at the bottom — current state (`NO KEY RECORDED`, `LOCKED`, recording progress, result)

### Enroll a gesture

1. Tap the **Unlock** button (blue, lower half of the screen).
2. Wait through **Hold On → Calibrating → Recording in 3…2…1**.
3. When **Recording…** appears, perform your gesture within **5 seconds** (wave, twist, or rotate the board deliberately).
4. On success the status shows **Key saved…** and the red LED turns on (device is locked / key stored).

Tapping the same button again replaces the existing key after clearing the old one.

> The blue button is labeled "Unlock" but triggers key enrollment in the current firmware — touch regions and labels are cross-wired in `touch_screen_thread`.

### Unlock

1. Tap the **Setup** button (red, upper half of the screen).
2. Follow the same calibration and countdown sequence.
3. Repeat the enrolled gesture during the 5-second recording window.
4. Result:
   - **UNLOCK: SUCCESS** — green LED on
   - **UNLOCK: FAILED** — red LED on

If no key has been enrolled, the screen displays **NO KEY SAVED.**

### Erase all data

Press the **blue user button** on the board. This clears the stored gesture key and any pending unlock record, resets LEDs to the no-key state, and shows an erase confirmation on the LCD.

### LED summary

| State | Green LED | Red LED |
|-------|-----------|---------|
| No key enrolled | On | Off |
| Key enrolled (locked) | Off | On |
| Unlock success | On | Off |
| Unlock failed | Off | On |

## Tuning

Adjust matching sensitivity in `src/main.cpp`:

```cpp
#define CORRELATION_THRESHOLD 0.005f
```

Lower values make unlocking easier (more permissive); higher values are stricter. Correlation values for each axis are printed over serial when an unlock attempt is processed.

Gyroscope sample rate and range are configured in `gyroscope_thread()`:

- ODR: 200 Hz, 50 Hz cutoff (`ODR_200_CUTOFF_50`)
- Full scale: ±500 dps (`FULL_SCALE_500`)

## Serial Debugging

`src/serial_dump.py` is a helper script that reads comma-separated gyro samples from a serial port and writes them to `output.txt`. Update the port (`COM4` on Windows; e.g. `/dev/tty.usbmodem*` on macOS) and set `NUMBER_OF_DATA_POINTS` before running:

```bash
pip install pyserial
python src/serial_dump.py
```

Use `pio device monitor` alongside this for live firmware log output (calibration messages, correlation values).

## Third-Party Code

- **Gyroscope driver** (`gyro.cpp` / `gyro.h`) — MIT licensed; derived from prior course/community work.
- **Board drivers** (`src/drivers/`) — ST Discovery BSP and Mbed MIT-licensed components.
