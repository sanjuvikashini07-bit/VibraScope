# VibraScope

**Portable Machine Vibration Diagnostic System** — built on an ESP32 and MPU6050.

VibraScope is a hand-held vibration health monitor for small rotating machinery (DC motors, cooling fans). It samples acceleration in real time, computes RMS and peak severity, runs an on-device FFT to find the dominant vibration frequency, and compares live readings against a machine-specific healthy baseline to report **NORMAL / WARNING / HIGH VIBRATION** — on an OLED display, through a buzzer, and on a live browser dashboard.

---

## Table of Contents

- [How It Works](#how-it-works)
- [Hardware](#hardware)
- [Wiring](#wiring)
- [Repository Contents](#repository-contents)
- [Firmware Setup](#firmware-setup)
- [Using the Device](#using-the-device)
- [Calibration](#calibration)
- [Status Thresholds](#status-thresholds)
- [Live Dashboard](#live-dashboard)
- [Safe Testing Procedure](#safe-testing-procedure)
- [Troubleshooting](#troubleshooting)
- [Future Scope](#future-scope)
- [License](#license)

---

## How It Works

A mechanically healthy machine vibrates only slightly, mostly at its rotation frequency (1× RPM), with a small, stable amplitude. Faults — imbalance, misalignment, bearing wear, loose mounting — increase vibration *amplitude* and/or introduce *new frequency components* (harmonics, sidebands). VibraScope measures both:

- **Time domain** — RMS and peak acceleration ("how much shaking")
- **Frequency domain** — FFT-derived dominant frequency ("at what rate")

and compares the result against a baseline learned from *that specific machine* while healthy, rather than a fixed universal threshold — the same principle industrial predictive-maintenance systems use, scaled down.

```
MPU6050 (accel, one axis)
        │  500 Hz sampling, 256-sample buffer (~512 ms window)
        ▼
Raw buffer → subtract mean (DC/gravity removal)
        │
        ├──► RMS / Peak            ─── time-domain severity
        └──► Hamming window → FFT  ─── dominant frequency
                        │
                        ▼
        Compare {RMS, Peak, Freq} vs stored baseline
                        │
                        ▼
        Status → OLED display + Buzzer + Live Dashboard
```

---

## Hardware

| Component | Notes |
|---|---|
| ESP32 DevKit V1 | Main controller — sampling, FFT, state machine, WiFi/USB output |
| MPU6050 | I2C accelerometer/gyro, one axis used, ±4g range |
| SSD1306 OLED (0.96", 128×64) | I2C display — live readings, baseline compare, FFT bars |
| Piezo buzzer | Audible alert — silent / intermittent / continuous by severity |
| Push buttons ×3 | Start/Stop, Calibrate, Screen Mode |

---

## Wiring

All I2C devices share one bus — no address conflict (MPU6050 = `0x68`, SSD1306 = `0x3C`).

| Component | Pin | ESP32 Pin | Notes |
|---|---|---|---|
| MPU6050 | VCC / GND | 3V3 / GND | Onboard 3.3V regulator on most breakouts |
| MPU6050 | SCL / SDA | GPIO22 / GPIO21 | Shared I2C bus |
| SSD1306 OLED | SCL / SDA | GPIO22 / GPIO21 | Shared I2C bus |
| Button — Start/Stop | signal | GPIO32 | `INPUT_PULLUP`, other leg to GND |
| Button — Calibrate | signal | GPIO33 | `INPUT_PULLUP`, other leg to GND |
| Button — Screen Mode | signal | GPIO25 | `INPUT_PULLUP`, other leg to GND |
| Buzzer | + / − | GPIO26 / GND | Bare piezo or active buzzer module |

No external pull-up resistors are needed on most breakout boards or the buttons (internal `INPUT_PULLUP` used).

---

## Repository Contents

| File | Description |
|---|---|
| `VibraScope.ino` | Main firmware — sampling, FFT, state machine, WiFi + USB Serial output |
| `VibraScope_Dashboard.html` | **Recommended.** Standalone browser dashboard using the Web Serial API — no build step, no WiFi required, connects straight to the ESP32's USB/COM port |
| `VibraScopeDashboard_Serial.jsx` | React version of the same USB-Serial dashboard, for projects already using a React build pipeline |
| `VibraScopeDashboard.jsx` | Earlier WiFi/HTTP-polling React dashboard (requires the ESP32 and browser on the same network) |
| `BuzzerTest.ino` | Minimal standalone sketch to test the buzzer in isolation |
| `I2CScanner.ino` | Minimal standalone sketch to verify the OLED/MPU6050 are detected on the I2C bus |

---

## Firmware Setup

**Board package:** Install **esp32 by Espressif Systems** via Arduino IDE's Boards Manager, select **ESP32 Dev Module**.

**Libraries** (Library Manager):

| Library | Purpose |
|---|---|
| `Adafruit MPU6050` + `Adafruit Unified Sensor` | Accelerometer driver |
| `Adafruit SSD1306` + `Adafruit GFX Library` | OLED driver |
| `arduinoFFT` (v2.x) | FFT computation |
| `ArduinoJson` | JSON packet formatting |

`Wire.h`, `Preferences.h`, `WiFi.h`, and `WebServer.h` ship with the ESP32 board package — no separate install needed.

> **Note on `arduinoFFT` v2.x:** the library is now templated (`ArduinoFFT<double>`); the sketch already accounts for this.

1. Open `VibraScope.ino` in Arduino IDE.
2. (Optional — only needed for the WiFi dashboard) Set your network credentials near the top of the file:
   ```cpp
   const char* WIFI_SSID = "YourWiFiName";
   const char* WIFI_PASS = "YourWiFiPassword";
   ```
   WiFi is optional — the device works standalone and the USB Serial dashboard doesn't need it at all.
3. Select the correct board and COM port, then Upload.

---

## Using the Device

Three buttons drive a simple state machine:

| State | Trigger | Notes |
|---|---|---|
| **IDLE** | Default / Stop pressed | Calibrate only works from here |
| **RUNNING** | Start/Stop pressed from IDLE | Acquire → RMS/Peak → FFT → Status → Display, once per cycle (~0.5–1s) |
| **CALIBRATING** | Calibrate pressed from IDLE | Blocks button input for ~2.5–5s while it averages 5 passes |

**Operating sequence:** `Stop → Calibrate → Start`. Calibrate is ignored if pressed while RUNNING.

**Screen Mode** (readable in any state, but only visibly changes the OLED while RUNNING) cycles between:
1. Main readings + status
2. Baseline vs current comparison
3. Mini FFT bar graph

---

## Calibration

VibraScope learns *your specific machine's* healthy signature instead of using a fixed threshold — every motor/fan/mounting combination vibrates differently.

1. Mount the MPU6050 rigidly on the machine housing.
2. Run the machine at its normal operating speed, let it settle ~10 seconds.
3. From IDLE, press **Calibrate**.
4. 5 buffers are captured (~2.5s) and averaged automatically.
5. Baseline RMS/Peak/Frequency is saved to flash (`Preferences`) — persists across power cycles.

**Re-calibrate whenever** you re-mount the sensor, swap machines, or perform maintenance.

---

## Status Thresholds

| Status | Condition |
|---|---|
| NORMAL | RMS ratio ≤ 1.5× baseline, frequency deviation ≤ 15% |
| WARNING | RMS ratio > 1.5× baseline, or frequency deviation > 15% |
| HIGH VIBRATION | RMS ratio > 2.5× baseline, or peak ratio > 3.0× baseline |

Tune `WARNING_RMS_RATIO`, `HIGH_RMS_RATIO`, `HIGH_PEAK_RATIO`, and `FREQ_DEV_WARN_PCT` near the top of the sketch during bench testing if needed.

---

## Live Dashboard

Two ways to view live data in a browser; **USB Serial is recommended** — it needs no network setup at all.

### Option A — USB Serial (recommended)

1. Open `VibraScope_Dashboard.html` directly in **Chrome or Edge on desktop** (double-click it — no server, no build step).
2. Close Arduino IDE's Serial Monitor first (only one program can hold the COM port at a time).
3. Click **Connect via USB**, select the ESP32's COM port from the picker.

The firmware prints one tagged JSON line per cycle over USB (`JSON:{...}`), which the dashboard reads directly — completely independent of WiFi.

### Option B — WiFi / HTTP polling

The sketch also runs a small web server (`GET /data` returns the same JSON), for the React dashboard (`VibraScopeDashboard.jsx`) or any HTTP client. Requires the ESP32 and your browser on the same network, and is more sensitive to network configuration (hotspot client isolation, campus WiFi restrictions, etc. — USB Serial avoids all of this).

---

## Safe Testing Procedure

Use a small 5V/12V DC fan or hobby motor, powered from its **own separate supply** — never share the ESP32's power with the motor.

1. **Baseline:** calibrate with the fan freshly balanced and running normally.
2. **Confirm NORMAL:** start monitoring, status should stay NORMAL.
3. **Induce a safe fault** — pick one:
   - Small blob of putty/tape/modeling clay on one blade tip (mass imbalance)
   - Slightly loosen (don't remove) the mounting screw
4. Confirm status escalates to WARNING/HIGH and the buzzer responds.
5. Remove the fault, confirm status returns to NORMAL within 1–2 cycles.

**Never** touch a spinning blade directly to stop or test it. Disconnect motor power before adjusting the mount or test weights.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| WiFi won't connect | Wrong SSID/password, 5GHz-only network (ESP32 needs 2.4GHz), hotspot client isolation, or a per-device approval setting on the phone |
| Dashboard shows stale/mismatched status vs OLED | Normal small lag between polling cycles; should be under ~1s |
| Buzzer silent | Confirm with `BuzzerTest.ino` in isolation first; check wiring polarity and GND continuity |
| OLED/MPU6050 not responding | Run `I2CScanner.ino` — should detect `0x3C` and `0x68`; if neither appears, check SDA/SCL wiring and power |
| `ArduinoFFT` compile error | Library v2.x is templated — use `ArduinoFFT<double>`, not bare `ArduinoFFT` |

---

## Future Scope

- Multi-axis (X/Y/Z) sensing for richer fault signatures
- Cloud logging for long-term trend analysis across multiple machines
- On-device or cloud-based ML fault classification
- Battery-powered enclosure for fully hand-held field use

---

## License

*(Add your chosen license here — e.g. MIT, GPL-3.0 — before publishing.)*
