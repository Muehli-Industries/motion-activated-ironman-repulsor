# Motion-Activated Iron Man Repulsor (ESP32-C3 + ADXL345 + DFPlayer Mini + NeoPixel Jewel)

An Arduino sketch for **Seeed Studio XIAO ESP32-C3** that detects a “repulsor” wrist pose with an **ADXL345** accelerometer, lights a **NeoPixel Jewel (7 LEDs)**, and triggers **blast** sound effects via a **DFPlayer Mini**.  
Blast triggering uses two complementary methods: a quick **angle snap** and a directional **Y-axis jerk** – both only while the pose is held.  
The Jewel displays solid white while in pose, and plays a flicker effect across all 7 LEDs when a blast is triggered.


---

## Contents

- [Features](#features)  
- [Hardware](#hardware)  
- [Wiring](#wiring)  
- [Libraries](#libraries)  
- [How it works](#how-it-works)  
  - [Pose angle & orientation gates](#pose-angle--orientation-gates)  
  - [State machine](#state-machine)  
  - [Blast detection](#blast-detection)  
- [Configuration & Tuning](#configuration--tuning)  
  - [Pins](#pins)  
  - [Pose thresholds](#pose-thresholds)  
  - [Orientation gates](#orientation-gates)  
  - [Debounce & timing](#debounce--timing)  
  - [Smoothing](#smoothing)  
  - [Blast timing](#blast-timing)  
  - [Calm window](#calm-window)  
  - [Angle-snap path](#angle-snap-path)  
  - [Y-jerk path](#y-jerk-path)  
- [LED Control (NeoPixel Jewel)](#led-control-neopixel-jewel)  
- [Audio (DFPlayer Mini)](#audio-dfplayer-mini)  
  - [File layout](#file-layout)  
  - [Common DFPlayer pitfalls](#common-dfplayer-pitfalls)  
- [Build & Upload](#build--upload)  
- [Troubleshooting](#troubleshooting)  
- [Advanced / Debugging](#advanced--debugging)  
- [License](#license)  

---

## Features

- **Pose detection** in the hand’s Y–Z plane (wrist raised, fingers up, back of hand facing the forearm).  
- **Hysteresis + orientation gates** to avoid flicker and false triggers.  
- **Two blast triggers** while in pose:  
  1. **Angle snap** – rapid increase in pose angle over a short time window.  
  2. **Directional Y-jerk** – sudden acceleration dominated by −Y (configurable).  
- **DFPlayer Mini** audio playback for:  
  - Power ON (`0002.mp3`)  
  - Power OFF (`0001.mp3`)  
  - Blast (`0003.mp3`)  
- **NeoPixel Jewel (7 LEDs)** control:  
  - Steady white when in pose.  
  - Flicker effect across all LEDs when a blast is triggered.  
- **Adjustable brightness** via one variable for quick tuning.  
- **Cool-downs & arming delays** to prevent accidental blasts.  
- **Post-blast grace period** to avoid LED flicker right after firing.

---

## Hardware

- **MCU:** Seeed Studio XIAO ESP32-C3  
- **Accelerometer:** ADXL345 (I²C, address `0x53`)  
- **Audio:** DFPlayer Mini + microSD (FAT32)  
- **LEDs:** NeoPixel Jewel, 7× WS2812B RGB LEDs, data in on center pin  

---

## Wiring

| Module             | Signal                   | XIAO ESP32-C3 (GPIO) | Notes |
|--------------------|--------------------------|----------------------|-------|
| **NeoPixel Jewel** | Data In                  | GPIO2                | Matches `#define LED_PIN` |
|                    | VCC                      | 5V                   | Jewel prefers 5V, works on 3V3 with reduced brightness |
|                    | GND                      | GND                  | Common ground |
| **DFPlayer Mini**  | TX → ESP RX               | GPIO4                | `DFPLAYER_TX_TO_ESP_RX = 4` |
| **DFPlayer Mini**  | RX ← ESP TX               | GPIO3                | `DFPLAYER_RX_TO_ESP_TX = 3` |
|                    | VCC                      | 5V                   | Required |
|                    | GND                      | GND                  | Common ground |
| **ADXL345**        | SDA                      | I²C SDA               | Default XIAO pins |
| **ADXL345**        | SCL                      | I²C SCL               | Default XIAO pins |
|                    | VCC / GND                | 3V3 / GND             | Per module spec |

<img width="900" height="450" alt="image" src="https://github.com/user-attachments/assets/ac004ce9-4ac1-473b-948d-d23698d8b319" />


---

## Libraries

Install via Arduino Library Manager:

- **Adafruit Unified Sensor**  
- **Adafruit ADXL345**  
- **DFRobotDFPlayerMini**  
- **FastLED**  

---

## How it works

> **Note:**  
> For this example sketch, the ADXL345 sensor must be mounted **flat on the back of the hand**.  
> - **Y-axis** points toward the fingertips  
> - **X-axis** points to the right (towards thumb on the left hand, towards pinky on the right hand.)  
> - **Z-axis** points upward away from the back of the hand

### Pose angle & orientation gates
- Gravity vector normalized to `(nx, ny, nz)`.  
- Pose angle in Y–Z plane:  
  ```cpp
  angle = atan2(max(ny,0), max(nz,1e-3)) * 180 / PI;
  ```
- **Enter pose**: angle within `ON_ANGLE…MAX_ENTER_ANGLE`, Y/Z above thresholds, roll within `MAX_ROLL_X_ENTER`, quasi-static.  
- **Hold pose**: relaxed gates, no Z requirement.  
- **Leave pose**: angle ≤ `OFF_ANGLE` or gates fail, confirmed after `OFF_CONFIRM_MS`.

### State machine
1. Idle → Pose Entered → LED ON (white) + sound.  
2. Pose Held → Blast arming possible.  
3. Pose Left → LED OFF + sound.

### Blast detection
- Armed after `BLAST_ARM_MS` & calm period (`CALM_REQUIRED_MS`).  
- Requires LED on ≥ `BLAST_MIN_ON_BEFORE_FIRE_MS`.  
- **Angle snap path**: delta angle ≥ `ANGLE_DELTA_MIN` within `ANGLE_WINDOW_MS`.  
- **Y-jerk path**: dominant jerk along ±Y with thresholds.

---

# Configuration & Tuning

## Pins
```cpp
#define LED_PIN 2                                  // NeoPixel data pin (GPIO)

// DFPlayer Mini UART1 mapping (ESP32-C3):
constexpr int DFPLAYER_TX_TO_ESP_RX = 4;           // DFPlayer TX  → ESP RX (GPIO4)
constexpr int DFPLAYER_RX_TO_ESP_TX = 3;           // DFPlayer RX  ← ESP TX (GPIO3)
```

## Brightness
```cpp
const uint8_t LED_BRIGHTNESS = 200;                // 0..255 overall brightness for the NeoPixel Jewel
```

## Pose thresholds
```cpp
const float ON_ANGLE          = 55.0f;             // enter pose when angle ≥ this (deg)
const float OFF_ANGLE         = 40.0f;             // leave pose when angle ≤ this (deg)
const float MAX_ENTER_ANGLE   = 80.0f;             // block entry if hand is already near vertical (deg)
```

## Orientation gates
```cpp
const float Y_MIN_ENTER       = 0.05f;             // +Y minimum to ENTER
const float Z_MIN_ENTER       = 0.10f;             // +Z minimum to ENTER
const float Y_MIN_HOLD        = 0.02f;             // relaxed +Y to KEEP HOLDING
const float MAX_ROLL_X_ENTER  = 0.35f;             // |nx| < ... allowed roll to ENTER
const float MAX_ROLL_X_HOLD   = 0.70f;             // |nx| < ... relaxed roll while HOLDING
const float G_TOL_ON          = 0.25f;             // quasi-static: ||g|| within ±0.25g of 1g to ENTER
```

## Debounce & timing
```cpp
const unsigned long DEADTIME_MS    = 300;          // after a toggle, block re-entry for this long
const unsigned long MIN_ON_MS      = 120;          // ensure LED stays ON at least this long
const unsigned long OFF_CONFIRM_MS = 150;          // must be outside pose continuously this long to turn OFF
```

## Smoothing
```cpp
const float ALPHA = 0.2f;                          // EMA smoothing for gravity vector (0..1), lower = smoother
```

## Blast timing
```cpp
#define BACKWARD_JERK_IS_POS_Y 0                   // 0 = expect -Y for recoil; 1 = +Y
const float BLAST_MIN_ANGLE       = 55.0f;         // must be a high-angle pose
const unsigned long BLAST_COOLDOWN_MS = 500;       // min time between blasts
const unsigned long BLAST_ARM_MS      = 150;       // arming delay after LED ON
const unsigned long BLAST_MIN_ON_BEFORE_FIRE_MS = 1000; // must hold ON ≥1s before any blast
const unsigned long POST_BLAST_GRACE_MS   = 350;   // ignore OFF for this long after blast
const unsigned long POST_BLAST_MIN_ON_MS  = 250;   // enforce min ON time after blast
```

## Calm window
```cpp
const float CALM_LMAG_THRESH  = 0.25f;             // | |g| - 1 |
const unsigned long CALM_REQUIRED_MS = 60;         // calm duration to arm blasts
```

## Angle-snap path
```cpp
const float ANGLE_WINDOW_MS     = 120.0f;          // time window to measure angle change
const float ANGLE_DELTA_MIN     = 12.0f;           // min angle change within window (deg)
const float MIN_LMAG_FOR_ANGLE  = 0.15f;           // min linear acceleration magnitude required
const unsigned long ANGLE_COOLDOWN_MS = 1500;      // extra cooldown for angle-based blasts
```

## Y-jerk path
```cpp
const float BLAST_Y_THRESH     = 0.20f;            // min |Y jerk|
const float BLAST_TOTAL_THRESH = 0.28f;            // min total linear acceleration magnitude
const float BLAST_Y_DOM        = 0.55f;            // |ly| / |l| ≥ dominance ratio
```


---

## LED Control (NeoPixel Jewel)

- Controlled via **FastLED** on `LED_PIN`.  
- Steady **white** at configured brightness when pose is held.  
- On blast: white **flicker effect** across all 7 LEDs for a short time.  
- Adjust brightness quickly by changing `LED_BRIGHTNESS`.

---

## Audio (DFPlayer Mini)

### File layout
| Track No. | File Name  | Description  |
|-----------|------------|--------------|
| 1         | `0001.mp3` | Power OFF    |
| 2         | `0002.mp3` | Power ON     |
| 3         | `0003.mp3` | Blast        |

### Common pitfalls
- Use **5 V power** and common ground.  
- Cross TX/RX properly.  
- FAT32, 8–32 GB cards, 8.3 filenames.  
- Some clones need a 1 kΩ resistor on DF RX line.

---

## Build & Upload

1. Install required libraries.  
2. Select board: **Seeed XIAO ESP32-C3**.  
3. Upload sketch.  
4. Test pose → steady light & sound.  
5. Test blast → flicker effect & sound.

---


## Troubleshooting

- **LED never turns on**
  - Verify the correct **GPIO** in `LED_PIN`.
  - Ensure angle thresholds and orientation gates are not too strict.
  - Check sensor orientation; swap Y/Z if your ADXL345 is mounted differently.
- **LED flickers**
  - Increase `OFF_CONFIRM_MS`, relax `MAX_ROLL_X_HOLD`, or increase `ALPHA`.
  - Ensure **post‑blast grace** is active (`POST_BLAST_GRACE_MS` > 0).
- **Blast triggers too early**
  - Increase `BLAST_MIN_ON_BEFORE_FIRE_MS` (e.g., 1200–1500 ms).
  - Increase `BLAST_COOLDOWN_MS` and/or `ANGLE_COOLDOWN_MS`.
  - Raise `ANGLE_DELTA_MIN` or `BLAST_Y_THRESH` / `BLAST_TOTAL_THRESH`.
- **DFPlayer not responding**
  - Double‑check TX/RX cross, 5V/GND, and try another DFPlayer module.
  - Try a different SD card and confirm file naming (`0001.mp3`, etc.).

---

## Advanced / Debugging
Uncomment:
```cpp
// #define VERBOSE_DEBUG 1
```
to see detailed logs.

---

## License
MIT License.
