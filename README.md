# 🤖 Smart Vacuum Robot — Subsumption Architecture v10.1

[![Platform](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Framework](https://img.shields.io/badge/Framework-Arduino-orange.svg)](https://github.com/espressif/arduino-esp32)
[![Architecture](https://img.shields.io/badge/Architecture-Subsumption-success.svg)](https://en.wikipedia.org/wiki/Subsumption_architecture)

> **Obrynex Edition** — A competition-grade autonomous vacuum robot built on the ESP32 NodeMCU, implementing Rodney Brooks' subsumption architecture with IMU-guided precision navigation, systematic boustrophedon coverage, BLE telemetry, and real-time occupancy mapping.

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Key Features](#-key-features)
- [Architecture](#-architecture)
- [Hardware Requirements](#-hardware-requirements)
- [Pin Assignment](#-pin-assignment)
- [Wiring Guide](#-wiring-guide)
- [Software Setup](#-software-setup)
- [Compile-Time Configuration](#-compile-time-configuration)
- [BLE Protocol](#-ble-protocol)
- [Telemetry & Dashboard](#-telemetry--dashboard)
- [Subsumption Layers](#-subsumption-layers-in-detail)
- [Calibration](#-calibration)
- [Troubleshooting](#-troubleshooting)
- [Version History](#-version-history)

---

## 🌟 Overview

This firmware drives a fully autonomous robot vacuum using a **priority-based subsumption architecture**. Independent behavioural layers run in parallel; higher-priority layers dynamically suppress lower ones. The result is a fast, predictable, and provably correct control system that degrades gracefully when sensors fail.

Designed for **robotics competitions** and **home hacking**, the robot features:

- **Precision navigation** via MPU6050 gyro integration
- **Systematic room coverage** with boustrophedon (lawn-mower) path planning
- **Real-time occupancy mapping** (200×200 grid, 5 cm resolution)
- **BLE 5.0 telemetry** with live map streaming to a mobile app
- **Competition timer** with automatic end-of-run speed boosting
- **Dual-core safe ISRs** with atomic data structures

---

## ✨ Key Features

| Category | Features |
|----------|----------|
| **Navigation** | MPU6050 IMU yaw integration, wheel encoder odometry (±2 %/m), dead-reckoning fallback (±20 %/m), boustrophedon row coverage, wall-following P-controller |
| **Sensing** | 3× HC-SR04 ultrasonic (front/right/left), 2× IR cliff sensors, IMU pitch/roll pickup detection, battery voltage monitoring |
| **Safety** | Battery critical shutdown, pickup/flip motor cutoff, cliff avoidance, thermal derating (L298N), Task Watchdog Timer, brownout recovery |
| **Intelligence** | Anti-pattern detection (loop & oscillation traps), coverage-biased random exploration, spiral mode in open spaces, stuck self-rescue |
| **Connectivity** | BLE status notifications (5 Hz), command interface, RLE-compressed occupancy grid streaming |
| **Persistence** | NVS flash telemetry (boot count, cliff saves, avoidances, rescues), competition lifetime stats |

---

## 🏗️ Architecture

```
flowchart TD
    A[LAYER 3.6: Battery Safety] -->|suppresses| B[LAYER 3.5: Pickup / Flip]
    B -->|suppresses| C[LAYER 3: Survival]
    C -->|suppresses| D[LAYER 2.5: Stuck Escape]
    D -->|suppresses| E[LAYER 2: Avoidance]
    E -->|suppresses| F[LAYER 1.5: Wall Following]
    F -->|suppresses| G[LAYER 1.2: Boustrophedon]
    G -->|suppresses| H[LAYER 1: Cruise & Clean]

    style A fill:#ff4444,stroke:#333,stroke-width:2px,color:#fff
    style H fill:#44ff44,stroke:#333,stroke-width:2px,color:#000

```

**Design principle:** Each layer is an independent `bool` function. If a layer returns `true`, it suppresses every layer below it for that `loop()` iteration. No scheduler, no RTOS tasks, no shared state beyond sensor data — simple, fast, and deterministic.

---

## 🔧 Hardware Requirements

### Core Components

| Component | Specification | Qty | Est. Cost |
|-----------|--------------|-----|-----------|
| Microcontroller | ESP32 NodeMCU (DevKit V1) | 1 | ~$6 |
| Motor Driver | L298N Dual H-Bridge | 1 | ~$3 |
| Motors | TT DC Motors + 65 mm wheels | 2 | ~$4 |
| Ultrasonic Sensors | HC-SR04 | 3 | ~$3 |
| IMU | MPU6050 (GY-521) | 1 | ~$3 |
| IR Sensors | TCRT5000 or similar (analog/digital) | 2 | ~$1 |
| Wheel Encoders | Hall-effect + 20-pole magnet ring | 2 | ~$4 |
| Cleaning Motors | 3–6 V DC (brush + vacuum) | 2 | ~$5 |
| Battery | 3S LiPo (11.1 V, 2200 mAh+) | 1 | ~$15 |
| Voltage Divider | 28 kΩ / 10 kΩ (for 3.3 V ADC) | 1 set | ~$0.50 |
| Chassis | Robot vacuum platform | 1 | ~$10 |

**Total estimated cost: ~$55–$70**

> ⚠️ **Critical:** GPIO 12 requires a **10 kΩ pull-down resistor to GND** at boot. It is an ESP32 strapping pin that selects flash voltage. Without the pull-down, the board may fail to boot when the encoder is connected.

---

## 📍 Pin Assignment

| Function | GPIO | Notes |
|----------|------|-------|
| **Left Motor IN1** | 27 | Direction |
| **Left Motor IN2** | 26 | Direction |
| **Left Motor ENA** | 25 | PWM (LEDC) |
| **Right Motor IN3** | 14 | Direction |
| **Right Motor IN4** | 4 | Safe alternative to GPIO 12 |
| **Right Motor ENB** | 13 | PWM (LEDC) |
| **US Front TRIG** | 5 | |
| **US Front ECHO** | 18 | ISR on CHANGE |
| **US Right TRIG** | 19 | |
| **US Right ECHO** | 16 | Moved from GPIO 21 in v6 |
| **US Left TRIG** | 17 | Moved from GPIO 22 in v6 |
| **US Left ECHO** | 23 | |
| **MPU6050 SDA** | 21 | I2C 400 kHz |
| **MPU6050 SCL** | 22 | I2C 400 kHz |
| **IR Cliff Left** | 34 | Input-only, no pull-up |
| **IR Cliff Right** | 35 | Input-only, no pull-up |
| **Brush Motor** | 32 | Transistor switch, HIGH = on |
| **Vacuum Motor** | 33 | Transistor switch, HIGH = on |
| **Status LED** | 2 | Onboard LED, 8 blink patterns |
| **Battery ADC** | 36 | VP (Input-only), 12-bit, 11 dB attenuation |
| **Emergency Stop** | 0 | BOOT button, active-LOW, FALLING edge |
| **Encoder Left** | 12 | Hall-effect RISING, **10 kΩ pull-down required** |
| **Encoder Right** | 15 | Hall-effect RISING, was MPU6050 INT |

---

## 🔌 Wiring Guide

### Power Distribution
```
3S LiPo (12.6 V) ──┬── L298N VCC ── Motors
                   ├── Buck converter ── 5 V ── ESP32 VIN
                   └── Voltage divider ── GPIO 36 (ADC)
```

### I2C Bus (MPU6050)
```
ESP32 GPIO 21 (SDA) ─── MPU6050 SDA
ESP32 GPIO 22 (SCL) ─── MPU6050 SCL
3.3 V ───────────────── MPU6050 VCC
GND ─────────────────── MPU6050 GND
```

### Motor Driver (L298N)
```
ENA (GPIO 25) ─── L298N ENA
IN1 (GPIO 27) ─── L298N IN1
IN2 (GPIO 26) ─── L298N IN2
ENB (GPIO 13) ─── L298N ENB
IN3 (GPIO 14) ─── L298N IN3
IN4 (GPIO 4)  ─── L298N IN4
```

### Ultrasonic Sensors
```
Front: TRIG→GPIO 5,  ECHO→GPIO 18
Right: TRIG→GPIO 19, ECHO→GPIO 16
Left:  TRIG→GPIO 17, ECHO→GPIO 23
```

> 🔄 **Migration from v3/v5:** Move `ECHO_RIGHT` from GPIO 21 → GPIO 16 and `TRIG_LEFT` from GPIO 22 → GPIO 17 to free I2C pins.

---

## 💻 Software Setup

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) ≥ 2.0 or [PlatformIO](https://platformio.org/)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32) ≥ 2.0.14
- (Optional) [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) library — **strongly recommended** to reclaim ~60 KB heap

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/obrynex-vacuum.git
   cd obrynex-vacuum
   ```

2. **Open in Arduino IDE**
   - Open `robot_vacuum_subsumption_v10.1.ino`
   - Select **Tools → Board → ESP32 Dev Module**
   - Select the correct **Port**

3. **Configure feature flags** (see next section)

4. **Upload**
   ```bash
   # PlatformIO
   pio run --target upload

   # Or click Upload in Arduino IDE
   ```

5. **Open Serial Monitor** at `115200 baud` to view the self-test and telemetry dashboard.

---

## ⚙️ Compile-Time Configuration

Edit the flags in **Section 1** of the sketch to enable/disable subsystems:

```cpp
#define VERBOSE_DEBUG            // Serial debug output (disable for competition)
#define ENABLE_IMU               // MPU6050 support
#define ENABLE_COMPETITION_TIMER // Speed boost in final 20% of run
#define ENABLE_BLE               // BLE telemetry (~100 KB flash)

// ── v8.0 Navigation Upgrades ──
#define ENABLE_ENCODERS          // Wheel encoders (requires GPIO 12 pull-down)
#define ENABLE_OCCUPANCY_GRID    // 200×200 grid (40 KB, needs PSRAM or free heap)
#define ENABLE_BOUSTROPHEDON     // Systematic row coverage (requires both above)
```

### Dependency Validation

The code includes a compile-time guard:

```cpp
#if defined(ENABLE_BOUSTROPHEDON) && (!defined(ENABLE_ENCODERS) || !defined(ENABLE_OCCUPANCY_GRID))
  #error "ENABLE_BOUSTROPHEDON requires both ENABLE_ENCODERS and ENABLE_OCCUPANCY_GRID"
#endif
```

### Memory Budget Guide

| Configuration | Heap Used | Notes |
|---------------|-----------|-------|
| Minimal (no BLE, no grid) | ~20 KB | Cruise + avoidance only |
| Standard (BLE + IMU) | ~110 KB | Good for most competitions |
| Full (BLE + grid + encoders) | ~150 KB | **Requires NimBLE** or PSRAM |
| Full + Bluedroid BLE | ~210 KB | May fail to allocate grid |

> 💡 **Pro tip:** If `reportMemory()` shows internal free heap < 80 KB, switch from Bluedroid to **NimBLE-Arduino** to reclaim ~60 KB. Change `#include <BLEDevice.h>` to `#include <NimBLEDevice.h>` and prefix BLE classes with `Nim`.

---

## 📡 BLE Protocol

### Service & Characteristics

| Char | UUID | Properties | Purpose |
|------|------|------------|---------|
| **Status** | `...8fcc-aa` | Notify, Read | 20-byte telemetry packet @ 5 Hz |
| **Command** | `...8fcc-bb` | Write, Write NR | 1-byte remote command |
| **Map Stream** | `...8fcc-cc` | Notify | RLE-compressed grid rows |

### Status Packet (20 bytes, little-endian)

```cpp
struct BleStatusPacket {
  uint8_t  state;       // RobotState enum value
  uint8_t  flags;       // bit 0: leftCliff, 1: rightCliff, 2: eStop, 3: imuOK, 4: thermalDerate
  uint16_t battMv;      // Battery millivolts (12600 = 12.600 V)
  uint16_t distF_mm;    // Front distance × 10 (mm precision)
  uint16_t distR_mm;    // Right distance × 10
  uint16_t distL_mm;    // Left distance × 10
  int16_t  yaw10;       // IMU yaw × 10 (degrees, wrapped ±180)
  int16_t  pitch10;     // IMU pitch × 10
  int16_t  roll10;      // IMU roll × 10
  uint16_t uptimeSec;   // Seconds since boot
  uint8_t  coveragePct; // Coverage grid 0–100%
  uint8_t  reserved;    // bit 0: boustrophedon active
};
```

### Command Codes

| Code | Name | Action |
|------|------|--------|
| `0x01` | `BLE_CMD_START` | Start/resume cleaning |
| `0x02` | `BLE_CMD_STOP` | Soft emergency stop |
| `0x03` | `BLE_CMD_RESUME` | Clear e-stop and resume |
| `0x04` | `BLE_CMD_RESET_STATS` | Zero counters, reset coverage, restart position |

### Map Stream Format

RLE-encoded occupancy rows sent at ~1 Hz (full map every ~200 s):

```
[0-1] Row index (uint16 BE)
[2-3] Total rows (uint16 BE)
[4..] Alternating (run_length, cell_value) pairs
```

Cell values: `0 = unknown`, `1 = cleaned`, `255 = obstacle`

---

## 📊 Telemetry & Dashboard

On boot, the robot prints a telemetry dashboard to Serial:

```
======================================================
     Robot Vacuum v10.1 - Competition Dashboard
======================================================
  Boots       : 42      Battery : 12.34V
  Cliff Saves : 7       IMU     : OK (precise)
  Avoidances  : 23      Brownout: Normal
  Rescues     : 3
======================================================
```

### LED Status Patterns

| Pattern | Period | Meaning |
|---------|--------|---------|
| Solid ON | — | Cruise (all clear) |
| Slow blink | 500 ms | Wall following |
| Medium blink | 300 ms | Spiral mode |
| Fast blink | 150 ms | Avoidance |
| Very fast | 75 ms | Stuck self-rescue |
| Ultra fast | 50 ms | Lifted / flipped |
| SOS dot | 100 ms | Survival (cliff/critical) |
| Double-flash | 150 ms ×2 | Battery critical shutdown |
| Steady 200 ms | 200 ms | Emergency stop |

---

## 🛡️ Subsumption Layers (in detail)

### Layer 3.6 — Battery Safety
Absolute hardware protection. If voltage drops below **9.9 V** (3S LiPo critical), all motors stop, cleaning motors shut off, and lifetime stats are flushed to NVS flash before the protection circuit cuts power.

### Layer 3.5 — Pickup / Flip Detection
Uses MPU6050 accelerometer pitch/roll. If `|pitch| > 30°` (lifted) or `> 100°` (flipped), motors stop within one loop iteration (~100 µs). Resets pattern detector and IMU tracking on resume to prevent stale data from corrupting navigation.

### Layer 3 — Survival
Triggers on cliff (IR sensors) or critical obstacles `< 10 cm`. Sequence: stop → reverse 900 ms → 180° turn toward open space. IMU ensures precise 170° turn; falls back to 1350 ms time-based if IMU is unavailable.

### Layer 2.5 — Stuck Escape
Detects wheel slip when front distance changes by `< 1.2 cm` over 2.5 seconds while advancing. Three-phase rescue: reverse → high-speed wiggle (left-right oscillation) → 135° escape turn.

### Layer 2 — Avoidance
Handles forward obstacles at **10–25 cm** with 5 cm hysteresis. Turns toward the roomier side (IMU-guided 85°). If still blocked, retries in the opposite direction. Records turn direction for oscillation detection.

### Layer 1.5 — Wall Following
Proportional controller (Kp = 2.5) locks onto a side wall at **20 cm** distance. Engages after 3 seconds of stable wall detection. Exits on front obstacle, wall loss (> 2 s), or oscillation trap detection. Correction is battery-compensated to maintain consistent responsiveness as the LiPo discharges.

### Layer 1.2 — Boustrophedon
Systematic row-by-row coverage. Requires encoders + occupancy grid.
- **Row advance:** Drive straight with IMU heading correction (±40 PWM max)
- **Row end:** Front obstacle `< 30 cm` triggers U-turn
- **U-turn:** 90° spin → lateral advance by `ROW_PITCH_CM` (20 cm) → 90° spin
- Alternates turn direction each row to always move into unexplored territory

### Layer 1 — Cruise & Clean
Default behaviour with three modes:
1. **Loop escape:** If yaw rotation exceeds 320° in 10 s, force a coverage-biased turn
2. **Spiral mode:** In open spaces (> 120 cm clearance), expands an outward spiral for up to 15 s
3. **Random cruise:** Turns every 3–8 s with 70/30 bias toward less-visited coverage cells

---

## 🔬 Calibration

### 1. Battery Voltage Divider

The default `BATTERY_CALIBRATION_FACTOR = 2.0` assumes a 10k/10k divider, but this produces **6.3 V at the ADC pin** for a full 3S LiPo — exceeding the 3.3 V maximum!

**Recommended:** Use a **28 kΩ / 10 kΩ** divider (factor ≈ 3.8) or similar. Verify with a multimeter and adjust the constant.

### 2. Wheel Encoder Geometry

Measure your actual hardware and update these constants:

```cpp
constexpr float ENC_TICKS_PER_REV = 20.0f;  // Poles on magnet ring
constexpr float WHEEL_DIAMETER_MM = 65.0f;  // Actual wheel diameter
constexpr float WHEEL_BASE_MM     = 150.0f; // Wheel centre-to-centre
```

### 3. Dead-Reckoning Speed Model

If running without encoders, calibrate by measuring actual distance travelled at 255 PWM for 1 second:

```cpp
constexpr float DR_SPEED_SCALE_CM_S = 30.0f; // cm/s at 255 PWM
```

### 4. IMU Calibration

The robot performs a 2-second stationary gyro calibration at boot. **Keep the robot still** during startup for best yaw accuracy.

---

## 🐛 Troubleshooting

| Symptom | Cause | Solution |
|---------|-------|----------|
| Boot loop / flash errors | GPIO 12 HIGH at boot | Add 10 kΩ pull-down from GPIO 12 to GND |
| Battery reads 4–6 V | Wrong ADC attenuation or divider | Verify `analogSetAttenuation(ADC_11db)` and divider ratio |
| IMU not found | Wrong I2C pins or bad wiring | Check SDA→GPIO 21, SCL→GPIO 22, pull-ups present |
| Grid allocation failed | Insufficient heap | Switch BLE to NimBLE or disable BLE |
| Robot spins in place | Encoder not counting | Check magnet alignment and ISR wiring |
| Front sensor always 9 cm | Sensor disconnected | Check wiring; stale sensor is clamped to `CRITICAL_DIST - 1` |
| WDT reset during run | `loop()` blocked | Ensure no `delay()` in main loop; check sensor ISR timing |
| Brownout resets | Weak battery or high inrush | Reduce `brownoutSpeedFactor` or use a battery with higher C-rating |

---

## 📈 Version History

| Version | Highlights |
|---------|------------|
| **v10.1** | ADC attenuation fix, atomic emergency stop, WDT during IMU cal, boustrophedon stuck detection, pickup resume resets pattern detector, dead reckoning during U-turn advance, battery divider sanity check |
| **v10.0** | BLE yaw overflow fix, ASCII-only output, `wrap180()` helper, static assertions, BLE IMU sentinel values, boustrophedon `readAtomic()` consistency |
| **v8.1** | Public `getHeadingDeg()`, stale front-US guard in boustrophedon, removed unused state variables, unified version strings, dependency `#error` guard |
| **v8.0** | Wheel encoders, 200×200 occupancy grid, boustrophedon planner, encoder odometry, MAP_STREAM_CHAR BLE streaming |
| **v7.2** | L298N thermal derating, competition timer, dead reckoning, coverage grid with 70/30 bias |
| **v7.1** | Sensor NaN guard, BLE command atomicity (`std::atomic`), wall-follow gain compensation |
| **v7.0** | BLE telemetry, IMU integration, wall following, anti-pattern detection, pickup/flip detection |
| **v6.x** | Subsumption architecture baseline, IR cliff sensors, ultrasonic ISR pipeline 

---

## 🙏 Acknowledgements

- **Rodney Brooks** — Subsumption Architecture (1986)
- **ESP32 Arduino Core Team** — Robust dual-core framework
- **NimBLE-Arduino contributors** — Lightweight BLE stack

> *"Fast, cheap, and provably correct — pick three."* — Obrynex Engineering

---

<p align="center">
  <b>Built with precision. Tested in competition. Ready to clean.</b><br>
  <sub>Obrynex Edition v10.1 | ESP32 NodeMCU</sub>
</p>
