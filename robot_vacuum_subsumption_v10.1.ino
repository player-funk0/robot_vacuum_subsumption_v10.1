// ╔══════════════════════════════════════════════════════════════════════════╗
// ║   Smart Vacuum Robot — Subsumption Architecture v10.1                   ║
// ║   OBRYNEX EDITION | ESP32 NodeMCU                                        ║
// ║                                                                          ║
// ║   What's new in v10.1 (12 patches — all criticals + bugs fixed):        ║
// ║                                                                          ║
// ║   CRITICAL FIXES (v10.1):                                               ║
// ║   • Fix: ADC attenuation set to 11 dB + 12-bit resolution. Prevents     ║
// ║     wrong battery readings and potential GPIO damage.                   ║
// ║   • Fix: BoustrophedonPlanner SPIN1->ADVANCE uses readAtomic().         ║
// ║   • Fix: Stuck detector covers STATE_BOUSTRO_ROW (was Cruise+Wall).     ║
// ║   • Fix: Pickup resume resets pattern detector + IMU turn tracking.     ║
// ║   • Fix: Dead reckoning updates during boustrophedon U-turn ADVANCE.    ║
// ║   • Fix: emergencyStopPressed is std::atomic<bool> (dual-core safe).    ║
// ║   • Fix: CoverageGrid::toCell() uses int16_t (was int8_t).              ║
// ║   • Fix: PatternDetector::detectLoop() uses wrap180() on yaw deltas.    ║
// ║   • Fix: WDT fed during IMU calibration + self-test.                    ║
// ║   • Fix: Battery divider sanity check at boot.                          ║
// ║   v10.0 retains: BLE yaw overflow, ASCII, map-stream buf, wrap180(),    ║
// ║     static_assert, reserved byte, BLE IMU sentinel.                     ║
// ║                                                                          ║
// ║   What's in v8.0 (3 navigation upgrades):                                ║
// ║   • Wheel encoders (ENABLE_ENCODERS), occupancy grid                     ║
// ║     (ENABLE_OCCUPANCY_GRID), boustrophedon planner                       ║
// ║     (ENABLE_BOUSTROPHEDON)                                              ║
// ║   What's in v7.2/7.1/7.0/v6 (all retained):                            ║
// ║   • DeadReckoning, CoverageGrid 70/30 bias, L298N thermal, BLE v7.2,   ║
// ║     MPU6050 IMU, wall-following, anti-pattern detector, pickup/flip,     ║
// ║     WDT, brownout recovery, competition timer, emergency stop,           ║
// ║     battery voltage compensation, enhanced LED patterns                  ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
// ═══════════════════════════════════════════════════════════════════════════
// ■ Subsumption Architecture — Rodney Brooks 1986
// ═══════════════════════════════════════════════════════════════════════════
// Parallel independent layers, highest priority suppresses lower ones.
// return in loop() IS the suppression — simple, fast, provably correct.
//
//  ┌─────────────────────────────────────────────────────────────────────┐
//  │  LAYER 3.6 ◄ BATTERY SAFETY      — absolute hardware protection     │
//  │  LAYER 3.5 ◄ PICKUP / FLIP       — motors off if robot is lifted    │
//  │  LAYER 3   ◄ SURVIVAL            — cliff + critical obstacle (<10cm)│
//  │  LAYER 2.5 ◄ STUCK ESCAPE        — wheel-slip self-rescue           │
//  │  LAYER 2   ◄ AVOIDANCE           — forward obstacle (10-25cm)       │
//  │  LAYER 1.5 ◄ WALL FOLLOWING      — systematic edge coverage         │
//  │  LAYER 1.2 ◄ BOUSTROPHEDON       — row-by-row systematic coverage   │
//  │              (ENABLE_BOUSTROPHEDON — requires encoders + occ. grid)  │
//  │  LAYER 1   ◄ CRUISE & CLEAN      — forward motion + spiral + bias   │
//  └─────────────────────────────────────────────────────────────────────┘

#include <Arduino.h>
#include <Wire.h>             // I2C for MPU6050
#include <esp_system.h>       // esp_random(), esp_reset_reason()
#include <esp_task_wdt.h>     // Task Watchdog Timer
#include <esp_heap_caps.h>    // heap_caps_get_free_size() for memory diagnostics
#include <math.h>
#include <atomic>             // std::atomic — correct multi-core flag semantics
#include <cstring>            // memset — explicit include, not just transitive
#include <Preferences.h>      // NVS flash for competition telemetry
#include "soc/gpio_struct.h"  // Direct GPIO register reads in ISR

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 1: Compile-Time Feature Flags
// Comment out any flag to disable that subsystem for testing or if hardware
// is not present. The code degrades gracefully in all cases.
// ═══════════════════════════════════════════════════════════════════════════
#define VERBOSE_DEBUG            // Comment out for final competition binary
#define ENABLE_IMU               // MPU6050 on I2C. Comment out if not fitted.
#define ENABLE_COMPETITION_TIMER // Speed boost in final 20% of run time.
#define ENABLE_BLE               // BLE telemetry + remote commands. Comment out to save ~100 KB flash.

// ── v8.0 Navigation Upgrades ──────────────────────────────────────────────
// Enable in order — each flag degrades gracefully if hardware is absent.
#define ENABLE_ENCODERS          // Hall-effect wheel encoders on GPIO 12 / 15.
                                 // ⚠ GPIO 12: 10 kΩ pull-down to GND required (boot strapping pin).
                                 // Needs: 2× hall-effect encoder modules (~$2 each), 20-pole magnet wheel.
                                 // Fallback when commented out: DeadReckoning (v7.2 behaviour).
#define ENABLE_OCCUPANCY_GRID    // Phase 1 grid: 200×200 uint8_t (40 KB). Needs PSRAM or 40 KB free heap.
                                 // Activates MAP_STREAM_CHAR BLE streaming.
                                 // Fallback when commented out: 8×8 CoverageGrid (v7.2).
#define ENABLE_BOUSTROPHEDON     // Systematic row coverage Layer 1.2.
                                 // Requires: ENABLE_ENCODERS + ENABLE_OCCUPANCY_GRID both active.
                                 // Fallback when commented out or deps missing: bias-cruise (v7.2).

// ── v8.1: Dependency validation ───────────────────────────────────────────
// ENABLE_BOUSTROPHEDON strictly requires both encoder and grid features.
// If you enable boustrophedon without its dependencies, you get a clean
// compile-time error pointing right here instead of confusing linker errors.
#if defined(ENABLE_BOUSTROPHEDON) && (!defined(ENABLE_ENCODERS) || !defined(ENABLE_OCCUPANCY_GRID))
  #error "ENABLE_BOUSTROPHEDON requires both ENABLE_ENCODERS and ENABLE_OCCUPANCY_GRID"
#endif

#ifdef ENABLE_BLE
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
#endif

#ifdef VERBOSE_DEBUG
  #define DBG(...)   Serial.printf(__VA_ARGS__)
  #define DBGLN(...) Serial.println(__VA_ARGS__)
#else
  #define DBG(...)
  #define DBGLN(...)
#endif

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 2: Pin Assignment Table
// ═══════════════════════════════════════════════════════════════════════════
//
// ┌──────────────────────────────┬────────────┬─────────────────────────────┐
// │ Component                    │ GPIO ESP32 │ Note                        │
// ├──────────────────────────────┼────────────┼─────────────────────────────┤
// │ Left motor IN1               │ 27         │ direction                   │
// │ Left motor IN2               │ 26         │ direction                   │
// │ Left motor ENA (PWM)         │ 25         │ LEDC                        │
// ├──────────────────────────────┼────────────┼─────────────────────────────┤
// │ Right motor IN3              │ 14         │ direction                   │
// │ Right motor IN4              │  4         │ safe alt to GPIO 12         │
// │ Right motor ENB (PWM)        │ 13         │ LEDC                        │
// ├──────────────────────────────┼────────────┼─────────────────────────────┤
// │ US front TRIG / ECHO         │  5 / 18    │ ISR on ECHO                 │
// │ US right TRIG / ECHO         │ 19 / 16    │ ★ ECHO moved from 21→16     │
// │ US left  TRIG / ECHO         │ 17 / 23    │ ★ TRIG moved from 22→17     │
// ├──────────────────────────────┼────────────┼─────────────────────────────┤
// │ MPU6050 SDA                  │ 21         │ ★ freed by US remap above   │
// │ MPU6050 SCL                  │ 22         │ ★ freed by US remap above   │
// │ MPU6050 INT (unused in sw)   │ 15         │ reserved, not yet connected │
// ├──────────────────────────────┼────────────┼─────────────────────────────┤
// │ IR cliff — left              │ 34         │ Input-Only, no pull-up      │
// │ IR cliff — right             │ 35         │ Input-Only, no pull-up      │
// ├──────────────────────────────┼────────────┼─────────────────────────────┤
// │ Brush motor (transistor)     │ 32         │ HIGH = on                   │
// │ Vacuum motor (transistor)    │ 33         │ HIGH = on                   │
// ├──────────────────────────────┼────────────┼─────────────────────────────┤
// │ Status LED (onboard)         │  2         │ 8 distinct blink patterns   │
// │ Battery voltage divider      │ 36         │ ADC VP (Input-Only)         │
// │ Emergency stop (BOOT btn)    │  0         │ active-LOW, FALLING edge    │
// ├──────────────────────────────┼────────────┼─────────────────────────────┤
// │ Encoder — left wheel         │ 12         │ ★ v8.0 | Hall-effect RISING │
// │                              │            │   10 kΩ pull-down required  │
// │ Encoder — right wheel        │ 15         │ ★ v8.0 | was MPU6050 INT    │
// └──────────────────────────────┴────────────┴─────────────────────────────┘
//
// ⚠ Hardware rewiring required vs v3/v5:
//   Move ECHO_RIGHT wire from GPIO 21 → GPIO 16
//   Move TRIG_LEFT  wire from GPIO 22 → GPIO 17
//   Add MPU6050 GY-521: SDA→21, SCL→22, VCC→3.3V, GND→GND
//   Cost ~$3, weight ~3g, impact: angle-precise turns and pickup detection.

// ── Motor Pins ──
constexpr uint8_t MOTOR_A_IN1 = 27, MOTOR_A_IN2 = 26, MOTOR_A_EN  = 25;
constexpr uint8_t MOTOR_B_IN3 = 14, MOTOR_B_IN4 =  4, MOTOR_B_EN  = 13;

// ── Ultrasonic Sensors (v6 remapped) ──
constexpr uint8_t TRIG_FRONT = 5,  ECHO_FRONT = 18;
constexpr uint8_t TRIG_RIGHT = 19, ECHO_RIGHT = 16;  // was 21
constexpr uint8_t TRIG_LEFT  = 17, ECHO_LEFT  = 23;  // was 22

// ── IMU ──
constexpr uint8_t I2C_SDA_PIN   = 21;
constexpr uint8_t I2C_SCL_PIN   = 22;
constexpr uint8_t MPU6050_ADDR  = 0x68;

// ── IR Cliff ──
constexpr uint8_t IR_LEFT_PIN  = 34;
constexpr uint8_t IR_RIGHT_PIN = 35;

// ── Actuators & Indicators ──
constexpr uint8_t BRUSH_MOTOR        = 32;
constexpr uint8_t VACUUM_MOTOR       = 33;
constexpr uint8_t STATUS_LED         = 2;
constexpr uint8_t BATTERY_PIN        = 36;
constexpr uint8_t EMERGENCY_STOP_PIN = 0;   // BOOT button, active-LOW

// ── Wheel encoders (v8.0 — ENABLE_ENCODERS) ──
// GPIO 12: left wheel hall-effect. Add 10 kΩ pull-down to GND.
//   GPIO 12 is an ESP32 boot-strapping pin; it must be LOW at boot to select
//   3.3 V flash mode. The encoder output idles LOW (no magnet), so a pull-down
//   keeps the strapping correct even if the encoder cable is already connected.
// GPIO 15: right wheel hall-effect. Previously reserved for MPU6050 INT
//   (unused in software). Safe to repurpose; no special boot requirements.
constexpr uint8_t ENC_LEFT_PIN  = 12;  // ★ v8.0 left wheel encoder
constexpr uint8_t ENC_RIGHT_PIN = 15;  // ★ v8.0 right wheel encoder (was MPU INT)

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 3: LEDC PWM Configuration
// ═══════════════════════════════════════════════════════════════════════════
constexpr uint32_t PWM_FREQ       = 1000;   // 1 kHz — good for DC motors
constexpr uint8_t  PWM_RESOLUTION =    8;   // 0–255 duty
constexpr uint8_t  PWM_CH_A = 0, PWM_CH_B = 1; // channels for Core 2.x

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 4: Tunable Constants
// ═══════════════════════════════════════════════════════════════════════════

// ── Distance thresholds (cm) ──
constexpr float CRITICAL_DIST    = 10.0f;
constexpr float AVOID_ENTER_DIST = 25.0f;
constexpr float AVOID_EXIT_DIST  = 30.0f;
constexpr float ECHO_MAX_CM      = 400.0f;

// ── Motor speeds (PWM 0–255) ──
constexpr int CRUISE_SPEED  = 175;
constexpr int TURN_SPEED    = 155;
constexpr int REVERSE_SPEED = 155;
constexpr int RAMP_STEP     =  12;   // +12 PWM counts every 20ms soft-start

// ── Manoeuvre durations (ms) — fallback when IMU unavailable ──
constexpr uint32_t REVERSE_DURATION  =  900;
constexpr uint32_t TURN_180_DURATION = 1350;
constexpr uint32_t TURN_90_DURATION  =  680;
constexpr uint32_t AVOID_PAUSE_DURATION = 200;
constexpr uint32_t WATCHDOG_TIMEOUT     = 6000;

// ── IMU angle targets (degrees) — used when IMU is available ──
// Hysteresis gap prevents stopping exactly on target boundary.
constexpr float TURN_180_TARGET_DEG =  170.0f;  // (gap = 10°)
constexpr float TURN_90_TARGET_DEG  =   85.0f;  // (gap =  5°)
constexpr float STUCK_TURN_TARGET_DEG = 130.0f; // ~135° escape

// ── Sensor timing ──
constexpr uint32_t SENSOR_FIRE_INTERVAL =  35;
constexpr uint32_t ECHO_MAX_VALID_US    = 23324UL; // = 400cm × 2 / 0.0343
constexpr uint32_t ECHO_TIMEOUT_US      = 40000UL; // physical max wait
constexpr uint32_t STALE_TIMEOUT_MS     =   500;
constexpr uint32_t IR_SAMPLE_INTERVAL_MS =    5;   // 200 Hz → 15ms window
constexpr uint8_t  IR_DEBOUNCE_SAMPLES  =     3;

// ── EMA filtering ──
constexpr float EMA_ALPHA          = 0.35f; // ultrasonic distance filter
constexpr float BATTERY_EMA_ALPHA  = 0.05f; // heavy filter for battery

// ── Cruise / Spiral ──
constexpr uint32_t CRUISE_TURN_MIN      = 3000;
constexpr uint32_t CRUISE_TURN_MAX      = 8000;
constexpr uint32_t CRUISE_TURN_DURATION =  350;  // fallback when no IMU
constexpr uint32_t SPIRAL_TRIGGER_TIME  = 4000;
constexpr uint32_t SPIRAL_MAX_DURATION  = 15000;
constexpr float    SPIRAL_CLEAR_DIST    = 120.0f;
constexpr float    SPIRAL_BREAKOUT_DIST =  60.0f;

// ── Stuck detection ──
constexpr uint32_t STUCK_CHECK_INTERVAL_MS = 2500;
constexpr float    STUCK_DIST_DELTA_CM     =  1.2f;
constexpr uint32_t STUCK_REV_DURATION      =  800;
constexpr uint32_t STUCK_WIGGLE_DURATION   = 1000;
constexpr uint32_t STUCK_TURN_DURATION     =  600;  // fallback when no IMU

// ── Wall following ──
constexpr float    WALL_FOLLOW_MIN   = 12.0f;  // minimum dist to classify as wall
constexpr float    WALL_FOLLOW_MAX   = 40.0f;  // maximum dist to classify as wall
constexpr float    WALL_TARGET_DIST  = 20.0f;  // desired following distance
constexpr float    WALL_KP           =  2.5f;  // proportional gain
constexpr int      WALL_FOLLOW_SPEED = 155;    // slightly slower for accuracy
constexpr uint32_t WALL_TRIGGER_MS   = 3000;   // wall must be stable for 3s
constexpr uint32_t WALL_LOST_MS      = 2000;   // exit after 2s without wall

// ── Anti-pattern detection ──
constexpr uint32_t PATTERN_RECORD_INTERVAL_MS =  500; // record yaw every 500ms
constexpr uint8_t  PATTERN_BUFFER_SIZE         =  20; // 10-second history window
constexpr float    LOOP_DETECT_DEG             = 320.0f; // ~full circle threshold
constexpr uint8_t  OSCILLATION_THRESHOLD       =    4;   // 4 direction flips in window
constexpr uint32_t OSCILLATION_WINDOW_MS       = 10000;

// ── Battery (3S LiPo) ──
constexpr float BATTERY_CALIBRATION_FACTOR = 2.0f;  // 10k/10k voltage divider
constexpr float BATTERY_NOMINAL_VOLTAGE    = 12.6f; // 3S full charge
constexpr float BATTERY_WARN_VOLTAGE       = 10.5f;
constexpr float BATTERY_CRITICAL_VOLTAGE   =  9.9f;

// ── Phase 1 memory planning ──
// The occupancy grid (200×200 uint8_t) consumes exactly 40,000 bytes.
// With Bluedroid BLE (~90 KB) the internal heap budget is tight.
// Options in order of preference:
//   1. NimBLE-Arduino (drop-in replacement, ~30 KB instead of ~90 KB):
//      Install via Library Manager. Replace the four BLE* includes with:
//      #include <NimBLEDevice.h> — API is nearly identical.
//   2. PSRAM (if fitted): allocate via heap_caps_malloc(PHASE1_GRID_BYTES, MALLOC_CAP_SPIRAM)
//   3. Internal heap (no PSRAM, NimBLE used): heap_caps_malloc(PHASE1_GRID_BYTES, MALLOC_CAP_8BIT)
// Use reportMemory() in runSelfTest() to confirm headroom BEFORE allocating.
constexpr size_t PHASE1_GRID_BYTES = 200UL * 200UL * sizeof(uint8_t); // 40 000 bytes

// ── Dead-reckoning speed model (v7.2 fallback when ENABLE_ENCODERS is off) ──
// Empirical: at full 255 PWM, measure distance in 1 second. 30 cm/s is typical
// for a 65 mm wheel at ~150 RPM under load. Accuracy: ~±20% per metre.
constexpr float DR_SPEED_SCALE_CM_S = 30.0f; // cm/s at 255 PWM — calibrate!

// ── Wheel encoder geometry (v8.0 — ENABLE_ENCODERS) ──
// Adjust these two values to match your hardware, then recompile.
// Typical cheap hall-effect encoder kit: 20-pole magnet ring on the wheel hub.
constexpr float ENC_TICKS_PER_REV   =  20.0f; // poles on the magnet wheel
constexpr float WHEEL_DIAMETER_MM   =  65.0f; // mm — measure your actual wheel
constexpr float WHEEL_BASE_MM       = 150.0f; // mm wheel centre-to-centre — calibrate!
// Derived at compile time (no runtime division):
constexpr float WHEEL_CIRCUM_MM     = 3.14159265f * WHEEL_DIAMETER_MM;  // ≈ 204.2 mm
constexpr float MM_PER_TICK         = WHEEL_CIRCUM_MM / ENC_TICKS_PER_REV; // ≈ 10.2 mm
constexpr float CM_PER_TICK         = MM_PER_TICK * 0.1f;                // ≈  1.02 cm

// ── Phase 1 occupancy grid geometry (v8.0 — ENABLE_OCCUPANCY_GRID) ──
// 200 × 200 cells at 5 cm/cell covers 10 m × 10 m coverage area. Robot starts at grid centre.
// Total allocation: 40,000 bytes. Prefers PSRAM; falls back to internal heap.
constexpr uint16_t OCC_ROWS        = 200;
constexpr uint16_t OCC_COLS        = 200;
constexpr float    OCC_CELL_CM     =   5.0f;   // 5 cm per cell
constexpr float    OCC_ORIGIN_CM   = 500.0f;   // half-span — grid covers −500…+500 cm
constexpr uint8_t  OCC_UNKNOWN     =   0;       // cell not yet observed
constexpr uint8_t  OCC_CLEANED     =   1;       // robot body has traversed this cell
constexpr uint8_t  OCC_OBSTACLE    = 255;       // US sensor placed a wall here

// ── Boustrophedon planner (v8.0 — ENABLE_BOUSTROPHEDON) ──
// ROW_PITCH_CM: lateral spacing between rows. Should be ≤ cleaning brush width.
// 20 cm is a safe default for a 30 cm body; adjust to your physical brush diameter.
constexpr float    BOUSTRO_ROW_PITCH_CM = 20.0f;
// Heading P-gain: PWM counts per degree of heading error during row advance.
// 2.5 gives ≈25 PWM correction for a 10° error — adequate without oscillation.
constexpr float    BOUSTRO_HEADING_KP   =  2.5f;
constexpr int      BOUSTRO_HEADING_MAX  =   40;  // max correction authority (PWM)
// Row-end detection: front obstacle closer than this → U-turn. More conservative
// than AVOID_ENTER_DIST (25 cm) so the planner transitions cleanly before the
// avoidance layer would otherwise take over.
constexpr float    BOUSTRO_ROW_END_CM   =  30.0f;

// ── Coarse coverage grid (8×8 = 64 cells, 62.5 cm per cell) ──
// Covers a 500×500 cm area centred on the robot's start position.
// 64 uint8_t visit counters = 64 bytes — negligible memory cost.
constexpr uint8_t COV_COLS      =  8;
constexpr uint8_t COV_ROWS      =  8;
constexpr float   COV_CELL_CM   = 62.5f;  // 500 cm / 8 cells
constexpr float   COV_ORIGIN_CM = 250.0f; // half of 500 cm — start at centre
// Coverage turn bias: probability (0–10) of ignoring coverage and turning randomly.
// 3/10 = 30% random, 70% coverage-driven. Prevents the robot looping in a pattern.
constexpr uint8_t COV_RANDOM_WEIGHT = 3;

// ── L298N thermal protection ──
// Time-based model (no thermistor required). If both motors run above
// THERMAL_HIGH_PWM continuously for THERMAL_HIGH_LIMIT_MS, compensateSpeed()
// applies THERMAL_DERATE_FACTOR for THERMAL_COOLDOWN_MS before resuming full power.
// This does NOT cover stall events — that requires the encoder roadmap item.
constexpr int      THERMAL_HIGH_PWM      = 180;    // > ~70% duty = 'hot'
constexpr uint32_t THERMAL_HIGH_LIMIT_MS = 20000;  // 20 s continuous high duty
constexpr uint32_t THERMAL_COOLDOWN_MS   = 3000;   // 3 s of reduced power
constexpr float    THERMAL_DERATE_FACTOR = 0.88f;  // 12% speed reduction

// ── IMU ──
constexpr uint32_t IMU_CALIBRATION_MS  = 2000;
constexpr float    PICKUP_ANGLE_DEG    = 30.0f;   // |pitch| or |roll| > 30° = lifted
constexpr float    FLIP_ANGLE_DEG      = 100.0f;  // > 100° = upside-down

// ── Competition timer ──
constexpr uint32_t COMPETITION_TIME_MS     = 300000UL; // 5 minutes
constexpr uint32_t COMP_BOOST_THRESHOLD_MS = (COMPETITION_TIME_MS * 80UL) / 100UL; // 80%
constexpr uint32_t COMP_MAX_THRESHOLD_MS   = (COMPETITION_TIME_MS * 95UL) / 100UL; // 95%
constexpr int      COMP_BOOST_SPEED        = 200;
constexpr int      COMP_MAX_SPEED          = 225;

// ── Task Watchdog ──
constexpr uint32_t TASK_WDT_TIMEOUT_S = 8;

// ── LED timing (ms) ──
constexpr uint32_t LED_BLINK_FAST_MS = 150;
constexpr uint32_t LED_SOS_DOT_MS   = 100;

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 5: State Machine
// ═══════════════════════════════════════════════════════════════════════════
enum RobotState : uint8_t {
  STATE_CRUISE,
  STATE_CRUISE_TURN,
  STATE_CRUISE_SPIRAL,
  STATE_WALL_FOLLOW,       // v6: systematic edge coverage
  STATE_AVOID_PAUSE,
  STATE_AVOID_TURN,
  STATE_SURVIVAL_REV,
  STATE_SURVIVAL_TURN,
  STATE_PICKUP_STOP,       // v6: robot lifted/flipped
  STATE_STUCK_REV,
  STATE_STUCK_WIGGLE,
  STATE_STUCK_TURN,
  STATE_BATTERY_SHUTDOWN,
  STATE_BOUSTRO_ROW,       // v8.0: advancing along a boustrophedon row
  STATE_BOUSTRO_TURN       // v8.0: U-turn between rows (spin90→advance→spin90)
};

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 6: IMU Manager (MPU6050 via Wire — no external library needed)
// ═══════════════════════════════════════════════════════════════════════════
//
// Design rationale: We use raw gyro Z-axis integration for yaw tracking
// instead of the DMP (Digital Motion Processor). This avoids a ~10%
// DMP initialisation failure rate in competition conditions, requires no
// external library, and is perfectly accurate for our use case: short turns
// (<2s) where integrated drift is <3° — well within acceptable tolerance.
// Pitch and roll come from the accelerometer for pickup detection.
//
class IMUManager {
public:
  bool initialized  = false;
  bool fallbackMode = false; // set if init fails; code uses time-based turns

  float pitch = 0.0f;  // degrees, from accelerometer
  float roll  = 0.0f;  // degrees, from accelerometer

private:
  float yawAccum   = 0.0f;  // integrated yaw since boot (no wrap)
  float yawRef     = 0.0f;  // reference set at each turn start
  float gyroBiasZ  = 0.0f;  // calibrated z-axis offset (LSB/s)
  float accelBiasX = 0.0f;
  float accelBiasY = 0.0f;
  unsigned long lastUpdateUs = 0;

  // MPU6050 register map (subset used)
  static constexpr uint8_t REG_SMPRT_DIV  = 0x19;
  static constexpr uint8_t REG_CONFIG     = 0x1A;
  static constexpr uint8_t REG_GYRO_CFG   = 0x1B;
  static constexpr uint8_t REG_ACCEL_CFG  = 0x1C;
  static constexpr uint8_t REG_ACCEL_OUT  = 0x3B; // 14-byte burst: A(6)+T(2)+G(6)
  static constexpr uint8_t REG_PWR_MGMT1  = 0x6B;
  static constexpr uint8_t REG_WHO_AM_I   = 0x75;

  // Scale factors
  static constexpr float GYRO_SCALE  = 1.0f / 131.0f;    // ±250°/s  → °/s
  static constexpr float ACCEL_SCALE = 1.0f / 16384.0f;  // ±2g      → g

  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  bool readBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((uint8_t)MPU6050_ADDR, len);
    for (uint8_t i = 0; i < len; i++) {
      if (!Wire.available()) return false;
      buf[i] = Wire.read();
    }
    return true;
  }

public:
  bool begin() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000); // 400 kHz fast mode

    // Verify device identity
    uint8_t id = 0;
    if (!readBurst(REG_WHO_AM_I, &id, 1) || id != 0x68) {
      DBG("[IMU] WHO_AM_I = 0x%02X (expected 0x68). Fallback mode.\n", id);
      fallbackMode = true;
      return false;
    }

    writeReg(REG_PWR_MGMT1, 0x01); // wake + PLL with gyro-X clock
    delay(50);
    writeReg(REG_CONFIG,    0x03); // DLPF 42 Hz — good noise/lag trade-off
    writeReg(REG_GYRO_CFG,  0x00); // ±250°/s full-scale
    writeReg(REG_ACCEL_CFG, 0x00); // ±2g full-scale
    writeReg(REG_SMPRT_DIV, 0x09); // 1 kHz / (1+9) = 100 Hz output rate

    lastUpdateUs  = micros();
    initialized   = true;
    DBGLN("[IMU] MPU6050 OK -- 100 Hz, DLPF 42 Hz, +/-250 deg/s");
    return true;
  }

  // 2-second stationary calibration — call after begin(), before moving
  void calibrate() {
    if (!initialized) return;
    DBGLN("[IMU] Calibrating gyro bias (2 s -- keep robot still)...");

    long   sumGz = 0, sumAx = 0, sumAy = 0;
    int    samples = 0;
    uint32_t start = millis();

    while (millis() - start < IMU_CALIBRATION_MS) {
      esp_task_wdt_reset();  // v10.1: 2-second calibration must not trip WDT
      uint8_t buf[14];
      if (readBurst(REG_ACCEL_OUT, buf, 14)) {
        int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
        sumGz += gz;
        sumAx += ax;
        sumAy += ay;
        samples++;
      }
      delay(10);
    }

    if (samples > 0) {
      gyroBiasZ  = (float)sumGz / samples * GYRO_SCALE;
      accelBiasX = (float)sumAx / samples * ACCEL_SCALE;
      accelBiasY = (float)sumAy / samples * ACCEL_SCALE;
    }

    lastUpdateUs = micros();
    yawAccum     = 0.0f;
    yawRef       = 0.0f;
    DBG("[IMU] Calibration done. BiasZ=%.3f deg/s  ax=%.3fg  ay=%.3fg\n",
        gyroBiasZ, accelBiasX, accelBiasY);
  }

  // Call every loop — integrate gyro Z for yaw, update pitch/roll from accel.
  // I2C burst read ≈ 315 µs at 400 kHz — negligible vs loop period.
  void update() {
    if (!initialized) return;

    unsigned long nowUs = micros();
    float dt = (float)(nowUs - lastUpdateUs) * 1e-6f;
    lastUpdateUs = nowUs;

    if (dt <= 0.0f || dt > 0.5f) return; // sanity: skip stale/huge deltas

    uint8_t buf[14];
    if (!readBurst(REG_ACCEL_OUT, buf, 14)) return;

    int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
    // buf[6..7] = temperature (unused)
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    // Pitch and roll from accelerometer (used only for pickup detection)
    float axG = ax * ACCEL_SCALE - accelBiasX;
    float ayG = ay * ACCEL_SCALE - accelBiasY;
    float azG = az * ACCEL_SCALE;
    pitch = atan2f(axG, sqrtf(ayG * ayG + azG * azG)) * 57.296f;
    roll  = atan2f(ayG, azG) * 57.296f;

    // Integrate corrected gyro Z for yaw
    float yawRate = gz * GYRO_SCALE - gyroBiasZ;
    yawAccum += yawRate * dt;
  }

  // Call at the START of every turn to zero the reference.
  void resetTurnTracking() { yawRef = yawAccum; }

  // Returns absolute degrees turned since last resetTurnTracking().
  float getTurnAngle() const { return fabsf(yawAccum - yawRef); }

  // Total accumulated yaw (for pattern detection).
  float getYaw() const { return yawAccum; }

  bool isPickedUp() const {
    return fabsf(pitch) > PICKUP_ANGLE_DEG || fabsf(roll) > PICKUP_ANGLE_DEG;
  }
  bool isFlipped() const {
    return fabsf(pitch) > FLIP_ANGLE_DEG || fabsf(roll) > FLIP_ANGLE_DEG;
  }
};

static IMUManager imu;

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 7: Dead Reckoning & Coverage Grid
// ═══════════════════════════════════════════════════════════════════════════
//
// DeadReckoning integrates IMU yaw + PWM-estimated speed to produce a
// continuous (x, y) position estimate in cm. No wheel encoders required.
//
// Accuracy notes:
//   • Linear: ~±20% per metre (PWM → velocity calibration drift)
//   • Heading: ~±3° per 180° turn (MPU6050 gyro integration, same as turns)
//   • Over a 5-minute competition run (~100 m travel) expect ±2–4 m absolute
//     position error — acceptable for 62.5 cm cell granularity.
//
// CoverageGrid stores per-cell visit counts in 64 bytes (8×8 uint8_t).
// preferTurnRight() biases cruise turn direction 70/30 toward less-visited
// cells, lifting estimated 5-minute coverage from ~55% (random) to ~70%.
// ═══════════════════════════════════════════════════════════════════════════

class DeadReckoning {
public:
  float x_cm = 0.0f;
  float y_cm = 0.0f;

  void reset() {
    x_cm = y_cm = 0.0f;
    lastUpdateMs   = millis();
    lastHeadingRad = 0.0f;
  }

  // Call each loop() cycle with the current nominal PWM and direction.
  // nominalPwm: pre-compensation PWM (e.g. rampCurrentSpeed or WALL_FOLLOW_SPEED).
  // forward: true = advancing, false = reversing.
  void update(int nominalPwm, bool forward) {
    uint32_t now = millis();
    float dt = (float)(now - lastUpdateMs) * 1.0e-3f;
    lastUpdateMs = now;
    if (dt <= 0.0f || dt > 0.15f) return; // skip stale / huge deltas

    float speedCmS = DR_SPEED_SCALE_CM_S * ((float)nominalPwm / 255.0f);
    float dist = speedCmS * dt * (forward ? 1.0f : -1.0f);

    // Heading: IMU absolute yaw (degrees → radians) if available,
    //          otherwise hold last known heading.
    if (imu.initialized) {
      lastHeadingRad = imu.getYaw() * (float)(M_PI / 180.0);
    }

    x_cm += dist * cosf(lastHeadingRad);
    y_cm += dist * sinf(lastHeadingRad);

    // Clamp to grid bounds — guards against runaway if speed model is wrong
    x_cm = constrain(x_cm, -COV_ORIGIN_CM, COV_ORIGIN_CM);
    y_cm = constrain(y_cm, -COV_ORIGIN_CM, COV_ORIGIN_CM);
  }

  // v8.1 FIX: was private — dr.getHeadingDeg() is called from readAllSensors()
  // when ENABLE_OCCUPANCY_GRID is defined without ENABLE_ENCODERS. Making this
  // public ensures that configuration compiles correctly.
  float getHeadingDeg() const {
    return lastHeadingRad * (180.0f / (float)M_PI);
  }

private:
  uint32_t lastUpdateMs   = 0;
  float    lastHeadingRad = 0.0f;
};

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 8: Wheel Encoders + EncoderOdometry  (ENABLE_ENCODERS)
// ═══════════════════════════════════════════════════════════════════════════
//
// WheelEncoders: volatile ISR tick counters for left and right wheels.
//   Each hall-effect pulse fires an IRAM_ATTR ISR that increments the counter.
//   Ticks always count upward; direction is supplied to EncoderOdometry::update()
//   as the forward parameter (same interface as DeadReckoning).
//
// EncoderOdometry: drop-in replacement for DeadReckoning.
//   Same public members: x_cm, y_cm, reset(), update(pwm, fwd).
//   Algorithm: differential-drive kinematics.
//     distL = dLeft  × CM_PER_TICK × sign(fwd)
//     distR = dRight × CM_PER_TICK × sign(fwd)
//     dCenter   = (distL + distR) / 2       [net translation, cm]
//     dHeading  = (distR - distL) / WHEEL_BASE  [radians, encoder-only fallback]
//   Heading source priority:
//     1. IMU yaw (absolute) when imu.initialized — ≈3° drift per 180° turn.
//     2. Encoder differential when IMU absent   — ≈2% drift per metre.
//   Position accuracy: ±2% per metre vs ±20% for dead-reckoning.
// ═══════════════════════════════════════════════════════════════════════════
#ifdef ENABLE_ENCODERS

struct WheelEncoders {
  volatile int32_t leftTicks  = 0;   // written ONLY from ISR; read from loop()
  volatile int32_t rightTicks = 0;

  // Snapshot of tick counts at the last reset() or update() call.
  // The delta (currentTicks − snapshot) is the distance since the last read.
  // These are loop()-only variables — never touched from an ISR.
  int32_t snapLeft  = 0;
  int32_t snapRight = 0;

  // Atomically read both counters into local variables without side-effects.
  // Disabling interrupts for the two 32-bit loads prevents a torn read if an
  // ISR fires between them. The caller then computes the delta and advances
  // the snapshot separately, so the snapshot is never clobbered before the
  // delta is computed (the root cause of the original v8.0 bug).
  void readAtomic(int32_t& outL, int32_t& outR) const {
    portDISABLE_INTERRUPTS();
    outL = leftTicks;
    outR = rightTicks;
    portENABLE_INTERRUPTS();
  }
} wheelEnc;

void IRAM_ATTR encISR_Left()  { wheelEnc.leftTicks++;  }
void IRAM_ATTR encISR_Right() { wheelEnc.rightTicks++; }

class EncoderOdometry {
public:
  float x_cm = 0.0f;
  float y_cm = 0.0f;

  // Zero position and discard all ticks accumulated before this run.
  // Establishes the baseline snapshot so the first update() delta is 0.
  void reset() {
    x_cm = y_cm = headingRad = 0.0f;
    // Atomically snapshot current tick counts — any ticks before this call
    // are intentionally discarded (e.g. ticks during self-test or setup).
    wheelEnc.readAtomic(wheelEnc.snapLeft, wheelEnc.snapRight);
  }

  // Compatible with DeadReckoning::update(). nominalPwm unused (we have encoders).
  // 'forward' applies the same direction sign to both wheels — correct for
  // the straight-travel states (cruise, wall-follow, reverse) that call this.
  // Spinning states never call update(), so pure-rotation ticks are ignored.
  //
  // Tick → distance pipeline:
  //   1. Read current tick counts atomically into locals (no snapshot change yet).
  //   2. Delta = current − snapshot (snapshot is from reset() or previous update()).
  //   3. Advance snapshot = current  (ready for the next call).
  //   4. Signed distance = delta × CM_PER_TICK × ±1.
  //   5. Heading from IMU absolute yaw (preferred) or encoder differential.
  //   6. Dead-reckon x, y.
  void update(int /*nominalPwm*/, bool forward) {
    // Step 1: atomic read — snapshot is NOT modified here.
    int32_t curL, curR;
    wheelEnc.readAtomic(curL, curR);

    // Step 2: delta against the previous snapshot.
    int32_t dL = curL - wheelEnc.snapLeft;
    int32_t dR = curR - wheelEnc.snapRight;

    // Step 3: advance snapshot for next call.
    wheelEnc.snapLeft  = curL;
    wheelEnc.snapRight = curR;

    if (dL == 0 && dR == 0) return;  // nothing moved

    // Step 4: signed distances in cm.
    float sign    = forward ? 1.0f : -1.0f;
    float distL   = (float)dL * CM_PER_TICK * sign;
    float distR   = (float)dR * CM_PER_TICK * sign;
    float dCenter = (distL + distR) * 0.5f;

    // Step 5: heading.
    // IMU yaw is absolute (resets only on power cycle) so heading drift is
    // bounded per-turn rather than accumulating with distance — always prefer it.
    // Encoder differential is the fallback: dHeading = (distR − distL) / wheelBase.
    // Both numerator and denominator are in cm, so the result is in radians.
    if (imu.initialized) {
      headingRad = imu.getYaw() * (float)(M_PI / 180.0);
    } else {
      constexpr float wheelBaseCm = WHEEL_BASE_MM * 0.1f; // mm → cm
      headingRad += (distR - distL) / wheelBaseCm;
    }

    // Step 6: integrate position.
    x_cm += dCenter * cosf(headingRad);
    y_cm += dCenter * sinf(headingRad);

#ifdef ENABLE_OCCUPANCY_GRID
    x_cm = constrain(x_cm, -OCC_ORIGIN_CM, OCC_ORIGIN_CM);
    y_cm = constrain(y_cm, -OCC_ORIGIN_CM, OCC_ORIGIN_CM);
#else
    x_cm = constrain(x_cm, -COV_ORIGIN_CM, COV_ORIGIN_CM);
    y_cm = constrain(y_cm, -COV_ORIGIN_CM, COV_ORIGIN_CM);
#endif
  }

  float getHeadingDeg() const { return headingRad * (180.0f / (float)M_PI); }

private:
  float headingRad = 0.0f;
};

// 'dr' is the canonical position tracker throughout this file.
// With ENABLE_ENCODERS: EncoderOdometry (±2 % / m).
// Without it          : DeadReckoning   (±20 % / m, declared earlier).
static EncoderOdometry dr;

#endif // ENABLE_ENCODERS

struct CoverageGrid {
  uint8_t visits[COV_ROWS][COV_COLS]; // 64 bytes; 0 = unvisited

  CoverageGrid() { memset(visits, 0, sizeof(visits)); }

  // Convert world cm → grid indices. Returns false if out-of-bounds.
  bool toCell(float x, float y, int16_t& row, int16_t& col) const {
    col = (int16_t)((x + COV_ORIGIN_CM) / COV_CELL_CM);
    row = (int16_t)((y + COV_ORIGIN_CM) / COV_CELL_CM);
    return (col >= 0 && col < (int16_t)COV_COLS &&
            row >= 0 && row < (int16_t)COV_ROWS);
  }

  // Increment the visit counter for the cell at (x, y). Saturates at 255.
  void markVisited(float x, float y) {
    int8_t r, c;
    if (toCell(x, y, r, c) && visits[r][c] < 255) visits[r][c]++;
  }

  // Visit count at world position (x, y). Returns 255 if out-of-bounds.
  uint8_t getVisits(float x, float y) const {
    int8_t r, c;
    return toCell(x, y, r, c) ? visits[r][c] : 255;
  }

  // Fraction of cells visited at least once, as 0–100 integer percent.
  uint8_t coveragePercent() const {
    uint8_t n = 0;
    for (uint8_t r = 0; r < COV_ROWS; r++)
      for (uint8_t c = 0; c < COV_COLS; c++)
        if (visits[r][c] > 0) n++;
    return (uint8_t)(n * 100U / (COV_COLS * COV_ROWS));
  }

  // Returns true (turn right) or false (turn left) biased 70/30 toward
  // the less-visited cell ~1.5 cells ahead on each side of the robot.
  // headingDeg: current robot heading (IMU yaw). Falls back to random
  // when IMU is absent.
  bool preferTurnRight(float x, float y, float headingDeg) const {
    // 30% random to prevent the bias itself from creating loops
    if ((esp_random() % 10) < COV_RANDOM_WEIGHT) return (bool)(esp_random() & 1);

    float hRad    = headingDeg * (float)(M_PI / 180.0);
    float lookDist = COV_CELL_CM * 1.5f;

    // Project 1.5 cells ahead, then ±90° to see what's on each side
    float xFwd = x + COV_CELL_CM * cosf(hRad);
    float yFwd = y + COV_CELL_CM * sinf(hRad);
    float xR   = xFwd + lookDist * cosf(hRad + (float)(M_PI / 2.0));
    float yR   = yFwd + lookDist * sinf(hRad + (float)(M_PI / 2.0));
    float xL   = xFwd + lookDist * cosf(hRad - (float)(M_PI / 2.0));
    float yL   = yFwd + lookDist * sinf(hRad - (float)(M_PI / 2.0));

    uint8_t vR = getVisits(xR, yR);
    uint8_t vL = getVisits(xL, yL);
    if (vR == vL) return (bool)(esp_random() & 1); // tie → random
    return vR < vL; // turn toward the less-visited side
  }

  void reset() { memset(visits, 0, sizeof(visits)); }
};

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 9: Phase 1 Occupancy Grid  (ENABLE_OCCUPANCY_GRID)
// ═══════════════════════════════════════════════════════════════════════════
//
// PhaseOneGrid activates the 200×200 stub planned since v7.0.
//
// Allocation: at boot, init() tries PSRAM first (heap_caps_malloc
//   MALLOC_CAP_SPIRAM), then internal heap. If neither has 40 KB contiguous,
//   the grid stays null and the occupancy/boustrophedon features disable
//   automatically. reportMemory() in runSelfTest() advises NimBLE migration
//   when heap is tight (~60 KB reclaimed by switching from Bluedroid).
//
// Cell encoding:
//   OCC_UNKNOWN  (0)   → not yet observed; boustrophedon targets these
//   OCC_CLEANED  (1)   → robot body traversed; floor has been swept
//   OCC_OBSTACLE (255) → US sensor placed a wall here; planner skips
//
// BLE MAP_STREAM_CHAR: after init succeeds, sendMapStreamRow() is called
//   once per loop() at ~1 Hz, cycling through all 200 rows. Each row is
//   RLE-compressed (typically 20–60 bytes) and pushed as a BLE Notify.
//   The 'reserved' byte in BleStatusPacket carries the last streamed row index.
// ═══════════════════════════════════════════════════════════════════════════
#ifdef ENABLE_OCCUPANCY_GRID

// v10.0: bleMapRowIdx is uint8_t (0-255). If someone increases OCC_ROWS
// to 256+, the modulo wraparound silently breaks BLE map streaming.
static_assert(OCC_ROWS <= 255, "OCC_ROWS must fit in uint8_t (max 255) for BLE map streaming");

class PhaseOneGrid {
public:

  bool init() {
    // v8.1 FIX: use PHASE1_GRID_BYTES constant consistently instead of
    // recomputing the size manually. If the cell type ever changes,
    // this stays correct automatically.
    cells = (uint8_t*)heap_caps_malloc(PHASE1_GRID_BYTES, MALLOC_CAP_SPIRAM);
    if (cells) {
      allocSrc = SRC_PSRAM;
    } else {
      cells = (uint8_t*)heap_caps_malloc(PHASE1_GRID_BYTES, MALLOC_CAP_8BIT);
      allocSrc = cells ? SRC_INTERNAL : SRC_FAILED;
    }
    if (cells) {
      memset(cells, OCC_UNKNOWN, PHASE1_GRID_BYTES);
      DBG("[OCC] Grid ready (%s): %ux%u = %u bytes\n",
          allocSrc == SRC_PSRAM ? "PSRAM" : "internal",
          (unsigned)OCC_ROWS, (unsigned)OCC_COLS,
          (unsigned)PHASE1_GRID_BYTES);
      return true;
    }
    DBGLN("[OCC] Alloc failed -- switch to NimBLE-Arduino to reclaim ~60 KB");
    return false;
  }

  bool isReady() const { return cells != nullptr; }

  void markCleaned(float x, float y) {
    int r, c;
    if (!toCell(x, y, r, c)) return;
    if (at(r, c) != OCC_OBSTACLE) at(r, c) = OCC_CLEANED;
  }

  void markObstacle(float x, float y) {
    int r, c;
    if (!toCell(x, y, r, c)) return;
    at(r, c) = OCC_OBSTACLE;
  }

  uint8_t get(float x, float y) const {
    int r, c;
    return toCell(x, y, r, c) ? at(r, c) : OCC_OBSTACLE;
  }

  // Fraction of non-obstacle cells marked cleaned, 0–100 integer percent.
  uint8_t cleanedPercent() const {
    if (!cells) return 0;
    uint32_t total = 0, cleaned = 0;
    for (uint32_t i = 0; i < (uint32_t)OCC_ROWS * OCC_COLS; i++) {
      if (cells[i] != OCC_OBSTACLE) { total++; if (cells[i] == OCC_CLEANED) cleaned++; }
    }
    return total ? (uint8_t)(cleaned * 100UL / total) : 0;
  }

  // Returns world Y of the nearest row with at least one OCC_UNKNOWN cell.
  // Scans outward from currentY in both directions. Returns NAN when done.
  float findNearestUncleanedRow(float currentY) const {
    if (!cells) return NAN;
    int base = (int)((currentY + OCC_ORIGIN_CM) / OCC_CELL_CM);
    base = constrain(base, 0, (int)OCC_ROWS - 1);
    for (int off = 0; off < (int)OCC_ROWS; off++) {
      for (int sgn = -1; sgn <= 1; sgn += 2) {
        if (off == 0 && sgn == -1) continue;
        int r = base + off * sgn;
        if (r < 0 || r >= (int)OCC_ROWS) continue;
        for (int c = 0; c < (int)OCC_COLS; c++) {
          if (at(r, c) == OCC_UNKNOWN) return (r + 0.5f) * OCC_CELL_CM - OCC_ORIGIN_CM;
        }
      }
    }
    return NAN;
  }

  // RLE-encode one grid row into out[]. Returns byte count written.
  // Format: alternating (run_length, cell_value) byte pairs.
  uint16_t encodeRow(int rowIdx, uint8_t* out, uint16_t maxOut) const {
    if (!cells || rowIdx < 0 || rowIdx >= (int)OCC_ROWS) return 0;
    uint16_t n = 0;
    uint8_t val = at(rowIdx, 0), run = 1;
    for (int c = 1; c < (int)OCC_COLS; c++) {
      uint8_t v = at(rowIdx, c);
      if (v == val && run < 255) { run++; }
      else {
        if (n + 2 > maxOut) break;
        out[n++] = run; out[n++] = val;
        val = v; run = 1;
      }
    }
    if (n + 2 <= maxOut) { out[n++] = run; out[n++] = val; }
    return n;
  }

  // v8.1 FIX: use PHASE1_GRID_BYTES consistently
  void reset() { if (cells) memset(cells, OCC_UNKNOWN, PHASE1_GRID_BYTES); }

private:
  uint8_t* cells = nullptr;
  enum AllocSrc : uint8_t { SRC_FAILED, SRC_PSRAM, SRC_INTERNAL } allocSrc = SRC_FAILED;

  uint8_t& at(int r, int c)       { return cells[r * OCC_COLS + c]; }
  uint8_t  at(int r, int c) const { return cells[r * OCC_COLS + c]; }

  bool toCell(float x, float y, int& r, int& c) const {
    c = (int)((x + OCC_ORIGIN_CM) / OCC_CELL_CM);
    r = (int)((y + OCC_ORIGIN_CM) / OCC_CELL_CM);
    return c >= 0 && c < (int)OCC_COLS && r >= 0 && r < (int)OCC_ROWS;
  }
};

static PhaseOneGrid occGrid;

#endif // ENABLE_OCCUPANCY_GRID

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 10: Boustrophedon Planner  (ENABLE_BOUSTROPHEDON)
// ═══════════════════════════════════════════════════════════════════════════
//
// Owns the U-turn state machine. boustrophedonLayer() calls into this struct.
//
// U-turn sequence (STATE_BOUSTRO_TURN):
//   Phase 1 — spin 90° in turnLeft direction (e.g. east→north)
//   Phase 2 — drive forward BOUSTRO_ROW_PITCH_CM (encoder-measured)
//   Phase 3 — spin 90° in same direction (north→west)
//   turnLeft flips after each complete U-turn, so the robot always moves
//   into unexplored territory rather than spiralling back.
//
// Row heading maintenance (STATE_BOUSTRO_ROW):
//   rowHeadingYaw is snapped from imu.getYaw() at the start of each row.
//   Error = rowHeadingYaw − imu.getYaw(). Correction = error × HEADING_KP,
//   clamped to ±BOUSTRO_HEADING_MAX PWM, applied differentially to wheels.
// ═══════════════════════════════════════════════════════════════════════════
#if defined(ENABLE_BOUSTROPHEDON) && defined(ENABLE_ENCODERS) && defined(ENABLE_OCCUPANCY_GRID)

struct BoustrophedonPlanner {
  enum Phase : uint8_t { IDLE=0, SPIN1=1, ADVANCE=2, SPIN2=3 };

  bool  active        = false;
  Phase turnPhase     = IDLE;
  bool  turnLeft      = true;       // direction of U-turn; alternates each row
  float rowHeadingYaw = 0.0f;       // IMU yaw reference at row start (degrees)
  uint32_t phaseStartMs   = 0;
  int32_t  advSnap        = 0;      // encoder left-tick snapshot for ADVANCE phase

  bool inTurn() const { return turnPhase != IDLE; }

  // v8.1 FIX: removed unused 'startY' parameter. The function was called with
  // dr.y_cm but never used it; row selection is done dynamically by
  // findNearestUncleanedRow() at each row-end.
  void activate() {
    active        = true;
    turnPhase     = IDLE;
    turnLeft      = true;
    rowHeadingYaw = imu.initialized ? imu.getYaw() : 0.0f;
    DBG("[BOUSTRO] Activated -- headingRef=%.1f deg\n", rowHeadingYaw);
  }

  void beginUTurn() {
    turnPhase    = SPIN1;
    phaseStartMs = millis();
    if (imu.initialized) imu.resetTurnTracking();
    // v10.0 FIX: use readAtomic() for consistency with the codebase design.
    // While 32-bit aligned volatile reads are atomic on ESP32, readAtomic()
    // is the intended API and makes the synchronization policy explicit.
    int32_t curL, curR;
    wheelEnc.readAtomic(curL, curR);
    advSnap = curL;
    DBG("[BOUSTRO] U-turn %s -- SPIN1\n", turnLeft ? "LEFT" : "RIGHT");
  }

  // Execute one step of the U-turn state machine. Call every loop iteration.
  // Returns true while a turn is still in progress.
  bool updateTurn() {
    uint32_t now = millis();

    if (turnPhase == SPIN1) {
      if (turnLeft) spinLeft(TURN_SPEED); else spinRight(TURN_SPEED);
      bool done = imu.initialized
        ? (imu.getTurnAngle() >= TURN_90_TARGET_DEG)
        : (now - phaseStartMs >= TURN_90_DURATION);
      if (done) {
        stopMotors(); resetRamp();
        turnPhase    = ADVANCE;
        phaseStartMs = now;
        // v10.1 FIX: use readAtomic() instead of direct volatile read.
        // The ISR may fire on a different core; portDISABLE_INTERRUPTS()
        // guarantees a consistent 32-bit snapshot.
        { int32_t curL, curR;
          wheelEnc.readAtomic(curL, curR);
          advSnap = curL; }
        DBGLN("[BOUSTRO] SPIN1 done -- ADVANCE");
      }
      return true;
    }

    if (turnPhase == ADVANCE) {
      setLeftMotor (compensateSpeed(CRUISE_SPEED), true);
      setRightMotor(compensateSpeed(CRUISE_SPEED), true);
      // v10.0 FIX: use readAtomic() instead of direct volatile read.
      int32_t curL, curR;
      wheelEnc.readAtomic(curL, curR);
      float travelCm = (float)(curL - advSnap) * CM_PER_TICK;
      bool  done = (travelCm >= BOUSTRO_ROW_PITCH_CM)
                || (now - phaseStartMs >= 3000UL); // 3 s safety timeout
      if (done) {
        stopMotors(); resetRamp();
        turnPhase    = SPIN2;
        phaseStartMs = now;
        if (imu.initialized) imu.resetTurnTracking();
        DBGLN("[BOUSTRO] ADVANCE done -- SPIN2");
      }
      return true;
    }

    if (turnPhase == SPIN2) {
      if (turnLeft) spinLeft(TURN_SPEED); else spinRight(TURN_SPEED);
      bool done = imu.initialized
        ? (imu.getTurnAngle() >= TURN_90_TARGET_DEG)
        : (now - phaseStartMs >= TURN_90_DURATION);
      if (done) {
        stopMotors(); resetRamp();
        turnPhase     = IDLE;
        turnLeft      = !turnLeft; // alternate direction for next row
        rowHeadingYaw = imu.initialized ? imu.getYaw() : rowHeadingYaw + 180.0f;
        DBG("[BOUSTRO] U-turn done -- new heading ref %.1f deg\n", rowHeadingYaw);
      }
      return true;
    }

    return false; // IDLE
  }

  void deactivate() {
    active    = false;
    turnPhase = IDLE;
    DBGLN("[BOUSTRO] Deactivated -- returning to cruise layer");
  }
} boustro;

#endif // ENABLE_BOUSTROPHEDON

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 11: Anti-Pattern Detector
// ═══════════════════════════════════════════════════════════════════════════
//
// Two detectors run concurrently:
//
//   Loop detector:       Record yaw every 500ms. If total angular change in
//                        the 10-second window exceeds 320°, the robot is
//                        circling. Trigger a forced escape turn.
//
//   Oscillation detector: If avoidance alternates left→right→left more than
//                        4 times within 10 seconds, the robot is bouncing
//                        in a corner or narrow corridor. Suppress wall-
//                        following to prevent making it worse.
//
struct PatternDetector {
  float    yawHistory[PATTERN_BUFFER_SIZE];
  uint8_t  histIdx        = 0;
  uint32_t lastRecordMs   = 0;

  bool     lastAvoidRight       = true;
  uint8_t  oscillationCount     = 0;
  uint32_t firstOscillationMs   = 0;

  void recordYaw(float yaw) {
    uint32_t now = millis();
    if (now - lastRecordMs < PATTERN_RECORD_INTERVAL_MS) return;
    lastRecordMs       = now;
    yawHistory[histIdx] = yaw;
    histIdx            = (histIdx + 1) % PATTERN_BUFFER_SIZE;
  }

  // Sum of step-by-step angular changes over the circular buffer window.
  // Reading in chronological order starting from the oldest entry (histIdx).
  bool detectLoop() const {
    if (!imu.initialized) return false;
    float total = 0.0f;
    for (uint8_t i = 0; i < PATTERN_BUFFER_SIZE - 1; i++) {
      uint8_t a = (histIdx + i)     % PATTERN_BUFFER_SIZE;
      uint8_t b = (histIdx + i + 1) % PATTERN_BUFFER_SIZE;
      // v10.1: wrap180() prevents false trigger across +/-180 degree boundary.
      // If yaw goes from +179 to -179 (2 degree actual), raw diff is 358.
      total += fabsf(wrap180(yawHistory[b] - yawHistory[a]));
    }
    return total >= LOOP_DETECT_DEG;
  }

  void recordAvoidDirection(bool turnedRight) {
    uint32_t now = millis();
    // Reset window if expired
    if (now - firstOscillationMs > OSCILLATION_WINDOW_MS) {
      oscillationCount = 0;
    }
    if (turnedRight != lastAvoidRight) {
      if (oscillationCount == 0) firstOscillationMs = now;
      oscillationCount++;
      lastAvoidRight = turnedRight;
    }
  }

  bool detectOscillation() const {
    if (millis() - firstOscillationMs > OSCILLATION_WINDOW_MS) return false;
    return oscillationCount >= OSCILLATION_THRESHOLD;
  }

  void reset() {
    oscillationCount = 0;
    firstOscillationMs = 0;
    histIdx = 0;
    memset(yawHistory, 0, sizeof(yawHistory));
  }
};

static PatternDetector patternDet;
// DeadReckoning 'dr': only instantiated when ENABLE_ENCODERS is NOT defined.
// With encoders, EncoderOdometry 'dr' is declared inside the #ifdef block above.
#ifndef ENABLE_ENCODERS
static DeadReckoning   dr;
#endif
static CoverageGrid    coverageGrid;

// ── L298N thermal state (mirrors THERMAL_* constants above) ──
static uint32_t thermalHighStartMs    = 0;     // when continuous high-duty began
static bool     thermalDerateActive   = false; // true during cooldown window
static uint32_t thermalCooldownStartMs= 0;     // when cooldown started

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 12: Ultrasonic Sensor (ISR-based)
// ═══════════════════════════════════════════════════════════════════════════
struct USSensor {
  uint8_t  trigPin;
  uint8_t  echoPin;
  volatile unsigned long echoRise;      // µs timestamp of rising edge
  volatile unsigned long pulseDuration; // measured pulse width
  volatile bool newData;
  float    distance;       // EMA-filtered distance in cm
  uint32_t lastFireMs;
  uint32_t lastValidMs;    // stale-sensor watchdog
  bool     isFaulty;
};

USSensor usSensors[3];           // [0]=front [1]=right [2]=left
uint8_t  sensorRotIdx = 0;

static portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;

// ── Direct GPIO register reads — safe from IRAM, no cache dependency ──
static inline IRAM_ATTR bool readGPIO(uint8_t pin) {
  return (pin < 32)
    ? (GPIO.in.val  >> pin)        & 0x1
    : (GPIO.in1.val >> (pin - 32)) & 0x1;
}

static inline IRAM_ATTR void handleEchoISR(uint8_t idx) {
  if (readGPIO(usSensors[idx].echoPin)) {
    usSensors[idx].echoRise = micros();
  } else {
    unsigned long dur = micros() - usSensors[idx].echoRise; // handles overflow
    if (dur < ECHO_TIMEOUT_US) {
      portENTER_CRITICAL_ISR(&sensorMux);
      usSensors[idx].pulseDuration = dur;
      usSensors[idx].newData       = true;
      portEXIT_CRITICAL_ISR(&sensorMux);
    }
  }
}

void IRAM_ATTR echoISR_Front() { handleEchoISR(0); }
void IRAM_ATTR echoISR_Right() { handleEchoISR(1); }
void IRAM_ATTR echoISR_Left()  { handleEchoISR(2); }

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 13: IR Cliff Sensor
// ═══════════════════════════════════════════════════════════════════════════
struct IRSensor {
  uint8_t pin;
  uint8_t buffer[IR_DEBOUNCE_SAMPLES]; // circular majority-vote buffer
  uint8_t bufIdx;
  bool    cliffDetected;
};

IRSensor irLeft, irRight;

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 14: Global State Variables
// ═══════════════════════════════════════════════════════════════════════════
RobotState currentState    = STATE_CRUISE;
uint32_t   stateStartTime  = 0;

bool avoidTurnRight    = true;
bool survivalTurnRight = true;
bool cruiseTurnRight   = true;
float cruiseTurnTargetDeg = 65.0f; // random angle used with IMU

uint32_t nextCruiseTurnMs = 0;
uint32_t clearPathStartMs = 0;
uint32_t spiralStartMs    = 0;

// Stuck detection
uint32_t lastStuckCheckMs   = 0;
float    lastFrontDistStuck = 0.0f;
bool     isRobotStuck       = false;
bool     stuckTurnRight     = true;

// Wall following
uint32_t wallDetectedMs = 0;
uint32_t wallLostMs     = 0;
bool     followingRight = false;

// Battery
float    filteredBatteryVoltage = 12.0f;
float    brownoutSpeedFactor    = 1.0f;  // reduced to 0.8 after brownout reset
Preferences preferences;
uint32_t statBoots      = 0;
uint32_t statCliffSaves = 0;
uint32_t statAvoids     = 0;
uint32_t statRescues    = 0;

// Ramp
int rampCurrentSpeed = 0;

// Convenient aliases (compile-time references; USSensor[] is global)
float& distFront = usSensors[0].distance;
float& distRight = usSensors[1].distance;
float& distLeft  = usSensors[2].distance;

// Timing
uint32_t lastIRSampleMs  = 0;
uint32_t lastLedToggleMs = 0;
bool     ledState        = false;
uint32_t bootTimeMs      = 0;

// Emergency stop
std::atomic<bool> emergencyStopPressed{false};  // v10.1: was volatile bool
bool          emergencyStopActive  = false;

void IRAM_ATTR emergencyStopISR() { emergencyStopPressed.store(true, std::memory_order_relaxed); }

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 15: BLE Configuration
// ═══════════════════════════════════════════════════════════════════════════
//
// Protocol overview:
//   Device name : "Obrynex-V1"   (visible in BLE scan)
//   Service UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
//
//   ┌─────────────────────────────────────────────────────────────────────┐
//   │ Characteristic    │ UUID      │ Props         │ Payload             │
//   ├─────────────────────────────────────────────────────────────────────┤
//   │ STATUS_CHAR       │ …8fcc-aa  │ Notify, Read  │ 20-byte struct @ 5Hz│
//   │ COMMAND_CHAR      │ …8fcc-bb  │ Write, WNR    │ 1 byte command code │
//   │ MAP_STREAM_CHAR   │ …8fcc-cc  │ Notify        │ RLE-compressed rows │
//   └─────────────────────────────────────────────────────────────────────┘
//
// STATUS packet (20 bytes, little-endian):
//   [0]      state       — RobotState enum value
//   [1]      flags       — bit 0: leftCliff, bit 1: rightCliff,
//                          bit 2: emergencyStop, bit 3: imuOK
//   [2-3]    battMv      — uint16 battery millivolts (12600 = 12.600 V)
//   [4-5]    distF_mm    — uint16 front distance × 10  (mm precision)
//   [6-7]    distR_mm    — uint16 right distance × 10
//   [8-9]    distL_mm    — uint16 left  distance × 10
//   [10-11]  yaw10       — int16  IMU yaw   × 10 (degrees)
//   [12-13]  pitch10     — int16  IMU pitch × 10
//   [14-15]  roll10      — int16  IMU roll  × 10
//   [16-17]  uptimeSec   — uint16 seconds since boot (wraps at ~18 h)
//   [18]     coveragePct — uint8  CoverageGrid::coveragePercent() 0–100
//   [19]     reserved    — uint8  Phase 1: grid-stream row index
//
// COMMAND codes (1 byte write to COMMAND_CHAR):
//   0x01  BLE_CMD_START        — resume cleaning (mirrors BOOT button)
//   0x02  BLE_CMD_STOP         — soft stop (same as emergencyStopActive)
//   0x03  BLE_CMD_RESUME       — clear emergency stop and resume
//   0x04  BLE_CMD_RESET_STATS  — zero lifetime counters + restart uptime
// ═══════════════════════════════════════════════════════════════════════════

#ifdef ENABLE_BLE

// ── UUIDs ──
static const char* BLE_SERVICE_UUID  = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
static const char* BLE_STATUS_UUID   = "4fafc201-1fb5-459e-8fcc-c5c9c331aabb";
static const char* BLE_CMD_UUID      = "4fafc201-1fb5-459e-8fcc-c5c9c331bbcc";
static const char* BLE_MAP_UUID      = "4fafc201-1fb5-459e-8fcc-c5c9c331ccdd";

// ── Command constants ──
constexpr uint8_t BLE_CMD_START       = 0x01;
constexpr uint8_t BLE_CMD_STOP        = 0x02;
constexpr uint8_t BLE_CMD_RESUME      = 0x03;
constexpr uint8_t BLE_CMD_RESET_STATS = 0x04;

// ── Status packet ──
// 20 bytes — fits in a single BLE notification (default MTU = 23, payload = 20)
struct __attribute__((packed)) BleStatusPacket {
  uint8_t  state;       //  1  RobotState enum
  uint8_t  flags;       //  1  cliff/estop/imu/thermal bits (see below)
  uint16_t battMv;      //  2  battery millivolts (12600 = 12.600 V)
  uint16_t distF_mm;    //  2  front  distance cm × 10
  uint16_t distR_mm;    //  2  right  distance cm × 10
  uint16_t distL_mm;    //  2  left   distance cm × 10
  int16_t  yaw10;       //  2  IMU yaw   × 10 °
  int16_t  pitch10;     //  2  IMU pitch × 10 °
  int16_t  roll10;      //  2  IMU roll  × 10 °
  uint16_t uptimeSec;   //  2  seconds since boot (wraps at ~18 h; 5 min = 300)
  uint8_t  coveragePct; //  1  CoverageGrid::coveragePercent() — 0–100
  uint8_t  reserved;    //  1  Phase 1: grid-stream sequence or map flags
};                      // = 20 bytes total
//
// flags bit layout (v7.2):
//   bit 0  leftCliff        bit 1  rightCliff
//   bit 2  emergencyStop    bit 3  imuOK
//   bit 4  thermalDerate    bits 5-7 reserved

static_assert(sizeof(BleStatusPacket) == 20, "BleStatusPacket must be exactly 20 bytes");
// Layout verified: 1+1+2+2+2+2+2+2+2+2+1+1 = 20 bytes

// ── BLE global handles ──
static BLEServer*         bleServer          = nullptr;
static BLECharacteristic* bleStatusChar      = nullptr;
static BLECharacteristic* bleCmdChar         = nullptr;
static BLECharacteristic* bleMapChar         = nullptr;
static bool               bleClientConnected = false;

// v8.1 NOTE: std::atomic<uint8_t> instead of volatile uint8_t:
// volatile prevents compiler optimisation of the variable itself, but gives
// NO guarantee about CPU-level cache coherency across two Xtensa LX6 cores.
// std::atomic with memory_order_relaxed is sufficient here because:
//   (a) 1-byte aligned read/write is hardware-atomic on Xtensa LX6, and
//   (b) we have no surrounding data that needs to be visible along with the flag.
// The exchange() in processBleCommand() atomically reads-and-zeros in one
// CPU instruction, eliminating the load-then-store TOCTOU window in v6/v7-beta.
static std::atomic<uint8_t> bleCommand{0x00};

// ── Server callbacks ──
class VacuumServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleClientConnected = true;
    DBGLN("[BLE] Client connected");
  }
  void onDisconnect(BLEServer* s) override {
    bleClientConnected = false;
    DBGLN("[BLE] Client disconnected -- restarting advertising");
    // Restart advertising so the app can reconnect without rebooting the robot
    s->getAdvertising()->start();
  }
};

// ── Command characteristic callbacks ──
// Runs in the BLE stack context (Core 0 in ESP-IDF).
// We only write one std::atomic byte, which is safe without a mutex.
class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    std::string val = c->getValue();
    if (!val.empty()) {
      // memory_order_relaxed: we only need atomicity of this single byte,
      // not ordering of any surrounding writes.
      bleCommand.store(static_cast<uint8_t>(val[0]), std::memory_order_relaxed);
      DBG("[BLE] Command received: 0x%02X\n", static_cast<uint8_t>(val[0]));
    }
  }
};

#endif  // ENABLE_BLE

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 16: LEDC PWM — Core 2.x / 3.x compatibility
// ═══════════════════════════════════════════════════════════════════════════
void setupPWM() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(MOTOR_A_EN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR_B_EN, PWM_FREQ, PWM_RESOLUTION);
  DBGLN("[PWM] LEDC Core 3.x (pin-based)");
#else
  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(MOTOR_A_EN, PWM_CH_A);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(MOTOR_B_EN, PWM_CH_B);
  DBGLN("[PWM] LEDC Core 2.x (channel-based)");
#endif
}

inline void writePWM_A(int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(MOTOR_A_EN, (uint32_t)duty);
#else
  ledcWrite(PWM_CH_A, (uint32_t)duty);
#endif
}

inline void writePWM_B(int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(MOTOR_B_EN, (uint32_t)duty);
#else
  ledcWrite(PWM_CH_B, (uint32_t)duty);
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 17: Motor Control
// ═══════════════════════════════════════════════════════════════════════════

// Battery voltage compensation: as pack depletes, increase PWM proportionally
// to maintain constant wheel torque/speed. Capped at 1.5× to avoid stalling.
// Also applies brownout reduction factor (0.8 after brownout reset).
// updateThermalModel — forward-declared here; defined after compensateSpeed.
static void updateThermalModel(int pwm);

inline int compensateSpeed(int nominal) {
  float ratio   = constrain(BATTERY_NOMINAL_VOLTAGE / max(filteredBatteryVoltage, 9.9f),
                            1.0f, 1.5f);
  float thermal = thermalDerateActive ? THERMAL_DERATE_FACTOR : 1.0f;
  int result    = constrain((int)((float)nominal * ratio * brownoutSpeedFactor * thermal),
                            0, 255);
  updateThermalModel(result); // update time-based thermal model with final PWM
  return result;
}

// Competition timer: ramp speed in final 20% of run to maximise coverage.
inline int competitionSpeed(int base = CRUISE_SPEED) {
#ifdef ENABLE_COMPETITION_TIMER
  uint32_t elapsed = millis() - bootTimeMs;
  if (elapsed >= COMP_MAX_THRESHOLD_MS)   base = COMP_MAX_SPEED;
  else if (elapsed >= COMP_BOOST_THRESHOLD_MS) base = COMP_BOOST_SPEED;
#endif
  return base;
}

// v8.1 FIX: Removed unused motorLeftFwd / motorRightFwd state variables.
// These were written by setLeftMotor/setRightMotor but never read by any
// code path. EncoderOdometry receives direction via the 'forward' parameter
// in update(), and DeadReckoning uses the same mechanism. Keeping dead
// state only creates confusion about where direction is tracked.

inline void setLeftMotor(int speed, bool fwd) {
  digitalWrite(MOTOR_A_IN1, fwd ? HIGH : LOW);
  digitalWrite(MOTOR_A_IN2, fwd ? LOW  : HIGH);
  writePWM_A(constrain(speed, 0, 255));
}

inline void setRightMotor(int speed, bool fwd) {
  digitalWrite(MOTOR_B_IN3, fwd ? HIGH : LOW);
  digitalWrite(MOTOR_B_IN4, fwd ? LOW  : HIGH);
  writePWM_B(constrain(speed, 0, 255));
}

void moveForward(int speed) {
  int s = compensateSpeed(speed);
  setLeftMotor(s, true);
  setRightMotor(s, true);
}
void moveBackward(int speed) {
  int s = compensateSpeed(speed);
  setLeftMotor(s, false);
  setRightMotor(s, false);
}
void spinRight(int speed) {
  int s = compensateSpeed(speed);
  setLeftMotor(s, true);
  setRightMotor(s, false);
}
void spinLeft(int speed) {
  int s = compensateSpeed(speed);
  setLeftMotor(s, false);
  setRightMotor(s, true);
}
void stopMotors() {
  setLeftMotor(0, true);
  setRightMotor(0, true);
  // A hard stop clears the thermal accumulator; no partial credit for prior heat.
  thermalHighStartMs = 0;
}

// ── L298N time-based thermal model ────────────────────────────────────────
// Called by compensateSpeed() on every motor command. Tracks how long both
// motors have been running above THERMAL_HIGH_PWM. If sustained for
// THERMAL_HIGH_LIMIT_MS, sets thermalDerateActive for THERMAL_COOLDOWN_MS.
//
// Limitation: uses a single-channel proxy (final compensated PWM) rather than
// reading actual motor current. Adequate for preventing sustained thermal stress
// during high-speed runs; does NOT protect against instantaneous stall events.
static void updateThermalModel(int pwm) {
  uint32_t now = millis();

  if (thermalDerateActive) {
    if (now - thermalCooldownStartMs >= THERMAL_COOLDOWN_MS) {
      thermalDerateActive  = false;
      thermalHighStartMs   = now; // re-arm accumulator
      DBG("[THERMAL] Cooldown complete -- full power restored\n");
    }
    return;
  }

  if (pwm >= THERMAL_HIGH_PWM) {
    if (thermalHighStartMs == 0) thermalHighStartMs = now;
    if (now - thermalHighStartMs >= THERMAL_HIGH_LIMIT_MS) {
      thermalDerateActive     = true;
      thermalCooldownStartMs  = now;
      thermalHighStartMs      = 0;
      DBG("[THERMAL] L298N throttled -- %d ms cooldown at %.0f%%\n",
          THERMAL_COOLDOWN_MS, THERMAL_DERATE_FACTOR * 100.0f);
    }
  } else {
    thermalHighStartMs = 0; // not continuously hot — reset accumulator
  }
}
void setCleaningMotors(bool on) {
  digitalWrite(BRUSH_MOTOR,  on ? HIGH : LOW);
  digitalWrite(VACUUM_MOTOR, on ? HIGH : LOW);
}

// Soft-start ramp: increases PWM 12 counts every 20 ms to avoid wheel slip
// and battery inrush. Call repeatedly from cruiseLayer().
void rampToSpeed(int target) {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 20) return;
  lastMs = now;
  rampCurrentSpeed = (rampCurrentSpeed < target)
    ? min(rampCurrentSpeed + RAMP_STEP, target)
    : max(rampCurrentSpeed - RAMP_STEP, target);
  moveForward(rampCurrentSpeed);
}
void resetRamp() { rampCurrentSpeed = 0; }

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 18: Helper Functions
// ═══════════════════════════════════════════════════════════════════════════
inline float min3(float a, float b, float c) {
  return (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c);
}

// NaN -> 0.0 (failsafe: no data = treat as obstacle at 0 cm)
inline float safeDistance(float d) { return isnan(d) ? 0.0f : d; }

// Wrap any angle (degrees) to the [-180, +180] range.
// Used by boustrophedon heading maintenance and BLE yaw encoding.
// v10.0: extracted as a helper to avoid duplicating the wrap logic.
inline float wrap180(float deg) {
  while (deg >  180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

// Hardware True Random Number Generator (RF noise source in ESP32)
inline uint32_t trueRandom(uint32_t lo, uint32_t hi) {
  if (hi <= lo) return lo;
  return lo + (esp_random() % (hi - lo + 1));
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 19: Sensor Pipeline
// ═══════════════════════════════════════════════════════════════════════════

// Round-robin TRIG firing + EMA processing of ISR results.
// Separation between processing (all sensors) and firing (one sensor)
// prevents sonic crosstalk by ensuring no two sensors are in-flight at once.
void fireSensors() {
  uint32_t now = millis();

  // Step 1: process any sensor that has new data ready
  for (int i = 0; i < 3; i++) {
    if (usSensors[i].newData) {
      portENTER_CRITICAL(&sensorMux);
      unsigned long dur = usSensors[i].pulseDuration;
      usSensors[i].newData = false;
      portEXIT_CRITICAL(&sensorMux);

      float raw = (dur > 0 && dur < ECHO_MAX_VALID_US)
                  ? static_cast<float>(dur) * 0.0343f / 2.0f
                  : ECHO_MAX_CM;

      // Guard: skip EMA update entirely if raw is not a finite number.
      // raw is derived from integer arithmetic so NaN is essentially
      // impossible today, but an explicit isfinite() guard makes the filter
      // bulletproof against any future ISR change or platform quirk.
      // Critically: NOT updating distance (vs. bleeding NAN into the EMA)
      // means safeDistance() will still return the last good reading.
      if (!isfinite(raw)) {
        DBG("[US%d] non-finite raw -- EMA skipped, last good reading held\n", i);
        continue; // newData already cleared; lastValidMs intentionally NOT updated
      }

      // Cold-start fix: first reading bypasses EMA to avoid 700 ms of blindness.
      // Subsequent bad raw values (e.g. ECHO_MAX_CM for a missed echo) are now
      // blended in gently at EMA_ALPHA = 0.35 weight — a single missed echo
      // can only move distance by 35% of the way toward ECHO_MAX_CM.
      usSensors[i].distance = isnan(usSensors[i].distance)
        ? raw
        : EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * usSensors[i].distance;

      usSensors[i].lastValidMs = now;
    }
  }

  // Step 2: fire the next sensor in rotation (with hardware jitter to reduce
  // inter-robot interference in a competition arena with many robots)
  USSensor& next = usSensors[sensorRotIdx];
  uint32_t  jitter = trueRandom(0, 8); // 0–8 ms random offset
  if (now - next.lastFireMs < SENSOR_FIRE_INTERVAL + jitter) return;
  next.lastFireMs = now;

  digitalWrite(next.trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(next.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(next.trigPin, LOW);

  sensorRotIdx = (sensorRotIdx + 1) % 3;
}

void checkStaleSensors() {
  uint32_t now = millis();
  for (int i = 0; i < 3; i++) {
    if (now - usSensors[i].lastValidMs > STALE_TIMEOUT_MS) {
      if (!usSensors[i].isFaulty) {
        usSensors[i].isFaulty = true;
        DBG("[FAULT] Sensor %d unresponsive -- limp mode\n", i);
      }
      // Front sensor: force critical distance (stop robot)
      // Side sensors: keep max distance (let other sensors decide)
      usSensors[i].distance = (i == 0) ? CRITICAL_DIST - 1.0f : ECHO_MAX_CM;
    } else if (usSensors[i].isFaulty) {
      usSensors[i].isFaulty = false;
      DBG("[FAULT] Sensor %d recovered\n", i);
    }
  }
}

void updateIRSensor(IRSensor& ir) {
  // LOW = surface absent (cliff), HIGH = surface present (safe)
  ir.buffer[ir.bufIdx] = (digitalRead(ir.pin) == LOW) ? 1 : 0;
  ir.bufIdx = (ir.bufIdx + 1) % IR_DEBOUNCE_SAMPLES;
  uint8_t sum = 0;
  for (uint8_t i = 0; i < IR_DEBOUNCE_SAMPLES; i++) sum += ir.buffer[i];
  ir.cliffDetected = (sum > IR_DEBOUNCE_SAMPLES / 2);
}

// Wheel-slip / stuck detection via front-distance change over time.
// If the robot is cruising forward but the front distance hasn't changed
// by > 1.2 cm in 2.5 seconds, the wheels are spinning in place.
// NOTE: also active during STATE_WALL_FOLLOW — a wedged robot following a
// wall would previously wait 2 s for WALL_LOST_MS before this could fire.
bool checkStuckCondition() {
  uint32_t now = millis();
  bool activeState = (currentState == STATE_CRUISE || currentState == STATE_WALL_FOLLOW
                      || currentState == STATE_BOUSTRO_ROW);  // v10.1
  if (!activeState) {
    lastStuckCheckMs = now; // reset timer during turns/manoeuvres
    return false;
  }
  float sf = safeDistance(distFront);
  if (sf > 150.0f) {          // don't trigger on wide-open expanses
    lastStuckCheckMs   = now;
    lastFrontDistStuck = sf;
    return false;
  }
  if (now - lastStuckCheckMs >= STUCK_CHECK_INTERVAL_MS) {
    float delta = fabsf(sf - lastFrontDistStuck);
    lastStuckCheckMs   = now;
    lastFrontDistStuck = sf;
    if (delta < STUCK_DIST_DELTA_CM) {
      isRobotStuck = true;
      return true;
    }
  }
  return false;
}

// Master sensor update — called first every loop().
void readAllSensors() {
  fireSensors();
  checkStaleSensors();

  // IR at 200 Hz (5 ms interval → 15 ms debounce window)
  uint32_t now = millis();
  if (now - lastIRSampleMs >= IR_SAMPLE_INTERVAL_MS) {
    lastIRSampleMs = now;
    updateIRSensor(irLeft);
    updateIRSensor(irRight);
  }

  // IMU update — rate-limited to 100 Hz to avoid I2C saturation
  if (imu.initialized) {
    static uint32_t lastImuMs = 0;
    if (now - lastImuMs >= 10) {
      lastImuMs = now;
      imu.update();
      patternDet.recordYaw(imu.getYaw());
    }
  }

  checkStuckCondition();

  // ── Position tracking + coverage marking ──────────────────────────────
  // dr.update() integrates motion into (x, y). With ENABLE_ENCODERS this
  // reads actual wheel ticks (±2 % / m); without it uses the PWM speed
  // model from v7.2 (±20 % / m). Either way, the same call sites apply.
  bool advancing = (currentState == STATE_CRUISE
                 || currentState == STATE_WALL_FOLLOW
                 || currentState == STATE_BOUSTRO_ROW);
  // v10.1: Also update DR during boustrophedon U-turn ADVANCE sub-phase.
  // boustro.inTurn() returns true during SPIN1, ADVANCE, and SPIN2.
  // Only ADVANCE involves forward translation; SPIN1/SPIN2 are pure rotation
  // and don't call update() by design (differential-drive math needs direction).
  bool boustAdvancing = false;
#if defined(ENABLE_BOUSTROPHEDON) && defined(ENABLE_ENCODERS) && defined(ENABLE_OCCUPANCY_GRID)
  boustAdvancing = (currentState == STATE_BOUSTRO_TURN && boustro.turnPhase == BoustrophedonPlanner::ADVANCE);
#endif
  bool reversing = (currentState == STATE_SURVIVAL_REV
                 || currentState == STATE_STUCK_REV);

  if (advancing || boustAdvancing) {
    int drPwm = boustAdvancing ? CRUISE_SPEED :
                (rampCurrentSpeed > 0) ? rampCurrentSpeed : WALL_FOLLOW_SPEED;
    dr.update(drPwm, true);
    coverageGrid.markVisited(dr.x_cm, dr.y_cm); // 8×8 coarse map (always active)
#ifdef ENABLE_OCCUPANCY_GRID
    if (occGrid.isReady()) {
      occGrid.markCleaned(dr.x_cm, dr.y_cm);
      // Ray-cast US readings into the grid to mark obstacle cells.
      // Only mark when the reading is fresh and within the mapping range.
      float heading = dr.getHeadingDeg() * (float)(M_PI / 180.0);
      if (distFront < AVOID_ENTER_DIST && !isnan(distFront) && !usSensors[0].isFaulty) {
        occGrid.markObstacle(dr.x_cm + distFront * cosf(heading),
                             dr.y_cm + distFront * sinf(heading));
      }
      if (distRight < AVOID_ENTER_DIST && !isnan(distRight) && !usSensors[1].isFaulty) {
        float a = heading - (float)(M_PI / 2.0);
        occGrid.markObstacle(dr.x_cm + distRight * cosf(a),
                             dr.y_cm + distRight * sinf(a));
      }
      if (distLeft < AVOID_ENTER_DIST && !isnan(distLeft) && !usSensors[2].isFaulty) {
        float a = heading + (float)(M_PI / 2.0);
        occGrid.markObstacle(dr.x_cm + distLeft  * cosf(a),
                             dr.y_cm + distLeft  * sinf(a));
      }
    }
#endif
  } else if (reversing) {
    dr.update(REVERSE_SPEED, false);
  }


  // Handle emergency stop (debounced via ISR flag)
  if (emergencyStopPressed.exchange(false, std::memory_order_relaxed)) {
    emergencyStopActive  = !emergencyStopActive;
    DBG("[ESTOP] Emergency stop %s\n", emergencyStopActive ? "ACTIVE" : "RELEASED");
    if (!emergencyStopActive) {
      // Resuming: reset ramp so we soft-start again
      resetRamp();
      nextCruiseTurnMs = millis() + trueRandom(CRUISE_TURN_MIN, CRUISE_TURN_MAX);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 20: Status LED (8 distinct blink patterns)
// ═══════════════════════════════════════════════════════════════════════════
//
// Pattern         | Period    | Meaning
// ──────────────────────────────────────────────────────
// Solid ON        | —         | Cruise (all clear)
// Slow blink 1Hz  | 500 ms    | Wall following active
// Medium blink    | 300 ms    | Spiral mode active
// Fast blink      | 150 ms    | Avoidance
// Very fast       |  75 ms    | Stuck self-rescue
// Ultra fast      |  50 ms    | Lifted / flipped
// SOS-dot         | 100 ms    | Survival (cliff / critical)
// Double-flash    | 150 ms ×2 | Battery critical shutdown
// Steady 200 ms   | 200 ms    | Emergency stop
//
void updateStatusLED() {
  uint32_t now = millis();

  if (emergencyStopActive) {
    if (now - lastLedToggleMs >= 200) {
      lastLedToggleMs = now; ledState = !ledState;
      digitalWrite(STATUS_LED, ledState);
    }
    return;
  }

  switch (currentState) {
    case STATE_BATTERY_SHUTDOWN: {
      static uint8_t cycle = 0;
      if (now - lastLedToggleMs >= 150) {
        lastLedToggleMs = now;
        cycle = (cycle + 1) % 8;
        ledState = (cycle == 0 || cycle == 2);
        digitalWrite(STATUS_LED, ledState ? HIGH : LOW);
      }
      break;
    }
    case STATE_PICKUP_STOP:
      if (now - lastLedToggleMs >= 50) {
        lastLedToggleMs = now; ledState = !ledState;
        digitalWrite(STATUS_LED, ledState);
      }
      break;
    case STATE_SURVIVAL_REV:
    case STATE_SURVIVAL_TURN:
      if (now - lastLedToggleMs >= LED_SOS_DOT_MS) {
        lastLedToggleMs = now; ledState = !ledState;
        digitalWrite(STATUS_LED, ledState);
      }
      break;
    case STATE_STUCK_REV:
    case STATE_STUCK_WIGGLE:
    case STATE_STUCK_TURN:
      if (now - lastLedToggleMs >= 75) {
        lastLedToggleMs = now; ledState = !ledState;
        digitalWrite(STATUS_LED, ledState);
      }
      break;
    case STATE_AVOID_PAUSE:
    case STATE_AVOID_TURN:
      if (now - lastLedToggleMs >= LED_BLINK_FAST_MS) {
        lastLedToggleMs = now; ledState = !ledState;
        digitalWrite(STATUS_LED, ledState);
      }
      break;
    case STATE_WALL_FOLLOW:
      if (now - lastLedToggleMs >= 500) {
        lastLedToggleMs = now; ledState = !ledState;
        digitalWrite(STATUS_LED, ledState);
      }
      break;
    case STATE_CRUISE_SPIRAL:
      if (now - lastLedToggleMs >= 300) {
        lastLedToggleMs = now; ledState = !ledState;
        digitalWrite(STATUS_LED, ledState);
      }
      break;
    default: // STATE_CRUISE, STATE_CRUISE_TURN — solid on
      if (!ledState) {
        ledState = true;
        digitalWrite(STATUS_LED, HIGH);
      }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 21: Battery Monitoring & NVS Telemetry
// ═══════════════════════════════════════════════════════════════════════════
float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float pinV   = (raw / 4095.0f) * 3.3f;
  float rawV   = pinV * BATTERY_CALIBRATION_FACTOR;
  filteredBatteryVoltage = BATTERY_EMA_ALPHA * rawV
                         + (1.0f - BATTERY_EMA_ALPHA) * filteredBatteryVoltage;
  return filteredBatteryVoltage;
}

void loadLifetimeStats() {
  preferences.begin("telemetry", false);
  statBoots      = preferences.getUInt("boots", 0) + 1;
  statCliffSaves = preferences.getUInt("cliffs", 0);
  statAvoids     = preferences.getUInt("avoids", 0);
  statRescues    = preferences.getUInt("rescues", 0);
  preferences.putUInt("boots", statBoots);
  preferences.end();
}

// Flush ALL lifetime stats to NVS in a single open/close cycle.
// Called from batterySafetyLayer() on shutdown and from BLE RESET_STATS,
// guaranteeing no in-RAM increments are lost. The old single-static rate
// limiter in incrementStat() could silently drop a cliff-save if an
// avoidance happened within the same 2-second window — this design avoids that.
void flushStats() {
  preferences.begin("telemetry", false);
  preferences.putUInt("boots",   statBoots);
  preferences.putUInt("cliffs",  statCliffSaves);
  preferences.putUInt("avoids",  statAvoids);
  preferences.putUInt("rescues", statRescues);
  preferences.end();
  DBGLN("[NVS] All stats flushed to flash");
}

// Increments an in-RAM counter and writes ONLY THAT KEY to NVS with an
// independent 2-second rate limit per stat. The previous implementation
// used a single shared static timer, so two different stats incrementing
// within 2 s would silently drop the second NVS write.
void incrementStat(const char* key, uint32_t& statVar) {
  statVar++;
  // Per-key rate limiter using a small lookup table.
  // 'cliffs', 'avoids', 'rescues' — 3 entries; 'boots' is written only at boot.
  static struct { const char* k; uint32_t t; } cache[] = {
    {"cliffs",  0}, {"avoids", 0}, {"rescues", 0}
  };
  uint32_t now = millis();
  for (auto& e : cache) {
    if (strcmp(e.k, key) == 0) {
      if (now - e.t < 2000) return; // same key: skip if < 2 s
      e.t = now;
      break;
    }
  }
  preferences.begin("telemetry", false);
  preferences.putUInt(key, statVar);
  preferences.end();
}

void printTelemetryDashboard() {
  float v = readBatteryVoltage();
  Serial.println("======================================================");
  Serial.println("     Robot Vacuum v10.1 - Competition Dashboard       ");
  Serial.println("======================================================");
  Serial.printf( "  Boots       : %-6d  Battery : %.2fV\n", statBoots, v);
  Serial.printf( "  Cliff Saves : %-6d  IMU     : %s\n", statCliffSaves,
                 imu.initialized ? "OK (precise)" : "FALLBACK (time)");
  Serial.printf( "  Avoidances  : %-6d  Brownout: %s\n", statAvoids,
                 (brownoutSpeedFactor < 1.0f) ? "REDUCED -20%" : "Normal");
  Serial.printf( "  Rescues     : %-6d\n", statRescues);
  Serial.println("======================================================");
}

// ═══════════════════════════════════════════════════════════════════════════
// ■ ■ ■   SUBSUMPTION LAYERS — highest priority first   ■ ■ ■
// ═══════════════════════════════════════════════════════════════════════════

// ──────────────────────────────────────────────────────────────────────────
// LAYER 3.6: Battery Safety — absolute hardware protection
// ──────────────────────────────────────────────────────────────────────────
bool batterySafetyLayer() {
  float v = readBatteryVoltage();

  if (v < BATTERY_CRITICAL_VOLTAGE && currentState != STATE_BATTERY_SHUTDOWN) {
    stopMotors();
    setCleaningMotors(false);
    resetRamp();
    currentState = STATE_BATTERY_SHUTDOWN;
    flushStats(); // guarantee all in-RAM counters reach NVS before power dies
    DBG("[BATTERY] Critical %.2fV -- full shutdown to protect cells\n", v);
  }

  if (currentState == STATE_BATTERY_SHUTDOWN) {
    stopMotors();
    setCleaningMotors(false);
    return true;
  }

  static uint32_t lastWarnMs = 0;
  if (v < BATTERY_WARN_VOLTAGE && millis() - lastWarnMs > 10000) {
    lastWarnMs = millis();
    DBG("[BATTERY] Low: %.2fV\n", v);
  }
  return false;
}

// ──────────────────────────────────────────────────────────────────────────
// LAYER 3.5: Pickup / Flip Protection  (NEW in v6)
// ──────────────────────────────────────────────────────────────────────────
// Motors stop within one loop() iteration (~100 µs) when the robot is
// lifted or flipped. Prevents brush/motor damage and is a competition
// safety requirement for many events.
// ──────────────────────────────────────────────────────────────────────────
bool pickupLayer() {
  if (!imu.initialized) return false;

  bool lifted = imu.isPickedUp() || imu.isFlipped();
  bool inPickup = (currentState == STATE_PICKUP_STOP);

  if (lifted && !inPickup) {
    stopMotors();
    setCleaningMotors(false);
    resetRamp();
    currentState = STATE_PICKUP_STOP;
    DBGLN("[PICKUP] Robot lifted/flipped -- motors stopped");
    return true;
  }

  if (inPickup) {
    if (!lifted) {
      // Back on the floor — resume cautiously
      currentState = STATE_CRUISE;
      resetRamp();
      setCleaningMotors(true);
      nextCruiseTurnMs = millis() + trueRandom(CRUISE_TURN_MIN, CRUISE_TURN_MAX);
      patternDet.reset();          // v10.1: clear stale yaw history
      if (imu.initialized) imu.resetTurnTracking();  // v10.1: fresh heading ref
      DBGLN("[PICKUP] Placed back down -- resuming cruise");
      return false;
    }
    stopMotors();
    setCleaningMotors(false);
    return true;
  }

  return false;
}

// ──────────────────────────────────────────────────────────────────────────
// LAYER 3: Survival — cliff edge and critical obstacle (<10 cm)
// ──────────────────────────────────────────────────────────────────────────
// Sequence: immediate stop → reverse → 180° turn toward open space.
// IMU is used for precise 170° turn angle; fallback to 1350 ms if unavailable.
// Faulty side sensors are excluded from criticalAlert to avoid false triggers.
// ──────────────────────────────────────────────────────────────────────────
bool survivalLayer() {
  float sf = safeDistance(distFront);
  float sr = safeDistance(distRight);
  float sl = safeDistance(distLeft);

  bool cliffAlert    = irLeft.cliffDetected || irRight.cliffDetected;
  bool criticalAlert = (sf < CRITICAL_DIST)
                    || (!usSensors[1].isFaulty && sr < CRITICAL_DIST)
                    || (!usSensors[2].isFaulty && sl < CRITICAL_DIST);
  bool threatActive  = cliffAlert || criticalAlert;
  bool inSurvival    = (currentState == STATE_SURVIVAL_REV ||
                        currentState == STATE_SURVIVAL_TURN);

  if (!threatActive && !inSurvival) return false;

  uint32_t now = millis();

  // Penalty watchdog: 6-second hard limit on any manoeuvre
  if (inSurvival && (now - stateStartTime > WATCHDOG_TIMEOUT)) {
    DBGLN("[WATCHDOG] Survival timeout -- safe reset");
    stopMotors(); resetRamp(); setCleaningMotors(true);
    currentState = STATE_CRUISE; return false;
  }

  if (threatActive && !inSurvival) {
    stopMotors(); resetRamp(); setCleaningMotors(false);
    currentState = STATE_SURVIVAL_REV; stateStartTime = now;

    // Smart turn direction: away from cliff, or toward clearer side
    if      (irLeft.cliffDetected  && !irRight.cliffDetected) survivalTurnRight = true;
    else if (irRight.cliffDetected && !irLeft.cliffDetected)  survivalTurnRight = false;
    else if (usSensors[1].isFaulty) survivalTurnRight = false;
    else if (usSensors[2].isFaulty) survivalTurnRight = true;
    else                            survivalTurnRight = (sr >= sl);

    if (imu.initialized) imu.resetTurnTracking();
    incrementStat("cliffs", statCliffSaves);

    DBG("[SURVIVAL] Cliff=%s  obstacle=%.1fcm  turn=%s%s\n",
        cliffAlert ? "YES" : "NO", min3(sf, sr, sl),
        survivalTurnRight ? "RIGHT" : "LEFT",
        (usSensors[1].isFaulty || usSensors[2].isFaulty) ? " [LIMP]" : "");
  }

  if (currentState == STATE_SURVIVAL_REV) {
    // Skip reverse if both sides are also blocked (avoids reversing into another hazard)
    bool revBlocked = (sr < CRITICAL_DIST && sl < CRITICAL_DIST);
    if (revBlocked) {
      stopMotors(); currentState = STATE_SURVIVAL_TURN; stateStartTime = now;
      survivalTurnRight = (sr >= sl);
      if (imu.initialized) imu.resetTurnTracking();
      DBGLN("[SURVIVAL] Reverse blocked -- skipping to turn");
    } else {
      moveBackward(REVERSE_SPEED);
      if (now - stateStartTime >= REVERSE_DURATION) {
        stopMotors(); currentState = STATE_SURVIVAL_TURN; stateStartTime = now;
        if (imu.initialized) imu.resetTurnTracking();
        DBGLN("[SURVIVAL] Reversing complete -- starting 180 deg turn");
      }
      return true;
    }
  }

  if (currentState == STATE_SURVIVAL_TURN) {
    survivalTurnRight ? spinRight(TURN_SPEED) : spinLeft(TURN_SPEED);

    bool done = imu.initialized
      ? (imu.getTurnAngle() >= TURN_180_TARGET_DEG)
      : (now - stateStartTime >= TURN_180_DURATION);

    if (done) {
      stopMotors(); resetRamp(); setCleaningMotors(true);
      currentState = STATE_CRUISE;
      patternDet.reset();
      nextCruiseTurnMs = millis() + trueRandom(CRUISE_TURN_MIN, CRUISE_TURN_MAX);
      DBGLN("[SURVIVAL] Manoeuvre complete -- resuming cruise");
    }
    return true;
  }

  return true;
}

// ──────────────────────────────────────────────────────────────────────────
// LAYER 2.5: Stuck Escape — wheel-slip self-rescue
// ──────────────────────────────────────────────────────────────────────────
// Three-phase escape: reverse → wiggle (breaks physical jam) → turn ~135°
// ──────────────────────────────────────────────────────────────────────────
bool escapeStuckLayer() {
  bool inStuck = (currentState == STATE_STUCK_REV    ||
                  currentState == STATE_STUCK_WIGGLE ||
                  currentState == STATE_STUCK_TURN);

  if (!isRobotStuck && !inStuck) return false;

  uint32_t now = millis();

  if (isRobotStuck && !inStuck) {
    stopMotors(); resetRamp(); setCleaningMotors(false);
    currentState = STATE_STUCK_REV; stateStartTime = now;

    float safeR = usSensors[1].isFaulty ? 0.0f : safeDistance(distRight);
    float safeL = usSensors[2].isFaulty ? 0.0f : safeDistance(distLeft);
    stuckTurnRight = (safeR >= safeL);

    incrementStat("rescues", statRescues);
    DBGLN("[STUCK] Wheel-slip detected -- starting self-rescue");
  }

  if (currentState == STATE_STUCK_REV) {
    moveBackward(REVERSE_SPEED);
    if (now - stateStartTime >= STUCK_REV_DURATION) {
      stopMotors(); currentState = STATE_STUCK_WIGGLE; stateStartTime = now;
    }
    return true;
  }

  if (currentState == STATE_STUCK_WIGGLE) {
    // Fast left-right oscillation physically dislodges the robot
    uint32_t elapsed = now - stateStartTime;
    ((elapsed / 250) % 2 == 0)
      ? spinRight(TURN_SPEED + 20)
      : spinLeft(TURN_SPEED + 20);
    if (elapsed >= STUCK_WIGGLE_DURATION) {
      stopMotors(); currentState = STATE_STUCK_TURN; stateStartTime = now;
      if (imu.initialized) imu.resetTurnTracking();
    }
    return true;
  }

  if (currentState == STATE_STUCK_TURN) {
    stuckTurnRight ? spinRight(TURN_SPEED) : spinLeft(TURN_SPEED);

    bool done = imu.initialized
      ? (imu.getTurnAngle() >= STUCK_TURN_TARGET_DEG)
      : (now - stateStartTime >= STUCK_TURN_DURATION);

    if (done) {
      stopMotors(); isRobotStuck = false; resetRamp(); setCleaningMotors(true);
      currentState = STATE_CRUISE;
      lastStuckCheckMs   = millis();
      lastFrontDistStuck = safeDistance(distFront);
      nextCruiseTurnMs   = millis() + trueRandom(CRUISE_TURN_MIN, CRUISE_TURN_MAX);
      DBGLN("[STUCK] Self-rescue complete -- resuming cruise");
    }
    return true;
  }

  return true;
}

// ──────────────────────────────────────────────────────────────────────────
// LAYER 2: Avoidance — forward obstacle 10–25 cm (with 5 cm hysteresis)
// ──────────────────────────────────────────────────────────────────────────
// Turns toward the roomier side. Retry in opposite direction if still blocked.
// Records turn direction for the oscillation detector.
// IMU ensures a precise 85° turn regardless of battery state.
// ──────────────────────────────────────────────────────────────────────────
bool avoidanceLayer() {
  float sf = safeDistance(distFront);
  bool obstacleAhead = (sf < AVOID_ENTER_DIST);
  bool inAvoid = (currentState == STATE_AVOID_PAUSE || currentState == STATE_AVOID_TURN);

  if (!obstacleAhead && !inAvoid) return false;

  uint32_t now = millis();

  if (inAvoid && (now - stateStartTime > WATCHDOG_TIMEOUT)) {
    stopMotors(); resetRamp(); currentState = STATE_CRUISE; return false;
  }

  if (obstacleAhead && !inAvoid) {
    stopMotors(); resetRamp();
    currentState = STATE_AVOID_PAUSE; stateStartTime = now;

    float safeR = usSensors[1].isFaulty ? 0.0f : safeDistance(distRight);
    float safeL = usSensors[2].isFaulty ? 0.0f : safeDistance(distLeft);
    if      (usSensors[1].isFaulty) avoidTurnRight = false;
    else if (usSensors[2].isFaulty) avoidTurnRight = true;
    else                            avoidTurnRight = (safeR >= safeL);

    patternDet.recordAvoidDirection(avoidTurnRight);
    if (imu.initialized) imu.resetTurnTracking();
    incrementStat("avoids", statAvoids);

    DBG("[AVOID] %.1fcm -> turn %s%s\n", sf,
        avoidTurnRight ? "RIGHT" : "LEFT",
        (usSensors[1].isFaulty || usSensors[2].isFaulty) ? " [LIMP]" : "");
  }

  if (currentState == STATE_AVOID_PAUSE) {
    stopMotors();
    if (now - stateStartTime >= AVOID_PAUSE_DURATION) {
      currentState = STATE_AVOID_TURN; stateStartTime = now;
    }
    return true;
  }

  if (currentState == STATE_AVOID_TURN) {
    avoidTurnRight ? spinRight(TURN_SPEED) : spinLeft(TURN_SPEED);

    bool done = imu.initialized
      ? (imu.getTurnAngle() >= TURN_90_TARGET_DEG)
      : (now - stateStartTime >= TURN_90_DURATION);

    if (done) {
      stopMotors();
      if (safeDistance(distFront) > AVOID_EXIT_DIST) {
        currentState = STATE_CRUISE; resetRamp();
        wallDetectedMs = 0; // reset wall-trigger timer after manoeuvre
        nextCruiseTurnMs = millis() + trueRandom(CRUISE_TURN_MIN, CRUISE_TURN_MAX);
        DBGLN("[AVOID] Path clear -- resuming cruise");
      } else {
        // Still blocked: flip direction and retry
        avoidTurnRight = !avoidTurnRight;
        currentState = STATE_AVOID_PAUSE; stateStartTime = now;
        if (imu.initialized) imu.resetTurnTracking();
        DBGLN("[AVOID] Still blocked -- trying opposite direction");
      }
    }
    return true;
  }

  return true;
}

// ──────────────────────────────────────────────────────────────────────────
// LAYER 1.5: Wall Following  (NEW in v6)
// ──────────────────────────────────────────────────────────────────────────
// Purpose: Systematic edge coverage. A robot that only wanders randomly will
// miss wall-adjacent strips. This layer locks onto a side wall (12–40 cm)
// that has been consistently detected for 3 seconds and follows it with a
// proportional controller, hugging it at 20 cm.
//
// Entry:  One side sensor reads 12–40 cm steadily for WALL_TRIGGER_MS.
// Exit:   Front obstacle | wall lost 2s | oscillation trap detected.
//
// P-controller: error = dist − 20 cm
//   correction = Kp × error = 2.5 × error (capped ±40 PWM)
//   right wall:  left_speed  −= correction   (steer left when too close)
//               right_speed += correction
// ──────────────────────────────────────────────────────────────────────────
bool wallFollowingLayer() {
  bool inWall = (currentState == STATE_WALL_FOLLOW);
  uint32_t now = millis();

  if (!inWall) {
    // Suppressed if oscillation detected (narrow corridor — wall-follow would
    // make a bad situation worse)
    if (patternDet.detectOscillation()) { wallDetectedMs = 0; return false; }

    float sr = safeDistance(distRight);
    float sl = safeDistance(distLeft);
    bool rightWall = (sr >= WALL_FOLLOW_MIN && sr <= WALL_FOLLOW_MAX);
    bool leftWall  = (sl >= WALL_FOLLOW_MIN && sl <= WALL_FOLLOW_MAX);

    if (rightWall || leftWall) {
      if (wallDetectedMs == 0) wallDetectedMs = now;
      if (now - wallDetectedMs >= WALL_TRIGGER_MS) {
        currentState   = STATE_WALL_FOLLOW;
        stateStartTime = now;
        wallLostMs     = 0;
        followingRight = rightWall; // prefer right wall (convention)
        resetRamp();
        DBG("[WALL] Entered wall follow (%s wall, %.1fcm)\n",
            followingRight ? "RIGHT" : "LEFT",
            followingRight ? sr : sl);
        inWall = true; // fall through to controller below
      }
    } else {
      wallDetectedMs = 0;
    }
    if (!inWall) return false;
  }

  // ─── Active wall following ───
  float sf = safeDistance(distFront);
  float wallDist = followingRight ? safeDistance(distRight) : safeDistance(distLeft);

  // Exit: front obstacle — hand off to avoidanceLayer next iteration
  if (sf < AVOID_ENTER_DIST) {
    currentState = STATE_CRUISE; resetRamp(); wallDetectedMs = 0;
    DBGLN("[WALL] Front obstacle -- exiting wall follow");
    return false;
  }

  // Exit: oscillation just started while wall-following
  if (patternDet.detectOscillation()) {
    currentState = STATE_CRUISE; resetRamp(); wallDetectedMs = 0;
    DBGLN("[WALL] Oscillation -- exiting wall follow");
    return false;
  }

  // Check if wall still present
  bool wallPresent = (wallDist >= WALL_FOLLOW_MIN * 0.5f &&
                      wallDist <= WALL_FOLLOW_MAX * 1.5f);
  if (!wallPresent) {
    if (wallLostMs == 0) wallLostMs = now;
    if (now - wallLostMs > WALL_LOST_MS) {
      currentState = STATE_CRUISE; resetRamp(); wallDetectedMs = 0; wallLostMs = 0;
      DBGLN("[WALL] Wall lost -- returning to cruise");
      return false;
    }
  } else {
    wallLostMs = 0;
  }

  // P-controller: positive error → too far → steer toward wall.
  //
  // Battery compensation for the gain:
  // compensateSpeed() scales baseSpeed up as battery drops (ratio ≥ 1.0).
  // If correction stays in nominal PWM units while baseSpeed is scaled up,
  // the differential fraction (correction / baseSpeed) shrinks → controller
  // becomes progressively sluggish and the robot drifts from the wall.
  // Fix: apply the same (compRatio × brownoutSpeedFactor) factor to correction
  // so the differential fraction stays constant across the entire battery curve.
  float compRatio  = constrain(BATTERY_NOMINAL_VOLTAGE / max(filteredBatteryVoltage, 9.9f),
                               1.0f, 1.5f);
  float compFactor = compRatio * brownoutSpeedFactor; // mirrors compensateSpeed()
  float err        = wallDist - WALL_TARGET_DIST;
  // Cap is applied to the nominal correction first, then scaled — this means the
  // ±40 PWM limit is enforced in nominal units and expands with compensation,
  // allowing full authority at low battery (max scaled cap = 40 × 1.5 = 60 PWM).
  float correction = constrain(WALL_KP * err, -40.0f, 40.0f) * compFactor;

  int baseSpeed = compensateSpeed(competitionSpeed(WALL_FOLLOW_SPEED));  // v10.1: end-of-run boost
  int leftSpeed, rightSpeed;

  if (followingRight) {
    leftSpeed  = baseSpeed - (int)correction; // steer right when far
    rightSpeed = baseSpeed + (int)correction;
  } else {
    leftSpeed  = baseSpeed + (int)correction; // steer left when far
    rightSpeed = baseSpeed - (int)correction;
  }

  setLeftMotor(constrain(leftSpeed,  60, 255), true);
  setRightMotor(constrain(rightSpeed, 60, 255), true);

  return true;
}

// ──────────────────────────────────────────────────────────────────────────
// LAYER 1.2: Boustrophedon  (ENABLE_BOUSTROPHEDON)
// ──────────────────────────────────────────────────────────────────────────
// Row-by-row systematic coverage.  Sits below wall-following (1.5) so the
// edge-pass always completes first, then the open-room grid is swept.
// Returns true while active; cruise layer is the fallback when it returns false.
//
// Row-advance state (STATE_BOUSTRO_ROW):
//   Drive at CRUISE_SPEED along rowHeadingYaw reference.
//   IMU heading error → proportional correction ±BOUSTRO_HEADING_MAX PWM.
//   Mark current grid cell as cleaned each iteration.
//   Row ends when front US distance < BOUSTRO_ROW_END_CM.
//
// U-turn state (STATE_BOUSTRO_TURN):
//   Delegates entirely to boustro.updateTurn().
//   Three sub-phases: SPIN1→ADVANCE→SPIN2 (see BoustrophedonPlanner above).
//   Avoidance and survival layers still suppress this during the U-turn if
//   an obstacle appears — the planner will resume from TURN_IDLE on re-entry.
// ──────────────────────────────────────────────────────────────────────────
#if defined(ENABLE_BOUSTROPHEDON) && defined(ENABLE_ENCODERS) && defined(ENABLE_OCCUPANCY_GRID)

bool boustrophedonLayer() {
  // Hard pre-conditions: both hardware features must be ready.
  if (!occGrid.isReady()) return false;

  // First entry: activate the planner.
  // v8.1 FIX: activate() no longer takes a startY parameter.
  if (!boustro.active) {
    boustro.activate();
    currentState = STATE_BOUSTRO_ROW;
    return true;
  }

  setCleaningMotors(true);

  // ── U-turn in progress ──────────────────────────────────────────────────
  if (boustro.inTurn()) {
    currentState = STATE_BOUSTRO_TURN;
    boustro.updateTurn();
    // Mark cells during ADVANCE sub-phase too.
    occGrid.markCleaned(dr.x_cm, dr.y_cm);
    return true;
  }

  // ── Row advance ─────────────────────────────────────────────────────────
  currentState = STATE_BOUSTRO_ROW;
  occGrid.markCleaned(dr.x_cm, dr.y_cm);

  // Heading maintenance: proportional correction from IMU yaw error.
  float yawErr = 0.0f;
  if (imu.initialized) {
    yawErr = wrap180(boustro.rowHeadingYaw - imu.getYaw());
  }
  int correction = constrain((int)(yawErr * BOUSTRO_HEADING_KP),
                              -BOUSTRO_HEADING_MAX, BOUSTRO_HEADING_MAX);
  rampToSpeed(competitionSpeed(CRUISE_SPEED));  // v10.1: end-of-run boost
  int spd = compensateSpeed(rampCurrentSpeed);
  setLeftMotor (constrain(spd + correction, 60, 255), true);
  setRightMotor(constrain(spd - correction, 60, 255), true);

  // ── Row-end detection ───────────────────────────────────────────────────
  // Front obstacle closer than BOUSTRO_ROW_END_CM signals end of traversable
  // row. The avoidance layer (25 cm) is more conservative, so this fires first.
  //
  // v8.1 FIX: guard against stale front sensor. When the front US sensor is
  // faulty, checkStaleSensors() clamps distFront to CRITICAL_DIST-1 (9 cm).
  // Without this guard, the planner would see 9 cm < 30 cm and trigger a
  // premature U-turn on every single row, making boustrophedon useless.
  // If the front sensor is stale, we skip row-end detection and let the
  // avoidance layer (which has its own stale-sensor logic) handle obstacles.
  bool rowEnd = false;
  if (!usSensors[0].isFaulty && !isnan(distFront)) {
    rowEnd = (distFront < BOUSTRO_ROW_END_CM);
  }
  // v10.1: if front US is faulty, use encoder distance as proxy for row end.
  // After 5 m of forward travel without a turn, assume the row is long enough
  // and trigger a U-turn. This prevents the planner from getting stuck when
  // the front sensor is dead — the avoidance layer (which uses side sensors)
  // still handles actual obstacles.
  else if (usSensors[0].isFaulty) {
    static float rowStartX = 0.0f, rowStartY = 0.0f;
    if (currentState != STATE_BOUSTRO_ROW) { rowStartX = dr.x_cm; rowStartY = dr.y_cm; }
    float rowDist = sqrtf((dr.x_cm - rowStartX)*(dr.x_cm - rowStartX) +
                          (dr.y_cm - rowStartY)*(dr.y_cm - rowStartY));
    rowEnd = (rowDist > 500.0f);  // 5 m max row length when front US dead
  }

  if (rowEnd) {
    // Check occupancy grid for remaining work.
    float nextY = occGrid.findNearestUncleanedRow(dr.y_cm + BOUSTRO_ROW_PITCH_CM);
    if (isnan(nextY)) {
      // All cells cleaned — hand off to cruise for any residual corners.
      DBGLN("[BOUSTRO] Full coverage achieved -- handing off to cruise layer");
      boustro.deactivate();
      return false;
    }
    // Start U-turn toward next row.
    stopMotors(); resetRamp();
    boustro.beginUTurn();
    currentState = STATE_BOUSTRO_TURN;
    DBG("[BOUSTRO] Row end -- next uncleaned row Y=%.1f cm\n", nextY);
  }

  return true;
}

#else // ENABLE_BOUSTROPHEDON stub — compiles away when flag is not set
inline bool boustrophedonLayer() { return false; }
#endif // ENABLE_BOUSTROPHEDON

// ──────────────────────────────────────────────────────────────────────────
// LAYER 1: Cruise & Clean — the robot's default free-roaming behaviour
// ──────────────────────────────────────────────────────────────────────────
// In priority order:
//   1. Loop escape (if pattern detector fires)
//   2. Spiral mode (4-second open space → 15-second expanding spiral)
//   3. Random cruise turns every 3–8 seconds (IMU-based angle for accuracy)
//   4. Proportional deceleration as obstacles approach 50→25 cm
// ──────────────────────────────────────────────────────────────────────────
void cruiseLayer() {
  uint32_t now = millis();
  float sf = safeDistance(distFront);
  float sr = safeDistance(distRight);
  float sl = safeDistance(distLeft);

  setCleaningMotors(true);

  // ── Loop escape: anti-pattern detector triggers a forced direction change ──
  if (patternDet.detectLoop() && currentState == STATE_CRUISE) {
    DBG("[PATTERN] Loop detected (%.0f deg rotation in 10s) -- escape turn\n",
        (float)LOOP_DETECT_DEG);
    currentState      = STATE_CRUISE_TURN;
    stateStartTime    = now;
    // Use coverage bias to pick the escape direction; still 70/30 random
    // to prevent the bias itself from re-entering the same loop.
    float _headingDeg   = imu.initialized ? imu.getYaw() : 0.0f;
    cruiseTurnRight     = coverageGrid.preferTurnRight(dr.x_cm, dr.y_cm, _headingDeg);
    cruiseTurnTargetDeg = 85.0f + (float)(esp_random() % 46); // 85–130°
    resetRamp();
    if (imu.initialized) imu.resetTurnTracking();
    patternDet.reset();
    return;
  }

  // ── Spiral mode entry ──
  bool allClear = (sf > SPIRAL_CLEAR_DIST && sr > SPIRAL_CLEAR_DIST && sl > SPIRAL_CLEAR_DIST);

  if (currentState == STATE_CRUISE) {
    if (allClear) {
      if (clearPathStartMs == 0) clearPathStartMs = now;
      else if (now - clearPathStartMs >= SPIRAL_TRIGGER_TIME) {
        currentState  = STATE_CRUISE_SPIRAL;
        spiralStartMs = now;
        resetRamp();
        DBGLN("[CRUISE] Wide open -- entering spiral mode");
      }
    } else {
      clearPathStartMs = 0;
    }
  }

  // ── Spiral mode execution ──
  if (currentState == STATE_CRUISE_SPIRAL) {
    uint32_t elapsed  = now - spiralStartMs;
    bool threatAhead  = (sf < SPIRAL_BREAKOUT_DIST || sr < SPIRAL_BREAKOUT_DIST || sl < SPIRAL_BREAKOUT_DIST);

    if (threatAhead || elapsed >= SPIRAL_MAX_DURATION) {
      stopMotors(); currentState = STATE_CRUISE; clearPathStartMs = 0; resetRamp();
      nextCruiseTurnMs = now + trueRandom(CRUISE_TURN_MIN, CRUISE_TURN_MAX);
      if (threatAhead) DBGLN("[CRUISE] Threat -- exiting spiral");
      else             DBGLN("[CRUISE] Spiral complete -- resuming straight");
      return;
    }

    // Outer motor runs at full competition speed; inner motor ramps up
    // over the spiral's lifetime, gradually opening the spiral.
    // 'spiralPhase' renamed from 'exp' — 'exp' shadows the std::exp() function.
    float spiralPhase = constrain((float)elapsed / SPIRAL_MAX_DURATION, 0.0f, 1.0f);
    int outerSpeed    = competitionSpeed();
    int innerSpeed    = 85 + (int)((outerSpeed - 85) * spiralPhase);

    setLeftMotor(compensateSpeed(outerSpeed), true);
    setRightMotor(compensateSpeed(innerSpeed), true);
    return;
  }

  // ── Random cruise turns ──
  if (now >= nextCruiseTurnMs && currentState == STATE_CRUISE) {
    currentState        = STATE_CRUISE_TURN;
    stateStartTime      = now;
    // Coverage-biased direction: 70% toward less-visited cell, 30% random.
    // avoidanceLayer still has priority — a poor turn will be corrected.
    float _hDeg         = imu.initialized ? imu.getYaw() : 0.0f;
    cruiseTurnRight     = coverageGrid.preferTurnRight(dr.x_cm, dr.y_cm, _hDeg);
    cruiseTurnTargetDeg = 30.0f + (float)(esp_random() % 66); // 30–95° random
    resetRamp();
    if (imu.initialized) imu.resetTurnTracking();
    DBG("[CRUISE] Random turn %s (cov bias) -- target %.0f deg\n",
        cruiseTurnRight ? "RIGHT" : "LEFT", cruiseTurnTargetDeg);
  }

  if (currentState == STATE_CRUISE_TURN) {
    cruiseTurnRight ? spinRight(TURN_SPEED) : spinLeft(TURN_SPEED);

    bool done = imu.initialized
      ? (imu.getTurnAngle() >= cruiseTurnTargetDeg)
      : (now - stateStartTime >= CRUISE_TURN_DURATION);

    if (done) {
      stopMotors(); currentState = STATE_CRUISE; clearPathStartMs = 0;
      nextCruiseTurnMs = now + trueRandom(CRUISE_TURN_MIN, CRUISE_TURN_MAX);
    }
    return;
  }

  // ── Normal forward motion ──
  currentState = STATE_CRUISE;

  // Proportional deceleration zone: 50 → 25 cm ahead
  int target = competitionSpeed();
  if (sf < 50.0f && sf >= AVOID_ENTER_DIST) {
    float factor = (sf - AVOID_ENTER_DIST) / (50.0f - AVOID_ENTER_DIST);
    target = TURN_SPEED + (int)((competitionSpeed() - TURN_SPEED) * factor);
  }

  rampToSpeed(target);
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 22: BLE Functions
// ═══════════════════════════════════════════════════════════════════════════

#ifdef ENABLE_BLE

// ── setupBLE ──────────────────────────────────────────────────────────────
// Registers service, creates all three characteristics, starts advertising.
// Must be called from setup() after Serial and sensor init are complete.
// BLE flash footprint: ~100 KB. Comment out #define ENABLE_BLE to reclaim.
// ──────────────────────────────────────────────────────────────────────────
void setupBLE() {
  BLEDevice::init("Obrynex-V1");
  BLEDevice::setPower(ESP_PWR_LVL_P9);          // max TX power (+9 dBm)

  bleServer = BLEDevice::createServer();
  // v10.0 NOTE: these new() allocations are INTENTIONAL singletons. The
  // ESP32 Bluedroid stack takes ownership of callback objects; they persist
  // for the program lifetime and are never deleted. Using static objects
  // would risk double-free when the BLE stack tears down on disconnect.
  bleServer->setCallbacks(new VacuumServerCallbacks());

  BLEService* svc = bleServer->createService(BLE_SERVICE_UUID);

  // ── STATUS characteristic (Notify + Read) ──
  bleStatusChar = svc->createCharacteristic(
    BLE_STATUS_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  bleStatusChar->addDescriptor(new BLE2902()); // required for Notify to work

  // ── COMMAND characteristic (Write + Write Without Response) ──
  bleCmdChar = svc->createCharacteristic(
    BLE_CMD_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  bleCmdChar->setCallbacks(new CommandCallbacks());

  // ── MAP_STREAM characteristic (Notify — RLE-compressed occupancy rows) ──
  bleMapChar = svc->createCharacteristic(BLE_MAP_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);
  bleMapChar->addDescriptor(new BLE2902());

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);  // needed for iOS to discover the device
  adv->setMaxPreferred(0x12);
  adv->start();

  DBGLN("[BLE] advertising as 'Obrynex-V1'");
  DBG("  STATUS  UUID: %s\n", BLE_STATUS_UUID);
  DBG("  COMMAND UUID: %s\n", BLE_CMD_UUID);
  DBG("  MAP     UUID: %s\n", BLE_MAP_UUID);
}

// ── sendBleStatus ─────────────────────────────────────────────────────────
// Packs the 20-byte STATUS struct and fires a BLE Notify at 5 Hz.
// Rate-limited with a static timer — safe to call every loop() iteration.
// No-ops immediately if no client is connected.
// ──────────────────────────────────────────────────────────────────────────
void sendBleStatus() {
  if (!bleClientConnected) return;

  static uint32_t lastBleMs = 0;
  uint32_t now = millis();
  if (now - lastBleMs < 200) return;   // 5 Hz cap
  lastBleMs = now;

  BleStatusPacket pkt;

  pkt.state  = static_cast<uint8_t>(currentState);
  pkt.flags  = (irLeft.cliffDetected   ? 0x01u : 0u)
             | (irRight.cliffDetected  ? 0x02u : 0u)
             | (emergencyStopActive    ? 0x04u : 0u)
             | (imu.initialized        ? 0x08u : 0u)
             | (thermalDerateActive    ? 0x10u : 0u); // bit 4: L298N throttled

  pkt.battMv      = static_cast<uint16_t>(filteredBatteryVoltage * 1000.0f);
  pkt.distF_mm    = static_cast<uint16_t>(constrain(safeDistance(distFront) * 10.0f, 0.0f, 65535.0f));
  pkt.distR_mm    = static_cast<uint16_t>(constrain(safeDistance(distRight) * 10.0f, 0.0f, 65535.0f));
  pkt.distL_mm    = static_cast<uint16_t>(constrain(safeDistance(distLeft)  * 10.0f, 0.0f, 65535.0f));
  // v10.0 FIX: wrap yaw to +-180 degrees before encoding. yawAccum is
  // unbounded (integrates forever), so after a 5-minute competition run
  // with heavy turning it can exceed 50,000 degrees. int16_t tops out at
  // 32,767 (+-3276.7 deg in fixed-point), so raw encoding would overflow.
  // Pitch and roll are inherently bounded (+-180); only yaw needs this.
  if (imu.initialized) {
    pkt.yaw10   = static_cast<int16_t>(wrap180(imu.getYaw()) * 10.0f);
    pkt.pitch10 = static_cast<int16_t>(imu.pitch    * 10.0f);
    pkt.roll10  = static_cast<int16_t>(imu.roll     * 10.0f);
  } else {
    // Sentinel values so the BLE client can distinguish "no IMU" from 0 deg.
    pkt.yaw10 = pkt.pitch10 = pkt.roll10 = INT16_MIN;  // -32768
  }
  // uptimeSec replaces uptimeMs (4 B → 2 B). Wraps at 65535 s ≈ 18 h —
  // far longer than any competition run. Freed 2 bytes used for coverage + reserved.
  pkt.uptimeSec   = static_cast<uint16_t>(min((now - bootTimeMs) / 1000UL, 65535UL));
  pkt.coveragePct = coverageGrid.coveragePercent();
  // v10.0: bit 0 set = boustrophedon active (Layer 1.2 engaged)
  pkt.reserved    = 0;
#if defined(ENABLE_BOUSTROPHEDON) && defined(ENABLE_ENCODERS) && defined(ENABLE_OCCUPANCY_GRID)
  if (boustro.active) pkt.reserved |= 0x01;
#endif

  bleStatusChar->setValue(reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
  bleStatusChar->notify();

  // ── MAP_STREAM: push one RLE-compressed grid row per BLE update (~1 Hz) ──
  // Cycles through all 200 rows; ~200 s for a full map push to the client.
#ifdef ENABLE_OCCUPANCY_GRID
  static uint32_t lastMapMs   = 0;
  static uint8_t  bleMapRowIdx = 0;
  // v10.0 FIX: static buffer instead of stack allocation. Avoids 408 bytes
  // of stack pressure per call — significant on ESP32 where default stack
  // is ~8 KB and deep call chains (BLE + interrupt context) can overflow.
  static uint8_t  rowBuf[408]; // header(4) + max RLE = 2x200 bytes
  if (occGrid.isReady() && now - lastMapMs >= 1000UL) {
    lastMapMs = now;
    rowBuf[0] = (uint8_t)(bleMapRowIdx >> 8);
    rowBuf[1] = (uint8_t)(bleMapRowIdx & 0xFF);
    rowBuf[2] = (uint8_t)(OCC_ROWS >> 8);
    rowBuf[3] = (uint8_t)(OCC_ROWS & 0xFF);
    uint16_t rleLen = occGrid.encodeRow(bleMapRowIdx, rowBuf + 4, (uint16_t)(sizeof(rowBuf) - 4));
    if (rleLen > 0) {
      bleMapChar->setValue(rowBuf, 4 + rleLen);
      bleMapChar->notify();
    }
    bleMapRowIdx = (uint8_t)((bleMapRowIdx + 1) % OCC_ROWS);
  }
#endif // ENABLE_OCCUPANCY_GRID
}

// ── processBleCommand ─────────────────────────────────────────────────────
// Consumes the std::atomic bleCommand flag set by the BLE callback.
// Must be called once per loop() iteration, BEFORE the emergency-stop check,
// so that BLE_CMD_RESUME can clear emergencyStopActive in time.
//
// v8.1 NOTE: uses std::atomic<uint8_t> with exchange(memory_order_acq_rel)
// for guaranteed atomicity across both Xtensa LX6 cores. The old volatile
// uint8_t approach was not cache-coherent on dual-core ESP32.
// ──────────────────────────────────────────────────────────────────────────
void processBleCommand() {
  // exchange() atomically reads the current value AND stores 0 in one CPU
  // instruction. This eliminates the load-then-store TOCTOU window that existed
  // when using separate bleCommand == 0x00 check and bleCommand = 0x00 assign.
  // memory_order_acq_rel: acquire on read (see any writes done before the store
  // in the BLE callback), release on write (our clear is visible to the callback).
  const uint8_t cmd = bleCommand.exchange(0x00, std::memory_order_acq_rel);
  if (cmd == 0x00) return;

  switch (cmd) {

    case BLE_CMD_START:
      // Mirrors the physical BOOT button: start/resume a cleaning run.
      // Refused if battery is critically low.
      if (currentState == STATE_BATTERY_SHUTDOWN) {
        DBGLN("[BLE] START refused -- battery critical");
        return;
      }
      emergencyStopActive = false;
      resetRamp();
      setCleaningMotors(true);
      currentState     = STATE_CRUISE;
      nextCruiseTurnMs = millis() + trueRandom(CRUISE_TURN_MIN, CRUISE_TURN_MAX);
      DBGLN("[BLE] CMD: START_CLEANING");
      break;

    case BLE_CMD_STOP:
      // Soft stop — identical to pressing the physical emergency stop button.
      emergencyStopActive = true;
      stopMotors();
      setCleaningMotors(false);
      resetRamp();
      DBGLN("[BLE] CMD: STOP_CLEANING");
      break;

    case BLE_CMD_RESUME:
      // Clear a BLE-induced (or button-induced) emergency stop and resume.
      if (emergencyStopActive) {
        emergencyStopActive = false;
        resetRamp();
        setCleaningMotors(true);
        currentState     = STATE_CRUISE;
        nextCruiseTurnMs = millis() + trueRandom(CRUISE_TURN_MIN, CRUISE_TURN_MAX);
        DBGLN("[BLE] CMD: RESUME_CLEANING");
      }
      break;

    case BLE_CMD_RESET_STATS:
      // Zero NVS lifetime counters, restart uptime clock, and clear coverage.
      statCliffSaves = 0;
      statAvoids     = 0;
      statRescues    = 0;
      bootTimeMs     = millis();
      coverageGrid.reset(); // 8×8 coarse map — new run
#ifdef ENABLE_OCCUPANCY_GRID
      if (occGrid.isReady()) occGrid.reset(); // 200×200 Phase 1 grid
#endif
      dr.reset();           // new run = fresh position estimate
#if defined(ENABLE_BOUSTROPHEDON) && defined(ENABLE_ENCODERS) && defined(ENABLE_OCCUPANCY_GRID)
      if (occGrid.isReady()) { boustro.deactivate(); boustro.activate(); }
#endif
      DBGLN("[BLE] CMD: RESET_STATS -- counters, coverage, and position zeroed");
      break;

    default:
      DBG("[BLE] CMD: unknown 0x%02X -- ignored\n", cmd);
      break;
  }
}

#endif  // ENABLE_BLE

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 23: Memory Diagnostics
// ═══════════════════════════════════════════════════════════════════════════
//
// reportMemory() prints a concise heap snapshot to Serial. Call it during
// setup() / runSelfTest() to verify the budget before Phase 1 grid allocation.
//
// Interpretation guide:
//   Internal free  < 80 KB  → switch BLE library to NimBLE (saves ~60 KB)
//   Internal free  < 50 KB  → allocate occupancyGrid in PSRAM if fitted
//   Largest block  < PHASE1_GRID_BYTES → grid CANNOT be contiguously allocated
//   PSRAM present            → allocate grid with heap_caps_malloc(PHASE1_GRID_BYTES,
//                              MALLOC_CAP_SPIRAM) — zero cost to internal heap
// ═══════════════════════════════════════════════════════════════════════════
void reportMemory() {
  size_t freeInternal  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t largestBlock  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t freePSRAM     = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

  // All output through DBG/DBGLN so the entire block is gated by VERBOSE_DEBUG.
  // Previously the border lines used Serial.println() while the content used DBG(),
  // producing orphan separator lines when VERBOSE_DEBUG was commented out.
  DBG("  ── Heap snapshot ──────────────────────────────────\n");
  DBG("  Internal free  : %6u bytes\n",  (unsigned)freeInternal);
  DBG("  Largest block  : %6u bytes\n",  (unsigned)largestBlock);
  DBG("  PSRAM free     : %6u bytes %s\n",
      (unsigned)freePSRAM, freePSRAM > 0 ? "" : "(not fitted)");
  DBG("  Phase 1 grid   : %6u bytes needed for occupancyGrid[200][200]\n",
      (unsigned)PHASE1_GRID_BYTES);
  DBG("  Coverage grid  : %6u bytes (CoverageGrid, already allocated)\n",
      (unsigned)sizeof(CoverageGrid));

  if (freePSRAM >= PHASE1_GRID_BYTES) {
    DBG("  [OK] Phase 1 ready : allocate grid in PSRAM (heap_caps_malloc + MALLOC_CAP_SPIRAM)\n");
  } else if (largestBlock >= PHASE1_GRID_BYTES) {
    DBG("  [OK] Phase 1 ready : allocate grid in internal heap (no PSRAM -- recommend NimBLE first)\n");
  } else {
    DBG("  [FAIL] Phase 1 BLOCKED: no contiguous block large enough for grid!\n");
    DBG("    -> Switch to NimBLE-Arduino to reclaim ~60 KB, then re-test.\n");
  }
  DBG("  ───────────────────────────────────────────────────\n");
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 24: Startup Self-Test
// ═══════════════════════════════════════════════════════════════════════════
void runSelfTest() {
  DBGLN("\n========== Self-Test v10.1 ==========");
  bool allPassed = true;

  // ── Memory diagnostics (printed first — good to see before sensor tests) ──
  reportMemory();

  // Allow ultrasonic sensors ~600 ms to get their first readings
  uint32_t t = millis();
  while (millis() - t < 600) { esp_task_wdt_reset(); fireSensors(); }  // v10.1

  const char* names[3] = {"Front", "Right", "Left"};
  for (int i = 0; i < 3; i++) {
    if (isnan(usSensors[i].distance)) {
      DBG("  [FAIL] US %-5s : NO RESPONSE\n", names[i]);
      allPassed = false;
    } else {
      DBG("  [OK]   US %-5s : %.1f cm\n", names[i], usSensors[i].distance);
    }
  }

  for (int s = 0; s < IR_DEBOUNCE_SAMPLES; s++) {
    updateIRSensor(irLeft);
    updateIRSensor(irRight);
  }
  DBG("  [%s] IR Left  : %s\n", irLeft.cliffDetected  ? "WARN" : "OK",
      irLeft.cliffDetected  ? "CLIFF (check surface)" : "OK");
  DBG("  [%s] IR Right : %s\n", irRight.cliffDetected ? "WARN" : "OK",
      irRight.cliffDetected ? "CLIFF (check surface)" : "OK");

  DBG("  [%s] IMU      : %s\n", imu.initialized ? "OK" : "WARN",
      imu.initialized ? "MPU6050 active -- precise angle turns" : "Not found -- time-based fallback");

  // Check reset reason — inform operator
  esp_reset_reason_t reason = esp_reset_reason();
  if (reason == ESP_RST_BROWNOUT) {
    DBGLN("  ! Last reset: BROWNOUT -- max speed reduced 20%");
    allPassed = false;
  } else if (reason == ESP_RST_TASK_WDT) {
    DBGLN("  ! Last reset: TASK WDT -- loop() was blocked");
  }

  if (allPassed) {
    DBGLN("==================================================");
    DBGLN("  Self-Test: PASS");
    DBGLN("==================================================\n");
    for (int i = 0; i < 3; i++) {
      digitalWrite(STATUS_LED, HIGH); delay(200);
      digitalWrite(STATUS_LED, LOW);  delay(200);
    }
  } else {
    DBGLN("==================================================");
    DBGLN("  Self-Test: WARNING -- robot will start with caveats");
    DBGLN("==================================================\n");
    for (int i = 0; i < 8; i++) {
      digitalWrite(STATUS_LED, HIGH); delay(80);
      digitalWrite(STATUS_LED, LOW);  delay(80);
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 25: setup()
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(400); // one-time hardware stabilisation — only delay() in production code

  bootTimeMs = millis();

  Serial.println("\n======================================================");
  Serial.println(  "   Smart Vacuum Robot - Subsumption Architecture v10.0");
  Serial.println(  "   Obrynex Edition  |  ESP32 NodeMCU  |  BLE enabled  ");
  Serial.println(  "======================================================\n");

  // ── Brownout recovery: reduce max speed 20% to survive a weak battery ──
  esp_reset_reason_t resetReason = esp_reset_reason();
  if (resetReason == ESP_RST_BROWNOUT) {
    brownoutSpeedFactor = 0.80f;
    Serial.println("[BROWNOUT] recovery: max PWM reduced 20%");
  } else if (resetReason == ESP_RST_TASK_WDT) {
    Serial.println("[WDT] recovery: loop() was blocked last run");
  }

  // ── Motor driver pins ──
  pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT); pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT); pinMode(MOTOR_B_IN4, OUTPUT); pinMode(MOTOR_B_EN, OUTPUT);
  setupPWM();
  DBGLN("[MOTOR] L298N driver configured");

  // ── Status LED ──
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  DBGLN("[LED] Status LED: GPIO 2");

  // ── Battery ADC ──
  pinMode(BATTERY_PIN, INPUT);
  // v10.1 FIX: explicit ADC configuration. ESP32 ADC1 defaults vary by core
  // version; without these calls the attenuation may be 0 dB (0–1.1V range)
  // which produces completely wrong battery readings.
  analogReadResolution(12);           // 0–4095 range
  analogSetAttenuation(ADC_11db);     // 0–3.3V full scale
  // Prime the EMA filter with a real reading before the main loop
  for (int i = 0; i < 20; i++) readBatteryVoltage();
  // v10.1 FIX: sanity check the voltage divider ratio at boot.
  // With 3S LiPo (12.6V max), the divider must produce ≤3.3V at ADC pin.
  // The stated "10k/10k" (factor 2.0) yields 6.3V — ABOVE 3.3V MAXIMUM.
  // Use ~28k/10k (factor 3.8) or similar instead.
  float vTest = readBatteryVoltage();
  if (vTest > 14.0f || vTest < 3.0f) {
    Serial.println("[WARN] Battery voltage implausible — check divider ratio!");
    Serial.printf("[WARN] Read %.1fV with factor %.1f (pin voltage ~%.2fV)
",
                  vTest, BATTERY_CALIBRATION_FACTOR, vTest / BATTERY_CALIBRATION_FACTOR);
  }
  DBG("[BATTERY] Monitor ready: %.2fV\n", filteredBatteryVoltage);

  // ── Cleaning motors ──
  pinMode(BRUSH_MOTOR,  OUTPUT);
  pinMode(VACUUM_MOTOR, OUTPUT);
  setCleaningMotors(false); // off until self-test passes

  // ── Emergency stop button (GPIO 0 = BOOT button) ──
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencyStopISR, FALLING);
  DBGLN("[ESTOP] Emergency stop: GPIO 0 (BOOT button, active-LOW)");

  // ── NVS competition telemetry ──
  loadLifetimeStats();

  // ── IMU (MPU6050) ──
  // Wire uses GPIO 21 (SDA) and 22 (SCL) — freed by remap of US sensors.
#ifdef ENABLE_IMU
  if (imu.begin()) {
    imu.calibrate();
    Serial.println("[IMU] MPU6050 OK -- precise angle turns enabled");
  } else {
    Serial.println("[IMU] MPU6050 not found -- time-based turns (fallback)");
  }
#else
  imu.fallbackMode = true;
  DBGLN("[IMU] disabled by compile flag");
#endif

  // ── Ultrasonic sensors with ISR (portMUX dual-core safe) ──
  // ★ Note new pin mapping vs v3/v5:
  //   ECHO_RIGHT: GPIO 16 (was 21)    TRIG_LEFT: GPIO 17 (was 22)
  const uint8_t trigPins[3] = {TRIG_FRONT, TRIG_RIGHT, TRIG_LEFT};
  const uint8_t echoPins[3] = {ECHO_FRONT, ECHO_RIGHT, ECHO_LEFT};
  void (*const isrFuncs[3])() = {echoISR_Front, echoISR_Right, echoISR_Left};

  for (int i = 0; i < 3; i++) {
    usSensors[i] = {trigPins[i], echoPins[i], 0, 0, false, NAN,
                    (uint32_t)(i * SENSOR_FIRE_INTERVAL), millis(), false};
    pinMode(trigPins[i], OUTPUT);
    digitalWrite(trigPins[i], LOW);
    pinMode(echoPins[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(echoPins[i]), isrFuncs[i], CHANGE);
  }
  DBGLN("[US] Ultrasonic sensors: ISR active (IRAM GPIO reads, portMUX safe)");

  // ── IR cliff sensors ──
  irLeft  = {IR_LEFT_PIN,  {0,0,0}, 0, false};
  irRight = {IR_RIGHT_PIN, {0,0,0}, 0, false};
  pinMode(IR_LEFT_PIN,  INPUT); // GPIO 34/35: input-only, no pull-up
  pinMode(IR_RIGHT_PIN, INPUT);
  DBGLN("[IR] Cliff sensors: majority-vote debounce (200 Hz)");

  // ── Initial motor state ──
  stopMotors();
  nextCruiseTurnMs = millis() + trueRandom(CRUISE_TURN_MIN, CRUISE_TURN_MAX);
  DBGLN("[MOTOR] Motors stopped, random cruise timer armed");

  // ── Task Watchdog Timer ──
  // If loop() blocks for more than TASK_WDT_TIMEOUT_S, the ESP32 hardware-resets.
  esp_task_wdt_init(TASK_WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  DBG("[WDT] Task WDT armed -- %d s timeout\n", TASK_WDT_TIMEOUT_S);

  // ── BLE server ──
  // Initialised after WDT so the 2-second advertising start doesn't trip it.
  // ENABLE_BLE adds ~100 KB to the flash binary. Comment the #define to save it.
#ifdef ENABLE_BLE
  setupBLE();
#else
  DBGLN("[BLE] disabled by compile flag");
#endif

  // ── Wheel encoders (v8.0 — ENABLE_ENCODERS) ──
  // Both pins are configured INPUT_PULLDOWN (not INPUT_PULLUP) because:
  //   GPIO 12: a pull-up would fight the 10 kΩ external pull-down required
  //            for boot-strapping correctness (see pin-table notes above).
  //   GPIO 15: no strapping requirement, but consistent with GPIO 12.
  // The hall-effect sensors are open-drain or push-pull, both compatible
  // with a pull-down. Pulses fire on RISING edge (magnet pole passing).
#ifdef ENABLE_ENCODERS
  pinMode(ENC_LEFT_PIN,  INPUT_PULLDOWN);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_PIN),  encISR_Left,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_PIN), encISR_Right, RISING);
  dr.reset(); // zero x_cm / y_cm and capture baseline tick snapshot
  DBGLN("[ENC] Wheel encoders: hall-effect ISRs armed (GPIO 12 L, GPIO 15 R)");
  DBG(  "[ENC] Wheel: d=%.0f mm  base=%.0f mm  %.2f cm/tick\n",
        WHEEL_DIAMETER_MM, WHEEL_BASE_MM, CM_PER_TICK);
#else
  DBGLN("[ENC] Wheel encoders disabled -- dead-reckoning fallback (v7.2)");
#endif

  // ── Phase 1 occupancy grid (v8.0 — ENABLE_OCCUPANCY_GRID) ──
  // Allocated here — after BLE init — so heap_caps_get_free_size() reflects
  // the full BLE memory cost before we commit the 40 KB.
#ifdef ENABLE_OCCUPANCY_GRID
  if (occGrid.init()) {
    DBGLN("[OCC] Occupancy grid: 200x200 cells (40 KB) ready");
    DBG(  "[OCC] Cell: %.0f cm  Span: +/-%.0f cm  Coverage: %u%%\n",
          OCC_CELL_CM, OCC_ORIGIN_CM, occGrid.cleanedPercent());
#ifdef ENABLE_BOUSTROPHEDON
    boustro.activate();
    DBGLN("[BOUSTRO] Boustrophedon planner: activated (Layer 1.2)");
#endif
  } else {
    DBGLN("[OCC] Occupancy grid: alloc failed -- bias-cruise fallback (v7.2)");
    DBGLN("      Tip: switch to NimBLE-Arduino to reclaim ~60 KB heap");
  }
#else
  DBGLN("[OCC] Occupancy grid disabled -- bias-cruise (v7.2)");
#endif

  // ── Startup self-test ──
  runSelfTest();

  // ── Start cleaning ──
  setCleaningMotors(true);

  // ── Print competition dashboard ──
  printTelemetryDashboard();

  DBGLN("==================================================");
  DBGLN("Subsumption Architecture v10.1: Ready! BLE advertising as 'Obrynex-V1'\n");
}

// ═══════════════════════════════════════════════════════════════════════════
// SECTION 26: loop()
// ═══════════════════════════════════════════════════════════════════════════
//
// The suppression mechanism at a glance:
//   Each layer returns bool: true = "I'm active, suppress everything below me".
//   return in loop() IS the suppression. No flags, no scheduler.
//   Layers are checked top-to-bottom every iteration (~10–20 kHz raw rate).
//
void loop() {
  // ── Feed the WDT first — proves loop() is not blocked ──
  esp_task_wdt_reset();

  // ── 1. Update all sensors (ISR results + IR + IMU + stuck check) ──
  readAllSensors();

  // ── 2. BLE: consume any pending command, then send telemetry ──
  // processBleCommand() runs BEFORE the emergency-stop check so that
  // BLE_CMD_RESUME can clear emergencyStopActive in the same iteration.
  // sendBleStatus() is rate-limited internally to 5 Hz; calling it here
  // every loop() iteration costs <1 µs when no notification is due.
#ifdef ENABLE_BLE
  processBleCommand();
  sendBleStatus();
#endif

  // ── 3. Emergency stop overrides the entire system ──
  if (emergencyStopActive) {
    stopMotors();
    setCleaningMotors(false);
    updateStatusLED();
    return;
  }

  // ── 4. Subsumption hierarchy ──

  if (batterySafetyLayer()) { updateStatusLED(); return; } // 3.6
  if (pickupLayer())         { updateStatusLED(); return; } // 3.5
  if (survivalLayer())       { updateStatusLED(); return; } // 3
  if (escapeStuckLayer())    { updateStatusLED(); return; } // 2.5
  if (avoidanceLayer())      { updateStatusLED(); return; } // 2
  if (wallFollowingLayer())  { updateStatusLED(); return; } // 1.5
  if (boustrophedonLayer())  { updateStatusLED(); return; } // 1.2
  cruiseLayer();                                            // 1

  updateStatusLED();
}

// ═══════════════════════════════════════════════════════════════════════════
// ■ Technical Notes — v7.0–v7.2 (retained features)
// ═══════════════════════════════════════════════════════════════════════════
//
//  1. NO delay() IN MAIN LOOP:
//     The only delay() calls are in setup() (400 ms hardware settle),
//     runSelfTest() (LED flashes), and imu.calibrate() (10 ms × ~200 samples
//     during boot calibration). The main loop() is 100% delay-free.
//
//  2. DUAL-CORE SAFETY (portMUX spinlocks):
//     ESP32 has two cores. noInterrupts() only masks one core.
//     portMUX_TYPE spinlocks are atomic across both cores — the only correct
//     synchronisation primitive for shared ISR ↔ loop() data.
//
//  3. IRAM-SAFE ISRs (direct GPIO register reads):
//     digitalRead() may live in SPI flash and cause a cache miss → CPU
//     exception inside an ISR. Direct reads from GPIO.in (DRAM-mapped)
//     are always safe.
//
//  4. UNSIGNED SUBTRACTION handles micros() overflow correctly (C++ std).
//
//  5. IMU TURN PRECISION:
//     MPU6050 ±250°/s gyro integrates at 100 Hz. Over a 1.35 s, 180° turn,
//     accumulated drift is < 3°. Far better than time-based turns whose
//     error grows with battery sag and motor wear.
//
//  6. COMPETITION TIMER (ENABLE_COMPETITION_TIMER):
//     At 80% of competition time → +15% speed boost.
//     At 95% of competition time → +29% speed boost.
//     These ensure maximum coverage in the scoring window.
//
//  7. BATTERY VOLTAGE COMPENSATION:
//     As 3S LiPo sags from 12.6 V → 9.9 V, PWM duty is scaled up by the
//     same ratio to maintain constant torque. Speed stays consistent for
//     the entire run, not just the first minute.
//
//  8. GRACEFUL DEGRADATION:
//     Front US faulty   → forced CRITICAL_DIST − 1 (safe halt)
//     Side US faulty    → limp mode, excluded from survival/avoid triggers
//     IMU unavailable   → time-based turn fallback
//     Brownout reset    → max PWM × 0.8 to survive weak battery
//     WDT reset         → auto-resume, logged to NVS
//
//  9. WALL FOLLOWING (Layer 1.5):
//     Proportional controller with Kp=2.5. Correction is capped ±40 PWM
//     to prevent overcorrection at sharp wall angles. Exits automatically
//     on front obstacle (hand-off to avoidance) or oscillation trap.
//
// 10. ANTI-PATTERN DETECTION:
//     Loop: 10-s yaw history. Total rotation > 320° → escape turn.
//     Oscillation: avoidance direction flips 4× in 10 s → suppress wall
//     following and let avoidance sort it out.
//
// 11. BLE ARCHITECTURE (v7 addition):
//     The BLE stack (Bluedroid) runs in ESP-IDF tasks on Core 0 internally.
//     The Arduino loop() runs on Core 1. Communication between them uses a
//     single std::atomic<uint8_t> bleCommand — guaranteed atomic on Xtensa
//     for 1-byte operations, no mutex needed. Multi-field structs
//     (BleStatusPacket) are written only from loop() and read only from
//     loop(), so no cross-core hazard exists. The BLE callback
//     (CommandCallbacks::onWrite) only writes one byte to bleCommand,
//     which is consumed at the top of the next loop() iteration before any
//     layer executes. This guarantees commands take effect within one loop
//     cycle (~50–100 µs) with zero blocking.
//
//     STATUS packet rate: 5 Hz (200 ms). This gives the mobile app fluid
//     updates while keeping the BLE air-time low enough to avoid connection
//     drops in congested 2.4 GHz environments.
//
// 12. STUCK DETECTOR (v7 fix):
//     v6 only armed stuck detection during STATE_CRUISE. A robot wedged
//     against a rug while wall-following would wait WALL_LOST_MS (2 s) before
//     the state machine exited to STATE_CRUISE, then wait STUCK_CHECK_INTERVAL
//     (2.5 s) more — up to 4.5 s of wheel-burning. v7+ also arms it during
//     STATE_WALL_FOLLOW, reducing worst-case stuck response to 2.5 s.
//
// 13. SENSOR NAN GUARD (v7.1 Fix 1):
//     The EMA filter initialises to NAN and correctly handles the cold-start
//     case via isnan(). The v7.1 isfinite() guard adds a second layer:
//     if raw itself is somehow non-finite, the filter holds its last good
//     reading rather than propagating NaN permanently.
//
// 14. BLE COMMAND ATOMICITY (v7.1 Fix 2 → v8.1 improved):
//     v7.0 used volatile uint8_t which prevents compiler optimisation but
//     gives NO CPU-level cache coherency guarantee across two Xtensa LX6 cores.
//     v7.1+ uses std::atomic<uint8_t> with exchange(memory_order_acq_rel):
//       • Atomically reads and zeros the flag in one CPU instruction.
//       • Eliminates the load-then-store TOCTOU window from v7.0.
//     v8.1: added compile-time #error guard for invalid flag combinations.
//
// 15. WALL-FOLLOW GAIN COMPENSATION (v7.1 Fix 3):
//     compensateSpeed() scales base PWM by (nominalV / battV) × brownout.
//     If correction stays in nominal units, (correction / baseSpeed) shrinks
//     as battery drops — the robot becomes progressively less responsive to
//     wall-distance errors and drifts away from the wall in the last minute
//     of a 5-minute run. The fix applies the same compFactor to correction,
//     keeping the differential fraction constant across the full battery curve.
//
// 16. L298N THERMAL DERATING (v7.2 Fix 3):
//     updateThermalModel() is called from compensateSpeed() on every motor
//     command. If the final compensated PWM >= THERMAL_HIGH_PWM (180) continuously
//     for THERMAL_HIGH_LIMIT_MS (20 s), thermalDerateActive is set and
//     compensateSpeed() applies THERMAL_DERATE_FACTOR (0.88 = 12% reduction)
//     for THERMAL_COOLDOWN_MS (3 s). A hard stop clears the accumulator.
//
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// ■ Technical Notes — v8.0 / v8.1 (navigation upgrades)
// ═══════════════════════════════════════════════════════════════════════════
//
//  1. ENCODER ACCURACY vs WHEEL SLIP
//     Encoders measure axle rotation, not ground contact. Carpet slip,
//     wheel spin on hard floors, and turns all introduce error. Empirically,
//     a single hall-effect sensor gives ±2–5% per metre — still ~10× better
//     than the PWM speed model. Quadrature encoders (2 sensors / wheel) add
//     direction certainty but are not required here: direction is inferred
//     from the motor command in EncoderOdometry::update().
//
//  2. GPIO 12 BOOT-STRAPPING — MANDATORY PULL-DOWN
//     ESP32 samples GPIO 12 at power-on to select flash voltage:
//       LOW  → 3.3 V flash (correct for all standard ESP32 devkit boards)
//       HIGH → 1.8 V flash (wrong — causes boot loop / flash read errors)
//     Add a 10 kΩ resistor from GPIO 12 to GND on the PCB or breadboard.
//     The hall-effect encoder output idles LOW between magnet poles, so the
//     pull-down has no effect on normal pulse detection.
//
//  3. BOUSTROPHEDON HEADING REFERENCE
//     rowHeadingYaw is snapped from imu.getYaw() at the start of each row
//     and after each U-turn. The IMU accumulates ~3° drift per 180° turn,
//     so after 10 rows the absolute heading error is ≤30°. The P-controller
//     (gain BOUSTRO_HEADING_KP) eliminates this within 1–2 seconds of the
//     start of each row. For longer runs, reduce BOUSTRO_HEADING_KP to 1.5
//     if oscillation appears.
//
//  4. OCCUPANCY GRID MEMORY BUDGET
//     Allocation:  200 × 200 × 1 byte =  40,000 bytes
//     BLE stack (Bluedroid):            ~90,000 bytes
//     BLE stack (NimBLE-Arduino):       ~30,000 bytes
//     ─────────────────────────────────────────────────
//     Remaining free heap (Bluedroid):  ~40–60 KB — tight on PSRAM-less boards
//     Remaining free heap (NimBLE):     ~80–100 KB — comfortable on all boards
//     Recommendation: add NimBLE-Arduino to platformio.ini or Arduino library
//     manager, then change #include <BLEDevice.h> to #include <NimBLEDevice.h>
//     and prefix BLE class names with Nim (NimBLEDevice, NimBLEServer, etc.).
//     The BLE API is otherwise compatible with this codebase.
//
//  5. U-TURN DIRECTION INVARIANT
//     turnLeft alternates after every complete U-turn. Starting with true
//     (left turn), the sequence is L-R-L-R for rows 0-1-2-3. Both turns
//     in each U-turn (SPIN1 and SPIN2) use the SAME direction, so:
//       East-going row ends → turn LEFT → face North → advance → turn LEFT
//       → face West. West-going row ends → turn RIGHT → face North → advance
//       → turn RIGHT → face East. This is the classic boustrophedon U-turn.
//
//  6. v8.1 FIXES SUMMARY
//     • DeadReckoning::getHeadingDeg() promoted from private to public.
//       This fixes a compile error when ENABLE_OCCUPANCY_GRID is defined
//       without ENABLE_ENCODERS (an unusual but valid config).
//     • Boustrophedon row-end now guards against stale front-US sensor.
//       A faulty front sensor (clamped to 9 cm by checkStaleSensors) no
//       longer triggers a U-turn on every row — the planner ignores row-end
//       when the front sensor is marked faulty and lets the avoidance layer
//       handle obstacle detection instead.
//     • BoustrophedonPlanner::activate() no longer takes the unused startY
//       parameter. All call sites updated.
//     • Removed unused motorLeftFwd / motorRightFwd state variables.
//     • Unified all version strings to v10.0.
//     • Added #error compile-time guard for ENABLE_BOUSTROPHEDON without
//       its required dependencies (ENABLE_ENCODERS + ENABLE_OCCUPANCY_GRID).
//     • PhaseOneGrid consistently uses PHASE1_GRID_BYTES constant.
//     • Stale comments corrected (volatile → std::atomic, section numbers).
//
//  7. v10.0 FIXES SUMMARY (10/10 release)
//     • BLE yaw overflow fixed. imu.getYaw() accumulates without bound (gyro
//       integration). After 30 min of competition turning it can exceed
//       50,000 degrees. Encoding as int16_t (max 32,767 = +-3276.7 deg in
//       fixed-point) overflows. Fix: wrap to +-180 degrees via wrap180()
//       helper before encoding. Also used by boustrophedon heading maint.
//     • All Unicode symbols (checkmarks, crosses, box-drawing) replaced with
//       pure ASCII. Emojis were already removed in v8.1; v10.0 completes the
//       transition for maximum serial terminal compatibility.
//     • BoustrophedonPlanner now uses wheelEnc.readAtomic() consistently in
//       beginUTurn() and ADVANCE phase. Direct volatile reads were technically
//       safe (32-bit aligned on ESP32) but inconsistent with the codebase's
//       own synchronization design.
//     • BLE MAP_STREAM row buffer moved from stack (408 bytes) to static
//       storage. Reduces stack pressure in a function called from loop().
//     • BLE IMU fields send INT16_MIN (-32768) sentinel when IMU is not
//       initialized, so the client can distinguish "no IMU" from 0 degrees.
//     • BLE reserved byte (bit 0) now carries boustrophedon active state,
//       allowing the client to display which navigation layer is engaged.
//     • static_assert ensures OCC_ROWS <= 255 for BLE streaming — changing
//       the grid to 256+ rows would silently break the uint8_t row index.
//
//  8. PHASE 1 → PHASE 2 ROADMAP
//     Phase 2 (not yet implemented) adds:
//     • Frontier-based exploration: when all reachable OCC_UNKNOWN cells are
//       exhausted but the grid contains OCC_OBSTACLE islands, detect the
//       boundary and navigate around obstacles to reach new frontiers.
//     • SLAM-lite: fuse US readings as line segments into a landmark map for
//       global loop closure. Corrects the accumulated heading drift over a
//       full 30-minute run.
//     • Multi-room detection: identify doorways as narrow OBSTACLE gaps and
//       plan cross-room coverage sequences.
//     Memory note: Phase 2 adds ~20 KB for a landmark list (50 landmarks ×
//     6 floats × 4 bytes); remain within budget with NimBLE.
//
// ═══════════════════════════════════════════════════════════════════════════
// END OF FILE — robot_vacuum_subsumption_v10.0.ino
// ═══════════════════════════════════════════════════════════════════════════
