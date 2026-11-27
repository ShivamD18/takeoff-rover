// config.h — pin map and tunables for the TakeOFF Rover (ESP32 + Xbox controller).
#pragma once
#include <stdint.h>

// ---------- Servos (signal only; power from the 5 V servo regulator, common ground) ----------
constexpr uint8_t PIN_SERVO_SHOULDER = 19;  // MG996R whose horn carries the 120 mm upper arm
constexpr uint8_t PIN_SERVO_ELBOW    = 18;  // MG996R driving the 30 mm crank -> push rod -> forearm
constexpr uint8_t PIN_SERVO_CLAW     = 21;  // SG90 on the gripper's spur gear

// ---------- L298N (left pair on bridge A, right pair on bridge B) ----------
// Not re-checked since the servo rewire: confirm against the wiring before driving.
constexpr uint8_t PIN_ENA = 25;  // left speed PWM  (ENA jumper removed, 10k pull-down)
constexpr uint8_t PIN_IN1 = 26;
constexpr uint8_t PIN_IN2 = 27;
constexpr uint8_t PIN_ENB = 33;  // right speed PWM (ENB jumper removed, 10k pull-down)
constexpr uint8_t PIN_IN3 = 32;
constexpr uint8_t PIN_IN4 = 13;

// ---------- Battery sense (optional) ----------
// Leave false unless the 100k/33k divider is fitted: a floating GPIO34 reads near 0 V,
// which would trip the critical-battery stop and freeze the rover.
constexpr bool    HAVE_VBAT_SENSE = false;
constexpr uint8_t PIN_VBAT = 34;            // ADC1, input-only, works with Bluetooth on
constexpr float   VBAT_DIVIDER = (100.0f + 33.0f) / 33.0f;
constexpr float   VBAT_LOW  = 6.8f;         // CAL for the real battery (these assume 2S LiPo)
constexpr float   VBAT_CRIT = 6.4f;

// ---------- LEDC (channels only matter on Arduino-ESP32 core 2.x) ----------
constexpr uint8_t CH_LEFT = 0, CH_RIGHT = 1;                 // share timer 0
constexpr uint8_t CH_SHOULDER = 2, CH_ELBOW = 4, CH_CLAW = 6;  // timers 1..3 at 50 Hz
constexpr uint32_t MOTOR_PWM_HZ   = 1000;
constexpr uint8_t  MOTOR_PWM_BITS = 10;
constexpr uint32_t SERVO_HZ       = 50;
constexpr uint8_t  SERVO_BITS     = 16;     // 20 ms frame, ~0.3 us per step

// ---------- Drive feel ----------
constexpr bool  LEFT_INVERT  = false;       // CAL: flip if a side drives backwards
constexpr bool  RIGHT_INVERT = true;
constexpr float STICK_DEADBAND  = 0.08f;
constexpr float EXPO            = 0.6f;     // 0 = linear, 1 = cubic
constexpr float TURN_GAIN       = 0.8f;
constexpr float PRECISION_SCALE = 0.35f;    // hold LB
constexpr float ACCEL_PER_S     = 3.3f;     // 0 -> full in ~0.3 s
constexpr float DECEL_PER_S     = 6.0f;
constexpr float MIN_EFFECTIVE   = 0.25f;    // CAL: smallest duty that turns the wheels

// ---------- Arm feel ----------
constexpr float ARM_TRIM_MM_S    = 120.0f;  // right-stick tip speed
constexpr float ARM_PRESET_DEG_S = 90.0f;   // fastest joint during a preset move

// ---------- Claw feel ----------
// The jaws run a speed- and acceleration-limited profile, so they ease in and out of
// every move instead of stepping to full speed in one 20 ms tick. An SG90 moves the gap
// at roughly 175 mm/s unloaded, so the peak below is the profile's limit, not the
// servo's: the fingers actually follow the ramp. Open -> LEGO grip takes ~0.70 s,
// quicker than the old constant-speed ramp and without the snatch at either end.
constexpr float CLAW_MM_S         = 55.0f;   // peak jaw speed
constexpr float CLAW_ACCEL_MM_S2  = 420.0f;  // ease in/out: reaches peak speed in ~0.13 s
constexpr float CLAW_TRIGGER_DB   = 0.06f;   // RT travel ignored before it takes over
constexpr float CLAW_TRIGGER_TAU  = 0.05f;   // RT smoothing: takes thumb shake and ADC noise out
constexpr float CLAW_RELIEF_MM_S  = 2.5f;    // slow creep when easing a long squeeze
constexpr uint32_t CLAW_RELIEF_MS = 20000;   // ease the squeeze after a long hold

// ---------- Presets: claw tip (x ahead of the shoulder axis, z = finger bottom above floor), mm ----------
// Checked by tools/arm_check.cpp: inside the joint limits with >= 17 deg to spare, clear of the
// deck and bracket, and every straight joint-space move between them is clear too.
struct Pose { float x, z; };
constexpr Pose POSE_PICK  = {170.0f,   5.0f};  // fingers 5 mm off the floor; trim down with the stick
constexpr Pose POSE_CARRY = {140.0f,  60.0f};  // block clears 1-inch walls and bumps
constexpr Pose POSE_DROP  = {200.0f,  55.0f};  // reach over a 1-inch wall, then open
constexpr Pose POSE_STOW  = {115.0f, 120.0f};  // folded up, matches the pose the CAD is drawn in
