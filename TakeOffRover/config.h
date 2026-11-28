// config.h — pin map and tunables for the TakeOFF Rover (ESP32 + Xbox controller).
#pragma once
#include <stdint.h>

// ---------- Arm master switch ----------
// FALSE: the arm and claw servos are never attached and never pulsed. The rover drives
// normally and the arm stays limp, so the firmware can be flown with the controller
// before the arm is calibrated. Keep it false until the shoulder and elbow calibration in
// the README is done, because the first servo pulse sends the arm straight to STOW at
// whatever angle the uncalibrated constants point at, which with the arm attached can
// drive it into the deck or jam the linkage flat.
// TRUE: full rover. Flip it only once arm::SHOULDER and arm::ELBOW hold measured numbers.
// A limp MG996R does not hold the arm up, so support or remove the arm while this is
// false and the arm is fitted.
constexpr bool ARM_ENABLED = true;

// The claw is a separate servo on its own gripper and does not use the shoulder/elbow
// geometry at all, so it can run while the arm is still disabled. Its travel is clamped
// to claw::MIN_US..MAX_US, which are the finger stops, so it cannot drive past them.
constexpr bool CLAW_ENABLED = true;

// ---------- Arm mode ----------
// TRUE: raw joint control. The right stick moves the two servo pulses directly, with no
// kinematics, no presets by position and no deck keep-out. It assumes nothing about where
// a pulse puts a joint, which is exactly why it works before the arm is calibrated and
// why a mechanically offset arm or a joint that cannot reach straight up does not matter.
// The cost: nothing stops the arm reaching the deck, so the driver's eyes are the limit.
// FALSE: the full IK build with Pick/Carry/Drop/Stow and the keep-out. Needs arm::SHOULDER
// and arm::ELBOW to hold measured numbers first, or the arm goes somewhere arbitrary.
constexpr bool ARM_RAW_MODE = true;

// Jog feel, and how far a preset may travel per second, in microseconds.
constexpr float ARM_RAW_US_S  = 320.0f;
constexpr float ARM_RAW_ACCEL = 1400.0f;

// Raw presets on A / B / Y / X, as (shoulder, elbow) pulses. Jog the arm where you want
// it, read the pair off the 5 Hz serial log, and paste it in here. 1500/1500 is mid
// travel on both joints, which is where the servos start.
struct RawPose { uint16_t s, e; };
constexpr RawPose RAW_PICK  = {1500, 1500};   // CAL
constexpr RawPose RAW_CARRY = {1500, 1500};   // CAL
constexpr RawPose RAW_DROP  = {1500, 1500};   // CAL
constexpr RawPose RAW_STOW  = {1500, 1500};   // CAL

// ---------- Bluetooth pairing ----------
// TRUE wipes every stored controller pairing at boot. Bluepad32 keeps pairing keys in
// flash, and a stale set makes a controller connect and drop straight back out, which
// looks like the Xbox light coming on and going off again with
// "could not find device for address ..." on the serial log.
// Set true, flash, pair the controller, then set it back to false and reflash - otherwise
// the keys are wiped on every boot and the controller has to be re-paired each time.
constexpr bool FORGET_BT_KEYS = false;

// ---------- Status LED ----------
// Says what the radio is doing with no laptop attached, which is the only way to tell on
// the field. GPIO2 is the blue onboard LED on most ESP32 dev boards; set HAVE_STATUS_LED
// false if yours has none or the pin is wanted for something else.
//   slow blink, once a second ... waiting for the controller
//   solid ....................... controller connected, driving
//   fast blink .................. controller lost mid-run, drive braked
constexpr bool    HAVE_STATUS_LED = true;
constexpr uint8_t PIN_STATUS_LED  = 2;

// ---------- Servos (signal only; power from the 5 V servo regulator, common ground) ----------
constexpr uint8_t PIN_SERVO_SHOULDER = 19;  // MG996R whose horn carries the 120 mm upper arm
constexpr uint8_t PIN_SERVO_ELBOW    = 18;  // MG996R driving the 30 mm crank -> push rod -> forearm
constexpr uint8_t PIN_SERVO_CLAW     = 21;  // SG90 on the gripper's spur gear

// ---------- L298N (left pair on bridge A, right pair on bridge B) ----------
// Taken from the working TakeoffBot sketch on 2026-11-28, which is the wiring actually on
// the rover. The previous map here (ENA 25, IN1 26, IN2 27, ENB 33, IN3 32, IN4 13) was
// never re-checked after the rewire and was wrong in a way that could not work: it drove
// GPIO13, which is not connected to anything, and never drove GPIO14, the right enable.
// With neither enable asserted the bridge stays disabled and the motors cannot turn.
constexpr uint8_t PIN_ENA = 27;  // left speed PWM
constexpr uint8_t PIN_IN1 = 32;
constexpr uint8_t PIN_IN2 = 33;
constexpr uint8_t PIN_ENB = 14;  // right speed PWM
constexpr uint8_t PIN_IN3 = 25;
constexpr uint8_t PIN_IN4 = 26;

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
// ---------- Stick mapping ----------
// Measured on the rover 2026-11-28: this controller reports the stick axes swapped, so
// left/right drove forwards/backwards and up/down steered. The swap applies to both
// sticks, since the right one feeds the arm the same way.
// If a direction is backwards rather than on the wrong axis, flip its INVERT instead.
// The 1 Hz serial log prints throttle and turn after this mapping, so push the stick
// forward and check throttle goes positive before anything is on the ground.
constexpr bool STICK_SWAP_XY    = true;
constexpr bool THROTTLE_INVERT  = false;
constexpr bool TURN_INVERT      = false;

constexpr float STICK_DEADBAND  = 0.08f;
constexpr float EXPO            = 0.6f;     // 0 = linear, 1 = cubic
constexpr float TURN_GAIN       = 0.8f;
constexpr float PRECISION_SCALE = 0.35f;    // hold LB (drive speed)
constexpr float ARM_PRECISION_SCALE = 0.4f; // hold LB: tip trim speed too, for lining up on a block
constexpr float ACCEL_PER_S     = 3.3f;     // 0 -> full in ~0.3 s
constexpr float DECEL_PER_S     = 6.0f;
constexpr float MIN_EFFECTIVE   = 0.25f;    // CAL: smallest duty that turns the wheels
                                            // DriveBenchTest's creep test measures this.
// Apply MIN_EFFECTIVE to the PAIR rather than to each side, so a turn keeps its commanded
// ratio instead of having the inner wheel floored up until it fights the turn. A side
// whose share lands below MIN_EFFECTIVE coasts, since that duty cannot turn a wheel.
// Set false to go back to the per-side floor if the rover drives better that way.
constexpr bool  PRESERVE_TURN_RATIO = true;
constexpr float DRIVE_IDLE_EPS      = 0.01f;  // below this a side is idle and braked

// ---------- Arm feel ----------
// The arm runs the same profile as the jaws (motion::step), so presets ease in and out
// instead of starting and stopping dead, and the right stick ramps the tip rather than
// snapping it to full speed in one tick. Trim decelerates harder than it accelerates, the
// same asymmetry drive.h uses, so letting go of the stick does not coast far.
constexpr float ARM_TRIM_MM_S           = 120.0f;  // right-stick tip speed
constexpr float ARM_TRIM_ACCEL_MM_S2    = 600.0f;  // tip ramps up in ~0.2 s
constexpr float ARM_TRIM_DECEL_MM_S2    = 1800.0f; // and stops in ~0.07 s (~4 mm of coast)
constexpr float ARM_PRESET_DEG_S        = 90.0f;   // fastest joint during a preset move
constexpr float ARM_PRESET_ACCEL_DEG_S2 = 320.0f;  // ease in/out of a preset move

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
// Expo on the trigger, like EXPO on the drive sticks. The jaws now open all the way to
// MIN_US, so without this most of the trigger travel is spent crossing a wide gap nobody
// is aiming at, and the part that matters -- closing the last few mm onto a block -- gets
// a sliver of travel. 0 = linear, 1 = fully cubic.
constexpr float CLAW_TRIGGER_EXPO = 0.6f;
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
