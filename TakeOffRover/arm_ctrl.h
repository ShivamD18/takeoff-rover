// arm_ctrl.h — arm (2x MG996R) and claw (SG90) control.
// Manual trim moves the claw tip in straight lines; presets move in a straight line in
// joint space. Every commanded pose is checked against the joint limits and the
// deck/bracket keep-out before it is sent to the servos.
#pragma once
#include <Arduino.h>
#include "config.h"
#include "pwm.h"
#include "arm_kinematics.h"

class Arm {
public:
  void begin() {
    pwm::attach(PIN_SERVO_SHOULDER, CH_SHOULDER, SERVO_HZ, SERVO_BITS);
    pwm::attach(PIN_SERVO_ELBOW,    CH_ELBOW,    SERVO_HZ, SERVO_BITS);
    // Power on folded: the first pulse commands STOW (the arm will move there quickly).
    arm::solveIK(POSE_STOW.x, POSE_STOW.z, t1_, t2_);
    arm::forwardTip(t1_, t2_, x_, z_);
    write();
  }

  // Start a preset move. Returns false if the target itself is not allowed.
  bool goTo(const Pose &p) {
    float a, b;
    if (!arm::solveIK(p.x, p.z, a, b)) return false;
    st1_ = t1_; st2_ = t2_; gt1_ = a; gt2_ = b; f_ = 0.0f;
    float span = fmaxf(fabsf(gt1_ - st1_), fabsf(gt2_ - st2_));
    rate_ = span > 0.01f ? ARM_PRESET_DEG_S / span : 1000.0f;   // fraction of the move per second
    presetActive_ = true;
    return true;
  }

  // vx, vz in [-1, 1] from the right stick. Any stick input cancels a preset move.
  void update(float vx, float vz, float dt) {
    blocked_ = false;
    if (vx != 0.0f || vz != 0.0f) {
      presetActive_ = false;
      float dx = vx * ARM_TRIM_MM_S * dt, dz = vz * ARM_TRIM_MM_S * dt;
      // Try the full move, then each axis alone, so the tip slides along a limit
      // instead of freezing when one direction is blocked.
      if (!tryTip(x_ + dx, z_ + dz) && !(dx != 0.0f && tryTip(x_ + dx, z_)) &&
          !(dz != 0.0f && tryTip(x_, z_ + dz)))
        blocked_ = true;
    } else if (presetActive_) {
      float f = fminf(1.0f, f_ + rate_ * dt);
      float a = st1_ + f * (gt1_ - st1_), b = st2_ + f * (gt2_ - st2_);
      if (arm::poseAllowed(a, b)) {
        f_ = f; t1_ = a; t2_ = b;
        arm::forwardTip(t1_, t2_, x_, z_);
        if (f_ >= 1.0f) presetActive_ = false;
      } else {
        presetActive_ = false;   // path runs into the deck or a limit: stop where we are
        blocked_ = true;
      }
    }
    write();
  }

  void hold() { presetActive_ = false; write(); }   // keep pulses alive, stop moving

  bool blocked() const { return blocked_; }
  float tipX() const { return x_; }
  float tipZ() const { return z_; }
  float t1() const { return t1_; }
  float t2() const { return t2_; }

private:
  float t1_ = arm::SHOULDER.centreDeg, t2_ = arm::ELBOW.centreDeg, x_ = 0, z_ = 0;
  float st1_ = 0, st2_ = 0, gt1_ = 0, gt2_ = 0, f_ = 0, rate_ = 0;
  bool presetActive_ = false, blocked_ = false;

  bool tryTip(float nx, float nz) {
    float a, b;
    if (!arm::solveIK(nx, nz, a, b)) return false;
    x_ = nx; z_ = nz; t1_ = a; t2_ = b;
    return true;
  }
  void write() {
    pwm::servoUs(PIN_SERVO_SHOULDER, CH_SHOULDER, arm::shoulderUs(t1_));
    pwm::servoUs(PIN_SERVO_ELBOW,    CH_ELBOW,    arm::elbowUs(t2_));
  }
};

class Claw {
public:
  void begin() {
    pwm::attach(PIN_SERVO_CLAW, CH_CLAW, SERVO_HZ, SERVO_BITS);
    gap_ = target_ = claw::OPEN_GAP;
    write();
  }
  void open()  { target_ = claw::OPEN_GAP; gripping_ = false; }
  void grip()  { target_ = claw::gapForGrip(block_, claw::SQUEEZE); gripping_ = true; gripSince_ = millis(); }
  void toggle() { gripping_ ? open() : grip(); }
  void setBlock(claw::Block b) { block_ = b; if (gripping_) grip(); }
  claw::Block block() const { return block_; }

  void update(float dt) {
    // Long hold: back off the squeeze so the SG90 is not stalled hard indefinitely.
    if (gripping_ && millis() - gripSince_ > CLAW_RELIEF_MS)
      target_ = claw::gapForGrip(block_, claw::SQUEEZE_RELIEVED);
    float s = CLAW_MM_S * dt;
    if (fabsf(target_ - gap_) <= s) gap_ = target_;
    else gap_ += (target_ > gap_ ? s : -s);
    write();
  }

private:
  float gap_ = claw::OPEN_GAP, target_ = claw::OPEN_GAP;
  bool gripping_ = false;
  uint32_t gripSince_ = 0;
  claw::Block block_ = claw::Block::LEGO;
  // Pulses are never stopped: an SG90 without signal goes limp and drops the block.
  void write() { pwm::servoUs(PIN_SERVO_CLAW, CH_CLAW, claw::usForGap(gap_)); }
};
