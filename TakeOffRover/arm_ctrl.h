// arm_ctrl.h — arm (2x MG996R) and claw (SG90) control.
// Manual trim moves the claw tip in straight lines; presets move in a straight line in
// joint space. Every commanded pose is checked against the joint limits and the
// deck/bracket keep-out before it is sent to the servos.
// The claw runs a speed- and acceleration-limited (trapezoidal) gap profile and takes a
// proportional command from the right trigger; see the Claw class below.
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
  // The move runs as a profile along its own length in degrees, so it accelerates away
  // from the start pose and decelerates into the target instead of jerking at both ends.
  bool goTo(const Pose &p) {
    float a, b;
    if (!arm::solveIK(p.x, p.z, a, b)) return false;
    st1_ = t1_; st2_ = t2_; gt1_ = a; gt2_ = b;
    span_ = fmaxf(fabsf(gt1_ - st1_), fabsf(gt2_ - st2_));
    move_ = motion::State{};
    if (span_ <= 0.01f) {                      // already there
      t1_ = gt1_; t2_ = gt2_;
      arm::forwardTip(t1_, t2_, x_, z_);
      presetActive_ = false;
      write();
      return true;
    }
    presetActive_ = true;
    return true;
  }

  // vx, vz in [-1, 1] from the right stick. Any stick input cancels a preset move.
  void update(float vx, float vz, float dt) {
    blocked_ = false;
    bool stick = (vx != 0.0f || vz != 0.0f);
    if (stick) presetActive_ = false;

    if (stick || tvx_ != 0.0f || tvz_ != 0.0f) {
      // Ramp the tip velocity rather than jumping to it. Releasing the stick glides to a
      // stop over ARM_TRIM_DECEL_MM_S2 instead of halting the servos mid-sweep.
      tvx_ = approach(tvx_, vx * ARM_TRIM_MM_S, dt);
      tvz_ = approach(tvz_, vz * ARM_TRIM_MM_S, dt);
      float dx = tvx_ * dt, dz = tvz_ * dt;
      // Try the full move, then each axis alone, so the tip slides along a limit
      // instead of freezing when one direction is blocked. Zero the velocity of whichever
      // axis is against the limit, or it winds up and lurches when the tip comes free.
      if (!tryTip(x_ + dx, z_ + dz)) {
        bool okX = dx != 0.0f && tryTip(x_ + dx, z_);
        bool okZ = !okX && dz != 0.0f && tryTip(x_, z_ + dz);
        if (okX)      tvz_ = 0.0f;
        else if (okZ) tvx_ = 0.0f;
        else { tvx_ = tvz_ = 0.0f; blocked_ = true; }
      }
    } else if (presetActive_) {
      bool done = motion::step(move_, span_, dt, ARM_PRESET_DEG_S, ARM_PRESET_ACCEL_DEG_S2);
      float f = move_.pos / span_;
      float a = st1_ + f * (gt1_ - st1_), b = st2_ + f * (gt2_ - st2_);
      if (arm::poseAllowed(a, b)) {
        t1_ = a; t2_ = b;
        arm::forwardTip(t1_, t2_, x_, z_);
        if (done) presetActive_ = false;
      } else {
        presetActive_ = false;   // path runs into the deck or a limit: stop where we are
        blocked_ = true;
      }
    }
    write();
  }

  // Keep pulses alive but stop moving. Velocities are cleared so a reconnect does not
  // resume a sweep the driver started before the link dropped.
  void hold() { presetActive_ = false; tvx_ = tvz_ = 0.0f; write(); }

  bool blocked() const { return blocked_; }
  float tipX() const { return x_; }
  float tipZ() const { return z_; }
  float t1() const { return t1_; }
  float t2() const { return t2_; }

private:
  float t1_ = arm::SHOULDER.centreDeg, t2_ = arm::ELBOW.centreDeg, x_ = 0, z_ = 0;
  float st1_ = 0, st2_ = 0, gt1_ = 0, gt2_ = 0, span_ = 0;
  float tvx_ = 0, tvz_ = 0;                  // tip velocity, mm/s
  motion::State move_;                       // progress in degrees along a preset move
  bool presetActive_ = false, blocked_ = false;

  // Same asymmetry as drive.h's ramp: accelerate gently, brake harder.
  static float approach(float cur, float tgt, float dt) {
    bool speedingUp = (cur * tgt >= 0.0f) && fabsf(tgt) > fabsf(cur);
    float step = (speedingUp ? ARM_TRIM_ACCEL_MM_S2 : ARM_TRIM_DECEL_MM_S2) * dt;
    if (tgt > cur) return fminf(tgt, cur + step);
    return fmaxf(tgt, cur - step);
  }

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
    vel_ = 0.0f;
    lastUs_ = 0;
    write();
  }

  void open()   { target_ = claw::OPEN_GAP; gripping_ = false; slow_ = false; }
  void grip()   { target_ = claw::gapForGrip(block_); gripping_ = true; slow_ = false; gripSince_ = millis(); }
  void toggle() { gripping_ ? open() : grip(); }
  void setBlock(claw::Block b) { block_ = b; if (gripping_ && !trigger_) grip(); }
  claw::Block block() const { return block_; }

  // Analog right trigger in [0, 1]: 0 = fully open, 1 = closed on the current block.
  // While it is held past the deadband it commands the gap directly and overrides the
  // RB toggle; let go and the jaws hold the gap they reached, so a block stays gripped
  // without keeping a finger on the trigger. The motion profile below smooths a shaky
  // thumb, so the fingers never twitch even if the trigger does.
  void trim(float trig, float dt) {
    // Release is tested on the RAW trigger, so letting go latches the gap at once. The
    // gap it commands while held comes from a one-pole filter, because the trigger is a
    // 10-bit analog axis under a thumb on a driving robot and its raw value jitters.
    // Starting the filter from zero on each press also gives the squeeze a soft start.
    if (trig > CLAW_TRIGGER_DB) {
      trigFilt_ += dt / (CLAW_TRIGGER_TAU + dt) * (trig - trigFilt_);
      float f = (fminf(trigFilt_, 1.0f) - CLAW_TRIGGER_DB) / (1.0f - CLAW_TRIGGER_DB);
      f = fmaxf(0.0f, f);
      float closed = claw::gapForGrip(block_);
      target_ = claw::OPEN_GAP + f * (closed - claw::OPEN_GAP);
      trigger_ = true;
      slow_ = false;
      gripping_ = f > 0.5f;
      gripSince_ = millis();
    } else if (trigger_) {
      trigger_ = false;
      trigFilt_ = 0.0f;
      target_ = gap_;               // latch the gap the driver let go at
      gripSince_ = millis();
    } else {
      trigFilt_ = 0.0f;
    }
  }

  void update(float dt) {
    // Long hold: back off the squeeze so the SG90 is not stalled hard indefinitely.
    // Crept in at CLAW_RELIEF_MM_S, because a step here would twitch the block.
    if (gripping_ && !trigger_ && !slow_ && millis() - gripSince_ > CLAW_RELIEF_MS) {
      float relieved = claw::gapForRelief(block_);
      if (relieved > target_) { target_ = relieved; slow_ = true; }
    }

    motion::State m{gap_, vel_};
    bool arrived = motion::step(m, target_, dt, slow_ ? CLAW_RELIEF_MM_S : CLAW_MM_S, CLAW_ACCEL_MM_S2);
    gap_ = m.pos;
    vel_ = m.vel;
    if (arrived) slow_ = false;                  // next command runs at full speed again
    write();
  }

  float gap() const { return gap_; }
  float vel() const { return vel_; }
  uint16_t us() const { return lastUs_; }
  bool gripping() const { return gripping_; }

private:
  float gap_ = claw::OPEN_GAP, target_ = claw::OPEN_GAP, vel_ = 0.0f, trigFilt_ = 0.0f;
  bool gripping_ = false, trigger_ = false, slow_ = false;
  uint32_t gripSince_ = 0;
  uint16_t lastUs_ = 0;
  claw::Block block_ = claw::Block::LEGO;

  // Pulses are never stopped: an SG90 without signal goes limp and drops the block.
  // LEDC keeps repeating the last duty on its own, so writing only when the rounded
  // pulse actually changes keeps the jaws held while stopping the sub-microsecond
  // dither at the end of a move from buzzing the servo.
  void write() {
    uint16_t us = claw::usForGap(gap_);
    if (us != lastUs_) { pwm::servoUs(PIN_SERVO_CLAW, CH_CLAW, us); lastUs_ = us; }
  }
};
