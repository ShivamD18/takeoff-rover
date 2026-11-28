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

// Raw joint control, for before the arm is calibrated. The sticks move the servo pulses
// themselves, so nothing here depends on us0, usPerDeg, the link lengths or the keep-out.
// Both joints run the same profile the rest of the rover uses, in microseconds instead of
// degrees, so jogging and presets are as smooth as everything else.
class ArmRaw {
public:
  // Deliberately does not attach. The servos stay limp until the driver asks for motion,
  // so plugging the battery in never moves the arm.
  void begin() {}

  // ds, de in [-1, 1] from the right stick.
  void jog(float ds, float de, float dt) {
    if (!live_) {
      if (ds == 0.0f && de == 0.0f) return;
      attach();
      return;                       // first input only wakes it; it does not also move
    }
    tgtS_ = clamp(tgtS_ + ds * ARM_RAW_US_S * dt);
    tgtE_ = clamp(tgtE_ + de * ARM_RAW_US_S * dt);
  }

  void goTo(const RawPose &p) {
    if (!live_) attach();
    tgtS_ = clamp((float)p.s);
    tgtE_ = clamp((float)p.e);
  }

  void update(float dt) {
    if (!live_) return;
    // The profile is given headroom over the jog rate so it tracks the stick instead of
    // sitting pinned at its own speed limit, which is what made jogging feel laggy and
    // leave the arm drifting on after the stick was released.
    const float track = ARM_RAW_US_S * ARM_RAW_TRACK;
    motion::step(ms_, tgtS_, dt, track, ARM_RAW_ACCEL);
    motion::step(me_, tgtE_, dt, track, ARM_RAW_ACCEL);
    write();
  }

  void hold() { if (live_) write(); }          // keep the pulses alive, stop moving
  bool live() const { return live_; }
  uint16_t shoulderUs() const { return (uint16_t)lroundf(ms_.pos); }
  uint16_t elbowUs() const { return (uint16_t)lroundf(me_.pos); }

private:
  motion::State ms_, me_;
  float tgtS_ = 1500.0f, tgtE_ = 1500.0f;
  bool live_ = false;

  // Waking the servos sends them to mid travel from wherever the arm physically sits, so
  // it can move a long way on that first command. That is unavoidable without feedback:
  // hold the arm the first time.
  void attach() {
    live_ = true;
    ms_ = motion::State(1500.0f, 0.0f);
    me_ = motion::State(1500.0f, 0.0f);
    tgtS_ = tgtE_ = 1500.0f;
    pwm::attach(PIN_SERVO_SHOULDER, CH_SHOULDER, SERVO_HZ, SERVO_BITS);
    pwm::attach(PIN_SERVO_ELBOW,    CH_ELBOW,    SERVO_HZ, SERVO_BITS);
    write();
  }
  static float clamp(float us) { return fmaxf(arm::SHOULDER.minUs, fminf(arm::SHOULDER.maxUs, us)); }
  void write() {
    pwm::servoUs(PIN_SERVO_SHOULDER, CH_SHOULDER, shoulderUs());
    pwm::servoUs(PIN_SERVO_ELBOW,    CH_ELBOW,    elbowUs());
  }
};

// Claw in raw microseconds: two taught endpoints and a profile between them. Nothing here
// uses the gap model, the pitch radius or any millimetre figure, so it describes the
// gripper that is actually on the rover.
class ClawRaw {
public:
  void begin(uint16_t openUs, uint16_t closedUs) {
    openUs_ = clamp(openUs);
    closedUs_ = clamp(closedUs);
    m_ = motion::State((float)openUs_, 0.0f);
    tgt_ = (float)openUs_;
    pwm::attach(PIN_SERVO_CLAW, CH_CLAW, SERVO_HZ, SERVO_BITS);
    write();
  }

  void openJaws()  { closed_ = false; tgt_ = (float)openUs_; }
  void closeJaws() { closed_ = true;  tgt_ = (float)closedUs_; closedAt_ = millis(); }
  void toggle()    { closed_ ? openJaws() : closeJaws(); }

  // Analog trigger, 0..1, straight across the taught span. Overrides the toggle while it
  // is held; letting go latches wherever the driver stopped.
  void trim(float trig, float dt) {
    if (trig > CLAW_TRIGGER_DB) {
      trigFilt_ += dt / (CLAW_TRIGGER_TAU + dt) * (trig - trigFilt_);
      float f = fmaxf(0.0f, (fminf(trigFilt_, 1.0f) - CLAW_TRIGGER_DB) / (1.0f - CLAW_TRIGGER_DB));
      f = (1.0f - CLAW_TRIGGER_EXPO) * f + CLAW_TRIGGER_EXPO * f * f * f;
      tgt_ = (float)openUs_ + f * (float)((int)closedUs_ - (int)openUs_);
      trigger_ = true;
      closed_ = f > 0.5f;
      closedAt_ = millis();
    } else if (trigger_) {
      trigger_ = false;
      trigFilt_ = 0.0f;
      tgt_ = m_.pos;
      closedAt_ = millis();
    } else {
      trigFilt_ = 0.0f;
    }
  }

  void update(float dt) {
    // Long hold: ease off the overlap so the servo is not pushing at full stall forever.
    if (closed_ && !trigger_ && millis() - closedAt_ > CLAW_RELIEF_MS) {
      float eased = (float)closedUs_ - (closedUs_ > openUs_ ? CLAW_RELIEF_BACKOFF : -CLAW_RELIEF_BACKOFF);
      if (fabsf(eased - tgt_) > 0.5f) tgt_ = clampf(eased);
    }
    motion::step(m_, tgt_, dt, CLAW_US_S, CLAW_ACCEL_US);
    write();
  }

  void hold() { write(); }
  void teachOpen()   { openUs_   = (uint16_t)lroundf(m_.pos); }
  void teachClosed() { closedUs_ = (uint16_t)lroundf(m_.pos); }
  uint16_t us() const { return (uint16_t)lroundf(m_.pos); }
  uint16_t openUs() const { return openUs_; }
  uint16_t closedUs() const { return closedUs_; }
  bool closed() const { return closed_; }

private:
  motion::State m_;
  float tgt_ = 1500.0f, trigFilt_ = 0.0f;
  uint16_t openUs_ = CLAW_OPEN_US_DEFAULT, closedUs_ = CLAW_CLOSED_US_DEFAULT, lastUs_ = 0;
  bool closed_ = false, trigger_ = false;
  uint32_t closedAt_ = 0;

  static uint16_t clamp(uint16_t u) { return u < CLAW_MIN_US ? CLAW_MIN_US : (u > CLAW_MAX_US ? CLAW_MAX_US : u); }
  static float clampf(float u) { return fmaxf((float)CLAW_MIN_US, fminf((float)CLAW_MAX_US, u)); }
  void write() {
    uint16_t u = (uint16_t)lroundf(clampf(m_.pos));
    if (u != lastUs_) { pwm::servoUs(PIN_SERVO_CLAW, CH_CLAW, u); lastUs_ = u; }
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

  void open()     { target_ = claw::OPEN_GAP; gripping_ = false; slow_ = false; }
  void approach() { target_ = claw::gapForApproach(block_); gripping_ = false; slow_ = false; }
  void grip()     { target_ = claw::gapForGrip(block_); gripping_ = true; slow_ = false; gripSince_ = millis(); }
  void toggle()   { gripping_ ? open() : grip(); }

  // Hold-to-open: held opens only as far as it takes to accept or release the block,
  // released closes onto it. Safer than a toggle, which leaves the jaws fully open and
  // wide enough to knock neighbouring blocks on the way down. The trigger still overrides
  // it while squeezed, for anything that needs a width this does not give.
  void hold(bool held) {
    if (trigger_) return;
    if (held) { if (gripping_ || !holding_) approach(); }
    else if (holding_) grip();
    holding_ = held;
  }
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
      // Expo: most of the travel sweeps the wide-open end quickly, and the squeeze onto
      // the block gets the fine end of the trigger.
      f = (1.0f - CLAW_TRIGGER_EXPO) * f + CLAW_TRIGGER_EXPO * f * f * f;
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
  bool gripping_ = false, trigger_ = false, slow_ = false, holding_ = false;
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
