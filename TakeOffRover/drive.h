// drive.h — skid steer on one L298N: left pair on bridge A, right pair on bridge B.
// L298 truth table: EN=H, IN1!=IN2 -> run; EN=H, IN1=IN2 -> fast stop; EN=L -> coast.
#pragma once
#include <Arduino.h>
#include "config.h"
#include "pwm.h"
#include "drive_mix.h"

class Drive {
public:
  void begin() {
    // EN pins are held low by the pull-downs during boot; drive everything low before
    // attaching PWM so the bridges stay off until the first command.
    for (uint8_t p : {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENA, PIN_ENB}) {
      pinMode(p, OUTPUT);
      digitalWrite(p, LOW);
    }
    pwm::attach(PIN_ENA, CH_LEFT, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
    pwm::attach(PIN_ENB, CH_RIGHT, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
    coast();
  }

  // throttle, turn in [-1, 1]; scale caps top speed (precision mode, low battery).
  void command(float throttle, float turn, float scale, float dt) {
    drivemix::mix(throttle, turn, TURN_GAIN, scale, tgtL_, tgtR_);
    curL_ = ramp(curL_, tgtL_, dt);
    curR_ = ramp(curR_, tgtR_, dt);
    bool idle = fabsf(tgtL_) < DRIVE_IDLE_EPS && fabsf(tgtR_) < DRIVE_IDLE_EPS;
    if (idle && fabsf(curL_) < 0.05f && fabsf(curR_) < 0.05f) {
      curL_ = curR_ = 0.0f;
      brake();                       // hold position: no rolling out of a parking zone
      return;
    }
    drivemix::Out ol, orr;
    drivemix::duties(LEFT_INVERT  ? -curL_ : curL_,
                     RIGHT_INVERT ? -curR_ : curR_,
                     MIN_EFFECTIVE, PRESERVE_TURN_RATIO, DRIVE_IDLE_EPS, ol, orr);
    apply(PIN_IN1, PIN_IN2, PIN_ENA, CH_LEFT,  ol);
    apply(PIN_IN3, PIN_IN4, PIN_ENB, CH_RIGHT, orr);
  }

  void brake() {
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
    pwm::write(PIN_ENA, CH_LEFT, maxDuty());
    pwm::write(PIN_ENB, CH_RIGHT, maxDuty());
  }

  void coast() {
    pwm::write(PIN_ENA, CH_LEFT, 0);
    pwm::write(PIN_ENB, CH_RIGHT, 0);
    curL_ = curR_ = tgtL_ = tgtR_ = 0.0f;
  }

  void emergencyStop() { curL_ = curR_ = tgtL_ = tgtR_ = 0.0f; brake(); }

private:
  float curL_ = 0, curR_ = 0, tgtL_ = 0, tgtR_ = 0;

  static uint32_t maxDuty() { return (1UL << MOTOR_PWM_BITS) - 1; }

  static float ramp(float cur, float tgt, float dt) {
    bool speedingUp = (cur * tgt >= 0.0f) && fabsf(tgt) > fabsf(cur);
    float step = (speedingUp ? ACCEL_PER_S : DECEL_PER_S) * dt;
    if (tgt > cur) return fminf(tgt, cur + step);
    return fmaxf(tgt, cur - step);
  }

  // Put one bridge into the state drive_mix.h worked out.
  static void apply(uint8_t inA, uint8_t inB, uint8_t en, uint8_t ch, const drivemix::Out &o) {
    switch (o.mode) {
      case drivemix::Mode::Brake:    // idle side: hold it, so it anchors a pivot turn
        digitalWrite(inA, LOW); digitalWrite(inB, LOW);
        pwm::write(en, ch, maxDuty());
        return;
      case drivemix::Mode::Coast:    // commanded too low to turn a wheel: freewheel, no
        pwm::write(en, ch, 0);       // stall current and less scrub on the inside of a turn
        return;
      case drivemix::Mode::Drive:
        digitalWrite(inA, o.forward ? HIGH : LOW);
        digitalWrite(inB, o.forward ? LOW : HIGH);
        pwm::write(en, ch, (uint32_t)(o.duty * maxDuty()));
        return;
    }
  }
};
