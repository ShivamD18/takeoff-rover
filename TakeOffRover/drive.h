// drive.h — skid steer on one L298N: left pair on bridge A, right pair on bridge B.
// L298 truth table: EN=H, IN1!=IN2 -> run; EN=H, IN1=IN2 -> fast stop; EN=L -> coast.
#pragma once
#include <Arduino.h>
#include "config.h"
#include "pwm.h"

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
    float l = throttle + turn * TURN_GAIN;
    float r = throttle - turn * TURN_GAIN;
    float m = fmaxf(1.0f, fmaxf(fabsf(l), fabsf(r)));   // keep the turn ratio when saturated
    tgtL_ = l / m * scale;
    tgtR_ = r / m * scale;
    curL_ = ramp(curL_, tgtL_, dt);
    curR_ = ramp(curR_, tgtR_, dt);
    bool idle = fabsf(tgtL_) < 0.01f && fabsf(tgtR_) < 0.01f;
    if (idle && fabsf(curL_) < 0.05f && fabsf(curR_) < 0.05f) {
      curL_ = curR_ = 0.0f;
      brake();                       // hold position: no rolling out of a parking zone
      return;
    }
    side(PIN_IN1, PIN_IN2, PIN_ENA, CH_LEFT,  LEFT_INVERT  ? -curL_ : curL_);
    side(PIN_IN3, PIN_IN4, PIN_ENB, CH_RIGHT, RIGHT_INVERT ? -curR_ : curR_);
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

  static void side(uint8_t inA, uint8_t inB, uint8_t en, uint8_t ch, float v) {
    float mag = fabsf(v);
    if (mag < 0.01f) {               // this side idle while the other turns: brake it
      digitalWrite(inA, LOW); digitalWrite(inB, LOW);
      pwm::write(en, ch, maxDuty());
      return;
    }
    // Map (0,1] onto [MIN_EFFECTIVE,1] so small stick moves still turn the wheels.
    float duty = MIN_EFFECTIVE + (1.0f - MIN_EFFECTIVE) * fminf(mag, 1.0f);
    digitalWrite(inA, v > 0 ? HIGH : LOW);
    digitalWrite(inB, v > 0 ? LOW : HIGH);
    pwm::write(en, ch, (uint32_t)(duty * maxDuty()));
  }
};
