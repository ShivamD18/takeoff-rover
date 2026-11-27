// pwm.h — LEDC wrapper that builds on Arduino-ESP32 core 2.x and 3.x.
#pragma once
#include <Arduino.h>
#include "config.h"

namespace pwm {

inline void attach(uint8_t pin, uint8_t ch, uint32_t hz, uint8_t bits) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, hz, bits);
#else
  ledcSetup(ch, hz, bits);
  ledcAttachPin(pin, ch);
#endif
}

inline void write(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

// Servo pulse width in microseconds (0 = no pulse, servo limp).
inline void servoUs(uint8_t pin, uint8_t ch, uint16_t us) {
  uint32_t duty = (uint32_t)((uint64_t)us * ((1UL << SERVO_BITS) - 1) / (1000000UL / SERVO_HZ));
  write(pin, ch, duty);
}

} // namespace pwm
