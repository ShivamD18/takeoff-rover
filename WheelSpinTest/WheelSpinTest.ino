// WheelSpinTest.ino — the simplest possible proof that the wheels turn.
//
// No commands, no typing, no Serial Monitor needed. Flash it, put the wheels off the
// ground, and watch. It cycles through every motor and direction forever, 2 s each with a
// 1 s pause between, and names each step on Serial at 115200 if you care to look.
//
// Deliberately dumb: the enables are driven HIGH with digitalWrite, exactly as the
// TakeoffBot sketch does, so nothing here depends on PWM, duty cycle or MIN_EFFECTIVE.
// If the wheels will turn at all, they turn for this. If they do not turn for this, no
// firmware will turn them and the fault is power, wiring or the L298N itself.
//
// Pins from the TakeoffBot sketch, 2026-11-28: the wiring actually on the rover.
#include <Arduino.h>

constexpr uint8_t PIN_ENA = 27, PIN_IN1 = 32, PIN_IN2 = 33;   // left pair
constexpr uint8_t PIN_ENB = 14, PIN_IN3 = 25, PIN_IN4 = 26;   // right pair

constexpr uint32_t RUN_MS = 2000, PAUSE_MS = 1000;

// in1, in2 drive the left bridge; in3, in4 the right. -1 reverse, 0 stop, +1 forward.
void setSides(int left, int right) {
  digitalWrite(PIN_IN1, left > 0 ? HIGH : LOW);
  digitalWrite(PIN_IN2, left < 0 ? HIGH : LOW);
  digitalWrite(PIN_ENA, left != 0 ? HIGH : LOW);
  digitalWrite(PIN_IN3, right > 0 ? HIGH : LOW);
  digitalWrite(PIN_IN4, right < 0 ? HIGH : LOW);
  digitalWrite(PIN_ENB, right != 0 ? HIGH : LOW);
}

struct Step { const char *what; int left, right; };
const Step STEPS[] = {
  {"LEFT  forward", +1,  0},
  {"LEFT  reverse", -1,  0},
  {"RIGHT forward",  0, +1},
  {"RIGHT reverse",  0, -1},
  {"BOTH  forward", +1, +1},
  {"BOTH  reverse", -1, -1},
};

void setup() {
  for (uint8_t p : {PIN_ENA, PIN_IN1, PIN_IN2, PIN_ENB, PIN_IN3, PIN_IN4}) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  Serial.begin(115200);
  delay(300);
  Serial.println("\nWheelSpinTest. Wheels off the ground. Cycling every motor forever.");
  Serial.printf("left  ENA %d IN1 %d IN2 %d\n", PIN_ENA, PIN_IN1, PIN_IN2);
  Serial.printf("right ENB %d IN3 %d IN4 %d\n", PIN_ENB, PIN_IN3, PIN_IN4);
  Serial.println("Nothing turning here means power, wiring or the L298N, not the code.");
}

void loop() {
  for (const Step &s : STEPS) {
    Serial.printf("-> %s\n", s.what);
    setSides(s.left, s.right);
    delay(RUN_MS);
    setSides(0, 0);
    delay(PAUSE_MS);
  }
}
