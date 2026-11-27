// ServoBenchTest.ino — TakeOFF Rover servo bench test and calibration.
// Serial Monitor at 115200, line ending "Newline".
//
// Wiring (rewired 2026-11-26):
//   s <us>   shoulder MG996R, GPIO19 (horn carries the long upper arm)
//   e <us>   elbow    MG996R, GPIO18 (horn carries the short crank that tilts the forearm)
//   c <us>   claw     SG90,   GPIO21
//   x        all servos limp (stop pulses)
//   t        telemetry on/off
//   h        help + calibration steps
// A trailing 'q' (e.g. "s 1500q") skips the echo; the TakeOFF Arm Live page uses it.
// Telemetry (20 Hz): T <s_us> <e_us> <c_us> <loop_max_us> <ms>   (0 = limp)
// Servos are LIMP at boot until you command them.
#include <Arduino.h>

struct Servo_ { char key; uint8_t pin; uint8_t ch; const char *name; int minUs, maxUs; int us; };
Servo_ servos[] = {
  {'s', 19, 2, "shoulder", 850, 2150, 0},   // arm limits match the rover firmware
  {'e', 18, 4, "elbow",    850, 2150, 0},
  {'c', 21, 6, "claw",     500, 2500, 0},   // wider for finding the finger stops
};
constexpr int PWM_HZ = 50, PWM_BITS = 16;
bool telemetry = true;

void writeUs(Servo_ &s, int us) {
  s.us = us;
  uint32_t duty = us ? (uint32_t)((uint64_t)us * ((1UL << PWM_BITS) - 1) / 20000UL) : 0;  // 0 = limp
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(s.pin, duty);
#else
  ledcWrite(s.ch, duty);
#endif
}

void help() {
  Serial.println(
    "Commands: s|e|c <us>, x (limp), t (telemetry), h (help)\n"
    "Calibration, horns OFF first, servo rail on the 5 V regulator:\n"
    " 1. c 1500, then s 1500, e 1500: each servo should move and hold. Measure the rail.\n"
    " 2. Fit the horns so at 1500 us the upper arm is 45 deg above forward-horizontal and\n"
    "    the forearm is 30 deg below it.\n"
    " 3. Direction: s 1550 should tip the upper arm UP/BACK; e 1450 should lift the front\n"
    "    of the forearm. If either is backwards, flip that sign in arm_kinematics.h\n"
    "    (SHOULDER.usPerDeg / ELBOW.usPerDeg). The two signs must stay opposite.\n"
    " 4. Step 50 us at a time toward each end; note where the linkage gets tight.\n"
    " 5. Claw: find fully open and fully closed pulses; measure the gap at 1500 us.");
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  char k = tolower(line[0]);
  bool quiet = line.endsWith("q");
  if (k == 'x') { for (auto &s : servos) writeUs(s, 0); Serial.println("> all limp"); return; }
  if (k == 't') { telemetry = !telemetry; Serial.printf("> telemetry %s\n", telemetry ? "on" : "off"); return; }
  if (k == 'h') { help(); return; }
  for (auto &s : servos) {
    if (k != s.key) continue;
    int us = line.substring(1).toInt();
    int c = constrain(us, s.minUs, s.maxUs);
    if (c != us) Serial.printf("! %s %d out of range [%d..%d], clamped to %d\n", s.name, us, s.minUs, s.maxUs, c);
    writeUs(s, c);
    if (!quiet) Serial.printf("> %s (GPIO%d) %d us\n", s.name, s.pin, c);
    return;
  }
  Serial.println("? use: s|e|c <us>, x, t, h");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  for (auto &s : servos) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(s.pin, PWM_HZ, PWM_BITS);
#else
    ledcSetup(s.ch, PWM_HZ, PWM_BITS);
    ledcAttachPin(s.pin, s.ch);
#endif
    writeUs(s, 0);
  }
  Serial.println("\nServoBenchTest ready. Servos limp. Shoulder=GPIO19, elbow=GPIO18, claw=GPIO21. Type h for help.");
}

String buf;

void loop() {
  static uint32_t lastLoop = micros(), loopMax = 0, lastTel = 0;
  uint32_t nowUs = micros();
  uint32_t dtUs = nowUs - lastLoop;
  lastLoop = nowUs;
  if (dtUs > loopMax) loopMax = dtUs;

  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') { handleLine(buf); buf = ""; }
    else if (buf.length() < 40) buf += ch;
  }

  uint32_t now = millis();
  if (telemetry && now - lastTel >= 50) {
    lastTel = now;
    Serial.printf("T %d %d %d %lu %lu\n", servos[0].us, servos[1].us, servos[2].us,
                  (unsigned long)loopMax, (unsigned long)now);
    loopMax = 0;
  }
}
