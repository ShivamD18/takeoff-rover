// DriveBenchTest.ino — TakeOFF Rover drive bench test.
// Verifies the L298N wiring, each side's direction, and the minimum duty that actually
// turns the wheels, WITHOUT flashing the full rover firmware (which waits on the arm
// calibration). Serial Monitor at 115200, line ending "Newline".
//
//   a <v>   left side  (bridge A), v = -1.00 .. 1.00   (negative = backwards)
//   b <v>   right side (bridge B)
//   d <v>   both sides together, for driving straight
//   c a|b   creep test: ramps that side up from zero. Send any line to stop it.
//           The duty it stops at is MIN_EFFECTIVE for config.h.
//   s       brake both      x  coast both (bridges off)
//   p       pin map         t  telemetry on/off      h  help
//
// WHEELS OFF THE GROUND for the first run: a miswired bridge drives at full duty.
// Every command restarts a 3 s deadman. With no new command the bridges brake on their
// own, so a typo cannot leave the rover driving off the bench.
#include <Arduino.h>

// Must match TakeOffRover/config.h. Duplicated because the Arduino IDE compiles only the
// sketch folder, the same reason ServoBenchTest carries its own copy of the servo pins.
constexpr uint8_t PIN_ENA = 25, PIN_IN1 = 26, PIN_IN2 = 27;   // left pair
constexpr uint8_t PIN_ENB = 33, PIN_IN3 = 32, PIN_IN4 = 13;   // right pair
constexpr uint32_t MOTOR_PWM_HZ = 1000;
constexpr uint8_t  MOTOR_PWM_BITS = 10;
constexpr uint8_t  CH_LEFT = 0, CH_RIGHT = 1;

constexpr uint32_t DEADMAN_MS = 3000;
constexpr uint32_t CREEP_STEP_MS = 400;
constexpr float    CREEP_STEP = 0.01f;

struct Side {
  char key; const char *name;
  uint8_t en, inA, inB, ch;
  float v;
};
Side sides[] = {
  {'a', "left  (bridge A)", PIN_ENA, PIN_IN1, PIN_IN2, CH_LEFT,  0.0f},
  {'b', "right (bridge B)", PIN_ENB, PIN_IN3, PIN_IN4, CH_RIGHT, 0.0f},
};

bool telemetry = true;
uint32_t lastCmd = 0;
int creepIdx = -1;                 // index into sides[], -1 = not creeping
float creepV = 0.0f;
uint32_t creepAt = 0;

static uint32_t maxDuty() { return (1UL << MOTOR_PWM_BITS) - 1; }

void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch; ledcWrite(pin, duty);
#else
  (void)pin; ledcWrite(ch, duty);
#endif
}

// Raw duty, no MIN_EFFECTIVE floor: this sketch exists to find that floor, so it must be
// able to command the duties below it that the rover firmware never would.
void drive(Side &s, float v) {
  s.v = constrain(v, -1.0f, 1.0f);
  float mag = fabsf(s.v);
  if (mag < 0.005f) {                       // brake: EN high, both inputs equal
    digitalWrite(s.inA, LOW); digitalWrite(s.inB, LOW);
    pwmWrite(s.en, s.ch, maxDuty());
    return;
  }
  digitalWrite(s.inA, s.v > 0 ? HIGH : LOW);
  digitalWrite(s.inB, s.v > 0 ? LOW : HIGH);
  pwmWrite(s.en, s.ch, (uint32_t)(mag * maxDuty()));
}

void brakeAll() { for (auto &s : sides) drive(s, 0.0f); }
void coastAll() {
  for (auto &s : sides) { s.v = 0.0f; digitalWrite(s.inA, LOW); digitalWrite(s.inB, LOW); pwmWrite(s.en, s.ch, 0); }
}

void pinMap() {
  Serial.printf("> left  ENA %d  IN1 %d  IN2 %d\n", PIN_ENA, PIN_IN1, PIN_IN2);
  Serial.printf("> right ENB %d  IN3 %d  IN4 %d\n", PIN_ENB, PIN_IN3, PIN_IN4);
  Serial.printf("> PWM %lu Hz, %u bits (max duty %lu)\n",
                (unsigned long)MOTOR_PWM_HZ, MOTOR_PWM_BITS, (unsigned long)maxDuty());
}

void help() {
  Serial.println(
    "Commands: a|b|d <-1..1>, c a|b (creep), s (brake), x (coast), p (pins), t, h\n"
    "Bring-up, WHEELS OFF THE GROUND:\n"
    " 1. p, and check the pins against the actual wiring before anything turns.\n"
    " 2. a 0.6: ONLY the left pair should turn, forwards. Then b 0.6 for the right.\n"
    "    A side that runs backwards -> set its INVERT in config.h. Nothing at all, or\n"
    "    both sides at once -> the wiring does not match the pin map above.\n"
    " 3. d 0.6 then d -0.6: both sides together, forwards then back.\n"
    " 4. c a: the duty creeps up until a wheel starts turning. Send any line to stop.\n"
    "    Repeat for c b, take the LARGER of the two, add ~0.02 -> MIN_EFFECTIVE.\n"
    " 5. Only once all four pass, put the wheels down and retest at 0.3.");
}

void startCreep(char which) {
  for (int i = 0; i < 2; i++)
    if (sides[i].key == which) {
      creepIdx = i; creepV = 0.0f; creepAt = millis();
      Serial.printf("> creep %s: duty climbing %.2f per %lu ms. Send any line when it turns.\n",
                    sides[i].name, CREEP_STEP, (unsigned long)CREEP_STEP_MS);
      return;
    }
  Serial.println("? use: c a  or  c b");
}

void stopCreep() {
  if (creepIdx < 0) return;
  Serial.printf("! %s started turning at duty %.2f -> MIN_EFFECTIVE about %.2f\n",
                sides[creepIdx].name, creepV, creepV + 0.02f);
  creepIdx = -1;
  brakeAll();
}

void handleLine(String line) {
  line.trim();
  lastCmd = millis();
  if (creepIdx >= 0) { stopCreep(); return; }     // any line stops a creep test
  if (line.length() == 0) return;
  char k = tolower(line[0]);
  String rest = line.substring(1); rest.trim();

  if (k == 's') { brakeAll(); Serial.println("> brake"); return; }
  if (k == 'x') { coastAll(); Serial.println("> coast (bridges off)"); return; }
  if (k == 'p') { pinMap(); return; }
  if (k == 'h') { help(); return; }
  if (k == 't') { telemetry = !telemetry; Serial.printf("> telemetry %s\n", telemetry ? "on" : "off"); return; }
  if (k == 'c') { startCreep(rest.length() ? tolower(rest[0]) : ' '); return; }
  if (k == 'd') {
    float v = rest.toFloat();
    for (auto &s : sides) drive(s, v);
    Serial.printf("> both %.2f\n", v);
    return;
  }
  for (auto &s : sides)
    if (k == s.key) {
      float v = rest.toFloat();
      drive(s, v);
      Serial.printf("> %s %.2f\n", s.name, s.v);
      return;
    }
  Serial.println("? use: a|b|d <-1..1>, c a|b, s, x, p, t, h");
}

void setup() {
  // Inputs low and the enables off before PWM is attached, so a bridge cannot twitch
  // while the pins are still floating during boot.
  for (uint8_t p : {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENA, PIN_ENB}) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  Serial.begin(115200);
  delay(300);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_ENA, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
  ledcAttach(PIN_ENB, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
#else
  ledcSetup(CH_LEFT, MOTOR_PWM_HZ, MOTOR_PWM_BITS);   ledcAttachPin(PIN_ENA, CH_LEFT);
  ledcSetup(CH_RIGHT, MOTOR_PWM_HZ, MOTOR_PWM_BITS);  ledcAttachPin(PIN_ENB, CH_RIGHT);
#endif
  coastAll();
  lastCmd = millis();
  Serial.println("\nDriveBenchTest ready. Bridges off. WHEELS OFF THE GROUND. Type h for help.");
  pinMap();
}

String buf;

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') { handleLine(buf); buf = ""; }
    else if (buf.length() < 40) buf += ch;
  }

  uint32_t now = millis();

  if (creepIdx >= 0 && now - creepAt >= CREEP_STEP_MS) {
    creepAt = now;
    creepV += CREEP_STEP;
    lastCmd = now;                                  // a running creep is not idle
    if (creepV > 1.0f) { Serial.println("! reached full duty without turning: check the battery and wiring"); stopCreep(); }
    else { drive(sides[creepIdx], creepV); Serial.printf("  duty %.2f\n", creepV); }
  }

  // Deadman: no command for DEADMAN_MS and anything still driving gets braked.
  if (creepIdx < 0 && now - lastCmd > DEADMAN_MS) {
    bool moving = false;
    for (auto &s : sides) if (fabsf(s.v) >= 0.005f) moving = true;
    if (moving) { brakeAll(); Serial.println("! deadman: no command for 3 s, braked"); }
    lastCmd = now;
  }

  static uint32_t lastTel = 0;
  if (telemetry && now - lastTel >= 500) {
    lastTel = now;
    Serial.printf("T a %.2f b %.2f %lu\n", sides[0].v, sides[1].v, (unsigned long)now);
  }
}
