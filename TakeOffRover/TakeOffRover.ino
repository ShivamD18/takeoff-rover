// TakeOffRover.ino — The Big 6ixers, TakeOFF Rover firmware (ESP32 + Xbox Wireless controller).
// Arm geometry rebuilt from the Fusion STEP export (2026-11-26); see arm_kinematics.h.
//
// Build: Arduino IDE / arduino-cli, board package esp32_bluepad32 4.1.0, board "ESP32 Dev Module".
//
// Wiring: GPIO19 shoulder MG996R (upper arm), GPIO18 elbow MG996R (crank), GPIO21 claw SG90.
//
// Controls (Xbox layout):
//   Left stick ........ drive (up/down = throttle, left/right = turn)
//   LB (hold) ......... precision mode: 35 % drive speed, 40 % arm tip speed
//   Right stick ....... ARM_RAW_MODE: up/down = shoulder, left/right = elbow (raw pulses)
//                       otherwise: move the claw tip, left/right = reach, up/down = height
//   B / X / Y / A ..... arm presets: UP / DOWN / DROP-over-wall / STOW
//   LB + preset ....... save where the arm is now into that preset (kept in flash)
//   RT (analog) ....... squeeze the claw proportionally; let go and it holds the gap
//                       (works with ARM_ENABLED false: the claw shares no arm geometry)
//   RB (hold) ......... open the jaws just enough to take or release the block;
//                       let go and they close onto it
//   D-pad left/right .. block size LEGO / Duplo (1 short or 1 long rumble)
//   Short buzz ........ the arm refused a move (limit, deck or bracket in the way)
//
// Safety: drive brakes and the arm/claw hold whenever the controller is not connected.
// The physical kill switch is still the primary emergency stop.
#include <Bluepad32.h>
#include <Preferences.h>
#include "config.h"
#include "drive.h"
#include "arm_ctrl.h"

Drive drive;
Arm arm_;
ArmRaw armRaw_;
Preferences prefs;

// Raw presets, in button order: A stow, B up, X down, Y drop. Loaded from flash at boot,
// falling back to the config.h values, and rewritten whenever one is taught.
RawPose poses[4] = {RAW_STOW, RAW_UP, RAW_DOWN, RAW_DROP};
const char *POSE_NAME[4] = {"STOW (A)", "UP (B)", "DOWN (X)", "DROP (Y)"};

void loadPoses() {
  prefs.begin("takeoff", true);                  // read-only
  for (int i = 0; i < 4; i++) {
    char ks[8], ke[8];
    snprintf(ks, sizeof ks, "p%ds", i);
    snprintf(ke, sizeof ke, "p%de", i);
    poses[i].s = prefs.getUShort(ks, poses[i].s);
    poses[i].e = prefs.getUShort(ke, poses[i].e);
  }
  prefs.end();
}

void savePose(int i, uint16_t s, uint16_t e) {
  poses[i].s = s;
  poses[i].e = e;
  prefs.begin("takeoff", false);
  char ks[8], ke[8];
  snprintf(ks, sizeof ks, "p%ds", i);
  snprintf(ke, sizeof ke, "p%de", i);
  prefs.putUShort(ks, s);
  prefs.putUShort(ke, e);
  prefs.end();
  Serial.printf("Saved %s = {%u, %u}\n", POSE_NAME[i], s, e);
}
Claw claw_;
ControllerPtr pad = nullptr;
uint32_t padLostAt = 0;        // millis the controller was last lost
bool everConnected = false;    // has one ever connected since boot

// ---------- Battery (only with the divider fitted) ----------
float vbat = 8.0f;
bool lowBatt = false, critBatt = false;

void updateBattery() {
  if (!HAVE_VBAT_SENSE) return;
  float v = analogReadMilliVolts(PIN_VBAT) / 1000.0f * VBAT_DIVIDER;
  vbat += 0.1f * (v - vbat);                                   // ~1 s filter at 10 Hz
  if (!critBatt && vbat < VBAT_CRIT) critBatt = true;
  if (critBatt && vbat > VBAT_CRIT + 0.2f) critBatt = false;
  bool wasLow = lowBatt;
  if (!lowBatt && vbat < VBAT_LOW) lowBatt = true;
  if (lowBatt && vbat > VBAT_LOW + 0.2f) lowBatt = false;
  if (lowBatt && !wasLow && pad) pad->playDualRumble(0, 600, 0xC0, 0xC0);
}

// ---------- Controller connection ----------
void onConnected(ControllerPtr c) {
  // Say why before hanging up on anything. Silently disconnecting here is
  // indistinguishable from a pairing fault at the bench: the light comes on, goes off,
  // and the log blames Bluetooth for what was actually this function's doing.
  if (pad != nullptr) {
    Serial.println("Second controller refused: one is already connected");
    c->disconnect();
    return;
  }
  if (!c->isGamepad()) {
    Serial.printf("Refused '%s': Bluepad32 does not report it as a gamepad\n", c->getModelName().c_str());
    c->disconnect();
    return;
  }
  pad = c;
  everConnected = true;
  BP32.enableNewBluetoothConnections(false);   // no other controller can pair mid-match
  c->playDualRumble(0, 250, 0x80, 0x40);
  Serial.printf("Controller connected: %s\n", c->getModelName().c_str());
}

void onDisconnected(ControllerPtr c) {
  if (c == pad) {
    pad = nullptr;
    padLostAt = millis();
    drive.emergencyStop();
    arm_.hold();
    BP32.enableNewBluetoothConnections(true);
    Serial.println("Controller lost: drive braked, arm holding");
  }
}

// ---------- Status LED ----------
// Blink rate carries the state. everConnected stays false until a controller has been
// seen, so a rover that never connected shows "waiting" rather than "lost".
void updateStatusLed(uint32_t now, bool connected) {
  if (!HAVE_STATUS_LED) return;
  bool on;
  if (connected) on = true;                                  // solid
  else if (everConnected && now - padLostAt < 10000) on = (now / 120) & 1;   // fast: just lost
  else on = (now / 500) & 1;                                 // slow: waiting
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

// ---------- Input shaping ----------
float stick(int32_t raw) {                       // Bluepad32 sticks: -511..512
  float v = constrain(raw / 512.0f, -1.0f, 1.0f);
  if (fabsf(v) < STICK_DEADBAND) return 0.0f;
  float s = (fabsf(v) - STICK_DEADBAND) / (1.0f - STICK_DEADBAND);
  s = (1.0f - EXPO) * s + EXPO * s * s * s;
  return v < 0 ? -s : s;
}

// Left stick -> throttle and turn, through the swap and invert flags in config.h.
// Stick up is a negative axisY, hence the sign on the unswapped path.
void readSticks(float &throttle, float &turn) {
  float ax = stick(pad->axisX()), ay = stick(pad->axisY());
  throttle = STICK_SWAP_XY ? ax : -ay;
  turn     = STICK_SWAP_XY ? ay : ax;
  if (THROTTLE_INVERT) throttle = -throttle;
  if (TURN_INVERT) turn = -turn;
}

// Right stick -> ax (horizontal: elbow, or tip reach) and az (vertical: shoulder, or tip
// height). Has its own swap flag because this controller swaps the left stick's axes but
// not the right one's.
// Expo on top of the drive shaping: the first part of the travel is much finer, which is
// what makes a servo jog feel controllable rather than twitchy.
void readArmSticks(float &ax, float &az, float dt) {
  static float fx = 0.0f, fz = 0.0f;
  float rx = stick(pad->axisRX()), ry = stick(pad->axisRY());
  float cx = ARM_STICK_SWAP_XY ? ry : rx;
  float cz = ARM_STICK_SWAP_XY ? rx : -ry;
  cx = (1.0f - ARM_RAW_EXPO) * cx + ARM_RAW_EXPO * cx * cx * cx;
  cz = (1.0f - ARM_RAW_EXPO) * cz + ARM_RAW_EXPO * cz * cz * cz;
  // One-pole filter on the shaped command. Returning to centre is not filtered, so
  // letting go of the stick stops the joint at once rather than easing out of the move.
  float k = dt / (ARM_STICK_TAU + dt);
  fx = (cx == 0.0f) ? 0.0f : fx + k * (cx - fx);
  fz = (cz == 0.0f) ? 0.0f : fz + k * (cz - fz);
  ax = ARM_ELBOW_INVERT ? -fx : fx;
  az = ARM_SHOULDER_INVERT ? -fz : fz;
}

struct Edge { bool prev = false; bool rose(bool now) { bool r = now && !prev; prev = now; return r; } };
Edge eA, eB, eX, eY, eRB, eLeft, eRight;

void preset(const Pose &p, const char *name) {
  if (!arm_.goTo(p)) {
    Serial.printf("Preset %s not reachable with the current geometry\n", name);
    pad->playDualRumble(0, 80, 0x00, 0xA0);
  }
}

// ---------- Main ----------
void setup() {
  drive.begin();                                 // bridges off before anything else
  if (HAVE_STATUS_LED) { pinMode(PIN_STATUS_LED, OUTPUT); digitalWrite(PIN_STATUS_LED, LOW); }
  Serial.begin(115200);
  if (HAVE_VBAT_SENSE) {
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  }
  if (ARM_RAW_MODE) loadPoses();
  if (ARM_ENABLED) { if (ARM_RAW_MODE) armRaw_.begin(); else arm_.begin(); }
  if (CLAW_ENABLED) claw_.begin();
  if (!ARM_ENABLED)
    Serial.printf("ARM DISABLED (config.h ARM_ENABLED=false): shoulder and elbow stay limp%s\n",
                  CLAW_ENABLED ? ", claw is live on RT/RB" : ", claw disabled too");
  BP32.setup(&onConnected, &onDisconnected);
  BP32.enableVirtualDevice(false);
  if (FORGET_BT_KEYS) {
    BP32.forgetBluetoothKeys();
    Serial.println("Bluetooth keys wiped. Pair the controller now, then set FORGET_BT_KEYS=false and reflash.");
  }
  Serial.println("TakeOFF Rover ready - turn on the Xbox controller (hold pair button the first time)");
}

void loop() {
  BP32.update();

  static uint32_t lastTick = 0, lastBatt = 0, lastLog = 0, lastBuzz = 0;
  uint32_t now = millis();
  if (now - lastBatt >= 100) { lastBatt = now; updateBattery(); }
  updateStatusLed(now, pad != nullptr && pad->isConnected());
  if (now - lastTick < 20) return;               // 50 Hz control loop
  float dt = (now - lastTick) / 1000.0f;
  lastTick = now;
  if (dt > 0.1f) dt = 0.1f;

  // Status for bring-up: 5 Hz with the arm live, 1 Hz without. Drive-only builds still
  // need a sign of life, or an idle rover looks like a dead one.
  if (now - lastLog >= (uint32_t)(ARM_ENABLED ? 200 : 1000)) {
    lastLog = now;
    if (ARM_ENABLED && ARM_RAW_MODE)
      Serial.printf("raw arm | S %u  E %u us%s | claw %u us %c\n",
                    armRaw_.shoulderUs(), armRaw_.elbowUs(),
                    armRaw_.live() ? "" : "  (limp - move the right stick to wake it)",
                    claw_.us(), claw_.gripping() ? 'G' : 'o');
    else if (ARM_ENABLED)
      Serial.printf("tip x %.0f z %.0f mm | t1 %.1f t2 %.1f | us S%u E%u C%u | gap %.1f mm %c\n",
                    arm_.tipX(), arm_.tipZ(), arm_.t1(), arm_.t2(),
                    arm::shoulderUs(arm_.t1()), arm::elbowUs(arm_.t2()), claw_.us(),
                    claw_.gap(), claw_.gripping() ? 'G' : 'o');
    else if (pad && pad->isConnected()) {
      float th, tn;
      readSticks(th, tn);
      if (CLAW_ENABLED)
        Serial.printf("drive+claw | throttle %+.2f turn %+.2f | claw %u us gap %.1f mm %c%s\n",
                      th, tn, claw_.us(), claw_.gap(), claw_.gripping() ? 'G' : 'o',
                      pad->l1() ? " | LB precision" : "");
      else
        Serial.printf("drive-only | controller ok | throttle %+.2f turn %+.2f%s\n",
                      th, tn, pad->l1() ? " | LB precision" : "");
    }
    else
      Serial.println("drive-only | waiting for the controller - turn it on, hold pair the first time");
  }

  if (pad == nullptr || !pad->isConnected()) {   // failsafe
    drive.emergencyStop();
    if (ARM_ENABLED) { if (ARM_RAW_MODE) armRaw_.hold(); else arm_.hold(); }
    if (CLAW_ENABLED) {
      claw_.trim(0.0f, dt);                      // release the trigger: latches the gap it
      claw_.update(dt);                          // holds, and lets the squeeze relief run
    }
    return;
  }

  // Drive
  float throttle, turn;
  readSticks(throttle, turn);
  float scale = pad->l1() ? PRECISION_SCALE : 1.0f;
  if (lowBatt) scale *= 0.5f;
  if (critBatt) drive.emergencyStop();
  else drive.command(throttle, turn, scale, dt);

  // Claw: RB latches open/grip, then RT overrides proportionally while it is held.
  // Runs whether or not the arm does, since it shares none of the arm's geometry.
  if (CLAW_ENABLED) {
    claw_.hold(pad->r1());                       // hold RB open, release to grip
    // Bluepad32 reports the analog triggers 0..1023. constrain() is a macro that would
    // call throttle() up to three times, so clamp with fminf/fmaxf instead.
    claw_.trim(fminf(1.0f, fmaxf(0.0f, pad->throttle() / 1023.0f)), dt);
    uint8_t dp = pad->dpad();
    if (eLeft.rose(dp & DPAD_LEFT))   { claw_.setBlock(claw::Block::LEGO);  pad->playDualRumble(0, 120, 0x60, 0x00); }
    if (eRight.rose(dp & DPAD_RIGHT)) { claw_.setBlock(claw::Block::DUPLO); pad->playDualRumble(0, 400, 0x60, 0x00); }
    claw_.update(dt);
  }

  if (!ARM_ENABLED) return;                      // shoulder and elbow stay out of it

  // Presets. LB held turns a press into "save where the arm is now" instead of a move,
  // so presets are set from the controller rather than by editing pulses and reflashing.
  if (ARM_RAW_MODE) {
    bool pressed[4] = {eA.rose(pad->a()), eB.rose(pad->b()), eX.rose(pad->x()), eY.rose(pad->y())};
    for (int p = 0; p < 4; p++) {
      if (!pressed[p]) continue;
      if (pad->l1()) {
        if (!armRaw_.live()) { Serial.println("Nothing to save: move the right stick to wake the arm first"); continue; }
        savePose(p, armRaw_.shoulderUs(), armRaw_.elbowUs());
        pad->playDualRumble(0, 90, 0x00, 0xB0);
        pad->playDualRumble(140, 90, 0x00, 0xB0);     // two buzzes = saved
      } else {
        armRaw_.goTo(poses[p]);
      }
    }
  } else {
    if (eA.rose(pad->a())) preset(POSE_PICK,  "PICK");
    if (eB.rose(pad->b())) preset(POSE_CARRY, "CARRY");
    if (eY.rose(pad->y())) preset(POSE_DROP,  "DROP");
    if (eX.rose(pad->x())) preset(POSE_STOW,  "STOW");
  }

  // Arm (manual trim) — frozen on critical battery to protect the servo rail
  // LB slows the tip as well as the wheels: lining up on a block wants both.
  float armScale = pad->l1() ? ARM_PRECISION_SCALE : 1.0f;
  float ax, az;
  readArmSticks(ax, az, dt);
  ax *= armScale;
  az *= armScale;
  if (ARM_RAW_MODE) {
    if (critBatt) armRaw_.hold();
    else { armRaw_.jog(az, ax, dt); armRaw_.update(dt); }   // stick up/down = shoulder
  } else {
    if (critBatt) arm_.hold(); else arm_.update(ax, az, dt);
    if (arm_.blocked() && now - lastBuzz > 400) { lastBuzz = now; pad->playDualRumble(0, 60, 0x00, 0x60); }
  }
}
