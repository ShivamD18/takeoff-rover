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
//   Right stick ....... move the claw tip: left/right = reach in/out, up/down = height
//   A / B / Y / X ..... presets: PICK / CARRY / DROP-over-wall / STOW
//   RT (analog) ....... squeeze the claw proportionally; let go and it holds the gap
//   RB ................ claw grip <-> open (latch, for carrying hands-free)
//   D-pad left/right .. block size LEGO / Duplo (1 short or 1 long rumble)
//   Short buzz ........ the arm refused a move (limit, deck or bracket in the way)
//
// Safety: drive brakes and the arm/claw hold whenever the controller is not connected.
// The physical kill switch is still the primary emergency stop.
#include <Bluepad32.h>
#include "config.h"
#include "drive.h"
#include "arm_ctrl.h"

Drive drive;
Arm arm_;
Claw claw_;
ControllerPtr pad = nullptr;

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
  BP32.enableNewBluetoothConnections(false);   // no other controller can pair mid-match
  c->playDualRumble(0, 250, 0x80, 0x40);
  Serial.printf("Controller connected: %s\n", c->getModelName().c_str());
}

void onDisconnected(ControllerPtr c) {
  if (c == pad) {
    pad = nullptr;
    drive.emergencyStop();
    arm_.hold();
    BP32.enableNewBluetoothConnections(true);
    Serial.println("Controller lost: drive braked, arm holding");
  }
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

// Right stick -> claw tip reach and height, through the same swap: horizontal stick
// moves the tip in and out, vertical moves it up and down.
// UNTESTED. ARM_ENABLED is false until the arm is calibrated, so this mirrors the left
// stick's mapping and wants checking the first time the arm runs.
void readArmSticks(float &ax, float &az) {
  float rx = stick(pad->axisRX()), ry = stick(pad->axisRY());
  ax = STICK_SWAP_XY ? ry : rx;        // horizontal stick = reach
  az = STICK_SWAP_XY ? rx : -ry;       // vertical stick = height
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
  Serial.begin(115200);
  if (HAVE_VBAT_SENSE) {
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  }
  if (ARM_ENABLED) { arm_.begin(); claw_.begin(); }
  else Serial.println("ARM DISABLED (config.h ARM_ENABLED=false): drive only, servos stay limp");
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
  if (now - lastTick < 20) return;               // 50 Hz control loop
  float dt = (now - lastTick) / 1000.0f;
  lastTick = now;
  if (dt > 0.1f) dt = 0.1f;

  // Status for bring-up: 5 Hz with the arm live, 1 Hz without. Drive-only builds still
  // need a sign of life, or an idle rover looks like a dead one.
  if (now - lastLog >= (uint32_t)(ARM_ENABLED ? 200 : 1000)) {
    lastLog = now;
    if (ARM_ENABLED)
      Serial.printf("tip x %.0f z %.0f mm | t1 %.1f t2 %.1f | us S%u E%u C%u | gap %.1f mm %c\n",
                    arm_.tipX(), arm_.tipZ(), arm_.t1(), arm_.t2(),
                    arm::shoulderUs(arm_.t1()), arm::elbowUs(arm_.t2()), claw_.us(),
                    claw_.gap(), claw_.gripping() ? 'G' : 'o');
    else if (pad && pad->isConnected()) {
      float th, tn;
      readSticks(th, tn);
      Serial.printf("drive-only | controller ok | throttle %+.2f turn %+.2f%s\n",
                    th, tn, pad->l1() ? " | LB precision" : "");
    }
    else
      Serial.println("drive-only | waiting for the controller - turn it on, hold pair the first time");
  }

  if (pad == nullptr || !pad->isConnected()) {   // failsafe
    drive.emergencyStop();
    if (ARM_ENABLED) {
      arm_.hold();
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

  if (!ARM_ENABLED) return;                      // drive-only build: nothing below runs

  // Presets
  if (eA.rose(pad->a())) preset(POSE_PICK,  "PICK");
  if (eB.rose(pad->b())) preset(POSE_CARRY, "CARRY");
  if (eY.rose(pad->y())) preset(POSE_DROP,  "DROP");
  if (eX.rose(pad->x())) preset(POSE_STOW,  "STOW");

  // Claw: RB latches open/grip, then RT overrides proportionally while it is held.
  if (eRB.rose(pad->r1())) claw_.toggle();
  // Bluepad32 reports the analog triggers 0..1023. constrain() is a macro that would
  // call throttle() up to three times, so clamp with fminf/fmaxf instead.
  claw_.trim(fminf(1.0f, fmaxf(0.0f, pad->throttle() / 1023.0f)), dt);
  uint8_t dp = pad->dpad();
  if (eLeft.rose(dp & DPAD_LEFT))   { claw_.setBlock(claw::Block::LEGO);  pad->playDualRumble(0, 120, 0x60, 0x00); }
  if (eRight.rose(dp & DPAD_RIGHT)) { claw_.setBlock(claw::Block::DUPLO); pad->playDualRumble(0, 400, 0x60, 0x00); }

  // Arm (manual trim) — frozen on critical battery to protect the servo rail
  // LB slows the tip as well as the wheels: lining up on a block wants both.
  float armScale = pad->l1() ? ARM_PRECISION_SCALE : 1.0f;
  float ax, az;
  readArmSticks(ax, az);
  ax *= armScale;
  az *= armScale;
  if (critBatt) arm_.hold(); else arm_.update(ax, az, dt);
  if (arm_.blocked() && now - lastBuzz > 400) { lastBuzz = now; pad->playDualRumble(0, 60, 0x00, 0x60); }
  claw_.update(dt);
}
