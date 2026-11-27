// arm_kinematics.h — TakeOFF Rover arm geometry, limits, collision keep-out and IK.
// Pure math (no Arduino calls), so tools/arm_check.cpp can test it on a PC.
//
// Every number below comes from the Fusion assembly exported as TakeOFF_Rover.stp
// (2026-11-26), except SHOULDER_H, which was measured on the built rover.
// Values marked CAL still need checking on the real arm.
//
// Coordinates: side view of the arm, origin on the shoulder servo axis,
//   x = forward (the way the claw reaches), z = up. Angles are absolute,
//   measured from forward-horizontal, counter-clockwise positive, in degrees.
// Claw tip = centre of the claw (spur-gear axis) at the BOTTOM of the fingers,
// so a tip height of 0 means the fingers touch the floor.
#pragma once
#include <math.h>
#include <stdint.h>

namespace arm {

constexpr float DEG = 57.2957795f;

// ---------- Links (STEP: pin-to-pin distances) ----------
constexpr float A2    = 120.0f;  // shoulder S -> elbow E      (upper arm, on the GPIO19 servo horn)
constexpr float A3    = 100.0f;  // elbow E -> wrist W         (forearm)
constexpr float CRANK = 30.0f;   // shoulder S -> crank pin C  (on the GPIO18 servo horn)
// Push rod C -> forearm tail T is 120 mm and the tail is 30 mm behind E, so S-C-T-E is a
// parallelogram: the crank always points along the forearm + 180 deg, i.e. the GPIO18
// servo sets the forearm's absolute angle directly.
// Level rods (25 mm arms at 135 deg on the bracket/elbow plate and 60 deg on the elbow/
// wrist plates) keep the claw vertical in every pose.

// ---------- Mounting ----------
constexpr float SHOULDER_H = 152.4f;  // CAL: shoulder axis above the floor. Measured ~6 in (CAD says 160.2)
constexpr float TIP_DOWN   = 84.3f;   // wrist pin W down to the bottom of the fingers
constexpr float TIP_FWD    = 15.7f;   // claw centre ahead of the wrist pin
constexpr float CLAW_BACK  = 6.0f;    // claw body extends this far behind W ...
constexpr float CLAW_FRONT = 35.7f;   // ... and this far ahead of W

// ---------- Obstacles around the arm (relative to the shoulder axis) ----------
constexpr float BRACKET_X   = 28.6f;  // servo bracket reaches this far forward ...
constexpr float BRACKET_TOP = 25.8f;  // ... and this high above the shoulder axis
constexpr float DECK_X      = 75.0f;  // front edge of the deck plate
constexpr float DECK_TOP    = -39.5f; // deck top surface (below the shoulder axis)
constexpr float CLEARANCE   = 8.0f;   // keep the claw this far from the deck/bracket
constexpr float FOREARM_R   = 6.0f;   // forearm half-thickness at the pins
constexpr float FLOOR_MIN   = 0.0f;   // lowest allowed tip height above the floor

// ---------- Joint limits ----------
// Level-keeping parallelograms go flat (jam) when the upper arm lines up with 135/-45 deg
// or the forearm with 60/-120 deg; the CAD notes keep 30 deg away from those. The crank
// parallelogram goes flat when the forearm folds onto the upper arm (t1 - t2 = 0 or 180).
// A further 5 deg margin is kept on every limit.
constexpr float T1_MIN = -10.0f, T1_MAX = 100.0f;
constexpr float T2_MIN = -85.0f, T2_MAX = 25.0f;
constexpr float REL_MIN = 20.0f, REL_MAX = 160.0f;   // t1 - t2

inline bool jointsAllowed(float t1, float t2) {
  float rel = t1 - t2;
  return t1 >= T1_MIN && t1 <= T1_MAX && t2 >= T2_MIN && t2 <= T2_MAX &&
         rel >= REL_MIN && rel <= REL_MAX;
}

// Highest solid surface under forward position x (relative to the shoulder axis).
inline float obstacleTop(float x) {
  if (x < BRACKET_X) return BRACKET_TOP;
  if (x < DECK_X)    return DECK_TOP;
  return -SHOULDER_H;                    // floor
}

struct Pins { float ex, ez, wx, wz; };
inline Pins forwardPins(float t1, float t2) {
  float r1 = t1 / DEG, r2 = t2 / DEG;
  Pins p;
  p.ex = A2 * cosf(r1);        p.ez = A2 * sinf(r1);
  p.wx = p.ex + A3 * cosf(r2); p.wz = p.ez + A3 * sinf(r2);
  return p;
}

// Claw and forearm must stay clear of the bracket, the deck and the floor.
inline bool collisionFree(float t1, float t2) {
  Pins p = forwardPins(t1, t2);
  float bottom = p.wz - TIP_DOWN;                // finger bottoms
  if (bottom < -SHOULDER_H + FLOOR_MIN) return false;
  // The profile only steps down with x, so the highest surface within CLEARANCE of the
  // claw's rear edge is the one to clear (the floor itself is handled above).
  float top = obstacleTop(p.wx - CLAW_BACK - CLEARANCE);
  if (top > -SHOULDER_H && bottom < top + CLEARANCE) return false;
  for (int i = 0; i <= 10; i++) {                // forearm, sampled every 10 mm
    float f = i / 10.0f;
    float x = p.ex + f * (p.wx - p.ex), z = p.ez + f * (p.wz - p.ez);
    float t = obstacleTop(x - FOREARM_R);
    if (t > -SHOULDER_H && z < t + FOREARM_R) return false;
  }
  return true;
}

inline bool poseAllowed(float t1, float t2) { return jointsAllowed(t1, t2) && collisionFree(t1, t2); }

// Claw tip (x ahead of the shoulder axis, z above the floor) -> joint angles.
// Elbow-up solution. Returns false if out of reach, outside the limits, or colliding.
inline bool solveIK(float x, float z, float &t1, float &t2) {
  float wx = x - TIP_FWD;
  float wz = z + TIP_DOWN - SHOULDER_H;          // wrist pin relative to the shoulder axis
  float d2 = wx * wx + wz * wz, d = sqrtf(d2);
  if (d > A2 + A3 - 2.0f || d < fabsf(A2 - A3) + 2.0f) return false;
  float c = (A2 * A2 + d2 - A3 * A3) / (2.0f * A2 * d);
  c = fmaxf(-1.0f, fminf(1.0f, c));
  float r1 = atan2f(wz, wx) + acosf(c);
  float r2 = atan2f(wz - A2 * sinf(r1), wx - A2 * cosf(r1));
  t1 = r1 * DEG;
  t2 = r2 * DEG;
  return poseAllowed(t1, t2);
}

inline void forwardTip(float t1, float t2, float &x, float &z) {
  Pins p = forwardPins(t1, t2);
  x = p.wx + TIP_FWD;
  z = SHOULDER_H + p.wz - TIP_DOWN;
}

// ---------- Servo calibration ----------
// The two MG996Rs sit on the same shoulder axis facing each other, so the same pulse
// change turns them opposite ways in this side view: the signs below must differ.
// Defaults assume "longer pulse = counter-clockwise seen from the horn" (TowerPro
// convention). CAL: check on the bench with ServoBenchTest and flip both if needed.
struct ServoCal { float centreDeg; float us0; float usPerDeg; float minUs; float maxUs; };
// Horns fitted so 1500 us puts each joint in the middle of its range.
constexpr ServoCal SHOULDER = {45.0f,  1500.0f, +10.0f, 850.0f, 2150.0f};  // GPIO19, upper arm
constexpr ServoCal ELBOW    = {-30.0f, 1500.0f, -10.0f, 850.0f, 2150.0f};  // GPIO18, crank/forearm

inline uint16_t toUs(const ServoCal &c, float angleDeg) {
  float us = c.us0 + (angleDeg - c.centreDeg) * c.usPerDeg;
  return (uint16_t)fmaxf(c.minUs, fminf(c.maxUs, us));
}
inline uint16_t shoulderUs(float t1) { return toUs(SHOULDER, t1); }
inline uint16_t elbowUs(float t2)    { return toUs(ELBOW, t2); }   // crank angle = t2 + 180

} // namespace arm

namespace claw {
// Rack-and-pinion gripper (STEP): 24-tooth module-0.7 spur gear (tip dia 18.2 mm) ->
// pitch radius 8.4 mm. Both fingers ride on the same pinion, so the gap changes by
// 2 x 8.4 mm per radian. Fingers are ~38 mm tall; fully open gap in the CAD is 45.8 mm.
constexpr float PITCH_R     = 8.4f;
constexpr float GAP_PER_DEG = 2.0f * PITCH_R / 57.2957795f;  // 0.293 mm per servo degree
constexpr float US_PER_DEG  = 10.0f;    // CAL: SG90 pulse per degree
constexpr float REF_US      = 1500.0f;  // CAL: pulse at which REF_GAP was measured
constexpr float REF_GAP     = 30.0f;    // CAL: measured gap at REF_US (mm)
constexpr float OPEN_SIGN   = -1.0f;    // a SHORTER pulse opens the claw (checked on the rover 2026-11-26)
constexpr float MIN_US = 700.0f, MAX_US = 2300.0f;  // CAL: just inside the finger stops

enum class Block : uint8_t { LEGO, DUPLO };
constexpr float LEGO_W   = 15.8f;   // 2xN brick, gripped across the narrow side
constexpr float DUPLO_W  = 31.7f;
constexpr float OPEN_GAP = 45.0f;   // just inside the 45.8 mm CAD maximum
constexpr float SQUEEZE  = 1.5f;    // mm past contact, taken up by the rubber pads
constexpr float SQUEEZE_RELIEVED = 0.8f;

inline float gapForGrip(Block b, float squeeze) { return (b == Block::LEGO ? LEGO_W : DUPLO_W) - squeeze; }
inline uint16_t usForGap(float gapMm) {
  float us = REF_US + OPEN_SIGN * (gapMm - REF_GAP) / GAP_PER_DEG * US_PER_DEG;
  return (uint16_t)fmaxf(MIN_US, fminf(MAX_US, us));
}
} // namespace claw
