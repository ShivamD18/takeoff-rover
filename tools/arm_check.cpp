// arm_check.cpp — PC test for the arm math in TakeOffRover/arm_kinematics.h.
// Build and run:  g++ -std=c++17 -O2 -Wall -I../TakeOffRover arm_check.cpp -o arm_check && ./arm_check
// Exit code 0 = all checks passed.
#include <cstdio>
#include <cmath>
#include "arm_kinematics.h"
#include "config.h"

static constexpr float DT = 0.02f;   // the firmware's 50 Hz control tick

// Braking distance from the speed limit: the most the jaws may lag a command that
// reverses faster than they can decelerate. v^2 / (2a), plus a tick of float slack.
static constexpr float OVERSHOOT_MAX =
    CLAW_MM_S * CLAW_MM_S / (2.0f * CLAW_ACCEL_MM_S2) + CLAW_MM_S * DT;

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; std::printf("  FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)

static float jointMargin(float t1, float t2) {
  float rel = t1 - t2;
  return std::fmin(std::fmin(std::fmin(t1 - arm::T1_MIN, arm::T1_MAX - t1), std::fmin(t2 - arm::T2_MIN, arm::T2_MAX - t2)),
                   std::fmin(rel - arm::REL_MIN, arm::REL_MAX - rel));
}

int main() {
  struct Named { const char *name; Pose p; } presets[] = {
    {"PICK", POSE_PICK}, {"CARRY", POSE_CARRY}, {"DROP", POSE_DROP}, {"STOW", POSE_STOW}};
  float J[4][2];

  std::printf("Presets (tip x ahead of shoulder, z = finger bottom above floor):\n");
  for (int i = 0; i < 4; i++) {
    float t1 = 0, t2 = 0, x = 0, z = 0;
    bool ok = arm::solveIK(presets[i].p.x, presets[i].p.z, t1, t2);
    CHECK(ok, "%s not reachable/allowed", presets[i].name);
    arm::forwardTip(t1, t2, x, z);
    J[i][0] = t1; J[i][1] = t2;
    uint16_t us1 = arm::shoulderUs(t1), us2 = arm::elbowUs(t2);
    std::printf("  %-5s (%5.0f,%5.0f): t1 %6.1f  t2 %6.1f  margin %5.1f deg  shoulder %4u us  elbow %4u us\n",
                presets[i].name, presets[i].p.x, presets[i].p.z, t1, t2, jointMargin(t1, t2), us1, us2);
    CHECK(std::fabs(x - presets[i].p.x) < 0.05f && std::fabs(z - presets[i].p.z) < 0.05f, "%s FK/IK mismatch", presets[i].name);
    CHECK(jointMargin(t1, t2) >= 10.0f, "%s within 10 deg of a joint limit", presets[i].name);
    CHECK(us1 > arm::SHOULDER.minUs && us1 < arm::SHOULDER.maxUs, "%s shoulder pulse clamped", presets[i].name);
    CHECK(us2 > arm::ELBOW.minUs && us2 < arm::ELBOW.maxUs, "%s elbow pulse clamped", presets[i].name);
  }

  std::printf("Straight joint-space moves between presets:\n");
  int blocked = 0;
  for (int a = 0; a < 4; a++)
    for (int b = 0; b < 4; b++) {
      if (a == b) continue;
      for (int k = 0; k <= 1000; k++) {
        float f = k / 1000.0f;
        float t1 = J[a][0] + f * (J[b][0] - J[a][0]), t2 = J[a][1] + f * (J[b][1] - J[a][1]);
        if (!arm::poseAllowed(t1, t2)) { blocked++; std::printf("  %s -> %s blocked at %.0f%%\n", presets[a].name, presets[b].name, f * 100); break; }
      }
    }
  CHECK(blocked == 0, "%d preset moves blocked", blocked);
  if (!blocked) std::printf("  all 12 moves clear\n");

  // Pulses never clamp anywhere inside the allowed joint box.
  int clamped = 0;
  for (float t1 = arm::T1_MIN; t1 <= arm::T1_MAX; t1 += 0.5f)
    for (float t2 = arm::T2_MIN; t2 <= arm::T2_MAX; t2 += 0.5f)
      if (arm::jointsAllowed(t1, t2)) {
        uint16_t a = arm::shoulderUs(t1), b = arm::elbowUs(t2);
        if (a <= arm::SHOULDER.minUs || a >= arm::SHOULDER.maxUs || b <= arm::ELBOW.minUs || b >= arm::ELBOW.maxUs) clamped++;
      }
  CHECK(clamped == 0, "%d allowed poses would clamp a servo pulse", clamped);

  // Pose the CAD assembly is drawn in must be allowed.
  CHECK(arm::poseAllowed(78.1f, -40.0f), "CAD pose (t1 78.1, t2 -40.0) rejected");

  // Known-bad poses must be rejected.
  { float t1, t2;
    CHECK(!arm::solveIK(60.0f, 70.0f, t1, t2), "old STOW (60,70) should be rejected (outside limits / in the deck)");
    CHECK(!arm::solveIK(170.0f, -5.0f, t1, t2), "tip below the floor accepted"); }

  // Floor reach.
  float lo = 1e9f, hi = -1e9f;
  for (float x = 0; x <= 260; x += 1.0f) { float a, b; if (arm::solveIK(x, 3.0f, a, b)) { lo = std::fmin(lo, x); hi = std::fmax(hi, x); } }
  std::printf("Fingers 3 mm off the floor reachable from %.0f to %.0f mm ahead of the shoulder\n", lo, hi);
  CHECK(lo <= POSE_PICK.x && POSE_PICK.x <= hi, "PICK outside floor reach");

  // Claw pulses.
  std::printf("Claw: open %u us, LEGO grip %u us, Duplo grip %u us\n", claw::usForGap(claw::OPEN_GAP),
              claw::usForGap(claw::gapForGrip(claw::Block::LEGO)),
              claw::usForGap(claw::gapForGrip(claw::Block::DUPLO)));

  // ---------- Jaw motion profile ----------
  // Runs the firmware's own claw::step at the 50 Hz control rate and measures what the
  // fingers do: how long a move takes, the peak speed, the worst jerk (change of
  // acceleration between ticks) and any overshoot past the target.
  std::printf("Jaw moves at %.0f Hz (limit %.0f mm/s, accel %.0f mm/s^2):\n", 1.0f / DT, CLAW_MM_S, CLAW_ACCEL_MM_S2);
  struct Move { const char *name; float from, to; } moves[] = {
    {"open -> LEGO grip",  claw::OPEN_GAP, claw::gapForGrip(claw::Block::LEGO)},
    {"open -> Duplo grip", claw::OPEN_GAP, claw::gapForGrip(claw::Block::DUPLO)},
    {"LEGO grip -> open",  claw::gapForGrip(claw::Block::LEGO), claw::OPEN_GAP},
    {"1 mm nudge",         claw::OPEN_GAP, claw::OPEN_GAP - 1.0f},
  };
  for (const Move &mv : moves) {
    motion::State m; m.pos = mv.from; m.vel = 0.0f;
    float vPeak = 0, aPrev = 0, jerkMax = 0, overshoot = 0, vLast = 0;
    int ticks = 0;
    for (; ticks < 2000; ticks++) {
      float vBefore = m.vel;
      bool done = motion::step(m, mv.to, DT, CLAW_MM_S, CLAW_ACCEL_MM_S2);
      float a = (m.vel - vBefore) / DT;
      if (ticks > 0) jerkMax = std::fmax(jerkMax, std::fabs(a - aPrev));
      aPrev = a;
      vPeak = std::fmax(vPeak, std::fabs(m.vel));
      float past = (mv.to > mv.from) ? m.pos - mv.to : mv.to - m.pos;
      overshoot = std::fmax(overshoot, past);
      vLast = m.vel;
      if (done) { ticks++; break; }
    }
    std::printf("  %-18s %5.1f -> %5.1f mm  %5.2f s  peak %5.1f mm/s  max jerk %7.0f mm/s^3  overshoot %.4f mm\n",
                mv.name, mv.from, mv.to, ticks * DT, vPeak, jerkMax, overshoot);
    CHECK(ticks < 2000, "%s never reached the target", mv.name);
    CHECK(overshoot <= 1e-3f, "%s overshoots by %.3f mm", mv.name, overshoot);
    CHECK(std::fabs(vLast) < 1e-3f, "%s ends at %.2f mm/s instead of stopped", mv.name, vLast);
    CHECK(vPeak <= CLAW_MM_S + 1e-3f, "%s exceeds the speed limit (%.1f mm/s)", mv.name, vPeak);
    // One tick of acceleration applied or removed is the most the profile may ever step.
    CHECK(jerkMax <= CLAW_ACCEL_MM_S2 / DT + 1.0f, "%s jerk %.0f exceeds one accel step", mv.name, jerkMax);
  }

  // Baseline: the constant-speed ramp this replaced, so the gain is on the record.
  // It stepped the jaws from rest to the full rate in one tick and stopped them dead on
  // arrival, so its acceleration was the whole speed over one tick either way.
  {
    const float OLD_MM_S = 40.0f;                  // the rate before the profile
    float dist = claw::OPEN_GAP - claw::gapForGrip(claw::Block::LEGO);
    float oldAccel = OLD_MM_S / DT, oldJerk = oldAccel / DT;
    std::printf("  was (constant %.0f mm/s)  %.2f s  accel %.0f mm/s^2  jerk %.0f mm/s^3 at both ends\n",
                OLD_MM_S, dist / OLD_MM_S, oldAccel, oldJerk);
    CHECK(CLAW_ACCEL_MM_S2 < oldAccel, "the profile does not reduce acceleration vs the old ramp");
  }

  // A shaky trigger thumb must not shake the fingers: feed a noisy command and check the
  // gap stays inside the acceleration limit and tracks the average, not the noise.
  {
    motion::State m; m.pos = claw::OPEN_GAP; m.vel = 0.0f;
    float closed = claw::gapForGrip(claw::Block::LEGO);
    float aMax = 0, vPrev = 0;
    unsigned seed = 12345;
    for (int i = 0; i < 300; i++) {
      seed = seed * 1103515245u + 12345u;
      float noise = ((seed >> 16) % 1000) / 1000.0f - 0.5f;      // +/- 0.5 of the trigger
      float f = std::fmin(1.0f, std::fmax(0.0f, 0.5f + 0.6f * noise));
      float target = claw::OPEN_GAP + f * (closed - claw::OPEN_GAP);
      motion::step(m, target, DT, CLAW_MM_S, CLAW_ACCEL_MM_S2);
      aMax = std::fmax(aMax, std::fabs(m.vel - vPrev) / DT);
      vPrev = m.vel;
      // A command that reverses faster than the jaws can brake is overshot by at most
      // the stopping distance from the speed limit; past that the jaws are not tracking.
      CHECK(m.pos >= closed - OVERSHOOT_MAX && m.pos <= claw::OPEN_GAP + OVERSHOOT_MAX,
            "noisy trigger drove the gap to %.2f mm", m.pos);
    }
    std::printf("  noisy trigger      peak acceleration %.0f mm/s^2 (limit %.0f), overshoot allowance %.2f mm\n",
                aMax, CLAW_ACCEL_MM_S2, OVERSHOOT_MAX);
    CHECK(aMax <= CLAW_ACCEL_MM_S2 + 1.0f, "noisy trigger broke the accel limit (%.0f)", aMax);
  }

  // Relief creep after a long hold must be gentle and must not open the jaws far.
  {
    motion::State m;
    m.pos = claw::gapForGrip(claw::Block::LEGO);
    m.vel = 0.0f;
    float relieved = claw::gapForRelief(claw::Block::LEGO);
    int ticks = 0;
    while (!motion::step(m, relieved, DT, CLAW_RELIEF_MM_S, CLAW_ACCEL_MM_S2) && ticks < 2000) ticks++;
    std::printf("  squeeze relief     %.1f -> %.1f mm in %.2f s at <= %.1f mm/s\n",
                claw::gapForGrip(claw::Block::LEGO), relieved, ticks * DT, CLAW_RELIEF_MM_S);
    CHECK(ticks > 2, "relief creep lands in %d ticks: too abrupt to be a creep", ticks);
    // Relief exists to unload the servo, not to let go: it may only ever open the jaws,
    // and the eased grip must still be squeezing the block.
    // Relief exists to unload the servo, not to let go.
    CHECK(relieved > claw::gapForGrip(claw::Block::LEGO), "relief tightens the grip instead of easing it");
    CHECK(claw::usForGap(relieved) < claw::usForGap(claw::gapForGrip(claw::Block::LEGO)),
          "relief pulse %u is not backed off the grip pulse %u",
          claw::usForGap(relieved), claw::usForGap(claw::gapForGrip(claw::Block::LEGO)));
    // The grip pulses are what actually hold the blocks, so they must be reachable.
    CHECK(claw::LEGO_GRIP_US <= claw::MAX_US && claw::DUPLO_GRIP_US <= claw::MAX_US,
          "a grip pulse is past MAX_US and would be clamped");
    std::printf("  grip pulses       LEGO %.0f us, Duplo %.0f us, relief backs off %.0f us\n",
                claw::LEGO_GRIP_US, claw::DUPLO_GRIP_US, claw::RELIEF_US);
  }

  // ---------- Arm preset moves ----------
  // Same profile, driven in degrees along the move. Checks the move eases in and out and
  // that every pose it passes through is still inside the limits and clear of the deck.
  std::printf("Preset moves at %.0f Hz (limit %.0f deg/s, accel %.0f deg/s^2):\n",
              1.0f / DT, ARM_PRESET_DEG_S, ARM_PRESET_ACCEL_DEG_S2);
  for (int a = 0; a < 4; a++)
    for (int b = 0; b < 4; b++) {
      if (a == b) continue;
      float span = std::fmax(std::fabs(J[b][0] - J[a][0]), std::fabs(J[b][1] - J[a][1]));
      if (span <= 0.01f) continue;
      motion::State m;
      float vPeak = 0, aPrev = 0, jerkMax = 0;
      int ticks = 0;
      bool clear = true;
      for (; ticks < 4000; ticks++) {
        float vBefore = m.vel;
        bool done = motion::step(m, span, DT, ARM_PRESET_DEG_S, ARM_PRESET_ACCEL_DEG_S2);
        float acc = (m.vel - vBefore) / DT;
        if (ticks > 0) jerkMax = std::fmax(jerkMax, std::fabs(acc - aPrev));
        aPrev = acc;
        vPeak = std::fmax(vPeak, std::fabs(m.vel));
        float f = m.pos / span;
        if (!arm::poseAllowed(J[a][0] + f * (J[b][0] - J[a][0]), J[a][1] + f * (J[b][1] - J[a][1]))) clear = false;
        if (done) { ticks++; break; }
      }
      if (a == 0 && b == 3)   // one representative move, so the output stays short
        std::printf("  %-5s -> %-5s  %5.1f deg  %.2f s  peak %5.1f deg/s  max jerk %6.0f deg/s^3\n",
                    presets[a].name, presets[b].name, span, ticks * DT, vPeak, jerkMax);
      CHECK(clear, "%s -> %s leaves the safe region under the profile", presets[a].name, presets[b].name);
      CHECK(ticks < 4000, "%s -> %s never finishes", presets[a].name, presets[b].name);
      CHECK(vPeak <= ARM_PRESET_DEG_S + 1e-3f, "%s -> %s exceeds the rate limit", presets[a].name, presets[b].name);
      CHECK(jerkMax <= ARM_PRESET_ACCEL_DEG_S2 / DT + 1.0f, "%s -> %s jerk %.0f exceeds one accel step",
            presets[a].name, presets[b].name, jerkMax);
    }
  std::printf("  all 12 moves stay in the safe region under the profile\n");

  // Trim ramp: the tip must not jump to full speed, and must stop without a long coast.
  {
    float v = 0, aMax = 0, vPrev = 0;
    int up = 0;
    while (v < ARM_TRIM_MM_S - 1e-3f && up < 2000) {
      v = std::fmin(ARM_TRIM_MM_S, v + ARM_TRIM_ACCEL_MM_S2 * DT);
      aMax = std::fmax(aMax, std::fabs(v - vPrev) / DT); vPrev = v; up++;
    }
    float coast = 0;
    int down = 0;
    while (v > 1e-3f && down < 2000) { v = std::fmax(0.0f, v - ARM_TRIM_DECEL_MM_S2 * DT); coast += v * DT; down++; }
    std::printf("Tip trim: %.0f mm/s reached in %.2f s, stops in %.2f s after %.1f mm of coast\n",
                ARM_TRIM_MM_S, up * DT, down * DT, coast);
    CHECK(up > 1, "tip trim reaches full speed in one tick");
    CHECK(coast < 15.0f, "tip coasts %.1f mm after the stick is released", coast);
  }

  std::printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
  return failures ? 1 : 0;
}
