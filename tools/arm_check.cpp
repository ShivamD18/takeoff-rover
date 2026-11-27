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
              claw::usForGap(claw::gapForGrip(claw::Block::LEGO, claw::SQUEEZE)),
              claw::usForGap(claw::gapForGrip(claw::Block::DUPLO, claw::SQUEEZE)));

  // ---------- Jaw motion profile ----------
  // Runs the firmware's own claw::step at the 50 Hz control rate and measures what the
  // fingers do: how long a move takes, the peak speed, the worst jerk (change of
  // acceleration between ticks) and any overshoot past the target.
  std::printf("Jaw moves at %.0f Hz (limit %.0f mm/s, accel %.0f mm/s^2):\n", 1.0f / DT, CLAW_MM_S, CLAW_ACCEL_MM_S2);
  struct Move { const char *name; float from, to; } moves[] = {
    {"open -> LEGO grip",  claw::OPEN_GAP, claw::gapForGrip(claw::Block::LEGO,  claw::SQUEEZE)},
    {"open -> Duplo grip", claw::OPEN_GAP, claw::gapForGrip(claw::Block::DUPLO, claw::SQUEEZE)},
    {"LEGO grip -> open",  claw::gapForGrip(claw::Block::LEGO, claw::SQUEEZE), claw::OPEN_GAP},
    {"1 mm nudge",         claw::OPEN_GAP, claw::OPEN_GAP - 1.0f},
  };
  for (const Move &mv : moves) {
    claw::Motion m; m.gap = mv.from; m.vel = 0.0f;
    float vPeak = 0, aPrev = 0, jerkMax = 0, overshoot = 0, vLast = 0;
    int ticks = 0;
    for (; ticks < 2000; ticks++) {
      float vBefore = m.vel;
      bool done = claw::step(m, mv.to, DT, CLAW_MM_S, CLAW_ACCEL_MM_S2);
      float a = (m.vel - vBefore) / DT;
      if (ticks > 0) jerkMax = std::fmax(jerkMax, std::fabs(a - aPrev));
      aPrev = a;
      vPeak = std::fmax(vPeak, std::fabs(m.vel));
      float past = (mv.to > mv.from) ? m.gap - mv.to : mv.to - m.gap;
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

  // A shaky trigger thumb must not shake the fingers: feed a noisy command and check the
  // gap stays inside the acceleration limit and tracks the average, not the noise.
  {
    claw::Motion m; m.gap = claw::OPEN_GAP; m.vel = 0.0f;
    float closed = claw::gapForGrip(claw::Block::LEGO, claw::SQUEEZE);
    float aMax = 0, vPrev = 0;
    unsigned seed = 12345;
    for (int i = 0; i < 300; i++) {
      seed = seed * 1103515245u + 12345u;
      float noise = ((seed >> 16) % 1000) / 1000.0f - 0.5f;      // +/- 0.5 of the trigger
      float f = std::fmin(1.0f, std::fmax(0.0f, 0.5f + 0.6f * noise));
      float target = claw::OPEN_GAP + f * (closed - claw::OPEN_GAP);
      claw::step(m, target, DT, CLAW_MM_S, CLAW_ACCEL_MM_S2);
      aMax = std::fmax(aMax, std::fabs(m.vel - vPrev) / DT);
      vPrev = m.vel;
      // A command that reverses faster than the jaws can brake is overshot by at most
      // the stopping distance from the speed limit; past that the jaws are not tracking.
      CHECK(m.gap >= closed - OVERSHOOT_MAX && m.gap <= claw::OPEN_GAP + OVERSHOOT_MAX,
            "noisy trigger drove the gap to %.2f mm", m.gap);
    }
    std::printf("  noisy trigger      peak acceleration %.0f mm/s^2 (limit %.0f), overshoot allowance %.2f mm\n",
                aMax, CLAW_ACCEL_MM_S2, OVERSHOOT_MAX);
    CHECK(aMax <= CLAW_ACCEL_MM_S2 + 1.0f, "noisy trigger broke the accel limit (%.0f)", aMax);
  }

  // Relief creep after a long hold must be gentle and must not open the jaws far.
  {
    claw::Motion m;
    m.gap = claw::gapForGrip(claw::Block::LEGO, claw::SQUEEZE);
    m.vel = 0.0f;
    float relieved = claw::gapForGrip(claw::Block::LEGO, claw::SQUEEZE_RELIEVED);
    int ticks = 0;
    while (!claw::step(m, relieved, DT, CLAW_RELIEF_MM_S, CLAW_ACCEL_MM_S2) && ticks < 2000) ticks++;
    std::printf("  squeeze relief     %.1f -> %.1f mm in %.2f s at <= %.1f mm/s\n",
                claw::gapForGrip(claw::Block::LEGO, claw::SQUEEZE), relieved, ticks * DT, CLAW_RELIEF_MM_S);
    CHECK(ticks > 2, "relief creep lands in %d ticks: too abrupt to be a creep", ticks);
    CHECK(relieved - claw::gapForGrip(claw::Block::LEGO, claw::SQUEEZE) < 1.0f, "relief opens the jaws more than 1 mm");
  }

  std::printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
  return failures ? 1 : 0;
}
