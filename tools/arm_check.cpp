// arm_check.cpp — PC test for the arm math in TakeOffRover/arm_kinematics.h.
// Build and run:  g++ -std=c++17 -O2 -Wall -I../TakeOffRover arm_check.cpp -o arm_check && ./arm_check
// Exit code 0 = all checks passed.
#include <cstdio>
#include <cmath>
#include "arm_kinematics.h"
#include "config.h"

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

  std::printf(failures ? "\n%d check(s) FAILED\n" : "\nAll checks passed\n", failures);
  return failures ? 1 : 0;
}
