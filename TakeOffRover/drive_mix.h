// drive_mix.h — skid-steer mixing and the command-to-duty map.
// Pure math (no Arduino calls), so tools/arm_check.cpp tests the same code the rover runs.
//
// The problem this solves: MIN_EFFECTIVE is the smallest duty that turns a wheel, and
// applying it to each side on its own floors the INNER wheel of a turn up to that duty.
// It then turns when it should barely move and fights the turn. A commanded 2:1 came out
// 1.60:1 at full stick and 1.13:1 in precision mode -- worst exactly where lining up on a
// block needs it most.
//
// Instead the floor is applied to the PAIR: the larger side gets the duty it always got,
// and the smaller keeps its commanded share of it. A side whose share lands below
// MIN_EFFECTIVE is coasted rather than driven, because that duty cannot turn a wheel --
// energising it would only hold the motor stalled, drawing current and making heat.
#pragma once
#include <math.h>
#include <stdint.h>

namespace drivemix {

enum class Mode : uint8_t { Brake, Coast, Drive };
struct Out {
  Mode mode = Mode::Brake;
  bool forward = true;
  float duty = 0.0f;      // only meaningful when mode == Drive
};

// Throttle and turn in [-1, 1] -> per-side commands in [-1, 1]. Dividing by the larger
// magnitude only when it exceeds 1 keeps the turn ratio intact when the mix saturates.
inline void mix(float throttle, float turn, float turnGain, float scale, float &l, float &r) {
  float a = throttle + turn * turnGain;
  float b = throttle - turn * turnGain;
  float m = fmaxf(1.0f, fmaxf(fabsf(a), fabsf(b)));
  l = a / m * scale;
  r = b / m * scale;
}

// Per-side commands -> what each bridge should do.
// eps is the magnitude below which a side counts as idle and is braked, so a stopped side
// anchors a pivot turn instead of rolling.
inline void duties(float l, float r, float minEff, bool preserveRatio, float eps, Out &ol, Out &orr) {
  float al = fabsf(l), ar = fabsf(r);
  float hi = fmaxf(al, ar);
  ol = Out{}; orr = Out{};
  if (hi < eps) return;                                  // both idle: brake both

  // Duty the larger side gets. Unchanged from the per-side map, so straight-line feel and
  // top speed are exactly what they were.
  float dutyHi = minEff + (1.0f - minEff) * fminf(hi, 1.0f);
  float k = dutyHi / hi;                                 // common scale, preserves the ratio

  const float v[2] = {l, r};
  Out *out[2] = {&ol, &orr};
  for (int i = 0; i < 2; i++) {
    float a = fabsf(v[i]);
    if (a < eps) continue;                               // idle side: brake (already set)
    float duty = preserveRatio ? a * k : minEff + (1.0f - minEff) * fminf(a, 1.0f);
    if (duty < minEff) { out[i]->mode = Mode::Coast; continue; }   // too low to turn: freewheel
    out[i]->mode = Mode::Drive;
    out[i]->forward = v[i] > 0.0f;
    out[i]->duty = fminf(duty, 1.0f);
  }
}

} // namespace drivemix
