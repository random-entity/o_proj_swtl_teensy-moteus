#pragma once

#include "meta.h"

namespace orbit {
struct {
  double i = 0.0;
  double ilim = 1.0;
  double kp = 1.0, ki = 0.1;

  void update(const double& tgt, const double& cur) {
    auto err = tgt - cur;
    i += err;
    // i = clamp();
  }
} pid;
}  // namespace orbit

void ModeRunners::Orbit(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.orbit;
  auto& w = b->cmd_.walk;

  switch (m) {
    case M::Orbit: {
      m = M::Walk;
      w.init_didimbal = BOOL_L;
      for (uint8_t f : IDX_LR) {
        w.tgt_yaw[f] = [](Basilisk* b) { return NaN; };
        w.stride[f] = [](Basilisk* b) { return 0.125; };
        w.bend[f] = 0.0;
        w.speed[f] = globals::stdval::speed::normal;
        w.acclim[f] = globals::stdval::acclim::standard;
        w.min_stepdur[f] = 0;
        w.max_stepdur[f] = globals::stdval::maxdur::safe;
        w.interval[f] = 0;
      }
      w.steps = -1;
      w.exit_condition = [](Basilisk* b) {
        auto& c = b->cmd_.sufi;
        return abs(b->imu_.GetYaw(true) - c.dest_yaw) < c.exit_thr;
      };
      w.exit_to_mode = M::Idle_Init;
    } break;
    default:
      break;
  }
}
