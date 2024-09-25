#pragma once

#include "mode_runners.h"

void ModeRunners::WalkToPos(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk_to_pos;

  switch (m) {
    case M::WalkToPos: {
      // Serial.println("ModeRunners::WalkToPos");

      m = M::Walk;
      auto& w = b->cmd_.walk;
      w.init_didimbal = c.init_didimbal;
      for (uint8_t f : IDX_LR) {
        w.tgt_yaw[f] = [](Basilisk* b) {
          const auto& c = b->cmd_.walk_to_pos;
          const auto tgt_delta_pos = c.tgt_pos - b->lps_.GetPos();
          return tgt_delta_pos.arg();
        };
        w.stride[f] = [](Basilisk* b) {
          auto& c = b->cmd_.walk_to_pos;
          return c.stride;
        };
        w.bend[f] = c.bend[f];
        w.speed[f] = c.speed;
        w.acclim[f] = c.acclim;
        w.min_stepdur[f] = c.min_stepdur;
        w.max_stepdur[f] = c.max_stepdur;
        w.interval[f] = c.interval;
      }
      w.steps = c.steps;
      w.exit_condition = [](Basilisk* b) {
        const auto& c = b->cmd_.walk_to_pos;
        return b->lps_.GetPos().dist(c.tgt_pos) < abs(c.prox_thr);
      };
    } break;
    default:
      break;
  }
}
