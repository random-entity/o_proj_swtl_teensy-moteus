#pragma once

#include "meta.h"

namespace bounce_walk {
double tgt_yaw;
bool moonwalk;
uint32_t muteki_until = 0;
}  // namespace bounce_walk

void ModeRunners::BounceWalk(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.bounce_walk;
  auto& w = b->cmd_.walk;

  switch (m) {
    case M::BounceWalk: {
      m = M::Walk;
      bounce_walk::tgt_yaw = c.init_tgt_yaw;
      w.init_didimbal = BOOL_L;
      for (LRIdx f : IDX_LR) {
        w.tgt_yaw[f] = [](Basilisk* b) {
          const auto cur_yaw = b->imu_.GetYaw(true);
          auto result = bounce_walk::tgt_yaw;
          result = nearest_pmn(cur_yaw, result);
          if (abs(result - cur_yaw) > 0.25) {
            bounce_walk::moonwalk = true;
            result = nearest_pmn(cur_yaw, result + 0.5);
          } else {
            bounce_walk::moonwalk = false;
          }
          return result;
        };
        w.stride[f] = [](Basilisk*) {
          double result = 0.125;
          return bounce_walk::moonwalk ? -result : result;
        };
        w.bend[f] = 0.0;
        w.speed[f] = globals::stdval::speed::normal;
        w.acclim[f] = globals::stdval::acclim::normal;
        w.min_stepdur[f] = 0;
        w.max_stepdur[f] = globals::stdval::maxdur::safe;
        w.interval[f] = 0;
      }
      w.steps = -1;
      w.exit_condition = [](Basilisk* b) {
        if (millis() <= bounce_walk::muteki_until) return false;

        bool start_muteki = false;

        const auto collision = b->Collision();
        Vec2 yaw_sum{};

        for (uint8_t other_suid = 1; other_suid <= 13; other_suid++) {
          if (other_suid == b->cfg_.suid) continue;
          if (!(collision & (1 << (other_suid - 1)))) continue;

          yaw_sum.add(Vec2{roster::db[other_suid - 1].yaw});
          start_muteki = true;
        }
        if (start_muteki) {
          bounce_walk::tgt_yaw = yaw_sum.arg();
        }

        if (!b->lps_.BoundMinX() || !b->lps_.BoundMaxX()) {
          bounce_walk::tgt_yaw = 0.5 - bounce_walk::tgt_yaw;
          start_muteki = true;
        }
        if (!b->lps_.BoundMinY() || !b->lps_.BoundMaxY()) {
          bounce_walk::tgt_yaw *= -1.0;
          start_muteki = true;
        }

        if (start_muteki) {
          bounce_walk::muteki_until = millis() + 5000;
        }

        return false;
      };
    } break;
    default:
      break;
  }
}
