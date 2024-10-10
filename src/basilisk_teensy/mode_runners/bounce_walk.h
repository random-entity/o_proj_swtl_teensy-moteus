#pragma once

#include "meta.h"

namespace bounce_walk {
double tgt_yaw;
bool moonwalk;
bool reinit;
}  // namespace bounce_walk

void ModeRunners::BounceWalk(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.bounce_walk;
  auto& w = b->cmd_.walk;

  switch (m) {
    case M::BounceWalk_Init: {
      bounce_walk::reinit = false;
      const auto cur_yaw = b->imu_.GetYaw(true);
      bounce_walk::tgt_yaw = nearest_pmn(cur_yaw, c.init_tgt_yaw);
      bounce_walk::moonwalk = (abs(bounce_walk::tgt_yaw - cur_yaw) > 0.25);

      m = M::Walk;
      w.init_didimbal = BOOL_L;
      for (LRIdx f : IDX_LR) {
        w.tgt_yaw[f] = [](Basilisk* b) {
          return bounce_walk::moonwalk ? nearest_pmn(b->imu_.GetYaw(true),
                                                     bounce_walk::tgt_yaw + 0.5)
                                       : bounce_walk::tgt_yaw;
        };
        w.stride[f] = [](Basilisk*) {
          double result = 0.125;
          return bounce_walk::moonwalk ? -result : result;
        };
        w.bend[f] = 0.0;
        w.speed[f] = globals::stdval::speed::normal;
        w.acclim[f] = globals::stdval::acclim::standard;
        w.min_stepdur[f] = 0;
        w.max_stepdur[f] = globals::stdval::maxdur::safe;
        w.interval[f] = 0;
      }
      w.steps = -1;
      w.exit_condition = [](Basilisk* b) {
        if (bounce_walk::reinit) return true;

        const auto my_pos = b->lps_.GetPos();
        const auto my_tgt_yaw = bounce_walk::tgt_yaw;

        Vec2 new_tgt_yaw{0.0, 0.0};

        for (uint8_t other_suid = 1; other_suid <= 13; other_suid++) {
          if (other_suid == b->cfg_.suid) continue;

          const auto& other = roster::db[other_suid - 1];
          const auto other_pos = Vec2{other.x, other.y};

          const auto dist_vec = other_pos - my_pos;

          if (dist_vec.mag() > b->coll_thr_) continue;

          const auto at_front =
              abs(nearest_pmn(0.0, dist_vec.arg() - my_tgt_yaw)) < 0.25;
          if (!at_front) continue;

          // At this point, the other is in collision boundary and at front.
          new_tgt_yaw = new_tgt_yaw + Vec2{dist_vec.arg() + 0.5};
          bounce_walk::reinit = true;
        }

        if (bounce_walk::reinit) bounce_walk::tgt_yaw = new_tgt_yaw.arg();

        if ((!b->lps_.BoundMinX() &&
             abs(0.5 - nearest_pmn(0.5, bounce_walk::tgt_yaw)) <= 0.25) ||
            (!b->lps_.BoundMaxX() &&
             abs(nearest_pmn(0.0, bounce_walk::tgt_yaw)) <= 0.25)) {
          bounce_walk::tgt_yaw = 0.5 - bounce_walk::tgt_yaw;
          bounce_walk::reinit = true;
        }

        if ((!b->lps_.BoundMinY() &&
             abs(-0.25 - nearest_pmn(-0.25, bounce_walk::tgt_yaw)) <= 0.25) ||
            (!b->lps_.BoundMaxY() &&
             abs(0.25 - nearest_pmn(0.25, bounce_walk::tgt_yaw)) <= 0.25)) {
          bounce_walk::tgt_yaw *= -1.0;
          bounce_walk::reinit = true;
        }

        return bounce_walk::reinit;
      };
      w.exit_to_mode = M::BounceWalk_Reinit;
    } break;
    case M::BounceWalk_Reinit: {
      bounce_walk::reinit = false;
      const auto cur_yaw = b->imu_.GetYaw(true);
      bounce_walk::tgt_yaw = nearest_pmn(cur_yaw, bounce_walk::tgt_yaw);
      bounce_walk::moonwalk = (abs(bounce_walk::tgt_yaw - cur_yaw) > 0.25);

      m = M::Walk;
    } break;
    default:
      break;
  }
}
