// Might NOT work with catwalk.

#pragma once

#include "meta.h"

namespace walk_to_pos_in_field {
bool moonwalk;
}  // namespace walk_to_pos_in_field

void ModeRunners::WalkToPosInField(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& w = b->cmd_.walk;

  switch (m) {
    case M::WalkToPosInField: {
      m = M::Walk;
      w.init_didimbal = BOOL_L;
      for (uint8_t f : IDX_LR) {
        w.tgt_yaw[f] = [](Basilisk* b) {
          const auto& c = b->cmd_.walk_to_pos_in_field;

          const auto pos = b->lps_.GetPos();
          const auto tgt_delta_pos = c.tgt_pos - pos;
          const auto pure_tgt_yaw_vec = tgt_delta_pos.normalize();
          Vec2 force{0.0, 0.0};

          for (uint8_t other_suid = 1; other_suid <= 13; other_suid++) {
            if (other_suid == b->cfg_.suid) continue;

            const auto& other = roster::db[other_suid - 1];
            const auto other_pos = Vec2{other.x, other.y};

            const auto dist_vec = other_pos - pos;
            const auto dist = dist_vec.mag();

            if (dist > b->coll_thr_) continue;

            const auto watch =
                nearest_pmn(0.0, dist_vec.argsub(pure_tgt_yaw_vec));

            const auto at_front = abs(watch) < 0.25;
            if (!at_front) continue;

            // At this point, the other is in collision boundary and at
            // front.

            const auto at_right = watch < 0.0;

            const double r = max((dist - b->coll_thr_) * 0.1, 0.001);
            const double mag = 1.0 / sq(r);
            force = force +
                    mag * Vec2{dist_vec.arg() + (at_right ? 1.0 : -1.0) * 0.25};
          }

          if (!b->lps_.BoundMinX())
            force = force + Vec2{0.0};
          else if (!b->lps_.BoundMaxX())
            force = force + Vec2{0.5};
          if (!b->lps_.BoundMinY())
            force = force + Vec2{0.25};
          else if (!b->lps_.BoundMaxY())
            force = force + Vec2{-0.25};

          const auto cur_yaw = b->imu_.GetYaw(true);
          const auto field_tgt_yaw_vec = pure_tgt_yaw_vec + force;
          auto field_tgt_yaw = nearest_pmn(cur_yaw, field_tgt_yaw_vec.arg());

          if (abs(field_tgt_yaw - cur_yaw) > 0.25) {
            walk_to_pos_in_field::moonwalk = true;
            field_tgt_yaw = nearest_pmn(cur_yaw, field_tgt_yaw + 0.5);
          } else {
            walk_to_pos_in_field::moonwalk = false;
          }
          return field_tgt_yaw;
        };
        w.stride[f] = [](Basilisk* b) {
          double result = 0.125;
          return walk_to_pos_in_field::moonwalk ? -result : result;
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
        const auto& c = b->cmd_.walk_to_pos_in_field;
        return b->lps_.GetPos().dist(c.tgt_pos) < 25.0;
      };
      w.exit_to_mode = M::Idle_Init;
    } break;
    default:
      break;
  }
}
