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
      bounce_walk::tgt_yaw = c.init_tgt_yaw;

      m = M::Walk;
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
        // if (millis() <= bounce_walk::muteki_until) {
        //   // Serial.println("MUTEKI");
        //   return false;
        // }
        // bool start_muteki = false;
        // const auto collision = b->Collision();
        // Vec2 yaw_sum{};
        // for (uint8_t other_suid = 1; other_suid <= 13; other_suid++) {
        //   if (other_suid == b->cfg_.suid) continue;
        //   if (!(collision & (1 << (other_suid - 1)))) continue;
        //   yaw_sum.add(Vec2{roster::db[other_suid - 1].yaw});
        //   start_muteki = true;
        // }
        // if (start_muteki) {
        //   bounce_walk::tgt_yaw = yaw_sum.arg();
        // }
        // if ((b->lps_.TrespassedMinX() &&
        //      abs(nearest_pmn(0.0, bounce_walk::tgt_yaw)) >= 0.25) ||
        //     (b->lps_.TrespassedMaxX() &&
        //      abs(nearest_pmn(0.0, bounce_walk::tgt_yaw)) <= 0.25)) {
        //   bounce_walk::tgt_yaw = 0.5 - bounce_walk::tgt_yaw;
        //   for (int i = 0; i < 10; i++) Serial.println("TrespassedX");
        //   Serial.print("Bouced tgt_yaw -> ");
        //   Serial.println(bounce_walk::tgt_yaw);
        //   start_muteki = true;
        // }
        // if (b->lps_.TrespassedMinY() || b->lps_.TrespassedMaxY()) {
        //   bounce_walk::tgt_yaw *= -1.0;
        //   for (int i = 0; i < 10; i++) Serial.println("TrespassedY");
        //   Serial.print("Bouced tgt_yaw -> ");
        //   Serial.println(bounce_walk::tgt_yaw);
        //   start_muteki = true;
        // }

        if (bounce_walk::reinit) return true;

        if ((!b->lps_.BoundMinX() &&
             abs(0.5 - nearest_pmn(0.5, bounce_walk::tgt_yaw)) <= 0.25) ||
            (!b->lps_.BoundMaxX() &&
             abs(nearest_pmn(0.0, bounce_walk::tgt_yaw)) <= 0.25)) {
          bounce_walk::tgt_yaw = 0.5 - bounce_walk::tgt_yaw;
          bounce_walk::reinit = true;

          Serial.println("X OUT");
          Serial.println("Bounced tgt_yaw ");
          Serial.println(bounce_walk::tgt_yaw);
        }

        if ((!b->lps_.BoundMinY() &&
             abs(-0.25 - nearest_pmn(-0.25, bounce_walk::tgt_yaw)) <= 0.25) ||
            (!b->lps_.BoundMaxY() &&
             abs(0.25 - nearest_pmn(0.25, bounce_walk::tgt_yaw)) <= 0.25)) {
          bounce_walk::tgt_yaw *= -1.0;
          bounce_walk::reinit = true;

          Serial.println("Y OUT");
          Serial.println("Bounced tgt_yaw ");
          Serial.println(bounce_walk::tgt_yaw);
        }

        // if (!b->lps_.BoundMinY() || !b->lps_.BoundMaxY()) {
        //   return true;
        //   Serial.println("Y NOT Bound");
        //   bounce_walk::tgt_yaw *= -1.0;
        //   start_muteki = true;
        // }
        // if (start_muteki) {
        //   Serial.println("Start Muteki");
        //   bounce_walk::muteki_until = millis() + 5000;
        // }
        // if (reinit) {
        //   b->cmd_.set_phis.exit_condition = [](Basilisk*) { return true; };
        //   b->cmd_.pivseq.exit_condition = [](Basilisk*) { return true; };
        //   b->cmd_.pivseq.exit_to_mode = M::BounceWalk;
        // }

        return bounce_walk::reinit;
      };
      w.exit_to_mode = M::BounceWalk_Reinit;
    } break;
    case M::BounceWalk_Reinit: {
      bounce_walk::reinit = false;
      m = M::Walk;
    } break;
    default:
      break;
  }
}
