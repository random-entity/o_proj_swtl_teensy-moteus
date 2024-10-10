#pragma once

#include "../helpers/imports.h"
#include "../servo_units/basilisk.h"

namespace tests {
using M = Basilisk::Command::Mode;

void Pivot(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.pivot;

  m = M::Pivot_Init;
  c.didimbal = BOOL_L;
  c.tgt_yaw = [](Basilisk*) { return NaN; };
  c.stride = 0.125;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = -0.125;
  c.speed = globals::stdval::speed::fast;
  c.acclim = globals::stdval::acclim::standard;
  c.min_dur = 2000;
  c.max_dur = -1;
  c.exit_to_mode = M::Idle_Init;
}

void PivSpin(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.piv_spin;

  m = M::PivSpin;
  c.didimbal = BOOL_L;
  c.dest_yaw = NaN;
  c.exit_thr = NaN;
  c.stride = 0.125;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = 0.0;
  c.speed = globals::stdval::speed::normal;
  c.acclim = globals::stdval::acclim::standard;
  c.min_stepdur = 0;
  c.max_stepdur = -1;
  c.interval = 0;
  c.steps = -1;
}

void Diamond(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.diamond;

  m = M::Diamond;
  c.init_didimbal = BOOL_L;
  c.init_stride = 0.3;
  c.speed = globals::stdval::speed::fast;
  c.acclim = globals::stdval::acclim::standard;
  c.min_stepdur = 0;
  c.max_stepdur = -1;
  c.interval = 100;
  c.steps = -1;
}

void WalkToDir(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk_to_dir;

  m = M::WalkToDir;
  c.init_didimbal = BOOL_L;
  c.tgt_yaw = 0.0;
  c.stride = 0.125;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = 0.0;
  c.speed = globals::stdval::speed::normal;
  c.acclim = globals::stdval::acclim::standard;
  c.min_stepdur = 1000;
  c.max_stepdur = 3000;
  c.interval = 0;
  c.steps = -1;
}

void WalkToPos(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk_to_pos;

  m = M::WalkToPos;
  c.init_didimbal = BOOL_L;
  c.tgt_pos = Vec2{(b->cfg_.lps.minx + b->cfg_.lps.maxx) * 0.5,
                   (b->cfg_.lps.miny + b->cfg_.lps.maxy) * 0.5};
  c.dist_thr = 30;
  c.stride = 0.125;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = 0.0;
  c.speed = globals::stdval::speed::normal;
  c.acclim = globals::stdval::acclim::standard;
  c.min_stepdur = 1000;
  c.max_stepdur = 3000;
  c.interval = 0;
  c.steps = -1;
}

void Sufi(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.sufi;

  m = M::Sufi;
  c.init_didimbal = BOOL_L;
  c.dest_yaw = NaN;
  c.exit_thr = NaN;
  c.stride = 0.125;
  c.bend[IDX_L] = 0.0;
  c.bend[IDX_R] = 0.0;
  c.speed = globals::stdval::speed::normal;
  c.acclim = globals::stdval::acclim::standard;
  c.min_stepdur = 1000;
  c.max_stepdur = 3000;
  c.interval = 0;
  c.steps = -1;
}

void BounceWalk(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.bounce_walk;

  m = M::BounceWalk_Init;
  c.init_tgt_yaw = -0.25;  //  random(360) / 360.0;
}

void WalkToPosInField(Basilisk* b) {
  auto& m = b->cmd_.mode;
  auto& c = b->cmd_.walk_to_pos_in_field;

  m = M::WalkToPosInField;
  c.tgt_pos = Vec2{500.0, 500.0};
}

}  // namespace tests
