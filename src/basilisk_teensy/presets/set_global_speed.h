#pragma once

#include "meta.h"

namespace set_global_speed {
Basilisk::Command::Pivot (*pivots)(Basilisk*, int);
PhiSpeed speed;
}  // namespace set_global_speed

void Presets::SetGlobalSetPhisSpeed(Basilisk* b, int level) {
  using namespace globals::stdval;

  set_global_speed::speed = level == 0   ? speed::slower
                            : level == 1 ? speed::slow
                            : level == 2 ? speed::normal
                            : level == 3 ? speed::fast
                            : level == 4 ? speed::faster
                                         : speed::normal;

  for (uint8_t f : IDX_LR) {
    b->cmd_.set_phis.tgt_phispeed[f] = set_global_speed::speed;
  }

  set_global_speed::pivots = b->cmd_.pivseq.pivots;

  b->cmd_.pivseq.pivots = [](Basilisk* b, int step) {
    Basilisk::Command::Pivot p = set_global_speed::pivots(b, step);
    p.speed = set_global_speed::speed;
    return p;
  };

  b->cmd_.mode = b->cmd_.do_preset.prev_mode;
}
