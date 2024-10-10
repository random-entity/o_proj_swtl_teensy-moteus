#pragma once

#include "meta.h"

void Presets::SetGlobalSetPhisSpeed(Basilisk* b, int level) {
  globals::var::speed = level == 0   ? globals::stdval::speed::slower
                        : level == 1 ? globals::stdval::speed::slow
                        : level == 2 ? globals::stdval::speed::normal
                        : level == 3 ? globals::stdval::speed::fast
                        : level == 4 ? globals::stdval::speed::faster
                                     : globals::stdval::speed::normal;

  b->cmd_.mode = b->cmd_.do_preset.prev_mode;
}
