#pragma once

#include <Arduino.h>

namespace xb_timing {

const uint32_t tmot_st_to_wa_us = 4000;                   // c_lim
const uint32_t rpl_snd_itv_us = tmot_st_to_wa_us + 3000;  // r
const uint32_t rpl_snd_tmot_us = 100;

}  // namespace xb_timing
