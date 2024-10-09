#pragma once

#include <Arduino.h>

#include <map>

namespace timing {
namespace xb {

const uint32_t tmot_st_to_wa_us = 10000;                  // c_lim
const uint32_t rpl_snd_itv_us = tmot_st_to_wa_us + 3000;  // r
const uint32_t rpl_snd_tmot_us = 9000;
uint8_t span;
inline static const std::map<uint8_t, uint32_t> suid_to_sndtim_us = [] {
  const auto& a = tmot_st_to_wa_us;
  const auto& r = rpl_snd_itv_us;

  std::map<uint8_t, uint32_t> result;

  uint8_t i_in_span = 0;
  uint8_t j = 0;

  for (uint8_t i = 0; i <= 12;) {  // i == SUID - 1
    uint32_t base = a + (j == 0 ? 0 : (100 * j - 10) * 1000);
    uint32_t limit = (100 * j + 80) * 1000;
    uint32_t t = base + i_in_span * r;

    if (t <= limit) {
      result[i + 1] = t;
      i++;
      i_in_span++;
    } else {
      j++;
      i_in_span = 0;
    }
  }

  span = j + 1;

  return result;
}();

}  // namespace xb
}  // namespace timing
