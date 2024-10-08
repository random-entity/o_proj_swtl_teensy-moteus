#pragma once

#include "../servo_units/basilisk.h"

struct BasiliskOneshots {
  static void CRMuxXbee(Basilisk*);
  static void SetBaseYaw(Basilisk*);

  static inline const std::map<uint8_t, void (*)(Basilisk*)> oneshots = {
      {ONESHOT_CRMuxXbee, &CRMuxXbee},
      {ONESHOT_SetBaseYaw, &SetBaseYaw},
  };

  static void Shoot(Basilisk* b) {
    for (uint8_t oneshot = 0; oneshot < 8; oneshot++) {
      if (b->cmd_.oneshots & (1 << oneshot)) {
        const auto maybe_method = SafeAt(oneshots, oneshot);
        if (maybe_method) {
          (*maybe_method)(b);
        }
      }
    }

    // It is each Oneshot method's responsibility to reset corresponding bit to
    // zero since there may be time-based Oneshots like ReplyNext.
    // b->cmd_.oneshots = 0;
  }
};
