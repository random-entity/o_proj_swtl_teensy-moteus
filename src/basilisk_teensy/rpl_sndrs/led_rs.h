#pragma once

#include <Arduino.h>

#include "../cmd_rcvrs/xbee_cr.h"
#include "../components/neokey.h"

void LedReplySender(Neokey& nk) {
  static Beat teensy_hearbeat{125};
  static bool on = true;
  if (teensy_hearbeat.Hit()) {
    nk.setPixelColor(0, on ? 0x400040 : 0x000000);
    nk.show();
    on = !on;
  }

  using XCR = XbeeCommandReceiver;
  nk.setPixelColor(1, XCR::led_got_start_bytes ? 0x002020 : 0x000000);
  nk.setPixelColor(2, XCR::got_full_packet ? 0x000020 : 0x000000);
  nk.setPixelColor(2, XCR::led_got_my_cmd ? 0x002020 : 0x000000);
  nk.setPixelColor(3, XCR::led_timeout_miss ? 0x800000 : 0x000000);
  nk.show();
}
