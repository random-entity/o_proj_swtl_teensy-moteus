#pragma once

#include "../servo_units/basilisk.h"

#ifndef XBEE_SERIAL
#define XBEE_SERIAL Serial4
#endif
#define XBEE_PACKET_LEN_INCLUDING_START_BYTES \
  50  // Including the 4 starting bytes.

class XbeeReplySender {
 public:
  inline static void Send(Basilisk::Reply rpl) {
    xbee_rpl_.decoded.suids = 1 << (*rpl.suid - 1);
    xbee_rpl_.decoded.mode = static_cast<uint8_t>(*rpl.mode);
    xbee_rpl_.decoded.lpsx = static_cast<float>(*rpl.lpsx);
    xbee_rpl_.decoded.lpsy = static_cast<float>(*rpl.lpsy);
    xbee_rpl_.decoded.yaw = static_cast<float>(rpl.yaw());

    XBEE_SERIAL.write(xbee_rpl_.raw_bytes,
                      XBEE_PACKET_LEN_INCLUDING_START_BYTES);
  }

  inline static union SendBuf {
    struct Decoded {
      const uint32_t start_bytes;  // Start bytes are included!
      uint16_t suids;
      const uint8_t oneshots;
      uint8_t mode;
      float lpsx;
      float lpsy;
      float yaw;
    } __attribute__((packed)) decoded;
    uint8_t raw_bytes[XBEE_PACKET_LEN_INCLUDING_START_BYTES];

    SendBuf()
        : decoded{.start_bytes{static_cast<uint32_t>(-1)},
                  .oneshots{1 << ONESHOT_SaveOthersReply}} {}
  } xbee_rpl_;
};
