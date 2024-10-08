#pragma once

#include "../helpers/timing.h"
#include "../servo_units/basilisk.h"

#ifndef XBEE_SERIAL
#define XBEE_SERIAL Serial4
#endif
#define XBEE_PACKET_LEN_INCLUDING_START_BYTES (50)

class XbeeReplySender {
 public:
  inline static Basilisk* b_;
  inline static bool waiting_send_ = false;
  inline static uint32_t TEMP_start_bytes_us;
  // inline static uint32_t send_at_us;

  // Should be called before use.
  inline static bool Setup(Basilisk* b) {
    if (!b) {
      Serial.println("XbeeReplySender: Null reference to Basilisk");
      return false;
    }
    b_ = b;
    xbee_rpl_.decoded.suids = 1 << (b->cfg_.suid - 1);
    Serial.println("XbeeReplySender: Setup complete");
    return true;
  }

  // Should be run continuously
  inline static void Run() {
    using namespace xb_timing;

    const static auto suid_m1 = b_->cfg_.suid - 1;
    const static uint32_t send_lb = tmot_st_to_wa_us + suid_m1 * rpl_snd_itv_us;
    const static uint32_t send_ub = send_lb + rpl_snd_tmot_us;

    if (!waiting_send_) return;
    if (globals::poll_clk_us < send_lb) return;
    waiting_send_ = false;
    if (globals::poll_clk_us >= send_ub) return;

    Serial.print("RB ");
    Serial.println(globals::poll_clk_us);

    Send();

    Serial.print("RD ");
    Serial.println(globals::poll_clk_us);
  }

  inline static void Send() {
    xbee_rpl_.decoded.mode = static_cast<uint8_t>(*b_->rpl_.mode);
    xbee_rpl_.decoded.lpsx = static_cast<float>(*b_->rpl_.lpsx);
    xbee_rpl_.decoded.lpsy = static_cast<float>(*b_->rpl_.lpsy);
    xbee_rpl_.decoded.yaw = static_cast<float>(b_->rpl_.yaw());

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
