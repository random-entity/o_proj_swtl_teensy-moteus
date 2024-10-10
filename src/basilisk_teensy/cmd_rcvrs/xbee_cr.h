#pragma once

#include "../helpers/timing.h"
#include "../roster/db.h"
#include "../rpl_sndrs/xbee_rs.h"
#include "../servo_units/basilisk.h"

#define XBEE_SERIAL Serial4
#define XBEE_PACKET_LEN 46  // NOT counting the 4 starting bytes.

// #define TIMEOUT_START_TO_WAITAGAIN_US (5000)
// #define T_FROM_START_TO_SEND_US (10)

class XbeeCommandReceiver {
  using C = Basilisk::Command;
  using M = C::Mode;

 public:
  bool Setup(Basilisk* b) {
    if (!b) {
      Serial.println("XbeeCommandReceiver: Null pointer to Basilisk");
      return false;
    }
    b_ = b;
    Serial.println("XbeeCommandReceiver: Registered reference to Basilisk");

    XBEE_SERIAL.begin(115200);
    if (!XBEE_SERIAL) {
      Serial.println("XbeeCommandReceiver: XBEE_SERIAL(Serial4) begin failed");
      return false;
    }

    Serial.println("XbeeCommandReceiver: Setup complete");
    return true;
  }

  void Run() {
    static uint8_t start = 0;
    static RecvBuf temp_rbuf;
    static uint8_t buf_idx;
    static uint32_t start_time_us;

    if (!receiving_) {
      while (XBEE_SERIAL.available() > 0 && start < 4) {
        const uint8_t rbyte = XBEE_SERIAL.read();
        if (rbyte == 255) {
          start++;
        } else {
          start = 0;
        }
      }

      if (start < 4) return;

      receiving_ = true;
      buf_idx = 0;
      got_full_packet = false;
      start_time_us = micros();
      globals::poll_clk_us = 0;  // Reset at start bytes anyway,
                                 // then set/reset waiting send flag later.

      // Serial.println("*****");
      // Serial.print("SUID ");
      // Serial.println(b_->cfg_.suid);
      // Serial.print("ST ");
      // Serial.print(start_time_us);
      // Serial.print(" -> ");
      // Serial.println(0);
    }

    // Cannot pass this point if (waiting mode).
    // From this point, (receiving mode)

    if (micros() > start_time_us + timing::xb::tmot_st_to_wa_us) {
      // Serial.println();
      // Serial.print("WA ");
      // Serial.println(micros() - start_time_us);

      if (!got_full_packet) Serial.println("TIMEOUT!");

      receiving_ = false;
      start = 0;
      return;
    }

    // Cannot pass this point if (receiving mode) && (receive timeout).
    // From this point, (receiving mode) && (before receive timeout).

    while (XBEE_SERIAL.available() > 0 && buf_idx < XBEE_PACKET_LEN) {
      temp_rbuf.raw_bytes[buf_idx] = XBEE_SERIAL.read();
      buf_idx++;
      // Serial.print(temp_rbuf.raw_bytes[buf_idx - 1]);
      // Serial.print(", ");
    }
    if (buf_idx < XBEE_PACKET_LEN) return;
    // Serial.println();

    // Cannot pass this point if !(got full packet).
    // From this point,
    // (receiving mode) && (before receive timeout) && (got full packet).

    got_full_packet = true;
    // Serial.print("FP ");
    // Serial.println(micros() - start_time_us);

    if (temp_rbuf.decoded.oneshots & (1 << ONESHOT_SaveOthersReply)) {
      // This is other's Reply. Parse and save to roster,
      // then immediately return to waiting mode.

      for (uint8_t other_suid = 1; other_suid <= 13; other_suid++) {
        if (other_suid == b_->cfg_.suid) continue;
        if (temp_rbuf.decoded.suids & (1 << (other_suid - 1))) {
          roster::db[other_suid - 1].x =
              temp_rbuf.decoded.u.save_others_reply.lpsx;
          roster::db[other_suid - 1].y =
              temp_rbuf.decoded.u.save_others_reply.lpsy;
          roster::db[other_suid - 1].yaw =
              temp_rbuf.decoded.u.save_others_reply.yaw;

          // Serial.print("Received Reply from SUID ");
          // Serial.println(other_suid);

          break;  // There should be no Reply with multiple SUIDs.
        }
      }

      // Serial.print("FD ");
      // Serial.println(micros() - start_time_us);
      receiving_ = false;
      start = 0;
      return;
    } else if (temp_rbuf.decoded.oneshots & (1 << ONESHOT_GlobalPoll)) {
      // This is a global Poll. SUIDs field does not matter. Reserve Reply send
      // as soon as 0 <= (poll clock) - (c_lim + (suid - 1) * r) < 100us holds.
      // Since poll clock is already reset to 0us at start bytes reception,
      // just set the waiting send flag now.

      XbeeReplySender::waiting_send_ = true;

      // Serial.println("Poll received, send flag set");
      // Serial.print("FD ");
      // Serial.println(micros() - start_time_us);
      receiving_ = false;
      start = 0;
      return;
    } else {
      // This is neither a Reply nor a Poll, so it must be a normal Command.

      if (temp_rbuf.decoded.suids & (1 << (b_->cfg_.suid - 1))) {
        // This Command is for me. Copy to memory and set waiting parse flag,
        // then immediately return to waiting mode.

        memcpy(xb_cmd_.raw_bytes, temp_rbuf.raw_bytes, XBEE_PACKET_LEN);
        waiting_parse_ = true;

        // Serial.println("Command for me received, copied to memory");
        // Serial.print("FD ");
        // Serial.println(micros() - start_time_us);
        receiving_ = false;
        start = 0;
        return;
      } else {
        // Serial.println("This Command is NOT for me");
        // Serial.print("FPD ");
        // Serial.println(micros() - start_time_us);
        receiving_ = false;
        start = 0;
        return;
      }
    }
  }

  inline static void Parse() {
    Serial.println("Parse begin");

    static auto& x = xb_cmd_.decoded;
    static auto& c = b_->cmd_;
    static auto& m = c.mode;

    c.oneshots = x.oneshots;
    if (c.oneshots) {
      if (c.oneshots & (1 << ONESHOT_SetBaseYaw)) {
        c.set_base_yaw.offset = static_cast<double>(x.u.set_base_yaw.offset);
      }

      return;  // Oneshot- and Mode- Commands are NOT processed from
               // a single Command since there are Oneshots that
               // require additional values.
    }

    const auto maybe_new_mode = static_cast<M>(x.mode);

    if (maybe_new_mode == M::DoPreset) {
      if (x.u.do_preset.idx[b_->cfg_.suid - 1] == 0) {
        return;  // Do not even switch Mode.  Previous DoPreset Command
                 // execution's future-chaining can be happening now.
      }

      c.do_preset.prev_mode = m;  // Save previous Mode before switching.
    }

    m = maybe_new_mode;
    switch (m) {
      case M::DoPreset: {
        c.do_preset.idx = x.u.do_preset.idx[b_->cfg_.suid - 1];

        // Serial.print("Copied to memory, Preset index ");
        // Serial.print(c.do_preset.idx);
        // Serial.println();
      } break;
      case M::Pivot_Init: {
        c.pivot.bend[IDX_L] = static_cast<double>(x.u.pivot.bend_l);
        c.pivot.bend[IDX_R] = static_cast<double>(x.u.pivot.bend_r);
        c.pivot.didimbal = static_cast<bool>(x.u.pivot.didimbal);
        c.pivot.speed = static_cast<double>(x.u.pivot.speed);
        c.pivot.stride = static_cast<double>(x.u.pivot.stride);
        c.pivot.tgt_yaw = [](Basilisk*) {
          return static_cast<double>(x.u.pivot.tgt_yaw);
        };
        c.pivot.min_dur = 0;
        c.pivot.max_dur = 6000;
        c.pivot.exit_condition = nullptr;
        c.pivot.acclim = 1.0;
        c.pivot.exit_to_mode = M::Idle_Init;
      } break;
      default: {
        // Serial.print("XbeeCommandReceiver: Mode ");
        // Serial.print(x.mode);
        // Serial.print(" is NOT registered");
        // Serial.println();
      } break;
    }
  }

  inline static union RecvBuf {
    struct Decoded {
      uint16_t suids;
      uint8_t oneshots;
      uint8_t mode;
      union {
        struct {
          float offset;
        } __attribute__((packed)) set_base_yaw;
        struct {
          float lpsx;
          float lpsy;
          float yaw;
        } __attribute__((packed)) save_others_reply;
        struct {
          uint16_t idx[13];  // The Goguma version of DoPreset protocol.
        } __attribute__((packed)) do_preset;
        struct {
          float bend_l;
          float bend_r;
          uint8_t didimbal;
          float speed;
          float stride;
          float tgt_yaw;
        } __attribute__((packed)) pivot;
      } u;
    } __attribute__((packed)) decoded;
    uint8_t raw_bytes[XBEE_PACKET_LEN];
  } xb_cmd_;

  inline static bool receiving_ = false;
  inline static bool waiting_parse_ = false;
  inline static bool waiting_xb_rpl_send_ = false;

  // Flags for LedReplySender
  inline static bool led_got_start_bytes = false;
  inline static bool got_full_packet = false;
  inline static bool led_got_my_cmd = false;
  inline static bool led_timeout_miss = false;

 private:
  inline static Basilisk* b_ = nullptr;
};
