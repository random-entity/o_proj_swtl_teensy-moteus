#pragma once

#include "../roster/db.h"
#include "../rpl_sndrs/xbee_rs.h"
#include "../servo_units/basilisk.h"

#define XBEE_SERIAL Serial4
#define XBEE_PACKET_LEN 46  // NOT counting the 4 starting bytes.

#define T_FROM_START_TO_WAITAGAIN_US 10
#define T_FROM_START_TO_SEND_US 10

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
      if (waiting_xb_rpl_send_ &&
          micros() >= start_time_us + T_FROM_START_TO_SEND_US) {
        Serial.print("RS_start ");
        Serial.println(micros() - start_time_us);

        XbeeReplySender::Send(b_->rpl_);
        waiting_xb_rpl_send_ = false;

        Serial.print("RS_done ");
        Serial.println(micros() - start_time_us);
      }

      if (XBEE_SERIAL.available() > 0) {
        uint8_t rbyte = XBEE_SERIAL.read();
        if (rbyte == 255) {
          start++;
        } else {
          start = 0;
        }

        if (start < 4) return;

        led_got_start_bytes = true;
        led_got_my_cmd = false;
        led_timeout_miss = false;

        receiving_ = true;
        buf_idx = 0;
        got_full_packet = false;
        start_time_us = micros();

        Serial.print("*****\nS ");
        Serial.print(start_time_us);
        Serial.print(" -> ");
        Serial.println(0);
      } else {
        return;
      }
    }

    if (micros() > start_time_us + T_FROM_START_TO_WAITAGAIN_US) {
      Serial.print("TU ");
      Serial.println(micros() - start_time_us);

      if (!got_full_packet) Serial.println("TIMEOUT!");

      receiving_ = false;
      start = 0;
      return;
    } else if (got_full_packet) {
      Serial.print("Done processing full packet but waiting TU ");
      Serial.println(micros() - start_time_us);
      return;
    }

    {  // Started receiving payload, but not yet timed out.
      if (got_full_packet) return;

      while (XBEE_SERIAL.available() > 0 && buf_idx < XBEE_PACKET_LEN) {
        temp_rbuf.raw_bytes[buf_idx] = XBEE_SERIAL.read();
        buf_idx++;
      }
      if (buf_idx < XBEE_PACKET_LEN) return;

      // Received full byte array within time limit since start bytes reception.
      // We will never come back here again once we return from here.
      got_full_packet = true;
      Serial.print("F ");
      Serial.println(micros() - start_time_us);

      /* Print for debug */ {
        Serial.print("SUID ");
        Serial.println(b_->cfg_.suid);
        // Serial.print(
        //     " received full Xbee packet within time limit since start
        //     bytes");
        // Serial.println();
        // Serial.print("micros() -> ");
        // Serial.print(micros());
        // Serial.println();
        // Serial.print("Packet bytes -> ");
        // for (size_t i = 0; i < XBEE_PACKET_LEN; i++) {
        //   Serial.print(temp_rbuf.raw_bytes[i]);
        //   Serial.print(", ");
        // }
        // Serial.println();
        // Serial.print("Oneshots -> ");
        // Serial.print(temp_rbuf.decoded.oneshots);
        // Serial.print("; Mode -> ");
        // Serial.print(temp_rbuf.decoded.mode);
        // Serial.println();
        if (temp_rbuf.decoded.oneshots & (1 << ONESHOT_SaveOthersReply)) {
          for (uint8_t other_suid = 1; other_suid <= 13; other_suid++) {
            if (temp_rbuf.decoded.suids & (1 << (other_suid - 1))) {
              Serial.print("Got Reply from SUID ");
              Serial.print(other_suid);
              //       Serial.print(" -> ");
              //       Serial.print(" x ");
              //       Serial.print(roster::db[other_suid - 1].x);
              //       Serial.print(", y ");
              //       Serial.print(roster::db[other_suid - 1].y);
              //       Serial.print(", yaw ");
              //       Serial.print(roster::db[other_suid - 1].yaw);
              Serial.println();
              break;  // There should be no Reply with multiple SUIDs.
            }
          }
        } else {
          Serial.print("For me? ");
          Serial.print((temp_rbuf.decoded.suids & (1 << (b_->cfg_.suid - 1)))
                           ? "True"
                           : "False");
          Serial.println();
          if (temp_rbuf.decoded.oneshots & (1 << ONESHOT_ReplyNext)) {
            for (uint8_t other_suid = 1; other_suid <= 13; other_suid++) {
              if (temp_rbuf.decoded.suids & (1 << (other_suid - 1))) {
                Serial.print("Poll to SUID ");
                Serial.print(other_suid);
                Serial.println();
                break;  // There should be no Reply with multiple SUIDs.
              }
            }
            //   } else if (temp_rbuf.decoded.mode ==
            //              static_cast<uint8_t>(M::DoPreset)) {
            //     Serial.println("This is a Preset Command");
            //     Serial.print("Preset indices for all Ahes -> ");
            //     for (uint8_t i = 0; i < 13; i++) {
            //       Serial.print(temp_rbuf.decoded.u.do_preset.idx[i]);
            //       Serial.print(", ");
            //     }
            //     Serial.println();
            //     Serial.print("My Preset index -> ");
            //     Serial.print(temp_rbuf.decoded.u.do_preset.idx[b_->cfg_.suid
            //     - 1]); Serial.println();
          }
        }
      }

      if (temp_rbuf.decoded.oneshots & (1 << ONESHOT_SaveOthersReply)) {
        // This is other's Reply. Parse and save to roster,
        // then immediately return.
        for (uint8_t other_suid = 1; other_suid <= 13; other_suid++) {
          if (other_suid == b_->cfg_.suid) continue;
          if (temp_rbuf.decoded.suids & (1 << (other_suid - 1))) {
            roster::db[other_suid - 1].x =
                temp_rbuf.decoded.u.save_others_reply.lpsx;
            roster::db[other_suid - 1].y =
                temp_rbuf.decoded.u.save_others_reply.lpsy;
            roster::db[other_suid - 1].yaw =
                temp_rbuf.decoded.u.save_others_reply.yaw;
            break;  // There should be no Reply with multiple SUIDs.
          }
        }

        Serial.print("FPD ");
        Serial.println(micros() - start_time_us);
        receiving_ = false;
        start = 0;
        return;
      }

      if (temp_rbuf.decoded.suids & (1 << (b_->cfg_.suid - 1))) {
        // This Command is for me.
        led_got_my_cmd = true;

        if (temp_rbuf.decoded.oneshots & (1 << ONESHOT_ReplyNext)) {
          // This Command is a poll. Reply will happen at start_time +
          // T_FROM_START_TO_SEND_US, independent to Executer.
          // No need to copy to Command to memory.
          waiting_xb_rpl_send_ = true;

          // Reply immediately if it's okay to send now.
          if (micros() >= start_time_us + T_FROM_START_TO_SEND_US) {
            Serial.print("RS_start ");
            Serial.println(micros() - start_time_us);

            XbeeReplySender::Send(b_->rpl_);
            waiting_xb_rpl_send_ = false;

            Serial.print("RS_done ");
            Serial.println(micros() - start_time_us);
          }

          Serial.print("FPD ");
          Serial.println(micros() - start_time_us);
          receiving_ = false;
          start = 0;
          return;
        }

        memcpy(xb_cmd_.raw_bytes, temp_rbuf.raw_bytes, XBEE_PACKET_LEN);
        waiting_parse_ = true;

        Serial.print("FPD ");
        Serial.println(micros() - start_time_us);
        receiving_ = false;
        start = 0;
        return;
      } else {
        // This is neither a Reply, nor a Command for me.
        Serial.print("FPD ");
        Serial.println(micros() - start_time_us);
        receiving_ = false;
        start = 0;
        return;
      }
    }

    //////////////////////////////////////////////////////////
    // if (micros() > start_time_us + 9000) {
    //   led_timeout_miss = true;
    //   // Not enough time if there are more Bailisks?
    //   Serial.println("Timeout. Back to waiting");
    //   while (XBEE_SERIAL.available() > 0) XBEE_SERIAL.read();
    //   receiving_ = false;
    //   start = 0;
    //   return;
    // }
    // while (XBEE_SERIAL.available() > 0 && buf_idx < XBEE_PACKET_LEN) {
    //   temp_rbuf.raw_bytes[buf_idx] = XBEE_SERIAL.read();
    //   buf_idx++;
    // }
    // if (buf_idx < XBEE_PACKET_LEN) return;
    // // Received full byte array within time limit since start bytes
    // reception. got_full_packet = true;
    // if (temp_rbuf.decoded.oneshots & (1 << ONESHOT_SaveOthersReply)) {
    //   // This is other's Reply. Parse and save to roster, then immediately
    //   // go back to waiting start bytes.
    //   for (uint8_t other_suid = 1; other_suid <= 13; other_suid++) {
    //     if (temp_rbuf.decoded.suids & (1 << (other_suid - 1))) {
    //       roster::db[other_suid - 1].x =
    //           temp_rbuf.decoded.u.save_others_reply.lpsx;
    //       roster::db[other_suid - 1].y =
    //           temp_rbuf.decoded.u.save_others_reply.lpsy;
    //       roster::db[other_suid - 1].yaw =
    //           temp_rbuf.decoded.u.save_others_reply.yaw;
    //       {
    //         Serial.print("Got Reply from SUID ");
    //         Serial.print(other_suid);
    //         Serial.print(" -> ");
    //         Serial.print(" x ");
    //         Serial.print(roster::db[other_suid - 1].x);
    //         Serial.print(", y ");
    //         Serial.print(roster::db[other_suid - 1].y);
    //         Serial.print(", yaw ");
    //         Serial.print(roster::db[other_suid - 1].yaw);
    //         Serial.println();
    //       }
    //       break;  // There should be no Reply with multiple SUIDs.
    //     }
    //   }
    //   while (XBEE_SERIAL.available() > 0) XBEE_SERIAL.read();
    //   receiving_ = false;
    //   start = 0;
    //   return;
    // }
    // if (temp_rbuf.decoded.suids & (1 << (b_->cfg_.suid - 1))) {
    //   // This Command is for me.
    //   if (temp_rbuf.decoded.oneshots & (1 << ONESHOT_ReplyNext)) {
    //     // This Command is a poll. Save time for synchronization.
    //     waiting_xb_rpl_send_ = true;
    //     // polled_time_us = micros();
    //   }
    //   memcpy(xb_cmd_.raw_bytes, temp_rbuf.raw_bytes, XBEE_PACKET_LEN);
    //   waiting_parse_ = true;
    //   led_got_my_cmd = true;
    // }
    // while (XBEE_SERIAL.available() > 0) XBEE_SERIAL.read();
    // receiving_ = false;
    // start = 0;
  }

  inline static void Parse() {
    Serial.println("Parse begin");

    static auto& x = xb_cmd_.decoded;
    static auto& c = b_->cmd_;
    static auto& m = c.mode;

    if (x.oneshots) {
      c.oneshots = x.oneshots;

      if (x.oneshots & (1 << ONESHOT_SetBaseYaw)) {
        c.set_base_yaw.offset = static_cast<double>(x.u.set_base_yaw.offset);
      }

      return;  // Oneshot- and Mode- Commands are NOT processed from
               // a single Command since there are Oneshots that
               // require additional values.
    }

    const auto maybe_new_mode = static_cast<M>(x.mode);
    if (maybe_new_mode == M::DoPreset &&
        x.u.do_preset.idx[b_->cfg_.suid - 1] == 0)
      return;  // Do not even switch Mode.  Previous DoPreset Command
               // execution's future-chaining can be happening now.

    m = maybe_new_mode;
    switch (m) {
      case M::DoPreset: {
        c.do_preset.idx = x.u.do_preset.idx[b_->cfg_.suid - 1];

        Serial.print("Copied to memory, Preset index ");
        Serial.print(c.do_preset.idx);
        Serial.println();
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
        Serial.print("XbeeCommandReceiver: Mode ");
        Serial.print(x.mode);
        Serial.print(" is NOT registered");
        Serial.println();
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
