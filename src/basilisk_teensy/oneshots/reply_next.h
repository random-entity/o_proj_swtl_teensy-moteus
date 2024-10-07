#pragma once

#include "../cmd_rcvrs/xbee_cr.h"
#include "../rpl_sndrs/xbee_rs.h"
#include "meta.h"

void BasiliskOneshots::ReplyNext(Basilisk* b) {
  if (XbeeCommandReceiver::waiting_xb_rpl_send_) {
    if (XbeeCommandReceiver::polled_time_us + 15000 < micros()) {
      if (micros() <= XbeeCommandReceiver::polled_time_us + 25000) {
        XbeeReplySender::Send(b->rpl_);
      }
      XbeeCommandReceiver::waiting_xb_rpl_send_ = false;
      b->cmd_.oneshots &= ~(1 << ONESHOT_ReplyNext);
    }
  }
}
