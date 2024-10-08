#include "cmd_rcvrs/neokey_cr.h"
#include "cmd_rcvrs/xbee_cr.h"
#include "components/specifics/neokey1x4_i2c0.h"
#include "components/specifics/neokey3x4_i2c0.h"
#include "executer.h"
#include "helpers/imports.h"
#include "helpers/utils.h"
#include "rpl_sndrs/led_rs.h"
#include "rpl_sndrs/serial_rs.h"
#include "servo_units/basilisk.h"

// Basilisk configuration.
Basilisk::Configuration cfg{
    .suid =
        [] {
          uint8_t suid = 0;
          const auto teensyid = GetTeensyId();
          if (teensyid_to_suid.find(teensyid) != teensyid_to_suid.end()) {
            suid = teensyid_to_suid.at(teensyid);
          }
          return suid;
        }(),  //
    .servo{.id_l = 1, .id_r = 2, .bus = 1},
    .lps{.c = 860.0,
         .x_c = 430.0,
         .y_c = 910.0,
         .minx = 100.0,
         .maxx = 760.0,
         .miny = 100.0,
         .maxy = 810.0},
    .lego{.pin_l = 23, .pin_r = 29, .run_interval = 20},  //
    .mags{.pin_la = 3,
          .pin_lt = 4,
          .pin_ra = 5,
          .pin_rt = 6,
          .run_interval = 100}};

// Basilisk and its executer.
Basilisk b{cfg};
Executer exec{&b};

// CommandReceivers.
XbeeCommandReceiver xb_cr;
Neokey nk = specifics::neokey1x4_i2c0;
NeokeyCommandReceiver nk_cr{nk};

// ReplySenders.
XbeeReplySender xb_rs;

void setup() {
  Serial.begin(9600);
  delay(250);

  Serial.println("******************************************");
  Serial.print("Basilisk SUID set to ");
  Serial.println(b.cfg_.suid);
  delay(250);

  if (!b.Setup()) {
    nk.setPixelColor(0, 0xF00000);
    while (1);
  }
  xb_cr.Setup(&b);
  nk_cr.Setup(&b);
  xb_rs.Setup(&b);
  delay(250);

  Serial.println("setup() done");
  Serial.println("******************************************");
}

void loop() {
  b.Run();

  xb_cr.Run();
  xb_rs.Run();

  static Beat nk_cr_beat{10};
  if (nk_cr_beat.Hit()) nk_cr.Run();

  static Beat exec_beat{10};
  if (exec_beat.Hit()) exec.Run();

  static Beat led_rs_beat{1};
  if (led_rs_beat.Hit()) LedReplySender(nk);

  // static Beat serial_rs_beat{2000};
  // if (serial_rs_beat.Hit()) SerialReplySender(b);
}
