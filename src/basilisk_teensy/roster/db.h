#pragma once

#ifndef NaN
#define NaN (0.0 / 0.0)
#endif

struct PosYaw {
  double x = NaN, y = NaN;
  double yaw = NaN;
};

namespace roster {
PosYaw db[13];
}
