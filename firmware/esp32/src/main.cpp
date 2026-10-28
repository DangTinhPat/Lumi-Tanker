#include "tap_hw/AppV3.hpp"

namespace {
tap_hw::AppV3 firmware;
}

void setup() {
  (void)firmware.begin();
}

void loop() {
  firmware.loop();
}
