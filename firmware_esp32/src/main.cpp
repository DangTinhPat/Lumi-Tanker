#include "tap_hw/FirmwareApp.hpp"

namespace {
tap_hw::FirmwareApp firmware;
}

void setup() {
  (void)firmware.begin();
}

void loop() {
  firmware.loop();
}
