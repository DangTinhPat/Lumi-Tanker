#if defined(TAP_FW_V3)
#include "tap_hw/AppV3.hpp"
#else
#include "tap_hw/FirmwareApp.hpp"
#endif

namespace {
#if defined(TAP_FW_V3)
tap_hw::AppV3 firmware;
#else
tap_hw::FirmwareApp firmware;
#endif
}

void setup() {
  (void)firmware.begin();
}

void loop() {
  firmware.loop();
}
