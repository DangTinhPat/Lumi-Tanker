#pragma once

// Marks the per-tick functions of the balance loop. On the ESP32-S3 the firmware runs from flash
// behind a 16 KB instruction cache shared by both cores; the ~40 KB of control code executed every
// 2 ms refilled it on most ticks (a Balancing update took 1.8-1.98 ms against 0.4 ms for the same
// call with the balancer off). TAP_HOT_IRAM places them in internal RAM instead. Empty on the host.
#if defined(TAP_HOT_IRAM)
#include <esp_attr.h>
#define TAP_HOT IRAM_ATTR
#else
#define TAP_HOT
#endif
