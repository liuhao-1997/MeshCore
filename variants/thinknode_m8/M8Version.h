#pragma once

// Frozen until the user accepts this version; increment only for the next iteration.
#define M8_BASE_FIRMWARE_VERSION "v1.17.5"

#ifndef FIRMWARE_APP_VERSION
#define FIRMWARE_APP_VERSION M8_BASE_FIRMWARE_VERSION "_elecrow"
#endif

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION FIRMWARE_APP_VERSION
#endif

#ifdef __cplusplus
static_assert(sizeof(FIRMWARE_APP_VERSION) <= 20, "M8 App version exceeds its 20-byte field");
#endif
