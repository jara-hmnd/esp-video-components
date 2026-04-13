/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_cam_sensor_types.h"

/** SCCB 7-bit address (VD56G3 family) */
#define VD66GY_SCCB_ADDR 0x10
#define VD66GY_PID       0x5603
#define VD66GY_SENSOR_NAME "VD66GY"

esp_cam_sensor_device_t *vd66gy_detect(esp_cam_sensor_config_t *config);

#ifdef __cplusplus
}
#endif
