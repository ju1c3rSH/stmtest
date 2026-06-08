/**
 * @file    mc_fusion.h
 * @brief   Sensor fusion: encoder, IMU, line sensor
 */

#ifndef __MC_FUSION_H
#define __MC_FUSION_H

#include <stdint.h>
#include "mc_platform.h"

/**
 * @brief Fused sensor state
 */
typedef struct
{
    /* Wheel state */
    float wheel_spd[2];        /* [0]=left, [1]=right, RPM */
    float wheel_pos[2];        /* [0]=left, [1]=right, accumulated counts */

    /* IMU state */
    float yaw_raw;             /* single-turn yaw from IMU (degrees) */
    float yaw_multi;           /* multi-turn cumulative yaw (degrees) */
    float yaw_prev;            /* previous yaw_raw for wrap detection */
    int32_t yaw_revs;          /* revolution counter */

    /* Car-level derived values */
    float forward_speed;       /* m/s */
    float turn_speed;          /* deg/s */
    float position;            /* cumulative distance (meters) */

    /* Line sensor */
    float line_raw;            /* raw reading */
    float line_filtered;       /* low-pass filtered */

} mc_fusion_t;

/**
 * @brief Initialize fusion state (zeros everything).
 */
void mc_fusion_init(mc_fusion_t *f);

/**
 * @brief Update all sensor readings and compute derived values.
 *
 * Call once per control loop tick (e.g. 1 kHz).
 *
 * @param f              Fusion state
 * @param hal            Platform HAL callbacks
 * @param ctrl_period_s  Control loop period in seconds (e.g. 0.001f)
 */
void mc_fusion_update(mc_fusion_t *f, const mc_hal_t *hal, float ctrl_period_s);

#endif /* __MC_FUSION_H */
