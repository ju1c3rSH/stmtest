/**
 * @file    mc_motion.h
 * @brief   Top-level motion control API
 *
 * This is the main entry point. It ties together:
 *   - Sensor fusion (encoder, IMU, line sensor)
 *   - Path executor (declarative path segments)
 *   - PID cascade (line -> yaw -> wheel)
 *   - Motor output
 *
 * Usage:
 *   mc_ctx_t mc;
 *   mc_hal_register(&my_hal);
 *   mc_init(&mc);
 *   mc_start_path(&mc, my_path);   // from button press, etc.
 *
 *   // In 1kHz ISR:
 *   mc_tick(&mc);
 */

#ifndef __MC_MOTION_H
#define __MC_MOTION_H

#include "mc_platform.h"
#include "mc_pid.h"
#include "mc_fusion.h"
#include "mc_path.h"
#include "mc_config.h"

/**
 * @brief Motion control context (all state in one struct)
 */
typedef struct
{
    /* Sensor fusion */
    mc_fusion_t fusion;

    /* Path executor */
    mc_path_ctx_t path;

    /* PID controllers */
    mc_pid_t pid_wheel_l;       /* left wheel speed PI */
    mc_pid_t pid_wheel_r;       /* right wheel speed PI */
    mc_pid_t pid_yaw;           /* yaw angle P(D) */
    mc_pid_t pid_line;          /* line-follow PI */

    /* Runtime configuration */
    float wheel_diameter;       /* meters */
    float wheel_base;           /* meters */
    uint16_t control_freq;      /* Hz */

    /* Status */
    uint8_t running;            /* 1 = path is active */

} mc_ctx_t;

/**
 * @brief Initialize motion control context with default PID parameters.
 *
 * Call mc_hal_register() before this.
 */
void mc_init(mc_ctx_t *ctx);

/**
 * @brief Reconfigure physical parameters at runtime.
 */
void mc_set_params(mc_ctx_t *ctx, float wheel_d, float wheel_base, uint16_t freq);

/**
 * @brief Main control loop tick. Call at the control frequency (e.g. 1 kHz).
 *
 * This function:
 *   1. Updates sensor fusion
 *   2. Ticks the path executor
 *   3. Runs line-follow PID if enabled
 *   4. Runs yaw PID
 *   5. Computes differential steering
 *   6. Runs wheel PID (left/right)
 *   7. Outputs motor PWM
 */
void mc_tick(mc_ctx_t *ctx);

/**
 * @brief Start executing a path.
 * @param ctx   Motion control context
 * @param path  Const path array (must end with MC_SEG_END)
 */
void mc_start_path(mc_ctx_t *ctx, const mc_seg_t *path);

/**
 * @brief Stop all motion (motors to zero, path to IDLE).
 */
void mc_stop(mc_ctx_t *ctx);

/**
 * @brief Check if path execution is complete.
 * @return 1 if done, 0 otherwise
 */
uint8_t mc_is_done(const mc_ctx_t *ctx);

/**
 * @brief Get current path state.
 */
mc_path_state_t mc_get_state(const mc_ctx_t *ctx);

/**
 * @brief Get current segment index.
 */
uint16_t mc_get_seg_index(const mc_ctx_t *ctx);

/**
 * @brief Get sensor fusion data (read-only access).
 */
const mc_fusion_t *mc_get_fusion(const mc_ctx_t *ctx);

/**
 * @brief Get PID controller for manual tuning (e.g. from OLED/buttons).
 * @param ctx
 * @param id  0=wheel_L, 1=wheel_R, 2=yaw, 3=line
 * @return Pointer to PID state, or NULL if id invalid
 */
mc_pid_t *mc_get_pid(mc_ctx_t *ctx, uint8_t id);

#endif /* __MC_MOTION_H */
