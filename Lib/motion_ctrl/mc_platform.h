/**
 * @file    mc_platform.h
 * @brief   Platform abstraction layer for motion control library
 *
 * Users register function pointers for hardware access. The library
 * never calls HAL/Arduino/ESP-IDF directly.
 */

#ifndef __MC_PLATFORM_H
#define __MC_PLATFORM_H

#include <stdint.h>

/**
 * @brief Hardware abstraction callbacks
 *
 * All fields must be non-NULL except debug_log (optional).
 * Register via mc_hal_register() before calling mc_init().
 */
typedef struct
{
    /**
     * Set motor PWM duty cycle.
     * @param left_pwm   Left motor duty in [-1.0, 1.0]. Positive = forward.
     * @param right_pwm  Right motor duty in [-1.0, 1.0]. Positive = forward.
     */
    void (*motor_set)(float left_pwm, float right_pwm);

    /**
     * Read encoder ticks since last call, then reset counters.
     * @param left   Output: left encoder delta (positive = forward)
     * @param right  Output: right encoder delta (positive = forward)
     */
    void (*encoder_read)(int32_t *left, int32_t *right);

    /**
     * Get current yaw angle from IMU.
     * @return Yaw in degrees, multi-turn cumulative (not wrapped to 360).
     */
    float (*imu_get_yaw)(void);

    /**
     * Read line sensor weighted position.
     * @return Position value. 0 = line centered. Positive/negative = offset.
     */
    float (*line_sensor_read)(void);

    /**
     * Get system tick in milliseconds.
     * @return Millisecond timestamp.
     */
    uint32_t (*get_tick_ms)(void);

    /**
     * Optional debug logging. May be NULL.
     */
    void (*debug_log)(const char *fmt, ...);

} mc_hal_t;

/**
 * @brief Register hardware abstraction callbacks.
 * @param hal  Pointer to filled mc_hal_t struct. The struct is copied internally.
 *             Call before mc_init().
 */
void mc_hal_register(const mc_hal_t *hal);

/**
 * @brief Get a pointer to the currently registered HAL.
 * @return Pointer to the internal HAL copy, or NULL if not registered.
 */
const mc_hal_t *mc_hal_get(void);

#endif /* __MC_PLATFORM_H */
