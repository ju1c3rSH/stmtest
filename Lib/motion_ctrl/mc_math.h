/**
 * @file    mc_math.h
 * @brief   Math utilities and constants for motion control library
 */

#ifndef __MC_MATH_H
#define __MC_MATH_H

#include <math.h>
#include <stdint.h>

/* ========== Constants ========== */
#ifndef MC_PI
#define MC_PI (3.14159265359f)
#endif
#ifndef MC_2PI
#define MC_2PI (6.28318530718f)
#endif
#define MC_RAD2DEG (57.2957795131f)
#define MC_DEG2RAD (0.01745329252f)

/* ========== Basic Math ========== */
static inline float mc_sq(float x) { return x * x; }
static inline float mc_abs(float x) { return x < 0.0f ? -x : x; }
static inline int mc_absi(int x) { return x < 0 ? -x : x; }
static inline float mc_sign(float x) { return x < 0.0f ? -1.0f : 1.0f; }
static inline float mc_min(float a, float b) { return a < b ? a : b; }
static inline float mc_max(float a, float b) { return a > b ? a : b; }

static inline float mc_clamp(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

static inline int mc_clampi(int x, int lo, int hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

/* ========== Angle Wrapping ========== */

/** Wrap angle to [-180, 180] degrees */
static inline float mc_wrap_pm180(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

/** Wrap angle to [0, 360] degrees */
static inline float mc_wrap_0_360(float deg)
{
    while (deg >= 360.0f)
        deg -= 360.0f;
    while (deg < 0.0f)
        deg += 360.0f;
    return deg;
}

/** Wrap angle to [-pi, pi] radians */
static inline float mc_wrap_pm_pi(float rad)
{
    while (rad > MC_PI)
        rad -= MC_2PI;
    while (rad < -MC_PI)
        rad += MC_2PI;
    return rad;
}

/* ========== Unit Conversions ========== */

/** RPM to m/s given wheel diameter (m) */
static inline float mc_rpm_to_ms(float rpm, float wheel_d)
{
    return rpm * wheel_d * MC_PI / 60.0f;
}

/** m/s to RPM given wheel diameter (m) */
static inline float mc_ms_to_rpm(float ms, float wheel_d)
{
    return ms * 60.0f / (wheel_d * MC_PI);
}

/** Encoder count delta to RPM */
static inline float mc_count_to_rpm(int32_t count, int32_t gear_ratio,
                                    int32_t ppr, float ctrl_freq)
{
    return (float)count / (float)(gear_ratio * ppr) * ctrl_freq * 60.0f;
}

/** Accumulated encoder counts to position in meters */
static inline float mc_count_to_meters(float accum_counts, float wheel_d,
                                       int32_t gear_ratio, int32_t ppr)
{
    return accum_counts / (float)(gear_ratio * ppr) * wheel_d * MC_PI;
}

/* ========== Safety ========== */

static inline uint8_t mc_is_nan(float x) { return x != x; }
static inline uint8_t mc_is_inf(float x)
{
    union
    {
        float f;
        uint32_t u;
    } fu = {.f = x};
    fu.u &= 0x7FFFFFFFu;
    return fu.u == 0x7F800000u;
}
static inline float mc_nan_to_zero(float x) { return mc_is_nan(x) ? 0.0f : x; }

#endif /* __MC_MATH_H */
