/**
 * @file    mc_config.h
 * @brief   Compile-time configuration defaults for motion control library
 *
 * Override any value by defining it before including this header,
 * or by passing -DMC_WHEEL_DIAMETER=0.050f in compiler flags.
 */

#ifndef __MC_CONFIG_H
#define __MC_CONFIG_H

/* ========== Physical Constants ========== */
#ifndef MC_WHEEL_DIAMETER
#define MC_WHEEL_DIAMETER           0.064f      /* wheel diameter (m), Wheel_Radius=0.032 */
#endif

#ifndef MC_WHEEL_BASE
#define MC_WHEEL_BASE               0.1657f     /* track width (m) */
#endif

#ifndef MC_GEAR_RATIO
#define MC_GEAR_RATIO               30          /* motor gearbox ratio */
#endif

#ifndef MC_ENCODER_PPR
#define MC_ENCODER_PPR              2000        /* encoder pulses per revolution */
#endif

/* ========== Control Frequencies ========== */
#ifndef MC_CONTROL_FREQ_HZ
#define MC_CONTROL_FREQ_HZ          200         /* main control loop (Hz), TIM1=200Hz */
#endif

#ifndef MC_CONTROL_PERIOD_S
#define MC_CONTROL_PERIOD_S         (1.0f / MC_CONTROL_FREQ_HZ)   /* control period (seconds) */
#endif

#ifndef MC_PID_WHEEL_FREQ_HZ
#define MC_PID_WHEEL_FREQ_HZ       200         /* wheel PID rate (Hz) */
#endif

#ifndef MC_PID_YAW_FREQ_HZ
#define MC_PID_YAW_FREQ_HZ         200         /* yaw PID rate (Hz) */
#endif

#ifndef MC_PID_LINE_FREQ_HZ
#define MC_PID_LINE_FREQ_HZ        200         /* line-follow PID rate (Hz) */
#endif

/* ========== PWM ========== */
#ifndef MC_PWM_MAX
#define MC_PWM_MAX                  3599        /* max PWM duty (TIM3 Period) */
#endif

/* ========== Line Sensor ========== */
#ifndef MC_LINE_SENSOR_CHANNELS
#define MC_LINE_SENSOR_CHANNELS     8           /* number of line sensor channels */
#endif

#ifndef MC_LINE_LPF_ALPHA
#define MC_LINE_LPF_ALPHA           0.08f       /* line sensor low-pass filter alpha */
#endif

/* ========== Path Executor ========== */
#ifndef MC_YAW_BLEND_MS
#define MC_YAW_BLEND_MS             50          /* yaw transition blend window (ms) */
#endif

#ifndef MC_YAW_DEADBAND_DEG
#define MC_YAW_DEADBAND_DEG         1.0f        /* spin completion deadband (degrees) */
#endif

/* ========== PID Defaults ========== */
#ifndef MC_PID_WHEEL_KP
#define MC_PID_WHEEL_KP             20.0f
#endif
#ifndef MC_PID_WHEEL_KI
#define MC_PID_WHEEL_KI             10.0f
#endif
#ifndef MC_PID_WHEEL_KD
#define MC_PID_WHEEL_KD             0.0f
#endif

#ifndef MC_PID_YAW_KP
#define MC_PID_YAW_KP               1.0f
#endif
#ifndef MC_PID_YAW_KI
#define MC_PID_YAW_KI               0.0f
#endif
#ifndef MC_PID_YAW_KD
#define MC_PID_YAW_KD               0.0f
#endif

#ifndef MC_PID_LINE_KP
#define MC_PID_LINE_KP              0.0f
#endif
#ifndef MC_PID_LINE_KI
#define MC_PID_LINE_KI              -0.158f
#endif
#ifndef MC_PID_LINE_KD
#define MC_PID_LINE_KD              0.0f
#endif

/* ========== Default Speed ========== */
#ifndef MC_DEFAULT_SPEED
#define MC_DEFAULT_SPEED            0.057f      /* default forward speed (m/s) */
#endif

/* ========== Spline ========== */
#ifndef MC_SPLINE_MAX_PTS
#define MC_SPLINE_MAX_PTS           8           /* max waypoints per spline segment */
#endif

#endif /* __MC_CONFIG_H */
