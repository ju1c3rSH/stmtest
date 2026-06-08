/**
 * @file    mc_pid.h
 * @brief   PID controller implementations (parallel, serial, PDFF)
 */

#ifndef __MC_PID_H
#define __MC_PID_H

#include <stdint.h>

/**
 * @brief PID controller state
 */
typedef struct
{
    /* Gains */
    float kp;
    float ki;
    float kd;

    /* Feed-forward gains (used by pdff only) */
    float kfp;          /* reference weight [0..1] */
    float kf_damp;      /* feedback damping factor */

    /* Timing */
    float ts;           /* sample period (seconds) */

    /* Integrator state */
    float i_term;
    float i_term_max;
    float i_term_min;
    uint8_t i_isolate;  /* 1 = freeze integrator to zero */

    /* Output limits */
    float out_min;
    float out_max;

    /* Internal state */
    float error;
    float pre_err;
    float p_term;
    float d_term;
    float out_value;

} mc_pid_t;

/**
 * @brief Initialize PID controller with gains and limits.
 */
void mc_pid_init(mc_pid_t *pid, float kp, float ki, float kd,
                 float ts, float out_min, float out_max);

/**
 * @brief Reset PID state (integrator, error history, output).
 */
void mc_pid_reset(mc_pid_t *pid);

/**
 * @brief Update PID gains at runtime.
 */
void mc_pid_set_gains(mc_pid_t *pid, float kp, float ki, float kd);

/**
 * @brief Parallel (ideal) PID: output = Kp*e + Ki*integral(e) + Kd*de
 * @param pid      PID state
 * @param ref      Reference (setpoint)
 * @param feedback Measured value
 * @return Control output (clamped to [out_min, out_max])
 */
float mc_pid_parallel(mc_pid_t *pid, float ref, float feedback);

/**
 * @brief Serial (interacting) PI: output = Kp*e + Ki*integral(Kp*e)
 *
 * Effective integral gain is Kp*Ki. No D-term.
 *
 * @param pid      PID state
 * @param ref      Reference (setpoint)
 * @param feedback Measured value
 * @return Control output (clamped to [out_min, out_max])
 */
float mc_pid_serial(mc_pid_t *pid, float ref, float feedback);

/**
 * @brief PD with Feed-Forward (PDFF):
 *        P-term uses weighted error: Kp*(ref*kfp - feedback*(1+kf_damp))
 *        I-term and D-term use normal error.
 *
 * @param pid      PID state (kfp and kf_damp must be set)
 * @param ref      Reference (setpoint)
 * @param feedback Measured value
 * @return Control output (clamped to [out_min, out_max])
 */
float mc_pid_pdff(mc_pid_t *pid, float ref, float feedback);

#endif /* __MC_PID_H */
