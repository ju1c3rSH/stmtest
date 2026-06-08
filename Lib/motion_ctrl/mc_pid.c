/**
 * @file    mc_pid.c
 * @brief   PID controller implementations
 */

#include "mc_pid.h"
#include "mc_math.h"

void mc_pid_init(mc_pid_t *pid, float kp, float ki, float kd,
                 float ts, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kfp = 1.0f;
    pid->kf_damp = 0.0f;
    pid->ts = ts;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->i_term_max = out_max;
    pid->i_term_min = out_min;
    pid->i_isolate = 0;
    pid->i_term = 0.0f;
    pid->error = 0.0f;
    pid->pre_err = 0.0f;
    pid->p_term = 0.0f;
    pid->d_term = 0.0f;
    pid->out_value = 0.0f;
}

void mc_pid_reset(mc_pid_t *pid)
{
    pid->i_term = 0.0f;
    pid->error = 0.0f;
    pid->pre_err = 0.0f;
    pid->p_term = 0.0f;
    pid->d_term = 0.0f;
    pid->out_value = 0.0f;
}

void mc_pid_set_gains(mc_pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/* ------------------------------------------------------------------ */

float mc_pid_parallel(mc_pid_t *pid, float ref, float feedback)
{
    pid->error = ref - feedback;

    /* P-term */
    pid->p_term = pid->kp * pid->error;

    /* D-term (derivative on error) */
    pid->d_term = pid->kd * (pid->error - pid->pre_err);
    pid->pre_err = pid->error;

    /* Dynamic anti-windup: I-term limited to remaining headroom */
    float i_max = mc_max(pid->out_max - pid->p_term, 0.0f);
    float i_min = mc_min(pid->out_min - pid->p_term, 0.0f);

    if (pid->i_isolate)
    {
        pid->i_term = 0.0f;
    }
    else
    {
        pid->i_term += pid->ki * pid->error * pid->ts;
        pid->i_term = mc_clamp(pid->i_term, i_min, i_max);
    }

    /* Output = P + I + D */
    pid->out_value = pid->p_term + pid->i_term + pid->d_term;
    pid->out_value = mc_clamp(pid->out_value, pid->out_min, pid->out_max);

    return pid->out_value;
}

/* ------------------------------------------------------------------ */

float mc_pid_serial(mc_pid_t *pid, float ref, float feedback)
{
    pid->error = ref - feedback;

    /* P-term */
    pid->p_term = pid->kp * pid->error;

    /* I-term (acts on P-term, not raw error -- serial form) */
    if (pid->i_isolate)
    {
        pid->i_term = 0.0f;
    }
    else
    {
        pid->i_term += pid->ki * pid->p_term * pid->ts;
        pid->i_term = mc_clamp(pid->i_term, pid->i_term_min, pid->i_term_max);
    }

    /* Output = P + I (no D-term) */
    pid->out_value = pid->p_term + pid->i_term;
    pid->out_value = mc_clamp(pid->out_value, pid->out_min, pid->out_max);

    return pid->out_value;
}

/* ------------------------------------------------------------------ */

float mc_pid_pdff(mc_pid_t *pid, float ref, float feedback)
{
    /* Feed-forward weighted error for P-term */
    float ref_temp = ref * pid->kfp;
    float fdb_temp = feedback * (1.0f + pid->kf_damp);
    float err_kf = ref_temp - fdb_temp;

    /* Normal error for I and D terms */
    pid->error = ref - feedback;

    /* P-term (on feed-forward error) */
    pid->p_term = pid->kp * err_kf;

    /* Dynamic anti-windup */
    float i_max = mc_max(pid->out_max - pid->p_term, 0.0f);
    float i_min = mc_min(pid->out_min - pid->p_term, 0.0f);

    /* I-term (on normal error) */
    if (pid->i_isolate)
    {
        pid->i_term = 0.0f;
    }
    else
    {
        pid->i_term += pid->ki * pid->error * pid->ts;
        pid->i_term = mc_clamp(pid->i_term, i_min, i_max);
    }

    /* D-term (on normal error) */
    pid->d_term = pid->kd * (pid->error - pid->pre_err);
    pid->pre_err = pid->error;

    /* Output = P + I + D */
    pid->out_value = pid->p_term + pid->i_term + pid->d_term;
    pid->out_value = mc_clamp(pid->out_value, pid->out_min, pid->out_max);

    return pid->out_value;
}
