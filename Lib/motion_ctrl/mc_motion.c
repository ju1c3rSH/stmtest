/**
 * @file    mc_motion.c
 * @brief   Top-level motion control orchestrator
 */

#include "mc_motion.h"
#include "mc_math.h"

void mc_init(mc_ctx_t *ctx)
{
    /* Physical defaults */
    ctx->wheel_diameter = MC_WHEEL_DIAMETER;
    ctx->wheel_base = MC_WHEEL_BASE;
    ctx->control_freq = MC_CONTROL_FREQ_HZ;
    ctx->running = 0;

    /* Fusion */
    mc_fusion_init(&ctx->fusion);

    /* Path executor */
    mc_path_init(&ctx->path);

    /* PID: wheel left */
    mc_pid_init(&ctx->pid_wheel_l,
                MC_PID_WHEEL_KP, MC_PID_WHEEL_KI, MC_PID_WHEEL_KD,
                1.0f / MC_PID_WHEEL_FREQ_HZ,
                -(float)MC_PWM_MAX, (float)MC_PWM_MAX);

    /* PID: wheel right */
    mc_pid_init(&ctx->pid_wheel_r,
                MC_PID_WHEEL_KP, MC_PID_WHEEL_KI, MC_PID_WHEEL_KD,
                1.0f / MC_PID_WHEEL_FREQ_HZ,
                -(float)MC_PWM_MAX, (float)MC_PWM_MAX);

    /* PID: yaw */
    mc_pid_init(&ctx->pid_yaw,
                MC_PID_YAW_KP, MC_PID_YAW_KI, MC_PID_YAW_KD,
                1.0f / MC_PID_YAW_FREQ_HZ,
                -15.0f, 15.0f);

    /* PID: line follow */
    mc_pid_init(&ctx->pid_line,
                MC_PID_LINE_KP, MC_PID_LINE_KI, MC_PID_LINE_KD,
                1.0f / MC_PID_LINE_FREQ_HZ,
                -20.0f, 20.0f);
}

void mc_set_params(mc_ctx_t *ctx, float wheel_d, float wheel_base, uint16_t freq)
{
    ctx->wheel_diameter = wheel_d;
    ctx->wheel_base = wheel_base;
    ctx->control_freq = freq;
}

void mc_start_path(mc_ctx_t *ctx, const mc_seg_t *path)
{
    mc_path_start(&ctx->path, path);
    mc_pid_reset(&ctx->pid_line);
    mc_pid_reset(&ctx->pid_yaw);
    ctx->running = 1;
}

void mc_stop(mc_ctx_t *ctx)
{
    mc_path_stop(&ctx->path);
    ctx->running = 0;

    /* Zero motors */
    const mc_hal_t *hal = mc_hal_get();
    if (hal && hal->motor_set)
        hal->motor_set(0.0f, 0.0f);
}

uint8_t mc_is_done(const mc_ctx_t *ctx)
{
    return (ctx->path.state == MC_PATH_DONE) ? 1 : 0;
}

mc_path_state_t mc_get_state(const mc_ctx_t *ctx)
{
    return mc_path_get_state(&ctx->path);
}

uint16_t mc_get_seg_index(const mc_ctx_t *ctx)
{
    return mc_path_get_seg_index(&ctx->path);
}

const mc_fusion_t *mc_get_fusion(const mc_ctx_t *ctx)
{
    return &ctx->fusion;
}

mc_pid_t *mc_get_pid(mc_ctx_t *ctx, uint8_t id)
{
    switch (id)
    {
        case 0: return &ctx->pid_wheel_l;
        case 1: return &ctx->pid_wheel_r;
        case 2: return &ctx->pid_yaw;
        case 3: return &ctx->pid_line;
        default: return (void *)0;
    }
}

/* ============================================================ */

/**
 * @brief Period division counter for sub-1kHz PIDs
 */
static uint32_t s_tick_cnt = 0;

void mc_tick(mc_ctx_t *ctx)
{
    const mc_hal_t *hal = mc_hal_get();
    if (!hal) return;

    float ctrl_period = 1.0f / (float)ctx->control_freq;

    /* ---- 1. Sensor fusion ---- */
    mc_fusion_update(&ctx->fusion, hal, ctrl_period);

    /* ---- 2. Path executor ---- */
    mc_path_output_t path_out;
    mc_path_tick(&ctx->path, ctx->fusion.yaw_multi, ctx->fusion.position,
                 ctx->wheel_diameter, &path_out);

    /* If path just finished, stop motors */
    if (ctx->path.state == MC_PATH_DONE)
    {
        if (hal->motor_set)
            hal->motor_set(0.0f, 0.0f);
        ctx->running = 0;
        return;
    }

    /* If idle, do nothing */
    if (ctx->path.state == MC_PATH_IDLE)
        return;

    /* ---- 3. Line-follow PID (500 Hz) ---- */
    float yaw_set = path_out.yaw_set;

    if (path_out.line_pid_en)
    {
        /* Decimation: run at MC_PID_LINE_FREQ_HZ */
        uint32_t line_period = ctx->control_freq / MC_PID_LINE_FREQ_HZ;
        if (line_period < 1) line_period = 1;
        if ((s_tick_cnt % line_period) == 0)
        {
            mc_pid_parallel(&ctx->pid_line, 0.0f, ctx->fusion.line_filtered);
        }
        yaw_set += ctx->pid_line.out_value;
    }

    /* ---- 4. Yaw PID (500 Hz) ---- */
    {
        uint32_t yaw_period = ctx->control_freq / MC_PID_YAW_FREQ_HZ;
        if (yaw_period < 1) yaw_period = 1;
        if ((s_tick_cnt % yaw_period) == 0)
        {
            mc_pid_parallel(&ctx->pid_yaw, yaw_set, ctx->fusion.yaw_multi);
        }
    }

    /* ---- 5. Differential steering ---- */
    float spd_rpm = path_out.speed_rpm;
    float yaw_corr = ctx->pid_yaw.out_value;
    float r_set = spd_rpm + yaw_corr;
    float l_set = spd_rpm - yaw_corr;

    /* ---- 6. Wheel PID (1 kHz) ---- */
    mc_pid_parallel(&ctx->pid_wheel_r, r_set, ctx->fusion.wheel_spd[1]);
    mc_pid_parallel(&ctx->pid_wheel_l, l_set, ctx->fusion.wheel_spd[0]);

    /* ---- 7. Motor output ---- */
    if (hal->motor_set)
    {
        /* Normalize PWM: [-PWM_MAX, PWM_MAX] -> [-1.0, 1.0] */
        float inv_max = 1.0f / (float)MC_PWM_MAX;
        float l_duty = ctx->pid_wheel_l.out_value * inv_max;
        float r_duty = ctx->pid_wheel_r.out_value * inv_max;

        /* Clamp to [-1, 1] */
        l_duty = mc_clamp(l_duty, -1.0f, 1.0f);
        r_duty = mc_clamp(r_duty, -1.0f, 1.0f);

        hal->motor_set(l_duty, r_duty);
    }

    s_tick_cnt++;
}
