/**
 * @file    mc_fusion.c
 * @brief   Sensor fusion implementation
 */

#include "mc_fusion.h"
#include "mc_math.h"
#include "mc_config.h"

void mc_fusion_init(mc_fusion_t *f)
{
    int i;
    for (i = 0; i < 2; i++)
    {
        f->wheel_spd[i] = 0.0f;
        f->wheel_pos[i] = 0.0f;
    }
    f->yaw_raw = 0.0f;
    f->yaw_multi = 0.0f;
    f->yaw_prev = 0.0f;
    f->yaw_revs = 0;
    f->forward_speed = 0.0f;
    f->turn_speed = 0.0f;
    f->position = 0.0f;
    f->line_raw = 0.0f;
    f->line_filtered = 0.0f;
}

void mc_fusion_update(mc_fusion_t *f, const mc_hal_t *hal, float ctrl_period_s)
{
    int32_t enc_left = 0, enc_right = 0;

    /* ---- Encoder ---- */
    if (hal->encoder_read)
    {
        hal->encoder_read(&enc_left, &enc_right);
    }

    /* Convert to RPM:
     *   spd = count / (gear * ppr) / period * 60
     *       = count / (gear * ppr) * freq * 60
     */
    float spd_factor = 1.0f / (float)(MC_GEAR_RATIO * MC_ENCODER_PPR) / ctrl_period_s * 60.0f;
    f->wheel_spd[0] = (float)enc_left  * spd_factor;   /* left  in RPM */
    f->wheel_spd[1] = (float)enc_right * spd_factor;   /* right in RPM */

    /* Accumulate position (in encoder counts) */
    f->wheel_pos[0] += (float)enc_left;
    f->wheel_pos[1] += (float)enc_right;

    /* ---- IMU multi-turn yaw ---- */
    if (hal->imu_get_yaw)
    {
        f->yaw_raw = hal->imu_get_yaw();

        /* Detect 0/360 boundary crossings */
        float error = f->yaw_raw - f->yaw_prev;
        if (error > 270.0f)
            f->yaw_revs -= 1;      /* crossed from 360->0: clockwise past zero */
        else if (error < -270.0f)
            f->yaw_revs += 1;      /* crossed from 0->360: counter-clockwise */

        f->yaw_multi = (float)f->yaw_revs * 360.0f + f->yaw_raw;
        f->yaw_prev = f->yaw_raw;
    }

    /* ---- Car-level kinematics ---- */
    /*
     * forward_speed = (R_spd + L_spd) * pi*D / 120   [m/s]
     * position      = (R_pos + L_pos) / (gear*ppr) * pi*D / 2   [m]
     * turn_speed    = (R_spd - L_spd) * pi*D / (60 * W)   [deg/s]
     */
    float pi_d = MC_PI * MC_WHEEL_DIAMETER;
    float inv_gear_ppr = 1.0f / (float)(MC_GEAR_RATIO * MC_ENCODER_PPR);

    f->forward_speed = (f->wheel_spd[1] + f->wheel_spd[0]) * pi_d / 120.0f;

    float pos_accum = (f->wheel_pos[1] + f->wheel_pos[0]) * inv_gear_ppr;
    f->position = pos_accum * pi_d / 2.0f;

    f->turn_speed = (f->wheel_spd[1] - f->wheel_spd[0]) * pi_d / (60.0f * MC_WHEEL_BASE);

    /* ---- Line sensor with low-pass filter ---- */
    if (hal->line_sensor_read)
    {
        f->line_raw = hal->line_sensor_read();
        f->line_filtered = MC_LINE_LPF_ALPHA * f->line_raw
                         + (1.0f - MC_LINE_LPF_ALPHA) * f->line_filtered;
    }
}
