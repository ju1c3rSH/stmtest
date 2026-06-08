#include "bsp_motion_hal.h"
#include "bsp_motor.h"
#include "inv_mpu.h"
#include "text_utils.h"

/* ------------------------------------------------------------------ */
/*  Motor: map [-1.0, 1.0] → [-MC_PWM_MAX, MC_PWM_MAX] for BSP       */
/* ------------------------------------------------------------------ */
static void motor_set(float left, float right)
{
    int16_t l = (int16_t)(left  * 3599.0f);
    int16_t r = (int16_t)(right * 3599.0f);
    Set_Motor_A_Speed(l);
    Set_Motor_B_Speed(r);
}

/* ------------------------------------------------------------------ */
/*  Encoder: read delta and reset counter                              */
/*  NOTE: right encoder is inverted to match forward-positive conv     */
/* ------------------------------------------------------------------ */
static void encoder_read(int32_t *left, int32_t *right)
{
    *left  = (int16_t)Encoder_Get_A();
    *right = (int16_t)(-(int16_t)Encoder_Get_B());
}

/* ------------------------------------------------------------------ */
/*  IMU yaw: DMP quaternion → degrees, single-turn [-180, 180]        */
/*  The library's mc_fusion handles multi-turn unwrapping.             */
/* ------------------------------------------------------------------ */
static float imu_get_yaw(void)
{
    float pitch, roll, yaw;
    if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0)
        return yaw;
    return 0.0f;
}

/* ------------------------------------------------------------------ */
/*  Line sensor: not installed → return 0 (centered / no line)        */
/* ------------------------------------------------------------------ */
static float line_sensor_read(void)
{
    return 0.0f;
}

/* ------------------------------------------------------------------ */
/*  System tick                                                        */
/* ------------------------------------------------------------------ */
static uint32_t get_tick_ms(void)
{
    return HAL_GetTick();
}

/* ------------------------------------------------------------------ */
/*  Register all callbacks with the motion control library             */
/* ------------------------------------------------------------------ */
void BSP_Motion_Init(void)
{
    mc_hal_t hal;

    hal.motor_set       = motor_set;
    hal.encoder_read    = encoder_read;
    hal.imu_get_yaw     = imu_get_yaw;
    hal.line_sensor_read = line_sensor_read;
    hal.get_tick_ms     = get_tick_ms;
    hal.debug_log       = NULL;   /* no va_list-forwarding helper yet */

    mc_hal_register(&hal);
}
