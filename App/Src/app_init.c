#include "app_init.h"
#include "main.h"
#include "bsp_delay.h"
#include "bsp_mpu9250.h"
#include "text_utils.h"
#include "inv_mpu.h"
#include "bsp_motor.h"
#include "mc_motion.h"
#include "bsp_motion_hal.h"

static MPU9250 mpu = {0};
static mc_ctx_t g_mc_ctx;

static const mc_seg_t s_test_path[] = {
    { MC_SEG_SPIN, MC_CMP_NONE, 0.0f, 0,0,0,0, 0.0f, {.spin = {180.0f, 90.0f}} },
    { MC_SEG_SPIN, MC_CMP_NONE, 0.0f, 0,0,0,0, 0.0f, {.spin = {0.0f, 90.0f}} },
    { MC_SEG_LINE, MC_CMP_DIST, 0.3f, 0,0,0,0, 0.0f, {.line = {0.05f, 0.3f, 0.0f}} },
    { MC_SEG_LINE, MC_CMP_DIST, 0.3f, 0,0,0,0, 0.0f, {.line = {-0.05f, 0.3f, 0.0f}} },
    { MC_SEG_END }
};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        mc_tick(&g_mc_ctx);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    }
}

void App_Init(void)
{
    DWT_Delay_Init();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

    DWT_Delay_us(20000);

    if (MPU_Init())
    {
        u1_printf("MPU9250 init SUCCESSFUL\r\n");
    }
    else
    {
        u1_printf("MPU9250 init FAILED\r\n");
    }
    while (mpu_dmp_init());

    mpu_get_gyro_bias(mpu.mpu_data.Gyro_Bias);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    u1_printf("Bias X=%.2f Y=%.2f Z=%.2f\n",
              mpu.mpu_data.Gyro_Bias[0],
              mpu.mpu_data.Gyro_Bias[1],
              mpu.mpu_data.Gyro_Bias[2]);

    BSP_Motion_Init();

    mc_init(&g_mc_ctx);
    mc_set_params(&g_mc_ctx, 0.064f, 0.1657f, 200);

    mc_start_path(&g_mc_ctx, s_test_path);
    u1_printf("motion_ctrl test started\r\n");
}

void App_MainLoop(void)
{
    const mc_fusion_t *f = mc_get_fusion(&g_mc_ctx);
    u1_printf("%.1f,%.1f,%.3f,%u,%u\r\n",
              f->yaw_multi,
              f->wheel_spd[0],
              f->forward_speed,
              mc_get_seg_index(&g_mc_ctx),
              mc_get_state(&g_mc_ctx));

    HAL_Delay(50);
}
