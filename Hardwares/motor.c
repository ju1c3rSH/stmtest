#include "motor.h"
//#include "gpio.h"
#include "main.h"
//#include "tim.h"
#include "cmsis_os.h"
#include "text_utils.h"
void Motor_Init(void)
{
    // Initialize motor control GPIOs
    HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_RESET);
}

void Set_Motor_A_Speed(int16_t speed)
{
    if (speed >= 0)
    {
        HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
        //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)speed); 
        u1_printf("Motor A PWM: %d\n", speed);
        htim3.Instance->CCR3 = speed; // Make speed positive for PWM
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_SET);
        //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)-speed); 
        htim3.Instance->CCR3 = -speed; // Make speed positive for PWM
    }

    // Set PWM duty cycle based on speed (assuming speed is in range 0-1000)
}

void Set_Motor_B_Speed(int16_t speed)
{
    if (speed >= 0)
    {
        HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_RESET);
        htim3.Instance->CCR4 = speed; // Make speed positive for PWM
        //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)speed); 
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(MOTOR_B_IN1_GPIO_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_B_IN2_GPIO_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_SET);
        //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)-speed); 
        htim3.Instance->CCR4 = -speed; // Make speed positive for PWM
    }

    // based on speed

    //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
}

uint32_t Encoder_Get_A(void)
{
    int32_t temp_count = __HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    return (short)temp_count;
}
uint32_t Encoder_Get_B(void)
{
    int32_t temp_count = __HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    return (short)temp_count;
}
