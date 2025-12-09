#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
//#include "gpio.h"
//#include "tim.h"

#define MOTOR_A_IN1_GPIO_PORT  GPIOA
#define MOTOR_A_IN1_PIN        GPIO_PIN_0
#define MOTOR_A_IN2_GPIO_PORT  GPIOA
#define MOTOR_A_IN2_PIN        GPIO_PIN_1

#define MOTOR_B_IN1_GPIO_PORT  GPIOB
#define MOTOR_B_IN1_PIN        GPIO_PIN_6
#define MOTOR_B_IN2_GPIO_PORT  GPIOB
#define MOTOR_B_IN2_PIN        GPIO_PIN_7

//A is for Left motor, B is for Right motor

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void Motor_Init(void);
void Set_Motor_A_Speed(int16_t speed);
void Set_Motor_B_Speed(int16_t speed);

uint32_t Encoder_Get_A(void);
uint32_t Encoder_Get_B(void);

#endif /* __MOTOR_H__ */
