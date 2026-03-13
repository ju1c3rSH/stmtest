/**
  ******************************************************************************
  * @file    soft_i2c.h
  * @brief   软件IIC驱动头文件 (PA5-SCL, PA7-SDA)
  ******************************************************************************
  */

#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include "stm32f1xx_hal.h"  // 根据您的STM32系列调整头文件，例如 stm32f4xx_hal.h

/* 引脚定义 */
#define I2C_SCL_PIN         GPIO_PIN_5
#define I2C_SDA_PIN         GPIO_PIN_7
#define I2C_GPIO_PORT       GPIOA

/* 宏定义 - 电平操作 */
#define I2C_SCL_HIGH()      HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET)
#define I2C_SCL_LOW()       HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET)

#define I2C_SDA_HIGH()      HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_SET)
#define I2C_SDA_LOW()       HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_RESET)

/* 读取SDA电平 */
#define I2C_SDA_READ()      HAL_GPIO_ReadPin(I2C_GPIO_PORT, I2C_SDA_PIN)



/* 函数声明 */
void Soft_I2C_Init(void);
void Soft_I2C_Start(void);
void Soft_I2C_Stop(void);
uint8_t Soft_I2C_WaitAck(void);
void Soft_I2C_Ack(void);
void Soft_I2C_NAck(void);
void Soft_I2C_SendByte(uint8_t data);
uint8_t Soft_I2C_ReadByte(uint8_t ack);
int MPU6050_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

int MPU6050_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
#endif
