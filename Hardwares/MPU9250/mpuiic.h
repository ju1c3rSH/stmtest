#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "main.h"

//正点原子MPU9250通讯线驱动
//由正点原子MPU6050驱动修改


#include "stm32f1xx_hal.h"  // 根据您的STM32系列调整头文件，例如 stm32f4xx_hal.h

/* 引脚定义 */
#define I2C_SCL_PIN         GPIO_PIN_5
#define I2C_SDA_PIN         GPIO_PIN_7
#define I2C_GPIO_PORT       GPIOA

#define MPU_SDA_IN()  { \
    GPIO_InitTypeDef GPIO_InitStruct = {0}; \
    GPIO_InitStruct.Pin = I2C_SDA_PIN; \
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; \
    GPIO_InitStruct.Pull = GPIO_PULLUP; \
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; \
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct); \
}

#define MPU_SDA_OUT() { \
    GPIO_InitTypeDef GPIO_InitStruct = {0}; \
    GPIO_InitStruct.Pin = I2C_SDA_PIN; \
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; \
    GPIO_InitStruct.Pull = GPIO_PULLUP; \
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; \
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct); \
}


#define MPU_IIC_SCL_SET     HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_SET)
#define MPU_IIC_SCL_RESET   HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, GPIO_PIN_RESET)
#define MPU_IIC_SDA_SET     HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_SET)
#define MPU_IIC_SDA_RESET   HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, GPIO_PIN_RESET)
#define MPU_READ_SDA        HAL_GPIO_ReadPin(I2C_GPIO_PORT, I2C_SDA_PIN)

//IIC所有操作函数
void MPU_IIC_Delay(void);				//MPU IIC延时函数	
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//发送IIC开始信号
void MPU_IIC_Stop(void);	  			//发送IIC停止信号
void MPU_IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MPU_IIC_Ack(void);					//IIC发送ACK信号
void MPU_IIC_NAck(void);				//IIC不发送ACK信号

void IMPU_IC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif
















