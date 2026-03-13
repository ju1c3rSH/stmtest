
/**
  ******************************************************************************
  * @file    soft_i2c.c
  * @brief   软件IIC驱动源文件 (PA5-SCL, PA7-SDA)
  ******************************************************************************
  */

#include "softiic9250.h"
#include "delay.h"
/**
  * @brief  软件IIC GPIO初始化
  * @param  无
  * @retval 无
  */
void Soft_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能GPIOA时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* 配置SCL和SDA引脚为开漏输出，并启用上拉 */
    GPIO_InitStruct.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;      // 开漏输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;              // 上拉电阻
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;    // 高速
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

    /* 初始状态：SCL和SDA均为高电平（空闲状态） */
    I2C_SCL_HIGH();
    I2C_SDA_HIGH();
}

/**
  * @brief  产生IIC起始信号
  * @param  无
  * @retval 无
  */
void Soft_I2C_Start(void)
{
    I2C_SDA_HIGH();   // 确保SDA为高
    I2C_SCL_HIGH();   // 确保SCL为高
    DWT_Delay_us(5);
    I2C_SDA_LOW();    // SDA拉低，产生起始信号
    DWT_Delay_us(5);
    I2C_SCL_LOW();    // 拉低SCL，准备传输数据
}

/**
  * @brief  产生IIC停止信号
  * @param  无
  * @retval 无
  */
void Soft_I2C_Stop(void)
{
    I2C_SDA_LOW();    // 确保SDA为低
    I2C_SCL_HIGH();   // SCL先高
    DWT_Delay_us(5);
    I2C_SDA_HIGH();   // SDA上升沿，产生停止信号
    DWT_Delay_us(5);
}

/**
  * @brief  等待从机应答信号
  * @param  无
  * @retval 0：收到应答(ACK)，1：未收到应答(NACK)
  */
uint8_t Soft_I2C_WaitAck(void)
{
    uint8_t timeout = 0;

    /* 释放SDA线，由从机控制 */
    I2C_SDA_HIGH();
    DWT_Delay_us(2);
    I2C_SCL_HIGH();   // 第9个时钟脉冲，从机拉低SDA表示应答
    DWT_Delay_us(2);

    /* 等待SDA被从机拉低（应答），并增加超时处理 */
    while (I2C_SDA_READ())
    {
        timeout++;
        if (timeout > 250)  // 超时判断
        {
            I2C_SCL_LOW();  // 拉低SCL，结束本次通信
            return 1;       // 返回NACK
        }
    }

    I2C_SCL_LOW();  // 拉低SCL，结束应答位
    return 0;       // 返回ACK
}

/**
  * @brief  主机发送应答信号(ACK)
  * @param  无
  * @retval 无
  */
void Soft_I2C_Ack(void)
{
    I2C_SDA_LOW();   // 拉低SDA表示应答
    DWT_Delay_us(2);
    I2C_SCL_HIGH();  // 产生时钟脉冲
    DWT_Delay_us(5);
    I2C_SCL_LOW();   // 拉低SCL，结束应答
    I2C_SDA_HIGH();  // 释放SDA
}

/**
  * @brief  主机发送非应答信号(NACK)
  * @param  无
  * @retval 无
  */
void Soft_I2C_NAck(void)
{
    I2C_SDA_HIGH();  // 拉高SDA表示非应答
    DWT_Delay_us(2);
    I2C_SCL_HIGH();  // 产生时钟脉冲
    DWT_Delay_us(5);
    I2C_SCL_LOW();   // 拉低SCL，结束非应答
}

/**
  * @brief  主机发送一个字节数据
  * @param  data 要发送的8位数据
  * @retval 无
  */
void Soft_I2C_SendByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        /* 先拉低SCL，准备设置数据 */
        I2C_SCL_LOW();
        DWT_Delay_us(2);

        /* 根据数据位设置SDA电平（从高位开始传输） */
        if (data & 0x80)
            I2C_SDA_HIGH();
        else
            I2C_SDA_LOW();

        DWT_Delay_us(2);
        /* 拉高SCL，从机在上升沿读取数据 */
        I2C_SCL_HIGH();
        DWT_Delay_us(5);

        data <<= 1;  // 左移一位，准备下一位
    }

    /* 传输完8位后拉低SCL，释放SDA，为应答位做准备 */
    I2C_SCL_LOW();
    I2C_SDA_HIGH();
}

/**
  * @brief  主机读取一个字节数据
  * @param  ack 读取后是否发送应答：1-发送应答(ACK)，0-发送非应答(NACK)
  * @retval 读取到的8位数据
  */
uint8_t Soft_I2C_ReadByte(uint8_t ack)
{
    uint8_t i, data = 0;

    /* 读取前释放SDA线（由从机控制） */
    I2C_SDA_HIGH();

    for (i = 0; i < 8; i++)
    {
        data <<= 1;      // 左移一位，为接收新位做准备
        I2C_SCL_HIGH();  // SCL高电平，从机输出数据
        DWT_Delay_us(2);

        /* 读取SDA电平并存入数据 */
        if (I2C_SDA_READ())
            data |= 0x01;

        I2C_SCL_LOW();   // 拉低SCL，从机准备下一位
        DWT_Delay_us(2);
    }

    /* 根据参数决定发送应答还是非应答 */
    if (ack)
        Soft_I2C_Ack();   // 发送应答，继续读取
    else
        Soft_I2C_NAck();  // 发送非应答，结束读取

    return data;
}

int MPU6050_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) //向指定地址写len个字节,用于dmp
{
	int i;
    Soft_I2C_Start();
    Soft_I2C_SendByte(addr << 1);
    Soft_I2C_WaitAck();
    Soft_I2C_SendByte(reg);
    Soft_I2C_WaitAck();
	for (i = 0; i < len; i++) 
	{
        Soft_I2C_SendByte(data[i]);
        Soft_I2C_WaitAck();
    }
    Soft_I2C_Stop();
	return 0;
}

int MPU6050_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)// 向指定从机指定地址读len个字节，用于dmp
{
    Soft_I2C_Start();
    Soft_I2C_SendByte(addr << 1);
    Soft_I2C_WaitAck();
    Soft_I2C_SendByte(reg);
    Soft_I2C_WaitAck();
    Soft_I2C_Start();
    Soft_I2C_SendByte((addr << 1)+1);
    Soft_I2C_WaitAck();
    while (len) {
        if (len == 1)
		{
            *buf = Soft_I2C_ReadByte(1);
		}
        else
		{
            *buf = Soft_I2C_ReadByte(0);
		}
        buf++;
        len--;
    }
	Soft_I2C_Stop();
	return 0;
}
