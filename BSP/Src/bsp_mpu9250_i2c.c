#include "bsp_mpu9250_i2c.h"

//����ԭ��MPU9250ͨѶ������
//������ԭ��MPU6050�����޸�
 
  //MPU IIC ��ʱ����
void MPU_IIC_Delay(void)
{
//	delay_us(2);
	int i=10;
	while(i--);
	
}

void MPU_IIC_Init(void)
{					     
  
}

//����IIC��ʼ�ź�
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda�����
	MPU_IIC_SDA_SET;	  	  
	MPU_IIC_SCL_SET;
	MPU_IIC_Delay();
 	MPU_IIC_SDA_RESET;//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay();
	MPU_IIC_SCL_RESET;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();//sda�����
	MPU_IIC_SCL_RESET;
	MPU_IIC_SDA_RESET;//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay();
	MPU_IIC_SCL_SET; 
	MPU_IIC_SDA_SET;//����I2C���߽����ź�
	MPU_IIC_Delay();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t MPU_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	MPU_SDA_IN();      //SDA����Ϊ����  
	MPU_IIC_SDA_SET ;MPU_IIC_Delay();	   
	MPU_IIC_SCL_SET;MPU_IIC_Delay();	 
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL_RESET;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL_RESET;
	MPU_SDA_OUT();
	MPU_IIC_SDA_RESET;
	MPU_IIC_Delay();
	MPU_IIC_SCL_SET;
	MPU_IIC_Delay();
	MPU_IIC_SCL_RESET;
}
//������ACKӦ��		    
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL_RESET;
	MPU_SDA_OUT();
	MPU_IIC_SDA_SET;
	MPU_IIC_Delay();
	MPU_IIC_SCL_SET;
	MPU_IIC_Delay();
	MPU_IIC_SCL_RESET;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MPU_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL_RESET;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        if((txd&0x80)>>7){MPU_IIC_SDA_SET;}
				else{MPU_IIC_SDA_RESET;}
        txd<<=1; 	  
		    MPU_IIC_SCL_SET;
		    MPU_IIC_Delay(); 
		    MPU_IIC_SCL_RESET;	
		    MPU_IIC_Delay();
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL_RESET; 
        MPU_IIC_Delay();
		MPU_IIC_SCL_SET;
        receive<<=1;
        if(MPU_READ_SDA)receive++;   
		MPU_IIC_Delay(); 
    }					 
    if (!ack)
        MPU_IIC_NAck();//����nACK
    else
        MPU_IIC_Ack(); //����ACK   
    return receive;
}

























