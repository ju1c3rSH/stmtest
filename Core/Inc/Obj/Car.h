#ifndef __CAR_H__
#define __CAR_H__

#include "main.h"
#include "PID.h"
#include "mpu9250.h"

#define __2PI 6.28318531f
//TODO: 调整小车半径参数

#define Wheel_Radius 0.065f
#define Car_Length 10  
#define Car_Width 0
#define Car_Hight 0

typedef struct
{
    bool Enable_Accelerate;
    bool Stop_PWM;
    // Add other flags as needed
} Car_FlagTypeDef;

typedef struct
{
    float Velocity_Left;
    float Velocity_Right;
    float Velocity_Target;
    float Distance_Left;
    float Distance_Right;
    float Distance_Target;
    float Pulse_Left;
    float Pulse_Right;
    float Full_Yaw; // 无限角
    float Yaw_Angle;
    float Pitch_Angle;
    float Roll_Angle;
    float Mid_Angle;//机械中值
    short Gyro_X;
    short Gyro_Y;
    short Gyro_Z;
    short Accel_X;
    short Accel_Y;
    short Accel_Z;

    float LastDistance;
    float dt;
} Car_PropTypeDef;


typedef struct 
{
   MPU9250 mpu;
}Car_DeviceTypeDef;

typedef struct
{
    float SetSpeed;
    float CurrentSpeed;
    float SetTempSpeed;
    float SetSpeedAcc;
    float SetDistance;
    float SetYaw;
    float SetMid_Angle;

    PID_TypeDef *SpeedPID;
    PID_TypeDef *DistancePID;
    PID_TypeDef *AnglePID;
    PID_TypeDef *PitchPID;
    PID_TypeDef *RollPID;
    PID_TypeDef *YawPID;

    Car_DeviceTypeDef Device;
    Car_PropTypeDef Prop;
    Car_FlagTypeDef Flag;

} Car_TypeDef;
extern Car_TypeDef g_car;
/*
积分项 (I): 误差 e(t) 需要对时间积分 ∫e(t)dt。在离散系统中，这通常近似为 sum_of_errors += e(t) * dt。
微分项 (D): 误差的变化率 de(t)/dt 在离散系统中近似为 (e(t) - e(t-1)) / dt。*/
void Car_Init(MPU9250 *mpu);
void Car_SetSpeed(Car_TypeDef *car, float speed);
void Car_Get_Real_Value(void);

#endif /* __CAR_H__ */
