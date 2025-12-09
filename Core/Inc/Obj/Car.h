#ifndef __CAR_H__
#define __CAR_H__

#include "main.h"
#include "PID.h"

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
    float Full_Yaw; // 无限角
    float Yaw_Angle;
    float Pitch_Angle;
    float Roll_Angle;
    float LastDistance;
    float dt;
} Car_PropTypeDef;

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
    
    Car_PropTypeDef Prop;
    Car_FlagTypeDef Flag;

} Car_TypeDef;

/*
积分项 (I): 误差 e(t) 需要对时间积分 ∫e(t)dt。在离散系统中，这通常近似为 sum_of_errors += e(t) * dt。
微分项 (D): 误差的变化率 de(t)/dt 在离散系统中近似为 (e(t) - e(t-1)) / dt。*/
void Car_Init(Car_TypeDef *car);
void Car_SetSpeed(Car_TypeDef *car, float speed);

#endif /* __CAR_H__ */
