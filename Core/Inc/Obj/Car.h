#ifndef __CAR_H__
#define __CAR_H__

#include "main.h"
#include "PID.h"
#include "mpu9250.h"
#include <stdbool.h>
#define __2PI 6.28318531f

#define PID_UART1_RX_BUF_SIZE 128

//在UART1使用的PID解析器
//Example:   {"type":"balance_pitch","kp":1.5,"ki":0.2,"kd":0.05}

#define Wheel_Radius 0.032f
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
    float Max_Accelerate;
    float Velocity_Left;
    float Velocity_Right;
    float Last_Velocity_Left;
    float Last_Velocity_Right;
    float Last_Velocity_Target;
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
    float Mid_Angle; // 机械中值
    short Gyro_X;
    short Gyro_Y;
    short Gyro_Z;
    short Accel_X;
    short Accel_Y;
    short Accel_Z;

    float LastDistance;
    float dt;
} Car_PropTypeDef;


typedef enum SpeedCurveTypeDef
{
    CURVE_NONE = 0, // 直启
    CURVE_TRAP = 1, // 梯形曲线
    CURVE_SPTA = 2  // S型曲线
} SpeedCurveTypeDef;


typedef struct
{
    float Velocity_Start;
    float Velocity_Current;
    float Velocity_Target;
    float Velocity_Max;
    float Velocity_Min;
    uint32_t aTimes; //（加速）步数计数
    uint32_t Max_Time;
    SpeedCurveTypeDef Curve_Type;
    float Accelerate;
    float Flexible;
} Car_WheelSpeedControlTypeDef;

typedef struct
{
    MPU9250 mpu;
} Car_DeviceTypeDef;

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
    Car_WheelSpeedControlTypeDef *SpeedControl;
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
void Car_Get_Real_Value(float dt);
void CorrectDate(float ax, float ay, float az,
                 float gx, float gy, float gz,
                 float ACCrange, float GYROrange, float *Date);
float InfiniteYaw(float Now_Yaw);
#endif /* __CAR_H__ */
