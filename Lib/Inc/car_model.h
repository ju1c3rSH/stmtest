#ifndef __CAR_H__
#define __CAR_H__

#include "main.h"
#include "pid_controller.h"

#include <stdbool.h>
#include "bsp_mpu9250.h"
#define __2PI 6.28318531f

//#define PID_UART1_RX_BUF_SIZE 128

//在UART1使用的PID解析�?
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
    int Pulse_Left;
    int Pulse_Right;
    float Full_Yaw; // 无限�?
    float Yaw_Angle ;
    float Pitch_Angle;
    float Roll_Angle;
    float Mid_Angle; // 机械中�?
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float LastDistance;
    float dt;
} Car_PropTypeDef;


typedef enum SpeedCurveTypeDef
{
    CURVE_NONE = 0, // 直启
    CURVE_TRAP = 1, // 梯形曲线
    CURVE_SPTA = 2  // S型曲�?
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
    MPU9250 *mpu;
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
void Car_Init(MPU9250 *mpu);
void Car_SetSpeed(float speed);
void Car_Get_Real_Value(void);
Car_TypeDef* Car_GetInstance(void);
float Car_GetPitchAngle(void);
void CorrectDate(float ax, float ay, float az,
                 float gx, float gy, float gz,
                 float ACCrange, float GYROrange, float *Date);
float InfiniteYaw(float Now_Yaw);
float Car_GetPitchAngle(void);
float Car_GetCurrentSpeed(void);
#endif /* __CAR_H__ */
