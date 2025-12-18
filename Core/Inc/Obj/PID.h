#ifndef __PID_H__
#define __PID_H__
#include <stdbool.h>
#include "main.h"


#include <stdint.h>

typedef enum
{
    PID_TYPE_BALANCE_PITCH,
    PID_TYPE_BALANCE_YAW,
    PID_TYPE_SPEED,
    // PID_TYPE_DISTANCE,
    PID_TYPE_INVALID,
    PID_TYPE_COUNT
} PID_Type_t;


typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Target;
    float Last_Error;
    float Error;
    float I_Max;
    float I_Out;
    float Out;
    float Out_Max;
    float Current;
    uint8_t Clear;
    PID_Type_t pid_type;

} PID_TypeDef;



// 只是处理func
float Position_PID(PID_TypeDef *PID, float Target);
float Distance_PID(PID_TypeDef *PID, float Target);
float Angle_PID(PID_TypeDef *PID, float Target, float Gyro);
void Set_PID(PID_TypeDef *PID, float Kp, float Ki, float Kd);

#endif /* __PID_H__ */