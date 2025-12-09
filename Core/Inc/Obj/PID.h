#ifndef __PID_H__
#define __PID_H__

#include "main.h"

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
} PID_TypeDef;

float Position_PID(PID_TypeDef *PID, float Target);
float Distance_PID(PID_TypeDef *PID, float Target);
float Angle_PID(PID_TypeDef *PID, float Target,float Gyro);
void Set_PID(PID_TypeDef *PID,float Kp,float Ki,float Kd);

#endif /* __PID_H__ */