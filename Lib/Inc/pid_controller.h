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
    const char *name;
    PID_Type_t type;
} PID_Type_Map_t;

extern const PID_Type_Map_t pid_type_map[3];
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
    float filter_alpha;     // 误差低通滤波系�?(0~1)
    uint8_t Clear;          // 清除标志 (保留未使�?
    PID_Type_t pid_type;

} PID_TypeDef;

extern PID_TypeDef g_stored_pid_params[PID_TYPE_COUNT];

// 只是处理func
float Position_PID(PID_TypeDef *PID, float Target);
float Distance_PID(PID_TypeDef *PID, float Target);
float Angle_PID(PID_TypeDef *PID, float Target, float Gyro);
void Set_PID(PID_TypeDef *PID, float Kp, float Ki, float Kd);
void PID_Set_Current(PID_TypeDef *PID, float current);
int PID_GetOutput(PID_TypeDef *PID);
#endif /* __PID_H__ */


