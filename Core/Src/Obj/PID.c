#include "PID.h"
#include "main.h"

#define SPEED_PID_KP 2.75537f

// NOTE: I_Max 和 Out_Max 不在此处初始化，而是在 Car_Init() 中硬编码。
// 原因：这些是固定配置参数，不需要通过串口动态调整。
// 如需修改，请编辑 Car.c 中的初始化代码。
PID_TypeDef g_stored_pid_params[PID_TYPE_COUNT] = {
    [PID_TYPE_BALANCE_PITCH] = {.pid_type = PID_TYPE_BALANCE_PITCH, .Kp =-140.0f, .Ki = 0.0f, .Kd = -4.273f},
    [PID_TYPE_BALANCE_YAW] = {.pid_type = PID_TYPE_BALANCE_YAW, .Kp = -300.0, .Ki = 0.0f, .Kd = -1.0f, .filter_alpha = 0.0f},
    //[PID_TYPE_SPEED] = {.pid_type = PID_TYPE_SPEED, .Kp = 3.354f, .Ki  = 0.01681f, .Kd = 0.00f, .filter_alpha = 0.9f},
    // filter_alpha: 误差低通滤波系数，0=无滤波，1=全滤波
    // 编码器速度滤波建议值: 0.3~0.7 (当前0.9偏大，相位滞后会明显)
    // 对于连续PID系统，建议从0.5开始调试，根据响应速度和噪声权衡
    [PID_TYPE_SPEED] = {.pid_type = PID_TYPE_SPEED, .Kp = SPEED_PID_KP, .Ki = (SPEED_PID_KP / 192.0f), .Kd = 0.00f, .filter_alpha = 0.9f},
};

float Position_PID(PID_TypeDef *PID, float Target)
{
    PID->Target = Target                                                                                                                                                                                                          ;
    PID->Error = PID->Target - PID->Current;
		
		PID->Error = (1 - PID->filter_alpha) * PID->Error + PID->filter_alpha * PID->Last_Error;
        //低通滤波，减少噪声对积分项的影响
	
    PID->I_Out += PID->Error;
    if (PID->I_Out > PID->I_Max)
        PID->I_Out = PID->I_Max;
    else if (PID->I_Out < -PID->I_Max)
        PID->I_Out = -PID->I_Max;

    PID->Out = PID->Kp * PID->Error + PID->Ki * PID->I_Out + PID->Kd * (PID->Error - PID->Last_Error);

    if (PID->Out > PID->Out_Max)
        PID->Out = PID->Out_Max;
    else if (PID->Out < -PID->Out_Max)
        PID->Out = -PID->Out_Max;

    PID->Last_Error = PID->Error;

    return PID->Out;
}

float Angle_PID(PID_TypeDef *PID, float Target, float Gyro)
{
    PID->Target = Target;
    PID->Error = PID->Target - PID->Current;

    PID->I_Out += PID->Error;
    if (PID->I_Out > PID->I_Max)
        PID->I_Out = PID->I_Max;
    else if (PID->I_Out < -PID->I_Max)
        PID->I_Out = -PID->I_Max;

    PID->Out = PID->Kp * PID->Error + PID->Ki * PID->I_Out - PID->Kd * Gyro;

    if (PID->Out > PID->Out_Max)
        PID->Out = PID->Out_Max;
    else if (PID->Out < -PID->Out_Max)
        PID->Out = -PID->Out_Max;

    PID->Last_Error = PID->Error;

//     float feedforward = 0;

//         if (fabsf(PID->Error) > 15.0f)
//     {

//         feedforward = 0.3f * PID->Out_Max * (PID->Error > 0 ? 1 : -1);
//     }

//    PID->Out += feedforward;

    return PID->Out;
}

float Distance_PID(PID_TypeDef *PID, float Target)
{
    PID->Target = Target;
    PID->Error = PID->Target - PID->Current;
    PID->I_Out += PID->Error;
    if (PID->I_Out > PID->I_Max)
        PID->I_Out = PID->I_Max;
    else if (PID->I_Out < -PID->I_Max)
        PID->I_Out = -PID->I_Max;

    PID->Out = PID->Kp * PID->Error + PID->Ki * PID->I_Out + PID->Kd * (PID->Error - PID->Last_Error);
    if (PID->Out > PID->Out_Max)
        PID->Out = PID->Out_Max;
    else if (PID->Out < -PID->Out_Max)
        PID->Out = -PID->Out_Max;
    PID->Last_Error = PID->Error;
    return PID->Out;
}
void PID_Set_Current(PID_TypeDef *PID, float current)
{
    PID->Current = current;
}
int PID_GetOutput(PID_TypeDef *PID)
{
    return (int)(PID->Out);
}
void Set_PID(PID_TypeDef *PID, float Kp, float Ki, float Kd)
{
    PID->Kp = Kp;
    PID->Ki = Ki;
    PID->Kd = Kd;
}
