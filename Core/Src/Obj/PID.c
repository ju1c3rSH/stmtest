#include "PID.h"
#include "main.h"

#define  kpo 1.0f
PID_TypeDef g_stored_pid_params[PID_TYPE_COUNT] = {
    [PID_TYPE_BALANCE_PITCH] = {.pid_type = PID_TYPE_BALANCE_PITCH, .Kp =-100.51f, .Ki = 0.0f, .Kd = -17.0f},
    [PID_TYPE_BALANCE_YAW] = {.pid_type = PID_TYPE_BALANCE_YAW, .Kp = -300.0, .Ki = 0.0f, .Kd = -1.0f,.a = 0.0f},
    [PID_TYPE_SPEED] = {.pid_type = PID_TYPE_SPEED, .Kp = kpo, .Ki  = (kpo /200), .Kd = 0.00f,.a = 0.9f},
    // [PID_TYPE_DISTANCE]    = { .pid_type = PID_TYPE_DISTANCE,    .kp = 1.0f, .ki = 0.1f, .kd = 0.01f },
};

float Position_PID(PID_TypeDef *PID, float Target)
{
    PID->Target = Target                                                                                                                                                                                                          ;
    PID->Error = PID->Target - PID->Current;
		
		PID->Error = (1 - PID->a)*PID->Error + PID->a * PID->Last_Error;
        //低通滤波，减少噪声对积分项的影响
	
    PID->I_Out += PID->Ki * PID->Error;
    if (PID->I_Out
			> PID->I_Max)
        PID->I_Out = PID->I_Max; 
    else if (PID->I_Out < -PID->I_Max)
        PID->I_Out = -PID->I_Max;

    PID->Out = PID->Kp * PID->Error +  PID->I_Out + PID->Kd * (PID->Error - PID->Last_Error);

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