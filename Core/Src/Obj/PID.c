#include "PID.h"
#include "main.h"
#include "cmsis_os.h"


PID_TypeDef g_stored_pid_params[PID_TYPE_COUNT] = {
    [PID_TYPE_BALANCE_PITCH] = {.pid_type = PID_TYPE_BALANCE_PITCH, .Kp = 1.0f, .Ki = 0.1f, .Kd = 0.01f},
    [PID_TYPE_BALANCE_YAW] = {.pid_type = PID_TYPE_BALANCE_YAW, .Kp = 1.0f, .Ki = 0.1f, .Kd = 0.01f},
    [PID_TYPE_SPEED] = {.pid_type = PID_TYPE_SPEED, .Kp = 1.0f, .Ki = 0.1f, .Kd = 0.01f},
    // [PID_TYPE_DISTANCE]    = { .pid_type = PID_TYPE_DISTANCE,    .kp = 1.0f, .ki = 0.1f, .kd = 0.01f },
};
float Position_PID(PID_TypeDef *PID, float Target)
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