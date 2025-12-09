#include "PID.h"
#include "main.h"

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

float Distance_PID(PID_Typedef *PID,float Target){
    PID->Target = Target;
    PID->Error = PID->Target - PID->Current;
    PID->I_Out += PID->Error;
    if (PID->I_Out > PID->I_Max)
        PID->I_Out = PID->I_Max;
    else if (PID->I_Out < -PID->I_Max)
        PID->I_Out = -PID->I_Max;

    PID->Out = PID->Kp * PID->Error + PID->Ki * PID -> I_Out + PID->Kd * (PID->Error - PID->Last_Error);
    if (PID->Out > PID->Out_Max)
        PID->Out = PID->Out_Max;
    else if (PID->Out < -PID->Out_Max)
        PID->Out = -PID->Out_Max;
    PID->Last_Error = PID->Error;
    return PID->Out;
}