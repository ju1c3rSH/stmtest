#include "subtask.h"
#include "main.h"
#include "Car.h"
#include "stdbool.h"
int M1_PWM_Out;
int M2_PWM_Out;
int STOP;
void Get_Data_SubTask(void)
{
    Car_Get_Real_Value(0.02);
    //... other subtasks
}

void Normal_Balance_SubTask(Car_TypeDef *car)
{
    //u1_printf("Normal Balance SubTask Running at time: %lu ms\r\n", HAL_GetTick());
    Position_PID(car->SpeedPID, car->SetSpeed);
    Angle_PID(car->PitchPID, car->Prop.Mid_Angle + car->SpeedPID->Out, (float)car->Prop.Gyro_Y);
    Angle_PID(car->YawPID, car->SetYaw, car->Prop.Gyro_Z);

    M1_PWM_Out = (int)car->PitchPID->Out + car->YawPID->Out;
    M2_PWM_Out = (int)car->PitchPID->Out - car->YawPID->Out;

    if (!car->Flag.Stop_PWM)
    {
        Set_Motor_A_Speed(M1_PWM_Out);
        Set_Motor_B_Speed(M2_PWM_Out);
    }
    else
    {
        Set_Motor_A_Speed(0);
        Set_Motor_B_Speed(0);
    }
}