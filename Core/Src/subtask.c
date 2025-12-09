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

void Normal_Balance_SubTask(void)
{
    //u1_printf("Normal Balance SubTask Running at time: %lu ms\r\n", HAL_GetTick());
    Position_PID(g_car.SpeedPID, g_car.SetSpeed);
    Angle_PID(g_car.PitchPID, g_car.Prop.Mid_Angle + g_car.SpeedPID->Out, (float)g_car.Prop.Gyro_Y);
     Angle_PID(g_car.YawPID, g_car.SetYaw, g_car.Prop.Gyro_Z);

    M1_PWM_Out = (int)&g_car.PitchPID->Out + g_car.YawPID->Out;
    M2_PWM_Out = (int)&g_car.PitchPID->Out - g_car.YawPID->Out;

    if (g_car.Flag.Stop_PWM == false)
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