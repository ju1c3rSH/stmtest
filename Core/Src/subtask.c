#include "subtask.h"
#include "main.h"
#include "Car.h"
void Get_Data_SubTask(void)
{
    Get_Car_Real_Value();
    //... other subtasks
}

void Normal_Balance_SubTask(void)
{
    Position_PID(&g_car.SpeedPID, g_car.SetSpeed);
    Angle_PID(&g_car.PitchPID, g_car.Prop.Mid_Angle + g_car.SpeedPID->Out, (float)g_car.Prop.Gyro_Y);
    //Angle_PID(&g_car.YawPID, g_car.SetYaw, g_car.Prop.Gyro_Z);
}