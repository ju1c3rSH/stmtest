#include "speed_control.h"
#include "Car.h"
#include <math.h>

static void CalculateSPTA(Car_WheelSpeedControlTypeDef *spta)
{
    float power = 0.0;
    float speed = 0.0;

    if (spta->Max_Time <= 0)
    {
        spta->Velocity_Current = spta->Velocity_Target;
        return;
    }

    power = (2.0f * ((float)spta->aTimes) - ((float)spta->Max_Time)) / ((float)spta->Max_Time);
    power = (0.0f - spta->Flexible) * power; // Flexible应该是正数，用于控制曲线形状

    speed = 1.0f + expf(power);
    if (speed != 0.0f)
    {
        speed = (spta->Velocity_Target - spta->Velocity_Start) / speed;
    }
    else
    {
        speed = 0.0f;
    }
    spta->Velocity_Current = speed + spta->Velocity_Start;

    if (spta->Velocity_Current > spta->Velocity_Max)
    {
        spta->Velocity_Current = spta->Velocity_Max;
    }
    else if (spta->Velocity_Current < spta->Velocity_Min)
    {
        spta->Velocity_Current = spta->Velocity_Min;
    }
}

void ApplySpeedCurveToCar(Car_TypeDef *car_obj)
{

    Car_WheelSpeedControlTypeDef *wheel_speed_ctrl = car_obj->SpeedControl;

    if (wheel_speed_ctrl == NULL)
    {

        return;
    }

    float temp = 0.0f;

    float target_speed = car_obj->SetSpeed;
    if (target_speed > wheel_speed_ctrl->Velocity_Max)
    {
        target_speed = wheel_speed_ctrl->Velocity_Max;
    }
    if (target_speed < wheel_speed_ctrl->Velocity_Min)
    {
        target_speed = wheel_speed_ctrl->Velocity_Min;
    }
    wheel_speed_ctrl->Velocity_Target = target_speed;

    if ((fabsf(wheel_speed_ctrl->Velocity_Current - wheel_speed_ctrl->Velocity_Start) <= wheel_speed_ctrl->Accelerate) && (wheel_speed_ctrl->Max_Time == 0))
    {

        if (wheel_speed_ctrl->Velocity_Start < wheel_speed_ctrl->Velocity_Min)
        {
            wheel_speed_ctrl->Velocity_Start = wheel_speed_ctrl->Velocity_Min;
        }

        temp = fabsf(wheel_speed_ctrl->Velocity_Target - wheel_speed_ctrl->Velocity_Start);
        temp = temp / wheel_speed_ctrl->Accelerate;
        wheel_speed_ctrl->Max_Time = (uint32_t)(temp) + 1;
        wheel_speed_ctrl->aTimes = 0;
    }

    if (wheel_speed_ctrl->aTimes < wheel_speed_ctrl->Max_Time)
    {
        CalculateSPTA(wheel_speed_ctrl);
        wheel_speed_ctrl->aTimes++; // 增加步数计数器
    }
    else
    {

        wheel_speed_ctrl->Velocity_Current = wheel_speed_ctrl->Velocity_Target;
        wheel_speed_ctrl->Max_Time = 0;
        wheel_speed_ctrl->aTimes = 0;
    }
}
