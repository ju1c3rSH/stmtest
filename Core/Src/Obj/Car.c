#include "Car.h"
#include "motor.h"
#include "math.h"
#include <stdbool.h>
#include "MahonyAHRS.h"
Car_TypeDef g_car;
#define PWM_ARR 599
#define PI 3.14159265358979323846f

extern PID_TypeDef g_stored_pid_params[PID_TYPE_COUNT];
static Car_WheelSpeedControlTypeDef SpeedControl;
static PID_TypeDef g_pitch_pid_instance;
static PID_TypeDef g_speed_pid_instance;
static PID_TypeDef g_yaw_pid_instance;
static PID_TypeDef g_distance_pid_instance;
static PID_TypeDef g_angle_pid_instance;
static PID_TypeDef g_roll_pid_instance;
static Car_FlagTypeDef g_car_flag_instance;

uint8_t s_pid_uart_rx_buf[PID_UART1_RX_BUF_SIZE];

void CorrectDate(float ax, float ay, float az,
                 float gx, float gy, float gz,
                 float ACCrange, float GYROrange, float *Date)
{
    Date[0] = ax * ACCrange * 9.8 / 32768;
    Date[1] = ay * ACCrange * 9.8 / 32768;
    Date[2] = az * ACCrange * 9.8 / 32768;
    Date[3] = gx * GYROrange / 32768 * 3.1415926 / 180;
    Date[4] = gy * GYROrange / 32768 * 3.1415926 / 180;
    Date[5] = gz * GYROrange / 32768 * 3.1415926 / 180;
    //	Date[0]=ax*0.0047856934f;
    //	Date[1]=ay*0.0047856934f;/*已确定量程可以提前算出来，减少计算*/
    //	Date[2]=az*0.0047856934f;
    //	Date[3]=gx*0.0010652644f;
    //	Date[4]=gy*0.0010652644f;
    //	Date[5]=gz*0.0010652644f;
}

void Car_Init(MPU9250 *mpu)
{

    g_car.SpeedControl = &SpeedControl;
    g_car.PitchPID = &g_pitch_pid_instance;
    g_car.SpeedPID = &g_speed_pid_instance;
    g_car.YawPID = &g_yaw_pid_instance;
    g_car.DistancePID = &g_distance_pid_instance;
    g_car.AnglePID = &g_angle_pid_instance;
    g_car.RollPID = &g_roll_pid_instance;
    g_car.Flag = g_car_flag_instance;
    /*初始化外设*/
    g_car.Device.mpu = *mpu;
    g_car.SetSpeed = 0.0f;
    g_car.CurrentSpeed = 0.0f;
    g_car.SetTempSpeed = 0.0f;
    g_car.SetSpeedAcc = 0.0f;
    g_car.SetDistance = 0.0f;
    g_car.SetYaw = 0.0f;
    //g_car.SetMid_Angle = 0.0f;
    g_car.Prop.Mid_Angle = 4.2f;
    // 初始化状态
    g_car.Flag.Enable_Accelerate = false;
    g_car.Flag.Stop_PWM = false;

    /* 初始化属性部分*/
    g_car.Prop.Velocity_Left = 0.0f;
    g_car.Prop.Velocity_Right = 0.0f;
    g_car.Prop.Velocity_Target = 0.0f;
    g_car.Prop.Last_Velocity_Left = 0.0f;
    g_car.Prop.Last_Velocity_Right = 0.0f;
    g_car.Prop.Last_Velocity_Target = 0.0f;
    g_car.Prop.Distance_Left = 0.0f;
    g_car.Prop.Distance_Right = 0.0f;
    g_car.Prop.Distance_Target = 0.0f;
    g_car.Prop.Pulse_Left = 0.0f;
    g_car.Prop.Pulse_Right = 0.0f;
    g_car.Prop.Full_Yaw = 0.0f;
    g_car.Prop.Yaw_Angle = 0.0f;
    g_car.Prop.Pitch_Angle = 0.0f;
    g_car.Prop.Roll_Angle = 0.0f;
    g_car.Prop.Gyro_X = 0;
    g_car.Prop.Gyro_Y = 0;
    g_car.Prop.Gyro_Z = 0;
    g_car.Prop.Accel_X = 0;
    g_car.Prop.Accel_Y = 0;
    g_car.Prop.Accel_Z = 0;
    g_car.Prop.LastDistance = 0.0f;
    g_car.Prop.dt = 0.02f;

    /* 初始化PID部分*/
    // 角度环
    g_car.PitchPID->Kp = g_stored_pid_params[PID_TYPE_BALANCE_PITCH].Kp;
    // g_car.PitchPID->Ki = 0.5f;
    // out 应该不是浮点数
    // TODO：后面写g_stored_pid_params和这里对接的逻辑
    // TODO已经完成
    g_car.PitchPID->Ki = g_stored_pid_params[PID_TYPE_BALANCE_PITCH].Ki;
    g_car.PitchPID->Kd = g_stored_pid_params[PID_TYPE_BALANCE_PITCH].Kd;
    g_car.PitchPID->Out = 0;
    g_car.PitchPID->Error = 0;
    g_car.PitchPID->Last_Error = 0;
    g_car.PitchPID->Current = 0;
    g_car.PitchPID->Target = 0;
    g_car.PitchPID->I_Max = 0.0f;//不需要积分
    g_car.PitchPID->Out_Max =PWM_ARR;
    g_car.PitchPID->pid_type = PID_TYPE_BALANCE_PITCH;
    // 速度环
    g_car.SpeedPID->Kp = g_stored_pid_params[PID_TYPE_SPEED].Kp;
    g_car.SpeedPID->Ki = g_stored_pid_params[PID_TYPE_SPEED].Ki;
    g_car.SpeedPID->Kd = g_stored_pid_params[PID_TYPE_SPEED].Kd;//不需要用到D
    // g_car.SpeedPID->Kd = 0.01f;
    g_car.SpeedPID->I_Max = 3000.0f;
    g_car.SpeedPID->Out_Max = 30.0f;
    g_car.SpeedPID->Out = 9000.0f;
    g_car.SpeedPID->Error = 0;
    g_car.SpeedPID->Last_Error = 0;
    g_car.SpeedPID->Current = 0;
    g_car.SpeedPID->Target = 0;
    g_car.SpeedPID->pid_type = PID_TYPE_SPEED;
    // Yaw角角度环
    g_car.YawPID->Kp = g_stored_pid_params[PID_TYPE_BALANCE_YAW].Kp;
    g_car.YawPID->Ki = g_stored_pid_params[PID_TYPE_BALANCE_YAW].Ki;
    g_car.YawPID->Kd = g_stored_pid_params[PID_TYPE_BALANCE_YAW].Kd;
    g_car.YawPID->I_Max = 0.0f;
    g_car.YawPID->Out_Max = 0.0f;//disable yaw control for now
    g_car.YawPID->Out = 0;
    g_car.YawPID->Error = 0;
    g_car.YawPID->Last_Error = 0;
    g_car.YawPID->Current = 0;
    g_car.YawPID->Target = 0;
    g_car.YawPID->pid_type = PID_TYPE_BALANCE_YAW;
}

float InfiniteYaw(float Now_Yaw)
{
    static int32_t wrap_count = 0; // 记录偏航角突变次数
    static float last_yaw = 0.0f;
    static bool is_first_call = true;

    if (is_first_call)
    {
        last_yaw = Now_Yaw;
        is_first_call = false;
        return Now_Yaw;
    }
    float delta_yaw = Now_Yaw - last_yaw;

    if (delta_yaw > 180.0f)
    {
        // 从-180跳到180
        wrap_count--;
    }
    else if (delta_yaw < -180.0f)
    {
        // 从 180跳到-180
        wrap_count++;
    }

    last_yaw = Now_Yaw;
    return Now_Yaw + (float)(wrap_count * 360);
}
float Car_GetCurrentSpeed(void)
{
    return (g_car.Prop.Velocity_Left + g_car.Prop.Velocity_Right) / 2.0f;
}
float Car_GetPitchAngle(void)
{
    return g_car.Prop.Pitch_Angle;
}
Car_TypeDef *Car_GetInstance(void)
{
    return &g_car; // 谨慎使用，仅在必要时（如初始化传参）
}
void Car_SetSpeed(float speed)
{
    g_car.SetSpeed = speed;
    // 对外只暴露操作接口
}
void Car_Get_Real_Value(float dt)
{
    static char Status;
    float Temp[6];

    g_car.Prop.Pulse_Left = Encoder_Get_A();
    g_car.Prop.Pulse_Right = Encoder_Get_B();

    g_car.Prop.Last_Velocity_Left = g_car.Prop.Velocity_Left;
    g_car.Prop.Last_Velocity_Right = g_car.Prop.Velocity_Right;
    g_car.Prop.Last_Velocity_Target = g_car.Prop.Velocity_Target;

    g_car.Prop.Velocity_Left = (float)(g_car.Prop.Pulse_Left) * Wheel_Radius * __2PI / 2000.0f / dt / 30; // 假设每转2000脉冲 获取轮真实速度[m/s]
    g_car.Prop.Velocity_Right = (float)(g_car.Prop.Pulse_Right) * Wheel_Radius * __2PI / 2000.0f / dt / 30;

    g_car.Prop.Distance_Left += g_car.Prop.Velocity_Left * dt;
    g_car.Prop.Distance_Right += g_car.Prop.Velocity_Right * dt;

    /*
    //计算轮子路程
    Blance.Car_RWheel_Distance += 1000*Blance.Car_RWheel_Pul*Wheel_Radius*__2PI/2000.0f/30;//获取右边轮真实路程[m]
    Blance.Car_LWheel_Distance += 1000*Blance.Car_RWheel_Pul*Wheel_Radius*__2PI/2000.0f/30;//获取左边轮真实路程*/

    g_car.Prop.dt = dt;

    // 9250对象，x是[0]，y是[1]，z是[2]

    MPU9250_ReadAccel(&g_car.Device.mpu);
    MPU9250_ReadGyro(&g_car.Device.mpu);
    MPU9250_ReadMag(&g_car.Device.mpu);
    /*
     static float acc_x_filtered = 0, acc_y_filtered = 0, acc_z_filtered = 0;
    const float alpha = 0.2f; // 0.1~0.3
    


    
    acc_x_filtered = alpha * g_car.Device.mpu.mpu_data.Accel[0] + (1 - alpha) * acc_x_filtered;
    acc_y_filtered = alpha * g_car.Device.mpu.mpu_data.Accel[1] + (1 - alpha) * acc_y_filtered;
    acc_z_filtered = alpha * g_car.Device.mpu.mpu_data.Accel[2] + (1 - alpha) * acc_z_filtered;

    float gx, gy, gz, ax, ay, az;
    gx = (g_car.Device.mpu.mpu_data.Gyro_row[0] - g_car.Device.mpu.mpu_data.Gyro_Bias[0]) / 16.384f;
    gy = (g_car.Device.mpu.mpu_data.Gyro_row[1] - g_car.Device.mpu.mpu_data.Gyro_Bias[1]) / 16.384f;
    gz = (g_car.Device.mpu.mpu_data.Gyro_row[2] - g_car.Device.mpu.mpu_data.Gyro_Bias[2]) / 16.384f;
    ax = g_car.Device.mpu.mpu_data.Accel[0];
    ay = g_car.Device.mpu.mpu_data.Accel[1];
    az = g_car.Device.mpu.mpu_data.Accel[2];
    Mahony_update_IMU(gx, gy, gz, acc_x_filtered, acc_y_filtered, acc_z_filtered);
    */
    // mpu->mpu_data.Gyro[0] = -mpu->mpu_data.Gyro[0]; //根据安装方向调整轴向
    
    g_car.Prop.Gyro_X = (g_car.Device.mpu.mpu_data.Gyro[0]);
    g_car.Prop.Gyro_Y = (g_car.Device.mpu.mpu_data.Gyro[1]);
    g_car.Prop.Gyro_Z = (g_car.Device.mpu.mpu_data.Gyro[2]);
    g_car.Prop.Accel_X = (g_car.Device.mpu.mpu_data.Accel[0]);
    g_car.Prop.Accel_Y = (g_car.Device.mpu.mpu_data.Accel[1]);
    g_car.Prop.Accel_Z = (g_car.Device.mpu.mpu_data.Accel[2]);
    //取消归一化
    //如果 Accel_X 是 m/s²，结果仍然正确（因为比例不变）
    //但没必要做单位转换，反而增加计算开销，效果不佳的时候，再试试吧
    float acc_pitch = atan2f(
        -g_car.Prop.Accel_X,
        sqrtf(g_car.Prop.Accel_Y*g_car.Prop.Accel_Y + g_car.Prop.Accel_Z*g_car.Prop.Accel_Z )
                            ) * 180.0f / PI;
    const float alpha = 0.985f;

    /*
    g_car.Prop.Gyro_X = (short)(gx);
    g_car.Prop.Gyro_Y = (short)(gy);
    g_car.Prop.Gyro_Z = (short)(gz); 
    g_car.Prop.Accel_X = (short)(ax);
    g_car.Prop.Accel_Y = (short)(ay);
    g_car.Prop.Accel_Z = (short)(az);
    
    */
   
    // Mahony_computeAngles();
    /* Mahony_computeAngles(); */
    /*
    g_car.Prop.Pitch_Angle = atan2f(-g_car.Prop.Accel_X,
                                        sqrtf(g_car.Prop.Accel_Y * g_car.Prop.Accel_Y +
                                              g_car.Prop.Accel_Z * g_car.Prop.Accel_Z)) *
                                 180.0f / PI;

    */
    //这里放弃使用纯加速度计的pitch计算方法，使用简单的互补滤波
    g_car.Prop.Pitch_Angle = alpha * (g_car.Prop.Pitch_Angle + g_car.Prop.Gyro_X *dt) + (1.0f - alpha)* acc_pitch;
    g_car.Prop.Roll_Angle = atan2f(g_car.Prop.Accel_Y, g_car.Prop.Accel_Z) * 180.0f / PI;
    g_car.Prop.Yaw_Angle += g_car.Prop.Gyro_Z * dt;


/*
    g_car.Prop.Pitch_Angle = getPitch();

    g_car.Prop.Roll_Angle = getRoll();
    g_car.Prop.Yaw_Angle = getYaw();
    //g_car.PitchPID->Current = g_car.Prop.Pitch_Angle;
    //g_car.RollPID->Current = g_car.Prop.Roll_Angle;
    g_car.Prop.Full_Yaw = InfiniteYaw(g_car.Prop.Yaw_Angle);
    //g_car.YawPID->Current = g_car.Prop.Full_Yaw;
    //g_car.SpeedPID->Current = (g_car.Prop.Velocity_Left + g_car.Prop.Velocity_Right) / 2.0f;
    //这两行被新封装取代
    */
    PID_Set_Current(g_car.RollPID, g_car.Prop.Roll_Angle);
    PID_Set_Current(g_car.PitchPID, g_car.Prop.Pitch_Angle);
    PID_Set_Current(g_car.SpeedPID, (g_car.Prop.Velocity_Left + g_car.Prop.Velocity_Right) / 2.0f);
    PID_Set_Current(g_car.YawPID, InfiniteYaw(g_car.Prop.Yaw_Angle));
    
    //u1_printf("{Pitch: %.2f, Roll: %.2f, Yaw: %.2f\r\n}", g_car.Prop.Pitch_Angle, g_car.Prop.Roll_Angle, g_car.Prop.Full_Yaw);
    //  pitch roll yaw
		
		u1_printf("{\"sensor\":\"mpu9250\",\"data\":{\"attitude\":{\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f},\"accel\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}}\r\n",
          g_car.Prop.Pitch_Angle,
          g_car.Prop.Roll_Angle,
          g_car.Prop.Full_Yaw,
          g_car.Prop.Accel_X ,
          g_car.Prop.Accel_Y ,
          g_car.Prop.Accel_Z );
					
    //u1_printf(" %.2f,  %.2f, %.2f\r\n", g_car.Prop.Pitch_Angle, g_car.Prop.Roll_Angle, g_car.Prop.Full_Yaw);
}