#ifndef __UART_PID_PARSE_H__
#define __UART_PID_PARSE_H__
#include <stdint.h>
#include <stdbool.h>
#include "PID.h"
#define UART1_PID_BUFFER_SIZE 256
static uint8_t s_uart1_rx_buf[UART1_PID_BUFFER_SIZE];
typedef struct
{
    uint32_t buffer[UART1_PID_BUFFER_SIZE];
    float kp;
    float ki;
    float kd;
    PID_Type_t pid_type;
    // uint32_t checkSum;
} PID_UART_PARSE_Params_t;


typedef struct
{
    uint32_t magic_number;
    float kp;
    float ki;
    float kd;
    PID_Type_t pid_type;
} PID_Flash_Params_t;

static float g_kp = 1.0f;
static float g_ki = 0.1f;
static float g_kd = 0.01f;




void LoadPIDParamsFromFlash(void);
bool SavePIDParamsToFlash(PID_Type_t pid_type, float kp, float ki, float kd);
void SetPIDParams(PID_Type_t pid_type,float kp, float ki, float kd);
bool UART1_ParsePIDData(uint8_t *buf, uint16_t len, PID_UART_PARSE_Params_t *pid_params);

#endif // __UART_PID_PARSE_H
