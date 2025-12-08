#ifndef __UART_PID_PARSE_H__
#define __UART_PID_PARSE_H__
#define UART1_PID_BUFFER_SIZE 256
#include <stdint.h>
static uint8_t s_uart1_rx_buf[UART1_PID_BUFFER_SIZE];

typedef struct {
    uint32_t buffer[UART1_PID_BUFFER_SIZE];
    float kp;
    float ki;
    float kd;
    // uint32_t checkSum;

} PID_Params_t;
static float g_kp = 1.0f;
static float g_ki = 0.1f;
static float g_kd = 0.01f;


void LoadPIDParamsFromFlash(void);
void SavePIDParamsToFlash(float kp, float ki, float kd);
void UART1_ParsePIDData(uint8_t* buf, uint16_t len, PID_Params_t* pid_params);
#endif // __UART_PID_PARSE_H
