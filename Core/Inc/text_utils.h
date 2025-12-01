#ifndef __TEXT_UTILS_H
#define __TEXT_UTILS_H

#ifdef __cplusplus
extern "C"{
#endif


#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart1; 
extern SPI_HandleTypeDef hspi1;

void Log_Print(const char *message);
void SWO_PrintChar(char c);
void SWO_Print(const char *message);
void u1_printf(const char *format, ...);

#ifdef __cplusplus
}
#endif
#endif