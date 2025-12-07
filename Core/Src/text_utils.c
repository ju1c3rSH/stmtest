#include <stdarg.h >
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "text_utils.h"


void Log_Print(const char *message, uint16_t len)
{
    if (len > 0) {
        HAL_UART_Transmit(&huart1, (uint8_t *)message, len, HAL_MAX_DELAY);
    }
}
void SWO_PrintChar(char ch)
{
  ITM_SendChar(ch);
}
void SWO_Print(const char *message)
{
  while (*message)
  {
    SWO_PrintChar(*message++);
  }
}

void u1_printf(const char *format, ...)
{
  char buffer[256];
  va_list args;

  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  Log_Print(buffer,strlen(buffer));
}
