#include "atgm336h.h"

#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include <string.h>
#include "semphr.h"
#include <stdbool.h>
#include "text_utils.h"
#include <stdlib.h> 
SemaphoreHandle_t xGnssMutex = NULL;
// GPS_PropTypeDef gps_props = {0};
static GPS_PropTypeDef *s_active_gps = NULL;
static const GPS_PropTypeDef s_default_gps = {
    .latitude = 0.0f,
    .longitude = 0.0f,
    .altitude = 0.0f,
    .speed = 0.0f,
    .heading = 0.0f,
    .fix_type = 0,
    .satellites = 0,
    .UTC_time = {0},
    .SYS_time = {0},
    .EW = 'E',
    .NS = 'N',
    .GPS_status = 'V',
    .GPS_Buffer = {0},
    .is_available = 0};

static volatile bool s_uart_idle_flag = false;
static volatile uint16_t s_uart_rx_len = 0;
static void extract_gnrmc_line(const uint8_t *buf, uint16_t len)
{
    char line[128] = {0};
    uint16_t i = 0, j = 0;

    for (i = 0; i < len; i++)
    {
        if (buf[i] == '\n')
        {
            line[j] = '\0';
            // 检查是否以 "$GNRMC" 开头
            if (strncmp(line, "$GNRMC", 6) == 0)
            {
                parse_gnrmc(line);
            }
            j = 0;
        }
        else if (buf[i] != '\r' && j < sizeof(line) - 1)
        {
            line[j++] = (char)buf[i];
        }
    }
}

void parse_uart_buffer(uint8_t *buf, uint16_t len)
{
    extract_gnrmc_line(buf, len);
}

void parse_gnrmc(const char *nmea)
{    // 此处简化 直接 strtok

    char copy[128];
    strncpy(copy, nmea, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';

    char *fields[14];
    int field_count = 0;
    char *token = strtok(copy, ",");

    while (token && field_count < 14)
    {
        fields[field_count++] = token;
        token = strtok(NULL, ",");
    }

    if (field_count < 12)
        return;

    if (fields[2][0] != 'A')
    {
        xSemaphoreTake(xGnssMutex, portMAX_DELAY);
        s_active_gps->is_available = 0;
        xSemaphoreGive(xGnssMutex);
        return;
    }

    GPS_PropTypeDef tmp = {0};
    strncpy(tmp.UTC_time, fields[1], sizeof(tmp.UTC_time) - 1);
    tmp.GPS_status = fields[2][0];

    if (fields[3] && strlen(fields[3]) > 0)
    {
        float lat = atof(fields[3]);
        int deg = (int)(lat / 100);
        float min = lat - deg * 100;
        tmp.latitude = deg + min / 60.0f;
        if (fields[4][0] == 'S')
        {
            tmp.latitude = -tmp.latitude;
            tmp.NS = 'S';
        }
        else
        {
            tmp.NS = 'N';
        }
    }


    if (fields[5] && strlen(fields[5]) > 0)
    {
        float lon = atof(fields[5]);
        int deg = (int)(lon / 100);
        float min = lon - deg * 100;
        tmp.longitude = deg + min / 60.0f;
        if (fields[6][0] == 'W')
        {
            tmp.longitude = -tmp.longitude;
            tmp.EW = 'W';
        }
        else
        {
            tmp.EW = 'E';
        }
    }

    tmp.is_available = 1;
    /// tmp.SYS_time = HAL_GetTick();


    if (xSemaphoreTake(xGnssMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        *s_active_gps = tmp;
        xSemaphoreGive(xGnssMutex);
    }
}

void ATGM336H_Init(GPS_PropTypeDef *gps_prop)
{
    if (xGnssMutex == NULL)
    {
        xGnssMutex = xSemaphoreCreateMutex();
        configASSERT(xGnssMutex != NULL);
    }
    s_active_gps = gps_prop;
    if (gps_prop != NULL)
    {
        *gps_prop = s_default_gps;
    }
}

void GPS_UART_IDLE_Callback(uint8_t *buf, uint16_t len)
{
    Log_Print("\r\n---START---\r\n", 15); 
    Log_Print((char*)buf, len); 
    Log_Print("\r\n---END---\r\n", 15);
    
    if (len > 0 && len <= GPS_UART_RX_BUF_SIZE) {
        extract_gnrmc_line(buf, len);
    }
}