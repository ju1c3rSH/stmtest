#include "uart_pid_parse.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
static SemaphoreHandle_t xPIDMutex = NULL;

static const uint32_t PID_FLASH_ADDRESS = 0x0807F800; // Example flash address (last page of 128KB flash)
void initPIDMutex(void)
{
    if (xPIDMutex == NULL)
    {
        xPIDMutex = xSemaphoreCreateMutex();
    }
}

bool UART1_ParsePIDData(uint8_t *buf, uint16_t len, PID_Params_t *pid_params)
{
    if (len < 12) // Minimum length check (3 floats = 12 bytes)
    {
        return;
    }

    char temp[UART1_PID_BUFFER_SIZE + 1];
    if (len > UART1_PID_BUFFER_SIZE)
    {
        len = UART1_PID_BUFFER_SIZE - 1;
    }

    memcpy(temp, buf, len);
    temp[len] = '\0';

    if (sscanf(temp, "{\"kp\":%f,\"ki\":%f,\"kd\":%f}", &pid_params->kp, &pid_params->ki, &pid_params->kd) == 3)
    {
        printf("Parsed PID params from JSON - Kp: %.3f, Ki: %.3f, Kd: %.3f\n", pid_params->kp, pid_params->ki, pid_params->kd);
        return true;
    }
    else
    {
        printf("Failed to parse PID params from JSON: %s\n", temp);
        return false;
    }

    return false;
}

void SavePIDParamsToFlash(float kp, float ki, float kd)
{
    // 加上互斥锁

    if (xPIDMutex == NULL)
    {
        printf("PID Mutex not initialized!\n");
        return;
    }

    PID_Flash_Params_t flash_params;
    flash_params.magic_number = 0xDEADBEEF; // Example magic number
    flash_params.kp = kp;
    flash_params.ki = ki;
    flash_params.kd = kd;

    HAL_FLASH_Unlock();

    // Erase the flash sector before writing
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = PID_FLASH_ADDRESS; // 用最后一页
    erase_init.NbPages = 1;

    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK)
    {
        printf("Flash erase failed!\n");
        HAL_FLASH_Lock();
        return;
    }

    // Write the data to flash

    uint32_t *data_ptr = (uint32_t *)&flash_params;
    int num_words = sizeof(PID_Flash_Params_t) / sizeof(uint32_t);
    for (uint32_t i = 0; i < num_words; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PID_FLASH_ADDRESS + i * 4, data_ptr[i]) != HAL_OK)
        {
            printf("Flash program failed at address 0x%08lX!\n", (address + i * 4));
            HAL_FLASH_Lock();
            return;
        }
    }

    HAL_FLASH_Lock();
    printf("PID params saved to Flash: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", kp, ki, kd);
}

void LoadPIDParamsFromFlash(void)
{
    // MUTEX,ALSOHEHE

    if (xMutex == NULL)
    {
        printf("PID Mutex not initialized!\n");
        return;
    }
    PID_Flash_Params_t flash_params;
    memcpy(&flash_params, (uint32_t *)PID_FLASH_ADDRESS, sizeof(PID_Flash_Params_t));

    if (flash_params.magic_number == 0xDEADBEEF)
    {
        if(xSemaphoreTake(xPIDMutex, portMAX_DELAY) == pdTRUE)
        {
            g_kp = flash_params.kp;
            g_ki = flash_params.ki;
            g_kd = flash_params.kd;
            xSemaphoreGive(xPIDMutex);
        }
        else
        {
            printf("Failed to take PID mutex!\n");
            return;
        }
        g_kp = flash_params.kp;
        g_ki = flash_params.ki;
        g_kd = flash_params.kd;
        printf("Loaded PID params from Flash: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", g_kp, g_ki, g_kd);
    }
    else
    {
        printf("No valid PID params found in Flash. Using defaults.\n");
    }
}

PID_Params_t GetPIDParams(void)
{
    PID_Params_t params;
    if(xPIDMutex == NULL)
    {
        printf("PID Mutex not initialized!\n");
        // Return default values
        params.kp = 1.0f;
        params.ki = 0.1f;
        params.kd = 0.01f;
        return params;
    }
    if (xSemaphoreTake(xPIDMutex, portMAX_DELAY) == pdTRUE)
    {
        params.kp = g_kp;
        params.ki = g_ki;
        params.kd = g_kd;
        xSemaphoreGive(xPIDMutex);
    }
    else
    {
        // If mutex take fails, return default values
        params.kp = 1.0f;
        params.ki = 0.1f;
        params.kd = 0.01f;
    }
    return params;
}