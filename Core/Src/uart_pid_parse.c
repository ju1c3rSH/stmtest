#include "uart_pid_parse.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include <semphr.h>

static SemaphoreHandle_t xParsePIDMutex = NULL; // 丢去main吧
// For 103C8T6 ONLLYY!!
#define PID_PARAMS_FLASH_ADDR_BASE 0x0800FC00 // 64KB Flash的最后一页起始地址
#define PID_PAGE_SIZE 1024                    // STM32F103 Flash 页大小通常是 1KB
#define PID_MAGIC_NUMBER 0xDEADBEEF
#define MAX_PIDS_PER_PAGE (PID_PAGE_SIZE / sizeof(PID_Flash_Params_t)) // 约 1024/20=51个PID
// static const uint32_t PID_FLASH_ADDRESS = 0x0807F800;
/*后来发现是 调试用的串口线在捣鬼，因为usb转TTL的串口线中TX是有一定的驱动能力的，如果设备断电后，但是串口线还插着，那么这时候的flash就会被擦除(固件的flash不会被擦除，仅仅是模拟eeprom那部分的flash)。我只是观察到有这个现象，但是根本原因不详。另外，解决的办法就是串口的TX、RX接上拉电阻就行了。*/ // Example flash address (last page of 128KB flash)
void initPIDMutex(void)
{
    if (xParsePIDMutex == NULL)
    {
        xParsePIDMutex = xSemaphoreCreateMutex();
    }
}

bool UART1_ParsePIDData(uint8_t *buf, uint16_t len, PID_UART_PARSE_Params_t *pid_params)
{
    if (len < 12) // Minimum length check (3 floats = 12 bytes)
    {
        return false;
    }

    if (buf == NULL || pid_params == NULL || len == 0)
    {
        printf("[UART1_ParsePIDData]: Invalid input parameters.\n");
        return false;
    }

    char temp[UART1_PID_BUFFER_SIZE + 1];
    if (len > UART1_PID_BUFFER_SIZE)
    {
        printf("[UART1_ParsePIDData]: Buffer length %d exceeds temp buffer size %d. Truncating.\n", len, UART1_PID_BUFFER_SIZE);
        len = UART1_PID_BUFFER_SIZE - 1;
    }

    memcpy(temp, buf, len);
    temp[len] = '\0';
    char pid_type_str[20];
    float kp, ki, kd;

    if (sscanf(temp, "{\"type\":\"%19[^\"],\"kp\":%f,\"ki\":%f,\"kd\":%f}", pid_type_str, &kp, &ki, &kd) == 4)
    {
        pid_params->kp = kp;
        pid_params->ki = ki;
        pid_params->kd = kd;

        if (strcmp(pid_type_str, "balance_pitch") == 0)
        {
            pid_params->pid_type = PID_TYPE_BALANCE_PITCH;
        }
        else if (strcmp(pid_type_str, "balance_yaw") == 0)
        {
            pid_params->pid_type = PID_TYPE_BALANCE_YAW;
        }
        else if (strcmp(pid_type_str, "speed") == 0)
        {
            pid_params->pid_type = PID_TYPE_SPEED;
        }
        else
        {
            printf("Failed to parse PID type from JSON: %s\n", pid_type_str);
            return false;
        }

        printf("Parsed PID params from JSON - Kp: %.3f, Ki: %.3f, Kd: %.3f\n", pid_params->kp, pid_params->ki, pid_params->kd);
        return true;
    }
    else
    {
        printf("Failed to parse PID params from JSON: %s\n", temp);
        return false;
    }
}
bool SavePIDParamsToFlash(PID_Type_t pid_type, float kp, float ki, float kd)
{
    if (xParsePIDMutex == NULL)
    {
        printf("PID Mutex not initialized!\n");
        return false;
    }

    PID_Flash_Params_t page_buffer[MAX_PIDS_PER_PAGE];
    memcpy(page_buffer, (void *)PID_PARAMS_FLASH_ADDR_BASE, PID_PAGE_SIZE);

    bool found = false;
    int empty_slot = -1;
    for (int i = 0; i < MAX_PIDS_PER_PAGE; i++)
    {
        if (page_buffer[i].magic_number == PID_MAGIC_NUMBER && page_buffer[i].pid_type == pid_type)
        {
            page_buffer[i].kp = kp;
            page_buffer[i].ki = ki;
            page_buffer[i].kd = kd;
            found = true;
            printf("Updated PID params in page buffer for type %d: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid_type, kp, ki, kd);
            break;
        }
        else if (page_buffer[i].magic_number != PID_MAGIC_NUMBER && empty_slot == -1)
        {
            empty_slot = i;
        }
    }

    if (!found)
    {
        if (empty_slot != -1)
        {
            page_buffer[empty_slot].magic_number = PID_MAGIC_NUMBER;
            page_buffer[empty_slot].pid_type = pid_type;
            page_buffer[empty_slot].kp = kp;
            page_buffer[empty_slot].ki = ki;
            page_buffer[empty_slot].kd = kd;
            printf("Added new PID params to page buffer for type %d: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid_type, kp, ki, kd);
        }
        else
        {
            printf("Error: No space to save PID params for type %d. Page full.\n", pid_type);
            return false;
        }
    }

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        printf("Flash unlock failed!\n");
        return false;
    }

    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = PID_PARAMS_FLASH_ADDR_BASE;
    erase_init.NbPages = 1;

    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK)
    {
        printf("Flash erase failed!\n");
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t *data_ptr = (uint32_t *)page_buffer;
    int num_words = PID_PAGE_SIZE / sizeof(uint32_t);
    for (uint32_t i = 0; i < num_words; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PID_PARAMS_FLASH_ADDR_BASE + i * 4, data_ptr[i]) != HAL_OK)
        {
            printf("Flash program failed at address 0x%08lX!\n", (PID_PARAMS_FLASH_ADDR_BASE + i * 4));
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    printf("PID params for type %d saved to Flash: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid_type, kp, ki, kd);

    return true;}
void LoadPIDParamsFromFlash(void)
{
    if (xParsePIDMutex == NULL)
    {
        printf("PID Mutex not initialized!\n");
        return;
    }

    PID_Flash_Params_t page_buffer[MAX_PIDS_PER_PAGE];
    memcpy(page_buffer, (void *)PID_PARAMS_FLASH_ADDR_BASE, PID_PAGE_SIZE);

    int loaded_count = 0;
    for (int i = 0; i < MAX_PIDS_PER_PAGE; i++)
    {
        if (page_buffer[i].magic_number == PID_MAGIC_NUMBER)
        {
            PID_Type_t type = page_buffer[i].pid_type;
            if (type < PID_TYPE_COUNT)
            {
                if (xSemaphoreTake(xParsePIDMutex, portMAX_DELAY) == pdTRUE)
                {
                    g_stored_pid_params[type].Kp = page_buffer[i].kp;
                    g_stored_pid_params[type].Ki = page_buffer[i].ki;
                    g_stored_pid_params[type].Kd = page_buffer[i].kd;
                    xSemaphoreGive(xParsePIDMutex);
                    loaded_count++;
                    printf("Loaded PID params from Flash for type %d: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", type, page_buffer[i].kp, page_buffer[i].ki, page_buffer[i].kd);
                }
                else
                {
                    printf("Failed to take PID mutex during load for type %d!\n", type);
                }
            }
            else
            {
                printf("Warning: Invalid PID type %d found in Flash at index %d.\n", type, i);
            }
        }
    }
    if (loaded_count == 0)
    {
        printf("No valid PID params found in Flash. Using defaults.\n");
    }
    else
    {
        printf("Loaded %d PID params from Flash.\n", loaded_count);
    }
}

PID_UART_PARSE_Params_t GetPIDParams(void)
{
    PID_UART_PARSE_Params_t params;
    if (xParsePIDMutex == NULL)
    {
        printf("PID Mutex not initialized!\n");
        // Return default values
        params.kp = 1.0f;
        params.ki = 0.1f;
        params.kd = 0.01f;
        return params;
    }
    if (xSemaphoreTake(xParsePIDMutex, portMAX_DELAY) == pdTRUE)
    {
        params.kp = g_kp;
        params.ki = g_ki;
        params.kd = g_kd;
        xSemaphoreGive(xParsePIDMutex);
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