#include "uart_pid_parse.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "text_utils.h"
#include "car_model.h"

float g_kp = 1.0f;
float g_ki = 0.1f;
float g_kd = 0.01f;
// For 103C8T6 ONLLYY!!
#define PID_PARAMS_FLASH_ADDR_BASE 0x0800FC00
#define PID_PAGE_SIZE 1024
#define PID_MAGIC_NUMBER 0xDEADBEEF
#define MAX_PIDS_PER_PAGE (PID_PAGE_SIZE / sizeof(PID_Flash_Params_t))

/*
bool UART1_ParsePIDData(uint8_t *buf, uint16_t len, PID_UART_PARSE_Params_t *pid_params)
{
    //测试用json:{"type":"balance_pitch","kp":850.0,"ki":0.0,"kd":2.6}
    // 发送时记得加换行（\n�?
    if (len < 12) // Minimum length check (3 floats = 12 bytes)
    {
        return false;
    }

    if (buf == NULL || pid_params == NULL || len == 0)
    {
        u1_printf("[UART1_ParsePIDData]: Invalid input parameters.\n");
        return false;
    }

    char temp[PID_UART1_RX_BUF_SIZE + 1];
    if (len > PID_UART1_RX_BUF_SIZE)
    {
        u1_printf("[UART1_ParsePIDData]: Buffer length %d exceeds temp buffer size %d. Truncating.\n", len, PID_UART1_RX_BUF_SIZE);
        len = PID_UART1_RX_BUF_SIZE - 1;
    }
    /
    代码审计�?
    UART1_ParsePIDData 中重复拷�?
    UartRecvTask 已拷贝到 local_buffer
    UART1_ParsePIDData 又拷贝一次到 temp
    浪费内存�?CPU
    /
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
            u1_printf("Failed to parse PID type from JSON: %s\n", pid_type_str);
            return false;
        }

        u1_printf("Parsed PID params from JSON - Kp: %.3f, Ki: %.3f, Kd: %.3f\n", pid_params->kp, pid_params->ki, pid_params->kd);
        return true;
    }
    else
    {
        u1_printf("Failed to parse PID params from JSON: %s\n", temp);
        return false;
    }
}
*/
bool UART1_ParsePIDData(uint8_t *buf, uint16_t len, PID_UART_PARSE_Params_t *pid_params)
{
    if (buf == NULL || pid_params == NULL || len == 0 || len > PID_UART1_RX_BUF_SIZE)
        return false;

    char *json = (char *)buf;

    // 1. 解析 type
    char *type_start = strstr(json, "\"type\":\"");
    if (!type_start)
        return false;
    type_start += 8; // 跳过 "\"type\":\""
    char *type_end = strchr(type_start, '"');
    if (!type_end)
        return false;
    int type_len = type_end - type_start;
    if (type_len >= 20)
        return false;
    char type_str[20] = {0};
    memcpy(type_str, type_start, type_len);

    char *kp_start = strstr(json, "\"kp\":");
    if (!kp_start)
        return false;
    float kp = strtof(kp_start + 5, NULL);

    char *ki_start = strstr(json, "\"ki\":");
    if (!ki_start)
        return false;
    float ki = strtof(ki_start + 5, NULL);

    char *kd_start = strstr(json, "\"kd\":");
    if (!kd_start)
        return false;
    float kd = strtof(kd_start + 5, NULL);

    pid_params->kp = kp;
    pid_params->ki = ki;
    pid_params->kd = kd;
    /*
    if (strcmp(type_str, "balance_pitch") == 0)
        pid_params->pid_type = PID_TYPE_BALANCE_PITCH;
    else if (strcmp(type_str, "balance_yaw") == 0)
        pid_params->pid_type = PID_TYPE_BALANCE_YAW;
    else if (strcmp(type_str, "speed") == 0)
        pid_params->pid_type = PID_TYPE_SPEED;
    else
        return false;
        */
    for (int i = 0; i < sizeof(pid_type_map) / sizeof(pid_type_map[0]); i++)
    {
        if (strcmp(type_str, pid_type_map[i].name) == 0)
        {
            pid_params->pid_type = pid_type_map[i].type;
            return true;
        }
    }
    printf("Parsed PID: type=%s, Kp=%.3f, Ki=%.3f, Kd=%.3f\n", type_str, kp, ki, kd);
    return true;
}
bool SavePIDParamsToFlash(PID_Type_t pid_type, float kp, float ki, float kd)
{
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

    __disable_irq();

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        printf("Flash unlock failed!\n");
        __enable_irq();
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
        __enable_irq();
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
            __enable_irq();
            return false;
        }
    }
    HAL_FLASH_Lock();
    __enable_irq();
    printf("PID params for type %d saved to Flash: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", pid_type, kp, ki, kd);

    return true;
}
void LoadPIDParamsFromFlash(void)
{
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
                g_stored_pid_params[type].Kp = page_buffer[i].kp;
                g_stored_pid_params[type].Ki = page_buffer[i].ki;
                g_stored_pid_params[type].Kd = page_buffer[i].kd;

                loaded_count++;
                printf("Loaded PID params from Flash for type %d: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", type, page_buffer[i].kp, page_buffer[i].ki, page_buffer[i].kd);
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
    params.kp = g_kp;
    params.ki = g_ki;
    params.kd = g_kd;
    return params;
}
/*
void Callback_ParsePID(uint8_t *buf, uint16_t len)
{
    Log_Print("\r\n---START---\r\n", 15);
    Log_Print((char *)buf, len);
    Log_Print("\r\n---END---\r\n", 15);

    if (len > 0 && len <= PID_UART1_RX_BUF_SIZE)
    {
        Parse_(buf, len);
    }
}
    */
