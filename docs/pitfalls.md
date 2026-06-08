# 项目重组编译坑点总结

## 环境

- MCU: STM32F103C8T6 (Cortex-M3)
- 构建: Keil MDK-ARM v5, ARM Compiler 5.06u7 (ARMCC5)
- 辅助工具: STM32CubeMX (`.ioc`), PowerShell 5.1, git

---

## 1. 编码灾难: GB2312 文件被 Set-Content 破坏

### 现象
`bsp_oled.c`, `bsp_oled_font.c` 等含中文注释/字模的文件，用 PowerShell `Set-Content` / `>` 重定向后，ARMCC5 报几十个语法错误，包括嵌套注释、缺失 `#endif`、类型不匹配等。

### 根因
- 这些文件是 **GB2312/GBK 编码**（不是 UTF-8）
- PowerShell `Set-Content` 默认转 UTF-8
- `> ` 重定向默认转 UTF-16LE
- ARMCC5 不支持 UTF-16LE，对 UTF-8 也不稳定
- 编码转换导致中文字节序列被破坏，部分字节对碰巧组成 `/*`，引发嵌套注释

### 解决方案
```powershell
# 错误: 会破坏编码
Set-Content -Path $file -Value $text

# 正确: 用 cmd 重定向保证原始字节不变
cmd /c "git show HEAD:path > output.c"

# 正确: 用字节级操作
$bytes = [System.IO.File]::ReadAllBytes($path)
$enc = [System.Text.Encoding]::GetEncoding(936)  # GBK code page
$text = $enc.GetString($bytes)
# ... 修改 ...
$bytes = $enc.GetBytes($text)
[System.IO.File]::WriteAllBytes($path, $bytes)

# 正确: 用 edit 工具 (逐字节精确匹配替换，不重新编码)
```

### 教训
- **绝对不要**用 PowerShell 文本工具编辑含中文的嵌入式源文件
- 用 `edit` 工具 (精确字符串替换) 或 `cmd /c` (原始二进制重定向)
- 需要修改时，从 git 提取原始二进制，用字节搜索+替换

---

## 2. CubeMX 重生成兼容性

### 问题
如果把 `main.c` `main.h` `MX_XXX_Init()` 移出自定义目录，CubeMX 重新生成时会:
- 在 `Core/Src/main.c` 重新创建文件（重复）
- 在新 `main.h` 中重新创建声明
- 不会更新已移走的文件

### 解决方案: "薄壳 main.c" 模式
```
Core/Src/main.c  → CubeMX 管控，仅保留 MX_XXX_Init() 实现 + 薄壳 main()
Core/Inc/main.h  → CubeMX 管控，USER CODE 区块保留

App/Src/app_init.c → App_Init() + App_MainLoop() (完全不受 CubeMX 影响)
BSP/ Lib/ Middlewares/ → 完全不受 CubeMX 影响
```

CubeMX 重生成时:
- `main.c`/`main.h` 被覆盖，但 USER CODE 块内的 `#include "app_init.h"` / `App_Init()` 自动保留
- **`stm32f1xx_hal_conf.h` 会无条件覆盖** → 需在 `.ioc` 中提前启用所有需要的 HAL 模块

---

## 3. 头文件 static 变量 = ODR 违规

### 问题文件
| 文件 | 违规 |
|------|------|
| `uart_pid_parse.h` | `static float g_kp = 1.0f;` |
| `pid_controller.h` | `static const PID_Type_Map_t pid_type_map[] = {...};` |
| `bsp_gnss.h` | `static uint8_t s_uart_rx_buf[1024];` |

每个包含该头文件的 `.c` 都会生成独立副本，浪费 RAM/Flash，且 `sizeof` 在 extern 数组上无法使用。

### 修复
- `.h` → 声明 `extern`
- `.c` → 定义实体
- 如果 `sizeof` 需要大小 → 头文件声明时指定数组维度: `extern const PID_Type_Map_t pid_type_map[3];`

---

## 4. ARMCC5 vs GCC 编译器差异

| 差异点 | ARMCC5 表现 | GCC 表现 | 影响 |
|--------|------------|----------|------|
| `strtof()` | 需要 `#include <stdlib.h>` | 同 | `uart_pid_parse.c` 编译报隐式声明 |
| `NULL` / `ptrdiff_t` | 需要 `#include <stddef.h>` | GCC 内置 | `sysmem.c` 报未定义 |
| `sys/stat.h` | 不存在 (bare-metal) | GCC newlib 提供 | `syscalls.c` 无法编译 |
| 嵌套注释 `/* /* */` | **错误** (#9-D) | 警告或忽略 | 含中文的 `/* */` 字节碰巧形成嵌套 |
| `sizeof(extern arr[])` | **错误** (incomplete type) | OK (linker 解析) | `pid_type_map` sizeof 失败 |
| microlib | 自带 `_sbrk` 堆管理 | 无 | 不需要 `sysmem.c` / `syscalls.c` |

### 修复总结
- `sysmem.c` / `syscalls.c`: ARMCC5 + microlib 项目应**从编译列表排除**
- 添加 `#include <stdlib.h>` (strtof)
- 添加 `#include <stddef.h>` (NULL/ptrdiff_t)
- `extern` 数组如需要 sizeof → 头文件中声明维度

---

## 5. FreeRTOS 依赖残留清理

### 问题
原项目 `uart_pid_parse.c`, `bsp_gnss.c`, `bsp_oled.c` 引用了 `FreeRTOS.h`, `semphr.h`, `cmsis_os.h`，但 FreeRTOS 源码目录完全缺失。

### 清理内容
| 文件 | 移除 |
|------|------|
| `uart_pid_parse.c` | `xSemaphoreCreateMutex`, `xSemaphoreTake/Give`, `portMAX_DELAY`, `SemaphoreHandle_t` |
| `bsp_gnss.c` | 同上，替换为直接访问 `volatile` 标志 |
| `bsp_oled.c` | `cmsis_os.h` include, `osMutexAttr_t` 结构体, `osMutexAcquire/Release` 注释代码 |

### Flash 写临界区替代
```c
// 旧: xSemaphoreTake(xParsePIDMutex, portMAX_DELAY);
// 新:
__disable_irq();
HAL_FLASH_Unlock();
// ... flash operations ...
HAL_FLASH_Lock();
__enable_irq();
```

---

## 6. git mv + Set-Content 的交互陷阱

### 问题
使用 `git mv old new` 后立即用 `Set-Content` 修改文件，导致:
1. 内容被 UTF-8 化（见坑点 #1）
2. 偶然引入多余字节 (如 bsp_gnss.h 的 \`\`\`\`\`\`\`\`)
3. `git checkout HEAD -- newpath` 会因为文件不在 HEAD commit 中而失败（staged 但未 commit）

### 教训
- `git mv` 后文件处于 staged 状态，HEAD 中仍以旧路径存在
- 如需恢复原始内容: `cmd /c "git show HEAD:旧路径 > 临时文件"`
- 修改文件用 `edit` 工具，避免 `Set-Content`

---

## 7. Include Path 管理

### 问题
`#include "eMPL/inv_mpu.h"` 在 Keil include path `../BSP/Src/eMPL` 下无法解析。

### 原因
ARMCC5 将 include path 作为搜索根目录，不会递归子目录。`"eMPL/inv_mpu.h"` 需要路径 `../BSP/Src` 才能找到 `eMPL/inv_mpu.h`。

### 解决方案
```c
// 方案A (推荐): 直接引用，因为 ../BSP/Src/eMPL 已在路径中
#include "inv_mpu.h"

// 方案B: 添加 ../BSP/Src 到 include path
// #include "eMPL/inv_mpu.h"
```

---

## 8. CubeMX 生成文件与工具链的宿命冲突

### 受影响文件
- `sysmem.c` / `syscalls.c`: CubeMX 为 GCC/newlib 生成，ARMCC5 用 microlib 不需要
- `stm32f1xx_hal_conf.h`: CubeMX **无条件覆盖**，无 USER CODE 保护

### 对策
- `sysmem.c` / `syscalls.c`: 从 Keil 编译列表排除，不在 `.uvprojx` 中引用
- `stm32f1xx_hal_conf.h`: 在 `.ioc` 的 Pinout → 外设列表 中启用所有使用的模块（GPIO/TIM/UART/DMA/SPI/FLASH/PWR），避免重生成后手动编辑

---

## 9. 恢复 git 文件后的清理盲区

### 问题
从 git HEAD 恢复 `bsp_oled.c` 后，虽然替换了 include 名，但**遗漏了源代码中嵌入的 RTOS 代码**:
```c
// 这段代码不在 #include 区域，而是嵌在函数体内:
const osMutexAttr_t mutex_attr = {
    .name = "OLED_Mutex",
    .attr_bits = osMutexRecursive
};
// oled_mutex = osMutexNew(&mutex_attr);
```

### 教训
- git restore 后必须**全文搜索** RTOS 相关符号: `osMutex`, `osSemaphore`, `cmsis_os`, `FreeRTOS`, `xSemaphore`
- 不止检查 include 区，还要检查函数体内的直接引用

---

## 编译通过后的残留 Warnings (无需处理)

| 文件 | Warning | 原因 |
|------|---------|------|
| `subtask.c` | `yaw_out` unreferenced | Yaw 控制被注释，保留变量供未来启用 |
| `car_model.c` | `Status`/`Temp` unreferenced | 遗留调试变量 |
| `stm32f1xx_it.c` | `rx_data` set but unused | CubeMX 生成的 DMA 接收变量模板 |
| `bsp_oled.c` | `enum mixed with another type` | HAL GPIO 函数的 PinState 参数类型兼容 |
