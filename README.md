# SJTU-DE-ICECS 固件说明

基于 **STM32L432KBU6** 的阻抗与温湿度采集节点固件。程序通过 AD5933 获取阻抗数据，通过 SHT20/SHT2x 读取温湿度，使用 USART1 与外接 ZigBee 模块交换数据，并支持串口在线调整测量频率和 PWM 占空比。

本文依据当前文件夹中的源码、`SJTU-DE-ICECS.ioc` 和构建配置整理。硬件接线以实际电路原理图为准；本文不代表已完成编译、烧录或实板精度验证。

## 1. 功能与默认参数

- **阻抗测量**：连续扫频或固定频率重复采样，输出经相位补偿后的阻抗实部和虚部。
- **温湿度采集**：每个测量点同时读取温度和相对湿度。
- **数据上报**：每条测量数据编码为一行 JSON，经 USART1 发送至 ZigBee 模块。
- **在线控制**：根据节点 ID 接收文本命令，切换测量模式、修改频率或调整 PWM 占空比。
- **任务调度**：使用 FreeRTOS、CMSIS-RTOS V1、消息队列和互斥锁组织采集、发送及命令处理。

| 项目 | 当前默认值 | 配置位置 |
| --- | --- | --- |
| MCU / 系统时钟 | STM32L432KBU6 / 80 MHz | `.ioc`、`Core/Src/main.c` |
| 节点 ID | 字符串 `0014` | `Core/Src/freertos.c`：`APP_NODE_ID` |
| 测量模式 | 扫频，完成后从起始频率重新开始 | `gAD5933Config` |
| 扫频范围 / 步长 | 5–100 kHz / 1 kHz，共 96 点 | `gAD5933Config` |
| 预置定频值 | 10 kHz，需切换至定频模式后使用 | `gAD5933Config` |
| AD5933 时钟设置 | 内部时钟，驱动按 16 MHz 计算频率寄存器 | `AD5933.h`、`AppTask_AD5933()` |
| AD5933 激励 / PGA | `AD5933_RANGE_1000mVpp` / ×1 | `AppTask_AD5933()` |
| 温湿度分辨率 | `RES_12_8`：温度 12 bit、湿度 8 bit | `AppTask_AD5933()` |
| USART1 | 38400 baud、8 数据位、无校验、1 停止位、无硬件流控 | `Core/Src/usart.c` |
| PWM | TIM1_CH1，1 kHz，上电占空比 30% | `Core/Src/tim.c`、`Core/Src/main.c` |
| ZigBee 启动配置 | PAN ID `0x0397`（919），节点类型 `0x02` | `AppTask_ZigBee()` |

> `tim.c` 和 `.ioc` 中的 PWM 初始比较值为 500，但 `main.c` 在启动 PWM 前将其改为 300，因此实际启动设置为 **30%**，不是 50%。

## 2. 硬件接口

下表是 MCU 引脚名，不是外部连接器针脚编号。

| 接口 / 信号 | MCU 引脚 | 源码配置与用途 |
| --- | --- | --- |
| AD5933 I2C1 SCL | PA9 | 开漏；`.ioc` 配置为 100 kHz |
| AD5933 I2C1 SDA | PA10 | AD5933 的 7 位地址为 `0x0D` |
| SHT2x I2C3 SCL | PA7 | 开漏；`.ioc` 配置为 50 kHz |
| SHT2x I2C3 SDA | PB4 | SHT2x 的 7 位地址为 `0x40` |
| USART1 TX | PB6 | MCU 发送，连接通信模块 RX |
| USART1 RX | PB7 | MCU 接收，连接通信模块 TX；逐字节中断接收命令 |
| PWM TIM1_CH1 | PA8 | 高电平有效的 PWM 输出 |
| ZigBee 唤醒控制 / P0_1 | PA4 | `ZIGBEE_TRANS(1)` 将引脚拉低；发送任务启动后保持该状态 |
| P0_0 | PA0 | 当前实际配置为推挽输出、初始低电平；虽定义了 `ZIGBEE_RECV_READY` 别名，但未用于输入握手 |
| 状态 LED | PA5 | 默认任务每 250 tick 翻转一次；当前 tick 为 1 ms |
| SWDIO / SWCLK | PA13 / PA14 | 下载与调试接口 |
| LSE | PC14 / PC15 | 当前程序启用外部低速晶振，并用于 MSI 自动校准 |

接线与上电前注意：

- 两路 I2C 均配置为 `GPIO_NOPULL`，需要板上适当的上拉电阻。HAL 调用中的设备地址由驱动左移一位，不要重复移位。
- 确认 MCU、传感器、串口模块的供电和逻辑电平兼容，并共地。直连 USB 转串口时，应避免转接器与 ZigBee 模块同时驱动 MCU 的 RX。
- PA8 会在调度器启动前输出 30% 占空比 PWM。连接实际负载前先确认驱动电路及允许工作范围。
- PA0 的名称与实际 GPIO 方向容易混淆，不能仅凭 `RECV_READY` 名称将其接作模块输出握手线。
- 若硬件没有配置所需 LSE 晶振，应先调整时钟初始化；时钟初始化失败会进入 `Error_Handler()`。

## 3. 目录结构

```text
SJTU-DE-ICECS/
├── Core/
│   ├── Inc/                         应用、驱动和 FreeRTOS 头文件
│   ├── Src/
│   │   ├── main.c                   时钟、外设、PWM 和调度器启动
│   │   ├── freertos.c               采集任务、ZigBee 发送、命令解析、JSON 上报
│   │   ├── AD5933.c                 AD5933 寄存器访问与阻抗计算
│   │   ├── Communication.c          AD5933 的 STM32 HAL I2C 适配
│   │   ├── sht2x_for_stm32_hal.c     温湿度传感器驱动
│   │   ├── sht2x_for_stm32_hal_example.c  保留的示例，未加入当前 CMake 目标
│   │   ├── cJSON.c                  JSON 序列化库
│   │   ├── gpio.c / i2c.c / tim.c / usart.c
│   │   └── stm32l4xx_it.c           SysTick、USART1 等中断入口
│   └── Startup/                     原 CubeIDE 工程启动文件
├── cmake/
│   ├── gcc-arm-none-eabi.cmake       交叉编译工具链、编译和链接选项
│   └── stm32cubemx/CMakeLists.txt    HAL、FreeRTOS 及生成代码的构建清单
├── SJTU-DE-ICECS.ioc                CubeMX 外设配置
├── CMakeLists.txt                  应用构建入口
├── CMakePresets.json               Debug / Release 等预设
├── startup_stm32l432xx.s            当前 CMake 使用的启动文件
├── STM32L432XX_FLASH.ld             当前 CMake 使用的链接脚本
├── STM32L432KBUX_FLASH.ld           保留的 CubeIDE 链接脚本
├── .project / .cproject             保留的 CubeIDE 工程配置
├── .vscode/                        本地构建、J-Link 烧录和调试配置
├── Drivers/                        不是完整的 HAL / CMSIS / FreeRTOS 依赖副本
├── build/Debug/                    CMake 构建输出，不应作为源码分发依赖
└── Debug/                          原 CubeIDE 构建输出
```

## 4. 编译与下载

### 4.1 构建依赖

当前 `.ioc` 的目标工具链为 **CMake**，记录的 CubeMX 版本为 **6.14.0**，固件包为 **STM32Cube FW_L4 V1.18.2**。构建需要：

- CMake 3.22 或更高版本，以及 Ninja。
- Arm GNU Toolchain，确保 `arm-none-eabi-gcc`、`arm-none-eabi-g++`、`arm-none-eabi-objcopy` 等工具在构建环境的 `PATH` 中。
- STM32CubeL4 固件包 V1.18.2，包含 HAL、CMSIS 和 FreeRTOS。
- 如需下载，还需要与实际探针匹配的下载工具和驱动。

**先处理外部依赖路径：** `cmake/stm32cubemx/CMakeLists.txt` 直接引用以下绝对路径：

```text
C:/Users/niwangze/STM32Cube/Repository/STM32Cube_FW_L4_V1.18.2/
```

在其他电脑上，需要将该文件中的依赖路径统一调整到实际安装位置，或备份后用 CubeMX 重新生成并核对差异。仅复制本项目文件夹不足以带齐依赖。修改 CubeMX 配置后，应检查自定义任务、PWM 启动、USART1 中断和应用源文件清单是否保留。

### 4.2 CMake 命令

在工程根目录打开配置好上述工具的终端：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Release 构建：

```powershell
cmake --preset Release
cmake --build --preset Release
```

如果使用 STM32Cube 集成终端中的 `cube-cmake`，可将上述命令中的 `cmake` 替换为 `cube-cmake`。普通终端不一定能直接找到该工具；本地 `.vscode/settings.json` 使用的就是 `cube-cmake`。

Debug 目标输出为 `build/Debug/SJTU-DE-ICECS.elf` 和同目录 `.map` 文件，另有用于代码索引的 `compile_commands.json`。当前 CMake 没有自动生成 HEX/BIN 的步骤，如有需要可手动转换：

```powershell
arm-none-eabi-objcopy -O ihex build/Debug/SJTU-DE-ICECS.elf build/Debug/SJTU-DE-ICECS.hex
arm-none-eabi-objcopy -O binary build/Debug/SJTU-DE-ICECS.elf build/Debug/SJTU-DE-ICECS.bin
```

当前工具链使用 Cortex-M4 硬浮点 ABI、C11 和 newlib-nano；根 `CMakeLists.txt` 已添加 `-u _printf_float`，用于 cJSON 浮点数字输出。移植构建配置时应保留这一链接选项。

### 4.3 下载与调试

本地 `.vscode/tasks.json` 提供 `Build Debug` 和 `Flash MCU (J-Link, no debug)` 任务，后者使用 J-Link、SWD、设备名 `STM32L432KB`，并依赖前者先构建。首次使用前必须先完成 CMake 配置。

迁移或使用其他探针前检查：

1. `.vscode/tasks.json` 中 J-Link 可执行文件路径是本机绝对路径。
2. `.vscode/flash.jlink` 中 `loadfile` 也是绝对路径，应指向本次构建的 ELF。
3. `.vscode/launch.json` 中名为 `Debug with JLink` 的配置仍有空设备名及 `./bin/executable.elf` 占位路径，不能直接当作可用调试配置。
4. 若使用 ST-LINK，可在相应下载工具中连接 SWD 并选择 ELF/HEX；选择 BIN 时，应明确其加载地址。当前链接脚本的 Flash 起始地址为 `0x08000000`。
5. 当前两份链接脚本都声明 Flash 为 256K、RAM 为 48K、RAM2 为 16K。**这只是工程声明，应对照实际芯片完整型号核验容量后再下载**。

保留的 `.project` / `.cproject` 仍引用 **STM32CubeL4 V1.18.1** 及旧链接资源路径，与当前 CMake 配置并不一致。若使用 CubeIDE，应先统一依赖、源文件清单、启动文件和链接脚本，不能直接认为两套工程配置等价。`.vscode/` 已被当前 `.gitignore` 忽略，单独通过 Git 获取工程时未必包含这些本地配置。

## 5. 启动流程与任务

`main()` 依次初始化 HAL、系统时钟、GPIO、I2C1、USART1、I2C3 和 TIM1，设置并启动 30% PWM，然后创建 RTOS 对象并启动调度器。采集无需上位机先发送启动命令。

| 任务 | 优先级 | 职责 |
| --- | --- | --- |
| `defaultTask` | Normal | 翻转状态 LED |
| `AD5933_Task` | Normal | 初始化 AD5933/SHT2x，读取测量配置，采样、相位补偿并生成 JSON |
| `ZigBee_Task` | High | 拉低唤醒脚，发送模块配置帧，随后从发送队列取出并发送消息 |
| `Command_Task` | AboveNormal | 每约 20 ms 检查待处理命令，校验参数并更新测量配置或 PWM |

测量数据和命令回复共用长度为 16 的 `xJsonQueue`，由发送任务统一经 USART1 发送。测量配置由互斥锁保护，并通过版本号通知采集任务切换；命令返回成功表示配置已更新，采集任务可能在当前阻塞操作结束后才采用新配置。

**采样周期不是固定 250 ms。** 代码在每次采样后延时 250 tick，但还存在 AD5933 状态轮询、温湿度读取、额外延时和队列等待；`Communication.c` 每次 I2C 写操作还包含 `HAL_Delay(100)`。不要直接按 4 Hz 解释数据或推算一轮扫频耗时。

## 6. 串口命令

### 6.1 发送规则

- 串口设置：**38400，8N1，无硬件流控**。
- 一行一条文本命令，以 `CR`、`LF` 或 `CRLF` 结束；选项间使用空格或制表符。
- 每条 `set` 命令必须恰好包含一个 `-id <节点ID>`，并与 `APP_NODE_ID` 完全匹配。`0014` 和 `14` 不相同；其他节点的命令会被静默忽略。
- `help` 或 `?` 不要求 ID。命令名和选项区分大小写。
- 接收行缓冲为 96 字节，正文最多 95 字节；分词最多 12 项，包含命令名、选项及其值。
- 当前仅保留一条待处理命令。应逐条发送、等待回复，避免连续粘贴多条；不支持分号分隔执行或带引号的复杂参数。

> 源码启动提示和帮助示例仍使用 `0011`，但当前默认节点为 **`0014`**。以下示例按当前默认 ID 编写；修改节点 ID 后也要相应修改命令。

### 6.2 常用命令

```text
help
set -o 0 -s 5 -e 100 -i 1 -id 0014
set -o 1 -q 10 -id 0014
set -q 20 -id 0014
set -pwm 0.3 -id 0014
set -pwm 0.5 -id 0014
```

以上各行分别表示：查看帮助；5–100 kHz、步长 1 kHz 扫频；10 kHz 定频；切换至 20 kHz 定频；设置 30% PWM；设置 50% PWM。每次只发送需要执行的那一行。

| 参数 | 含义与限制 |
| --- | --- |
| `-id` | 目标节点字符串 ID，所有 `set` 命令必填 |
| `-o 0` / `-o 1` | 扫频 / 定频模式 |
| `-q` | 定频频率，单位 kHz，只接受正整数 |
| `-s` / `-e` / `-i` | 扫频起点 / 终点 / 步长，单位 kHz，只接受正整数 |
| `-pwm` | 占空比比例，范围 0–1；`0.3` 表示 30%，不是 0.3% |

扫频要求终点不小于起点，且 `(终点 - 起点)` 能被步长整除；增量数最多 511，对应最多 512 个测量点。只给 `-q` 会自动选择定频，只给扫频参数会自动选择扫频；未指定的频率参数沿用当前值，同时混用两类频率参数时需要显式给出 `-o`。即使选择定频，解析器也会校验保留的扫频参数是否合法。

PWM 命令应单独发送，不与频率参数混写。占空比解析到千分之一并四舍五入；当前计数周期为 1000，对应 0.1 个百分点的占空比步进。命令只改占空比，不改 PWM 频率。

成功回复示例：

```text
CMD OK: mode=sweep start=5000 Hz end=100000 Hz step=1000 Hz points=96
CMD OK: mode=fixed freq=10000 Hz
CMD OK: pwm duty=30.0% pulse=300
```

频率命令的输入单位是 **kHz**，上述回复单位是 **Hz**。错误一般以 `CMD ERR:` 开头。参数仅保存在 RAM，复位后恢复源码默认设置；当前没有保存配置、停止采集或在线校准命令。

## 7. 上报数据格式

测量数据为单行 JSON，末尾追加 `\r\n`。下面仅用于展示格式，不是实测数据：

```json
{"ID":"0014","Freq":10,"Zr":12345.6,"Zi":-789.1,"T":25.2,"H":48.6}
```

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `ID` | 字符串 | 节点 ID，保留前导零 |
| `Freq` | 数值 | 当前频率，单位 kHz；由内部 Hz 值整数除以 1000 得到 |
| `Zr` | 数值 | 经相位补偿计算的阻抗实部，单位 Ω，准确度取决于标定 |
| `Zi` | 数值 | 经相位补偿计算的阻抗虚部，单位 Ω，符号取决于相位约定与标定 |
| `T` | 数值 | 驱动换算后的温度，单位 °C |
| `H` | 数值 | 驱动换算后的相对湿度，单位 %RH |

扫频和定频使用相同字段。数据没有时间戳、序号、扫频轮次编号或传感器状态字段，上位机需要自行记录接收时间和分组。

USART1 同时承载 JSON、`CMD READY` / `CMD HELP` / `CMD OK` / `CMD ERR` 文本，以及启动时发给 ZigBee 模块的二进制配置帧。因此上位机应区分消息类型、验证 JSON，不能对串口每一行都直接按测量 JSON 解析。

## 8. 标定与实现边界

### 8.1 阻抗与相位

`Core/Src/freertos.c` 使用固定增益因子和线性相位补偿：

```text
gainFactor = 4.3675e-9
phase_comp_deg = 7.53601e-4 × frequency_Hz - 90.46298
phase_corrected_deg = phase_raw_deg - phase_comp_deg
Zr = |Z| × cos(phase_corrected_deg × π / 180)
Zi = |Z| × sin(phase_corrected_deg × π / 180)
```

这些数值应针对实际模拟前端、标准阻抗、激励幅度、PGA 和工作频段核验，不能直接视为不同板卡通用的标定结果。驱动虽然提供 `AD5933_CalculateGainFactor()`，当前采集任务没有执行在线校准。JSON 中的 `Zr` / `Zi` 是应用层用幅值和补偿相位重新计算的结果。

频率解析器只检查正整数及扫频点数等条件，未完整限制器件可用频率范围或极大整数溢出。不要把“命令接受”当作“硬件能够准确测量”，应使用经过板卡验证的频率范围。

### 8.2 温湿度

当前 SHT2x 驱动的实际换算公式为：

```text
T = -56.85 + 175.72 × raw_temperature / 65536
H =   4.00 + 125.00 × raw_humidity    / 65536
```

源码未说明上述偏移常数的标定依据。正式使用前应确认它们是否包含实验修正，并用实际传感器资料和标准仪器复核。当前读取了第三个返回字节但没有校验 CRC，也未明确屏蔽原始值状态位或向上层传递 I2C 错误；不能仅凭输出了数字就判定温湿度采集正常。

### 8.3 其他边界

- AD5933 多处等待数据有效的循环没有总超时，底层 I2C 错误也未完整向上传递。设备异常可能导致测量停滞或无效输出。
- ZigBee 配置帧对应当前代码所用模块的协议，未提供通用模块适配或确认应答流程；PAN ID 和节点类型字节不能直接套用到任意型号。虽然节点类型注释为低功耗终端，当前固件保持唤醒脚状态，也没有实现完整的 MCU 低功耗策略。
- `Communication.c` 中 SPI 接口仅为占位实现；当前采集实际使用 I2C。
- FreeRTOS 堆设置为 30000 字节；cJSON 默认使用 C 库分配器，发送缓冲使用 `pvPortMalloc()`。扩展任务或消息大小时需要同时关注两类内存分配。

## 9. 常见问题

| 现象 | 优先检查 |
| --- | --- |
| 找不到 HAL / FreeRTOS 头文件或源文件 | V1.18.2 固件包是否存在；CMake 中绝对路径是否已更新 |
| 找不到 `cmake`、`cube-cmake`、Ninja 或编译器 | 是否在正确的构建终端中运行；工具是否加入 PATH |
| 换目录后 CMake 缓存报错 | 使用新的构建目录重新配置，避免沿用其他电脑或路径生成的缓存 |
| 串口无输出或乱码 | 38400 / 8N1、TX/RX、共地、电平、模块供电与网络配置 |
| `set` 命令无回复 | ID 是否为当前节点 `0014`，是否发送行结束符，是否命令发送过快 |
| JSON 数值缺失 | 构建链接选项是否保留 `-u _printf_float` |
| 有 LED 闪烁但没有测量数据 | I2C 接线、上拉、地址、AD5933 状态等待及传感器供电 |
| 测量比预期慢 | 底层 I2C 写延时、状态轮询、传感器读取及发送队列等待 |
| 温湿度或阻抗偏差明显 | 增益因子、相位公式、温湿度偏移常数及实际模拟前端标定 |
| 烧录后仍表现为旧程序 | 是否选择了本次构建的 `build/Debug/` 产物，而非旧 `Debug/` 目录文件 |

建议首次联调依次检查：电源与接口 → 构建依赖 → 烧录目标 → LED/PWM → 串口帮助 → 默认扫频 JSON → 定频命令 → 标准阻抗及温湿度校验。仅收到 `CMD OK` 不等于传感器数据已验证。

## 10. 第三方代码与维护说明

项目包含 ST HAL/CMSIS、FreeRTOS、Analog Devices AD5933/Communication 驱动、cJSON 1.7.14 及 eepj 的 SHT2x HAL 驱动。各组件保留各自源码中的版权和许可说明；根目录未提供统一的项目级 LICENSE，本文不额外授予授权。

本次文档整理仅新增 `README.md`，不修改固件和构建配置；未执行重新编译、烧录及实板测试。目录中已有的 ELF/MAP 属于已有构建产物，不作为当前源码已通过验证的依据。
