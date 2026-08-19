# CANopen CiA402 FOC 伺服驱动系统

基于 STM32F407ZGT6 的 CANopen 伺服驱动器：CANopenNode v4 协议栈 + CiA402 驱动状态机 + FOC 电机控制。

硬件：2804 无刷电机 + SimpleFOC Mini（DRV8313 3-PWM）+ AS5600 磁编码器（I2C）+ TJA1050 CAN 收发器。当前部署未安装 INA240。

---

## 目录结构

```
canopen/
├── platformio.ini            # PlatformIO 工程配置（STM32Cube HAL）
├── ld/STM32F407ZGT6.ld       # 链接脚本（1MB Flash，0x08000000）
├── src/
│   ├── main.c                # 入口：HAL 初始化 + CANopen + 主循环
│   ├── board.c/h             # 时钟/GPIO/USART/CAN/TIM/I2C，ADC 按需初始化
│   ├── OD.h/.c               # 对象字典（DS301 + CiA402 + 运动参数）
│   ├── canopen_app.c/h       # CANopenNode 初始化与对象字典扩展
│   ├── cia402.c/h            # CiA402 电源状态机（pp/pv/hm 模式）
│   ├── motor.c/h             # 电机驱动：TIM1 PWM + 电压/速度/位置环 + 堵转保护
│   ├── stall_guard.c/h       # 无电流传感器时的编码器堵转保护
│   ├── foc.c/h               # FOC 数学：Clarke/Park/SVPWM/单相重构
│   ├── encoder.c/h           # AS5600 磁编码器
│   ├── pid.c/h               # 浮点 PID
│   ├── tft_display.c/h       # ST7789V 240x240 调试屏（照搬 cheji407）
│   ├── cn_font.c/h           # 16x16 中文字库（可选）
│   └── config.h              # 电机/FOC/PID 参数
├── lib/
│   ├── CANopenNode/          # CANopenNode v4 协议栈（Apache 2.0）
│   └── CO_STM32/             # STM32 bxCAN 驱动移植 + 协议栈裁剪配置
└── tools/
    ├── drive.eds             # EDS 电子数据表
    └── drive_test.py         # 上位机测试脚本（python-can + canopen）
```

## 硬件接线（与 开发参考方案.md 一致）

| 功能 | 引脚 | 外设 |
|------|------|------|
| U/V/W 相 PWM | PE9 / PE11 / PE13 | TIM1_CH1/2/3 -> SimpleFOC IN1/2/3 |
| 驱动使能 EN | PE14 | GPIO 输出 |
| nSLEEP | PC4 | GPIO 输出（必须拉高） |
| U 相电流 | PA3 | 可选 ADC1_IN3 <- INA240 OUT；当前未接，不初始化 ADC |
| 编码器 | PB6 / PB7 | I2C1 -> AS5600 |
| CAN | PD0 / PD1 | CAN1 -> TJA1050 |
| 调试串口 | PA9 / PA10 | USART1 115200 |
| TFT 屏 | PB3 / PB5 / PD11 / PD12 / PD13 | SPI3 SCK/SDA + CS/DC/BL |

> TFT 为 ST7789V 240x240，软排线接法与 cheji407 一致；引脚与伺服外设无冲突。
> 上电后 TFT 显示启动信息，之后每秒刷新一次状态（AS5600 / CANopen 状态 / 状态字 / 位置 / 速度 / 电流 / 故障码）。

## 构建与烧录

```bash
pio run                 # 编译
pio run -t upload       # ST-Link 烧录
```

本项目实际配置为 STM32 ROM 串口下载：`upload_protocol = serial`、`upload_port = COM3`。下载时将 BOOT0 置 1 后复位，执行 `pio run -t upload`；完成后将 BOOT0 置 0 并复位运行。

交付前还会运行不依赖硬件的控制算法测试：

```powershell
powershell -ExecutionPolicy Bypass -File tools/run_native_tests.ps1
```

晶振频率：`platformio.ini` 中 `HSE_VALUE=8000000`（8MHz，与 cheji407 板一致）。若 HSE 起振失败，程序会自动回退到内部 HSI 16MHz。

## CANopen 通信参数

- 节点 ID：0x01（编译期配置，见 `src/canopen_app.c`）
- 波特率：1 Mbps
- Heartbeat：1s
- RPDO1（0x200+0x01）：Controlword(6040h) + Target position(607Ah)，SYNC 触发
- RPDO2（0x300+0x01）：Controlword(6040h) + Target velocity(60FFh)，SYNC 触发
- TPDO1（0x180+0x01）：Statusword(6041h) + Position actual(6064h)，异步 1ms
- TPDO2（0x280+0x01）：Velocity actual(606Ch) + Mode display(6061h)，异步 10ms

> 注：参考文档中“TPDO1 包含 Statusword+Position+Velocity”为 10 字节，超过单帧 PDO 上限 8 字节，故拆分为两个 TPDO。

## CiA402 对象字典

| 索引 | 名称 | 类型 | 说明 |
|------|------|------|------|
| 603Fh | Error code | U16 | 故障码 |
| 6040h | Controlword | U16 | 状态机控制字 |
| 6041h | Statusword | U16 | 状态字 |
| 6060h | Modes of operation | I8 | 1=pp, 3=pv, 6=hm |
| 6061h | Modes display | I8 | 当前模式 |
| 6064h | Position actual | I32 | 编码器计数 |
| 606Ch | Velocity actual | I32 | counts/s |
| 607Ah | Target position | I32 | 轮廓位置目标 |
| 60FFh | Target velocity | I32 | 轮廓速度目标 |
| 6081h | Profile velocity | U32 | counts/s |
| 6083h | Profile acceleration | U32 | counts/s² |
| 6084h | Profile deceleration | U32 | counts/s² |
| 6502h | Supported drive modes | U32 | pp+pv+hm |

位置单位为编码器计数（4096/圈）；速度单位 counts/s。

## 控制架构

```
┌─────────────────────────────────────────────────┐
│              CiA402 状态机 (主循环)              │
│  控制字 -> 状态迁移 -> 使能/禁止电机/模式分发    │
└──────────────┬──────────────────────────────────┘
               │ 目标位置/速度
┌──────────────▼──────────────────────────────────┐
│       1kHz 速度/位置环 (TIM7 中断)               │
│   消费AS5600缓存 + 差分测速 -> 运动规划 -> 速度PI │
└──────────────┬──────────────────────────────────┘
               │ Vq / Iq*
┌──────────────▼──────────────────────────────────┐
│       20kHz 电压型 FOC (TIM1 更新中断)            │
│   电角度 -> 反Park -> SVPWM -> TIM1 CCR1/2/3      │
└─────────────────────────────────────────────────┘
```

- 默认无电流传感器电压型 FOC（`CURRENT_SENSE_PRESENT=0`、`FOC_CURRENT_LOOP_ENABLE=0`）：速度环直接输出 Vq；PA3/ADC1 不初始化，TFT 显示 `Iq: N/A`。
- 无 INA240 时最大 Vq 为 1.0 V；目标速度绝对值至少 250 counts/s、Vq 至少 0.5 V，但实际速度持续低于 50 counts/s 达 0.75 s 时，驱动自动关 EN 并锁存 `0x8611` 堵转故障。
- TIM1 使用普通 PWM 启动并单独开启更新中断；中心对齐模式每个完整 PWM 周期只执行一次 20kHz FOC。
- AS5600 运行期使用 I2C 中断采样，TIM7 ISR 不再执行最长 50ms 的阻塞式 I2C 读取。
- FOC 对齐会检查 TIM1 更新计数、编码器健康状态和转子实际位移；任一条件失败都会关闭 EN 并进入故障，不再打印假成功。
- 电流型 FOC（`FOC_CURRENT_LOOP_ENABLE=1`）只允许在 `CURRENT_SENSE_PRESENT=1` 时编译；需重新安装并标定 INA240。
- 单相电流重构：仅采样 U 相，按 B/C 相占空比分配估算另两相，低速/低占空比下精度有限，属成本优化方案。
- 未使用 FreeRTOS：CANopenNode 本身非阻塞可裸机运行（1ms 定时器 + 主循环），避免 RTOS 移植复杂度。

## 快速调试（上位机）

```bash
pip install python-can canopen pyserial
python tools/drive_test.py --port COM7 --cmd enable   # slcan 接口（USB-CAN 工具）
python tools/drive_test.py --port COM7 --cmd vel --value 5000
python tools/drive_test.py --port COM7 --cmd status
python tools/drive_test.py --port COM7 --cmd disable
```

接口类型可用 `--interface gs_usb / pcan / socketcan` 切换，取决于 USB-CAN 工具固件。

### PcanView / VIEW 手工帧

节点 1 的 SDO 请求 COB-ID 为 `0x601`，响应 COB-ID 为 `0x581`。先在 COB-ID `0x000` 发送 NMT Start 数据 `01 01`，再依次发送：

| 操作 | 0x601 数据（8 bytes） |
|---|---|
| 读状态字 6041h | `40 41 60 00 00 00 00 00` |
| Shutdown 6040h=0006h | `2B 40 60 00 06 00 00 00` |
| Switch on 6040h=0007h | `2B 40 60 00 07 00 00 00` |
| Enable operation 6040h=000Fh | `2B 40 60 00 0F 00 00 00` |
| 速度模式 6060h=03h | `2F 60 60 00 03 00 00 00` |
| 目标速度 60FFh=1000 | `23 FF 60 00 E8 03 00 00` |

`C:\Users\13957\Desktop\固件源码工具\view\PcanView.exe` 是 PEAK PcanView；它通常要求 PCAN 兼容适配器。若 UCC-T01 无法被它识别，应使用适配器附带的 VIEW/串口 CAN 软件发送同样的标准帧。

## 首次上电调试步骤

1. 烧录固件，串口观察启动信息与 AS5600 是否识别。
2. CAN 工具读 0x1008/0x1018 确认节点上线，观察 Heartbeat。
3. 空载手转电机，读 6064h 确认编码器计数连续变化、方向正确。
4. 发送控制字 0x06 -> 0x07 -> 0x0F，每步读 6041h，预期基础状态依次为 0x0231、0x0233、0x0237。
5. 设置 6060h=3，再给目标 60FFh=500 验证转向；首次测试必须使用限流电源。测试完成先写 60FFh=0，再写 6040h=0 禁用。
6. 速度环 PID 从极小增益开始整定，再切位置模式调位置 P。
7. 若需要真实电流显示、过流保护或电流型 FOC，重新安装 INA240，并将 `CURRENT_SENSE_PRESENT` 改为 1 后再标定。

## 参考项目

- [CANopenNode](https://github.com/CANopenNode/CANopenNode)（Apache 2.0 协议栈）
- [CanOpenSTM32](https://github.com/CANopenNode/CanOpenSTM32)（STM32 HAL 移植）
- [OpenBLT](https://github.com/feaser/openblt)（Bootloader 参考，本项目未使用）
