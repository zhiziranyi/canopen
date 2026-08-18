# CANopen CiA402 FOC 伺服驱动重构设计

**日期：** 2026-08-18  
**目标硬件：** STM32F407ZGT6、SimpleFOC Mini/DRV8313（三 PWM）、2804 七对极电机、AS5600、INA240A1、TJA1050  
**验收工具：** `C:\Users\13957\Desktop\固件源码工具\view\PcanView.exe`（以及实际可识别该 USB-CAN 适配器的兼容工具）

## 目标与现状

目标是在不改变现有引脚分配和串口下载流程的前提下，让 CANopen/CiA402 控制的电机能够稳定受控转动，并保留故障安全行为。当前基线可编译，AS5600 能被识别，屏幕显示 `Status=0x0240`（Switch On Disabled + Remote），但电机完全无动作。

已确认的第一根因是：`motor_init()` 使用 `HAL_TIM_PWM_Start_IT()` 启动 TIM1 三个 PWM 通道。该 HAL API 只打开 CC1/CC2/CC3 中断，不打开更新中断；而现有 FOC 代码挂在 `HAL_TIM_PeriodElapsedCallback()`，因此 `motor_current_loop_isr()` 不会执行。对齐流程没有检查更新事件计数，导致可能打印成功但没有实际 PWM。

第二个必要条件是 CiA402 标准使能。上电默认保持 `Switch On Disabled`，VIEW 或其他 CANopen 主站必须依次写入 `6040h=0006h`、`0007h`、`000Fh`，然后再设置模式和目标。固件不自动上电驱动电机。

## 方案范围

### 保留的部分

- CANopenNode v4 协议栈、STM32 bxCAN 驱动和现有对象字典索引。
- STM32F407ZGT6、168 MHz 时钟、1 Mbps CAN、节点 ID 1。
- PE9/PE11/PE13 PWM、PE14 EN、PC4 nSLEEP、PA3 ADC、PB6/PB7 I2C、PD0/PD1 CAN、PA9/PA10 调试串口。
- 串口下载配置；最终固件必须通过 PlatformIO 编译后再交付。

### 重构的部分

- TIM1 PWM 与更新中断启动顺序。
- FOC/速度/位置环的调度边界与共享状态。
- AS5600 采样接口，禁止在高频控制 ISR 内执行不可控的阻塞式 I2C。
- 对齐成功判据、DRV8313 使能和故障停机路径。
- CiA402 状态迁移、模式切换和对象字典同步的测试覆盖。
- 诊断日志、主机 CAN 验收步骤和单元测试。

## 架构与数据流

```text
CAN RX ISR/协议栈
        │
        ▼
CANopenNode 1 ms process ──► OD RAM (6040/6060/607A/60FF)
        │                                  │
        ▼                                  ▼
CiA402 状态机（主循环） ───────► Motor command snapshot
        │                                  │
        ├────────────── 1 kHz control tick ┘
        │                编码器采样 → 位置/速度环 → Vq/Iq target
        ▼
TIM1 update ISR（20 kHz）
    读取稳定快照 → 电角度 → Clarke/Park → PI/电压型 Vq → SVPWM → CCR1/2/3
        │
        ├── ADC 注入采样完成：更新 U 相电流与过流状态
        └── 保护动作：清零 PWM、拉低 EN、锁存故障
```

### 中断与任务规则

1. `HAL_TIM_PWM_Start()` 只负责打开 PWM 输出；随后显式调用 `HAL_TIM_Base_Start_IT(&htim1)` 打开 TIM1 更新中断。中心对齐模式配置重复计数器，使每个完整 PWM 周期只产生一次控制更新；只在 `HAL_TIM_PeriodElapsedCallback()` 调用 `motor_current_loop_isr()`。
2. TIM1 ISR 不执行 CANopen、不执行 `HAL_Delay()`、不发串口、不做动态内存分配。它只处理固定时间预算内的数学、PWM 写入和保护。
3. 1 kHz 控制周期不在 ISR 内启动无界等待的 I2C 事务。编码器驱动提供“启动采样/完成更新/读取最近有效值”接口；若总线失败，保留最后有效角度并累计错误计数，超过阈值进入故障。
4. 主循环以非阻塞方式调用 `canopen_app_process()`、`cia402_process()` 和参数同步。OD 与 ISR 共享的值通过快照或临界区保护。
5. 所有 PWM 占空比都限制在 `[0, 1]`，比较值按 `ARR+1` 归一化，禁用时清零比较值并拉低 EN；对齐时 EN 先拉高且必须确认 TIM1 更新计数在增长。

## FOC 和运动控制行为

- 第一阶段保持电压型 FOC（`FOC_CURRENT_LOOP_ENABLE=0`），速度环输出限幅后的 `Vq`；INA240 仅用于监控和过流保护。
- FOC 对齐在固定电压矢量下采集两个稳定机械角，计算方向与零点；若 PWM 更新计数、编码器更新计数或角度变化不满足条件，打印明确原因并保持驱动关闭。
- 电角度只在一个控制周期内计算一次，避免重复 I2C/浮点读取。
- 速度单位继续使用 AS5600 counts/s，位置单位继续使用 counts；目标速度按 `VELOCITY_LIMIT_CPS` 限幅。
- 轮廓位置模式保留 CiA402 PP 接口，但规划器的时间和距离计算使用 64 位中间值并处理零速度、零加速度、小距离和反向目标，避免无符号下溢。
- CiA402 模式仅接受 PP(1)、PV(3)、HM(6)。不支持模式写入不会启动电机，并通过错误码/诊断日志说明原因。

## CiA402 和安全行为

- 上电状态：`Switch On Disabled`，状态字至少为 `0x0240`。
- 标准迁移：`0x0006 → 0x0007 → 0x000F`；缺少任一步不会拉高驱动 EN。
- `Quick Stop`、`Disable Operation`、`Shutdown`、通信复位和无效模式均使目标归零并关闭 PWM/EN。
- 过流、编码器连续失败、ADC 越界或 CAN 总线严重错误进入 Fault；`603Fh` 写入 CiA402 错误码，`1001h` 置通用错误，`6041h` 置 bit3。
- 故障复位只接受控制字 bit7 的上升沿；复位后回到 Switch On Disabled，不自动重新转动。
- 看门狗刷新放在主循环健康路径；主循环卡死时不继续保持驱动使能。

## 测试和验收

### 主机可执行单元测试

为不依赖 STM32 HAL 的 `foc.c`、`pid.c`、CiA402 状态转换和编码器回绕逻辑添加 native 测试，至少覆盖：

- PID 输出和积分限幅。
- Clarke/Park 与逆变换的基本不变量。
- SVPWM 输出范围和零输入约 50% 占空比。
- 0/4095 回绕产生 ±1 附近的增量，而不是大跳变。
- `0006/0007/000F` 状态序列、急停、故障复位边沿。
- 位置规划的零距离、小距离、正向和反向目标。

### 固件构建验收

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
```

交付前必须看到 exit code 0；同时记录 Flash/RAM 使用量，不以“代码看起来正确”代替编译结果。

### VIEW/CAN 现场验收

1. 设置 CAN 1 Mbps，确认节点 1 的 Boot-up/Heartbeat。
2. 读取 `1008h`、`1018h`，确认设备身份。
3. 读 `6041h`，上电应为 `0x0240`。
4. 写 `6040h=0006h`、`0007h`、`000Fh`，逐步读回 `6041h`，确认状态迁移。
5. 写 `6060h=03h`、`60FFh` 小速度正值，观察 `606Ch` 和 `6064h` 变化，再测试负值。
6. 记录 TPDO/Heartbeat、EN 引脚、PWM 输出和故障码；任何过流或编码器错误都应关闭 EN。

若 `PcanView.exe` 无法识别 UCC-T01 适配器，使用该适配器随附的 VIEW/兼容 CAN 工具完成同样的 CAN 帧验收，不改变固件协议。

## 交付与版本管理

- 在本地初始化 Git 仓库，提交设计、测试和代码变更。
- 配置用户提供的 GitHub 远程仓库后推送；远程地址、分支名和认证方式以用户环境实际配置为准，不在源码中保存凭据。
- 每次交付前先执行全量编译和测试，再报告结果；编译失败时只报告实际错误，不宣称完成。
