# CANopen CiA402 FOC 伺服驱动系统交接文档

更新时间：2026-08-25  
目标：STM32F407ZGT6 + CANopenNode v4 + SimpleFOC Mini + 2804 + AS5600 + TJA1050

## 项目状态和边界

该仓库是 CANopen/CiA402 伺服驱动台架工程。电机侧默认运行**无电流传感器电压型 FOC**，并用速度/位置环与编码器堵转判定补充安全控制。当前未安装 INA240，因此电流闭环、真实电流读数和硬件过流保护不属于默认验收范围。

## 架构

```text
CANopen NMT/SDO/PDO -> CiA402 state machine -> target position/velocity
                                            -> 1 kHz motion/velocity loop
AS5600 I2C interrupt -> angle cache ------> 20 kHz voltage FOC -> TIM1 3-PWM
TJA1050 CAN <------------------------------------ status/heartbeat/PDO
```

## 日常开发流程

1. 使用 `tools/run_native_tests.ps1` 运行不依赖硬件的控制算法测试。
2. 用 `pio run -e black_f407zg` 构建；ROM 串口下载时设置 BOOT0 并使用 COM3。
3. 空载读取 `6064h`，先确认编码器连续性和方向。
4. 按 `6040h: 0x06 -> 0x07 -> 0x0F` 驱动 CiA402 状态机，再从低速度目标开始。
5. 发生异常时先将速度目标置零、写禁用控制字，必要时断开驱动电源；不要在堵转状态继续增大 PID 或 Vq。

## 安全风险

- 三相驱动电源、电机和逻辑地接法必须符合驱动板要求；CANH/CANL 只能通过 TJA1050 等收发器连接。
- 真实电机测试必须使用限流电源、可靠机械固定和可触及的断电措施。
- 任何高压电机、实际车辆或人机协作场景都需要项目外的安全设计与验证，本仓库不提供此类认证。

## 仓库卫生

提交源码、EDS、构建脚本、测试与文档；不提交 `.pio/`、本机 USB-CAN 配置、私有密钥或现场数据。
