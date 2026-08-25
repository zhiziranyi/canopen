# CANopen CiA402 FOC 验证清单

## 自动化/构建检查

```powershell
powershell -ExecutionPolicy Bypass -File tools/run_native_tests.ps1
pio run -e black_f407zg
```

## 台架测试

| 项目 | 操作 | 通过判据 |
|---|---|---|
| 编码器 | 手转电机，读 SDO `6064h` | 位置连续变化，方向符合配置。 |
| Heartbeat | NMT Start 后监听总线 | 节点 ID 1 周期发送 Heartbeat。 |
| CiA402 | 依次写 `0x06/0x07/0x0F` | `6041h` 状态字按状态机转换。 |
| 速度模式 | 写 `6060h=3`、低速 `60FFh` | 电机低速平稳运行，可通过 606Ch 观察速度。 |
| 位置模式 | 写目标位置 | 电机在限速范围内趋向目标，无明显失控。 |
| 堵转保护 | 低风险条件短时阻挡 | 系统关 EN 并锁存预期故障，需按恢复策略重新使能。 |
| PDO | 发送 RPDO、监听 TPDO | 目标控制和反馈格式与 EDS/README 一致。 |

测试日志、CAN 抓包或上位机截图应作为硬件在环证据保存。未接 INA240 时不要将 `Iq`、电流环或硬件过流保护判定为已验证。
