/**
 * @file    main.c
 * @brief   CANopen CiA402 FOC 伺服驱动 应用入口
 *
 * 编译位置：0x08000000（1MB Flash 起始）
 */
#include "board.h"
#include "CO_app_STM32.h"
#include "canopen_app.h"
#include "cia402.h"
#include "encoder.h"
#include "motor.h"
#include "OD.h"
#include "tft_display.h"
#include <stdio.h>

static void sync_od_to_motor(void)
{
    /* 将对象字典中的运动参数同步到电机层（简单轮询） */
    motor_set_profile(OD_RAM.x6081_profileVelocity,
                      OD_RAM.x6083_profileAcceleration,
                      OD_RAM.x6084_profileDeceleration);
}

/* ---------------- TFT 调试屏 ---------------- */
static int s_enc_ok = 0;
static uint32_t s_boot_count = 0;
static uint32_t s_hf_flag = 0;

/* 在固定 x 位置画定宽文本，旧内容被背景色覆盖，避免残留 */
static void tft_value(uint16_t y, uint16_t color, const char* s)
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%-15s", s);
    TFT_SetTextColor(color, COLOR_BLACK);
    TFT_DrawString(90, y, buf);
}

static void tft_static_layout(void)
{
    /* 自检：依次闪红/绿/蓝，确认 SPI 通路与屏工作 */
    TFT_FillScreen(COLOR_RED);
    HAL_Delay(300);
    TFT_FillScreen(COLOR_GREEN);
    HAL_Delay(300);
    TFT_FillScreen(COLOR_BLUE);
    HAL_Delay(300);

    TFT_FillScreen(COLOR_BLACK);
    TFT_SetTextColor(COLOR_WHITE, COLOR_BLACK);
    TFT_DrawString(0, 0, "== CANopen FOC Servo ==");
    TFT_DrawString(0, 20, "AS5600:");
    TFT_DrawString(0, 40, "CANopen:");
    TFT_DrawString(0, 60, "Node:");
    TFT_DrawString(0, 84, "Status:");
    TFT_DrawString(0, 104, "Pos:");
    TFT_DrawString(0, 124, "Vel:");
    TFT_DrawString(0, 144, "Iq:");
    TFT_DrawString(0, 164, "Fault:");
    TFT_DrawString(90, 60, "0x01  1Mbps");
    TFT_DrawString(0, 184, "Boot:");
    TFT_DrawString(0, 200, "HF:");
}

static void tft_status_update(void)
{
    char buf[24];
    uint16_t sw = OD_RAM.x6041_statusword;
    int32_t iq_ma = (int32_t)(motor_get_current_iq() * 1000.0f);

    s_enc_ok = encoder_is_healthy();
    tft_value(20, s_enc_ok ? COLOR_GREEN : COLOR_RED, s_enc_ok ? "OK" : "WARN");

    /* 驱动状态按状态字判定（错误寄存器会包含无总线时的通讯错误位，不代表真实故障） */
    if (sw & 0x0008u) {
        tft_value(40, COLOR_RED, "FAULT");
    } else if (sw & 0x0004u) {
        tft_value(40, COLOR_GREEN, "OPERATION");
    } else if (sw & 0x0001u) {
        tft_value(40, COLOR_YELLOW, "READY");
    } else {
        tft_value(40, COLOR_WHITE, "DISABLED");
    }

    snprintf(buf, sizeof(buf), "0x%04X", (unsigned)sw);
    tft_value(84, COLOR_WHITE, buf);

    snprintf(buf, sizeof(buf), "%ld", (long)motor_get_position());
    tft_value(104, COLOR_WHITE, buf);

    snprintf(buf, sizeof(buf), "%ld", (long)motor_get_velocity());
    tft_value(124, COLOR_WHITE, buf);

    snprintf(buf, sizeof(buf), "%ldmA", (long)iq_ma);
    tft_value(144, (iq_ma > 500 || iq_ma < -500) ? COLOR_RED : COLOR_GREEN, buf);

    snprintf(buf, sizeof(buf), "0x%04X", (unsigned)OD_RAM.x603F_errorCode);
    tft_value(164, (OD_RAM.x603F_errorCode != 0u) ? COLOR_RED : COLOR_WHITE, buf);

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)s_boot_count);
    tft_value(184, COLOR_WHITE, buf);

    snprintf(buf, sizeof(buf), "%lu", (unsigned long)s_hf_flag);
    tft_value(200, s_hf_flag ? COLOR_RED : COLOR_GREEN, buf);
}

int main(void)
{
    HAL_Init();

    /* 与 cheji407 一致：先配时钟，立刻初始化 TFT */
    SystemClock_Config();
    TFT_Init();
    tft_static_layout();

    board_init();

    dbg_printf("\r\n========================================\r\n");
    dbg_printf("  CANopen CiA402 FOC Servo Drive v1.0\r\n");
    dbg_printf("  Node ID: 0x01  Baud: 1Mbps\r\n");
    dbg_printf("========================================\r\n");

    /* 重启诊断：备份寄存器 1 = 上电次数，2 = HardFault 标志 */
    s_boot_count = bkp_read(1) + 1u;
    bkp_write(1, s_boot_count);
    s_hf_flag = bkp_read(2);
    dbg_printf("[DIAG] boot=%lu hf=%lu\r\n",
               (unsigned long)s_boot_count, (unsigned long)s_hf_flag);

    dbg_printf("[TFT] init done\r\n");

    if (encoder_init() != 0) {
        dbg_printf("[WARN] AS5600 not responding (check I2C wiring)\r\n");
        s_enc_ok = 0;
    } else {
        dbg_printf("[OK] AS5600 encoder ready\r\n");
        s_enc_ok = 1;
    }

    if (motor_init() != 0) {
        dbg_printf("[FAULT] motor init failed: %s\r\n",
                   motor_fault_name(motor_get_fault()));
    } else if (s_enc_ok) {
        /* FOC 上电对齐校准：电机会短暂跳动对齐（正常现象） */
        dbg_printf("[FOC] aligning rotor...\r\n");
        if (motor_align_foc() != 0) {
            dbg_printf("[FAULT] FOC alignment failed: %s\r\n",
                       motor_fault_name(motor_get_fault()));
        }
    }

    app_canopen_init();
    cia402_init();
    board_watchdog_init();

    dbg_printf("[OK] CANopen node started, waiting for NMT/controlword...\r\n");
    tft_status_update();

    uint32_t last_sync = 0;
    uint32_t last_hb = 0;
    uint32_t last_led = 0;
    uint32_t last_watchdog = HAL_GetTick();
    uint32_t previous_control_updates = motor_get_control_update_count();
    uint32_t previous_velocity_ticks = motor_get_velocity_tick_count();
    uint8_t led_state = 0;
    while (1) {
        canopen_app_process();
        cia402_process();

        uint32_t now = HAL_GetTick();
        if (now - last_sync >= 5) {
            last_sync = now;
            sync_od_to_motor();
        }
        if (now - last_watchdog >= 100u) {
            uint32_t control_updates = motor_get_control_update_count();
            uint32_t velocity_ticks = motor_get_velocity_tick_count();
            last_watchdog = now;
            if (control_updates != previous_control_updates
                && velocity_ticks != previous_velocity_ticks) {
                board_watchdog_refresh();
            }
            previous_control_updates = control_updates;
            previous_velocity_ticks = velocity_ticks;
        }
        /* 每秒心跳：用于确认串口与固件正常运行 */
        if (now - last_hb >= 1000) {
            last_hb = now;
            int32_t iq_ma = (int32_t)(motor_get_current_iq() * 1000.0f);
            int32_t iu_ma = (int32_t)(motor_get_current_u() * 1000.0f);
            int32_t vq_mv = (int32_t)(motor_get_voltage_cmd() * 1000.0f);
            dbg_printf("[T] sw=%04X cw=%04X mode=%d tgt=%ld pos=%ld vel=%ld "
                       "vq=%ldmV iu=%ldmA iq=%ldmA fault=%d ctrl=%lu encErr=%lu canErr=%08lX\r\n",
                       (unsigned)OD_RAM.x6041_statusword,
                       (unsigned)OD_RAM.x6040_controlword,
                       (int)OD_RAM.x6061_modesOfOperationDisplay,
                       (long)OD_RAM.x60FF_targetVelocity,
                       (long)motor_get_position(),
                       (long)motor_get_velocity(),
                       (long)vq_mv,
                       (long)iu_ma,
                       (long)iq_ma,
                       (int)motor_get_fault(),
                       (unsigned long)motor_get_control_update_count(),
                       (unsigned long)encoder_get_error_count(),
                       (unsigned long)app_canopen_error_status());
            tft_status_update();
        }
        /* 板载 LED 500ms 翻转：固件运行指示（不依赖串口） */
        if (now - last_led >= 500) {
            last_led = now;
            led_state = !led_state;
            HAL_GPIO_WritePin(PIN_LED0_GPIO, PIN_LED0_PIN,
                              led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(PIN_LED1_GPIO, PIN_LED1_PIN,
                              led_state ? GPIO_PIN_RESET : GPIO_PIN_SET);
        }
    }
}
