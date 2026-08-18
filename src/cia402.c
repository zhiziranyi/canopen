#include "cia402.h"
#include "OD.h"
#include "encoder.h"
#include "motor.h"

/* ---------------- 电源状态机状态 ---------------- */
typedef enum {
    ST_NOT_READY_TO_SWITCH_ON = 0,
    ST_SWITCH_ON_DISABLED,
    ST_READY_TO_SWITCH_ON,
    ST_SWITCHED_ON,
    ST_OPERATION_ENABLED,
    ST_QUICK_STOP_ACTIVE,
    ST_FAULT_REACTION_ACTIVE,
    ST_FAULT
} cia402_state_t;

/* ---------------- 支持的操作模式 ---------------- */
#define MODE_NO_MODE        0
#define MODE_PROFILE_POS    1
#define MODE_PROFILE_VEL    3
#define MODE_HOMING         6

/* CiA 402 支持的驱动模式位：bit0=pp, bit2=pv, bit5=hm */
#define SUPPORTED_DRIVE_MODES (0x01u | 0x04u | 0x20u)

static cia402_state_t s_state = ST_NOT_READY_TO_SWITCH_ON;
static uint16_t s_prev_cw = 0;
static uint8_t s_prev_pp_bit4 = 0;
static uint16_t s_error_code = 0;
static uint8_t s_homing_done = 0;

static void update_statusword(void)
{
    uint16_t sw = 0;
    int32_t pos = motor_get_position();
    int32_t target = OD_RAM.x607A_targetPosition;
    float vel = motor_get_velocity();
    float vel_cmd = motor_get_velocity_cmd();

    switch (s_state) {
        case ST_SWITCH_ON_DISABLED:
            sw |= 0x0040;  /* bit6: switch on disabled */
            break;
        case ST_READY_TO_SWITCH_ON:
            sw |= 0x0021;  /* bit0 + bit5(quick stop) */
            break;
        case ST_SWITCHED_ON:
            sw |= 0x0023;  /* bit0+bit1+bit5 */
            break;
        case ST_OPERATION_ENABLED:
            sw |= 0x0027;  /* bit0+bit1+bit2+bit5 */
            break;
        case ST_QUICK_STOP_ACTIVE:
            sw |= 0x0007;  /* bit0+bit1+bit2 (quick stop) */
            break;
        case ST_FAULT:
        case ST_FAULT_REACTION_ACTIVE:
            sw |= 0x0008;  /* bit3: fault */
            break;
        default:
            break;
    }

    /* bit4: voltage enabled（除 switch on disabled / fault 外） */
    if (s_state != ST_SWITCH_ON_DISABLED && s_state != ST_FAULT && s_state != ST_NOT_READY_TO_SWITCH_ON) {
        sw |= 0x0010;
    }
    /* bit9: remote */
    sw |= 0x0200;

    /* bit10: target reached */
    if (s_state == ST_OPERATION_ENABLED) {
        if (OD_RAM.x6060_modesOfOperation == MODE_PROFILE_POS) {
            if ((pos > target - 4 && pos < target + 4) && (vel > -50.0f && vel < 50.0f)) {
                sw |= 0x0400;
            }
        } else if (OD_RAM.x6060_modesOfOperation == MODE_PROFILE_VEL) {
            if ((vel_cmd > -50.0f && vel_cmd < 50.0f) && (vel > -50.0f && vel < 50.0f)) {
                sw |= 0x0400;
            }
        } else if (OD_RAM.x6060_modesOfOperation == MODE_HOMING) {
            if (s_homing_done) sw |= 0x0400;
        }
    }

    OD_RAM.x6041_statusword = sw;
}

static void process_controlword(uint16_t cw)
{
    uint8_t fr_edge = ((cw & 0x80u) != 0u) && ((s_prev_cw & 0x80u) == 0u);
    s_prev_cw = cw;

    /* 故障复位：0->1 边沿 */
    if (s_state == ST_FAULT) {
        if (fr_edge) {
            if (motor_get_fault()) motor_clear_fault();
            s_error_code = 0;
            OD_RAM.x603F_errorCode = 0;
            s_state = ST_SWITCH_ON_DISABLED;
        }
        return;
    }
    if (fr_edge) return;

    /* bit1=0: disable voltage -> switch on disabled */
    if ((cw & 0x02u) == 0u) {
        s_state = ST_SWITCH_ON_DISABLED;
        return;
    }

    switch (s_state) {
        case ST_SWITCH_ON_DISABLED:
            if ((cw & 0x07u) == 0x06u) {           /* shutdown */
                s_state = ST_READY_TO_SWITCH_ON;
            }
            break;
        case ST_READY_TO_SWITCH_ON:
            if ((cw & 0x07u) == 0x07u) {           /* switch on */
                s_state = ST_SWITCHED_ON;
            }
            break;
        case ST_SWITCHED_ON:
            if ((cw & 0x0Fu) == 0x0Fu) {           /* enable operation */
                s_state = ST_OPERATION_ENABLED;
            } else if ((cw & 0x07u) == 0x06u) {    /* shutdown */
                s_state = ST_READY_TO_SWITCH_ON;
            }
            break;
        case ST_OPERATION_ENABLED:
            if ((cw & 0x04u) == 0u) {              /* quick stop */
                s_state = ST_QUICK_STOP_ACTIVE;
            } else if ((cw & 0x0Fu) == 0x07u) {    /* disable operation */
                s_state = ST_SWITCHED_ON;
            } else if ((cw & 0x07u) == 0x06u) {    /* shutdown */
                s_state = ST_READY_TO_SWITCH_ON;
            }
            break;
        case ST_QUICK_STOP_ACTIVE:
            if ((cw & 0x0Fu) == 0x0Fu) {           /* enable operation again */
                s_state = ST_OPERATION_ENABLED;
            }
            break;
        default:
            break;
    }
}

static void apply_mode_logic(void)
{
    uint8_t mode = OD_RAM.x6060_modesOfOperation;
    uint16_t cw = OD_RAM.x6040_controlword;
    uint8_t pp_bit4 = (cw >> 4) & 0x01u;

    /* halt 位（bit8） */
    if (cw & 0x0100u) {
        motor_stop();
    }

    switch (mode) {
        case MODE_PROFILE_POS:
            /* bit4 上升沿锁存目标位置 */
            if (pp_bit4 && !s_prev_pp_bit4) {
                motor_set_position_target(OD_RAM.x607A_targetPosition);
            }
            s_prev_pp_bit4 = pp_bit4;
            break;
        case MODE_PROFILE_VEL:
            motor_set_velocity_target((float)OD_RAM.x60FF_targetVelocity);
            break;
        case MODE_HOMING:
            if (pp_bit4 && !s_prev_pp_bit4) {
                encoder_set_position_zero();
                s_homing_done = 1;
            }
            s_prev_pp_bit4 = pp_bit4;
            break;
        default:
            s_prev_pp_bit4 = pp_bit4;
            motor_stop();
            break;
    }
}

void cia402_init(void)
{
    s_state = ST_NOT_READY_TO_SWITCH_ON;
    s_prev_cw = 0;
    s_prev_pp_bit4 = 0;
    s_error_code = 0;
    s_homing_done = 0;

    OD_RAM.x603F_errorCode = 0;
    OD_RAM.x6040_controlword = 0;
    OD_RAM.x6041_statusword = 0;
    OD_RAM.x6061_modesOfOperationDisplay = 0;
    OD_RAM.x6064_positionActualValue = 0;
    OD_RAM.x606C_velocityActualValue = 0;
    OD_RAM.x6502_supportedDriveModes = SUPPORTED_DRIVE_MODES;

    /* 上电自动进入 switch on disabled（对应 CiA402 上电流程） */
    s_state = ST_SWITCH_ON_DISABLED;
}

void cia402_process(void)
{
    uint16_t cw = OD_RAM.x6040_controlword;

    /* 电机层故障（过流等）上报到 CiA402 状态机 */
    if (motor_get_fault()) {
        cia402_raise_fault(0x2310u);   /* 过流 */
    }

    process_controlword(cw);

    /* 使能/禁止电机 */
    if (s_state == ST_OPERATION_ENABLED) {
        if (!motor_is_enabled()) {
            motor_enable();
        }
        apply_mode_logic();
    } else {
        if (motor_is_enabled()) {
            motor_disable();
        }
    }

    /* 刷新对象字典 */
    OD_RAM.x6064_positionActualValue = motor_get_position();
    OD_RAM.x606C_velocityActualValue = (int32_t)motor_get_velocity();
    OD_RAM.x6061_modesOfOperationDisplay = OD_RAM.x6060_modesOfOperation;
    update_statusword();
}

void cia402_raise_fault(uint16_t error_code)
{
    if (s_state == ST_FAULT) return;
    s_error_code = error_code;
    OD_RAM.x603F_errorCode = error_code;
    OD_RAM.x1001_errorRegister = 0x01u;   /* generic error */
    s_state = ST_FAULT;
    motor_disable();
}
