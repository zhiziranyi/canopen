#include "cia402.h"

#include "OD.h"
#include "cia402_sm.h"
#include "encoder.h"
#include "motor.h"

#define MODE_PROFILE_POS 1
#define MODE_PROFILE_VEL 3
#define MODE_HOMING 6
#define SUPPORTED_DRIVE_MODES (0x01u | 0x04u | 0x20u)

static cia402_sm_t s_power_sm;
static uint8_t s_prev_pp_bit4 = 0u;
static uint8_t s_prev_fault_reset_bit = 0u;
static uint8_t s_homing_done = 0u;
static uint8_t s_external_fault = 0u;
static int8_t s_previous_mode = 0;

static uint16_t motor_fault_error_code(motor_fault_t fault)
{
    switch (fault) {
        case MOTOR_FAULT_OVERCURRENT:
            return 0x2310u;
        case MOTOR_FAULT_ENCODER:
            return 0x7300u;
        case MOTOR_FAULT_INIT:
        case MOTOR_FAULT_CONTROL_TIMING:
        case MOTOR_FAULT_ALIGNMENT:
            return 0xFF01u;
        case MOTOR_FAULT_NONE:
        default:
            return 0u;
    }
}

static void set_error(uint16_t error_code)
{
    OD_RAM.x603F_errorCode = error_code;
    OD_RAM.x1001_errorRegister = (error_code != 0u) ? 0x01u : 0x00u;
}

static void apply_mode_logic(void)
{
    int8_t mode = OD_RAM.x6060_modesOfOperation;
    uint16_t controlword = OD_RAM.x6040_controlword;
    uint8_t pp_bit4 = (uint8_t)((controlword >> 4) & 0x01u);

    if (mode != s_previous_mode) {
        motor_stop();
        s_homing_done = 0u;
        s_prev_pp_bit4 = pp_bit4;
        s_previous_mode = mode;
        if (mode == MODE_PROFILE_VEL) {
            OD_RAM.x60FF_targetVelocity = 0;
        }
    }

    if (!cia402_mode_supported(mode)) {
        OD_RAM.x6061_modesOfOperationDisplay = 0;
        s_prev_pp_bit4 = pp_bit4;
        motor_stop();
        return;
    }
    OD_RAM.x6061_modesOfOperationDisplay = mode;

    if ((controlword & 0x0100u) != 0u) {
        motor_stop();
        s_prev_pp_bit4 = pp_bit4;
        return;
    }

    switch (mode) {
        case MODE_PROFILE_POS:
            if (pp_bit4 != 0u && s_prev_pp_bit4 == 0u) {
                motor_set_position_target(OD_RAM.x607A_targetPosition);
            }
            break;
        case MODE_PROFILE_VEL:
            motor_set_velocity_target((float)OD_RAM.x60FF_targetVelocity);
            break;
        case MODE_HOMING:
            if (pp_bit4 != 0u && s_prev_pp_bit4 == 0u) {
                encoder_set_position_zero();
                s_homing_done = 1u;
            }
            break;
        default:
            motor_stop();
            break;
    }
    s_prev_pp_bit4 = pp_bit4;
}

static void update_statusword(void)
{
    uint16_t statusword = cia402_sm_statusword(&s_power_sm);
    int8_t mode = OD_RAM.x6060_modesOfOperation;

    if (cia402_sm_operation_enabled(&s_power_sm)) {
        if (mode == MODE_PROFILE_POS) {
            int64_t position_error = (int64_t)OD_RAM.x607A_targetPosition
                                   - (int64_t)motor_get_position();
            float velocity = motor_get_velocity();
            if (position_error >= -4 && position_error <= 4
                && velocity > -50.0f && velocity < 50.0f) {
                statusword |= 0x0400u;
            }
        } else if (mode == MODE_PROFILE_VEL) {
            float velocity_command = motor_get_velocity_cmd();
            float velocity = motor_get_velocity();
            if (velocity_command > -50.0f && velocity_command < 50.0f
                && velocity > -50.0f && velocity < 50.0f) {
                statusword |= 0x0400u;
            }
        } else if (mode == MODE_HOMING && s_homing_done != 0u) {
            statusword |= 0x0400u;
        }
    }

    OD_RAM.x6041_statusword = statusword;
}

void cia402_init(void)
{
    cia402_sm_init(&s_power_sm);
    s_prev_pp_bit4 = 0u;
    s_prev_fault_reset_bit = 0u;
    s_homing_done = 0u;
    s_external_fault = 0u;
    s_previous_mode = 0;

    set_error(0u);
    OD_RAM.x6040_controlword = 0u;
    OD_RAM.x6041_statusword = cia402_sm_statusword(&s_power_sm);
    OD_RAM.x6061_modesOfOperationDisplay = 0;
    OD_RAM.x6064_positionActualValue = motor_get_position();
    OD_RAM.x606C_velocityActualValue = (int32_t)motor_get_velocity();
    OD_RAM.x6502_supportedDriveModes = SUPPORTED_DRIVE_MODES;
}

void cia402_process(void)
{
    uint16_t controlword = OD_RAM.x6040_controlword;
    uint8_t fault_reset_bit = ((controlword & 0x0080u) != 0u) ? 1u : 0u;
    uint8_t fault_reset_edge = (fault_reset_bit != 0u && s_prev_fault_reset_bit == 0u) ? 1u : 0u;
    motor_fault_t motor_fault;

    s_prev_fault_reset_bit = fault_reset_bit;
    if (cia402_sm_state(&s_power_sm) == CIA402_STATE_FAULT && fault_reset_edge != 0u) {
        motor_clear_fault();
        if (motor_get_fault() == MOTOR_FAULT_NONE) {
            s_external_fault = 0u;
        }
    }

    motor_fault = motor_get_fault();
    if (motor_fault != MOTOR_FAULT_NONE) {
        set_error(motor_fault_error_code(motor_fault));
    }

    cia402_sm_step(&s_power_sm, controlword,
                   motor_fault != MOTOR_FAULT_NONE || s_external_fault != 0u);

    if (cia402_sm_operation_enabled(&s_power_sm)) {
        if (!motor_is_enabled()) {
            motor_enable();
        }
        apply_mode_logic();
    } else if (motor_is_enabled()) {
        motor_disable();
    }

    if (cia402_sm_state(&s_power_sm) != CIA402_STATE_FAULT
        && motor_get_fault() == MOTOR_FAULT_NONE && s_external_fault == 0u) {
        set_error(0u);
    }

    OD_RAM.x6064_positionActualValue = motor_get_position();
    OD_RAM.x606C_velocityActualValue = (int32_t)motor_get_velocity();
    if (!cia402_mode_supported(OD_RAM.x6060_modesOfOperation)) {
        OD_RAM.x6061_modesOfOperationDisplay = 0;
    }
    update_statusword();
}

void cia402_raise_fault(uint16_t error_code)
{
    s_external_fault = 1u;
    set_error(error_code);
    cia402_sm_step(&s_power_sm, OD_RAM.x6040_controlword, true);
    motor_disable();
    update_statusword();
}
