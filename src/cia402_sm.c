#include "cia402_sm.h"

#include <stddef.h>

void cia402_sm_init(cia402_sm_t* sm)
{
    if (sm == NULL) {
        return;
    }
    sm->state = CIA402_STATE_SWITCH_ON_DISABLED;
    sm->previous_controlword = 0u;
}

void cia402_sm_step(cia402_sm_t* sm, uint16_t controlword, bool hardware_fault)
{
    bool fault_reset_edge;

    if (sm == NULL) {
        return;
    }

    fault_reset_edge = ((controlword & 0x0080u) != 0u)
                    && ((sm->previous_controlword & 0x0080u) == 0u);
    sm->previous_controlword = controlword;

    if (hardware_fault) {
        sm->state = CIA402_STATE_FAULT;
        return;
    }

    if (sm->state == CIA402_STATE_FAULT) {
        if (fault_reset_edge) {
            sm->state = CIA402_STATE_SWITCH_ON_DISABLED;
        }
        return;
    }

    if ((controlword & 0x0002u) == 0u) {
        sm->state = CIA402_STATE_SWITCH_ON_DISABLED;
        return;
    }

    switch (sm->state) {
        case CIA402_STATE_SWITCH_ON_DISABLED:
            if ((controlword & 0x0007u) == 0x0006u) {
                sm->state = CIA402_STATE_READY_TO_SWITCH_ON;
            }
            break;

        case CIA402_STATE_READY_TO_SWITCH_ON:
            if ((controlword & 0x0007u) == 0x0007u) {
                sm->state = CIA402_STATE_SWITCHED_ON;
            }
            break;

        case CIA402_STATE_SWITCHED_ON:
            if ((controlword & 0x000Fu) == 0x000Fu) {
                sm->state = CIA402_STATE_OPERATION_ENABLED;
            } else if ((controlword & 0x0007u) == 0x0006u) {
                sm->state = CIA402_STATE_READY_TO_SWITCH_ON;
            }
            break;

        case CIA402_STATE_OPERATION_ENABLED:
            if ((controlword & 0x0004u) == 0u) {
                sm->state = CIA402_STATE_QUICK_STOP_ACTIVE;
            } else if ((controlword & 0x000Fu) == 0x0007u) {
                sm->state = CIA402_STATE_SWITCHED_ON;
            } else if ((controlword & 0x0007u) == 0x0006u) {
                sm->state = CIA402_STATE_READY_TO_SWITCH_ON;
            }
            break;

        case CIA402_STATE_QUICK_STOP_ACTIVE:
            if ((controlword & 0x000Fu) == 0x000Fu) {
                sm->state = CIA402_STATE_OPERATION_ENABLED;
            } else if ((controlword & 0x0007u) == 0x0006u) {
                sm->state = CIA402_STATE_READY_TO_SWITCH_ON;
            }
            break;

        case CIA402_STATE_NOT_READY_TO_SWITCH_ON:
        case CIA402_STATE_FAULT_REACTION_ACTIVE:
        case CIA402_STATE_FAULT:
        default:
            break;
    }
}

uint16_t cia402_sm_statusword(const cia402_sm_t* sm)
{
    uint16_t statusword = 0x0200u;

    if (sm == NULL) {
        return statusword;
    }

    switch (sm->state) {
        case CIA402_STATE_SWITCH_ON_DISABLED:
            statusword |= 0x0040u;
            break;
        case CIA402_STATE_READY_TO_SWITCH_ON:
            statusword |= 0x0031u;
            break;
        case CIA402_STATE_SWITCHED_ON:
            statusword |= 0x0033u;
            break;
        case CIA402_STATE_OPERATION_ENABLED:
            statusword |= 0x0037u;
            break;
        case CIA402_STATE_QUICK_STOP_ACTIVE:
            statusword |= 0x0017u;
            break;
        case CIA402_STATE_FAULT:
        case CIA402_STATE_FAULT_REACTION_ACTIVE:
            statusword |= 0x0008u;
            break;
        case CIA402_STATE_NOT_READY_TO_SWITCH_ON:
        default:
            break;
    }

    return statusword;
}

bool cia402_sm_operation_enabled(const cia402_sm_t* sm)
{
    return sm != NULL && sm->state == CIA402_STATE_OPERATION_ENABLED;
}

cia402_state_t cia402_sm_state(const cia402_sm_t* sm)
{
    return (sm != NULL) ? sm->state : CIA402_STATE_NOT_READY_TO_SWITCH_ON;
}

bool cia402_mode_supported(int8_t mode)
{
    return mode == 1 || mode == 3 || mode == 6;
}
