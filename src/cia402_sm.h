#ifndef CIA402_SM_H
#define CIA402_SM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    CIA402_STATE_NOT_READY_TO_SWITCH_ON = 0,
    CIA402_STATE_SWITCH_ON_DISABLED,
    CIA402_STATE_READY_TO_SWITCH_ON,
    CIA402_STATE_SWITCHED_ON,
    CIA402_STATE_OPERATION_ENABLED,
    CIA402_STATE_QUICK_STOP_ACTIVE,
    CIA402_STATE_FAULT_REACTION_ACTIVE,
    CIA402_STATE_FAULT
} cia402_state_t;

typedef struct {
    cia402_state_t state;
    uint16_t previous_controlword;
} cia402_sm_t;

void cia402_sm_init(cia402_sm_t* sm);
void cia402_sm_step(cia402_sm_t* sm, uint16_t controlword, bool hardware_fault);
uint16_t cia402_sm_statusword(const cia402_sm_t* sm);
bool cia402_sm_operation_enabled(const cia402_sm_t* sm);
cia402_state_t cia402_sm_state(const cia402_sm_t* sm);
bool cia402_mode_supported(int8_t mode);

#endif /* CIA402_SM_H */
