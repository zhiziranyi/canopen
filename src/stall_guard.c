#include "stall_guard.h"

#include <math.h>

void stall_guard_init(stall_guard_t* guard)
{
    if (guard != 0) {
        guard->stalled_seconds = 0.0f;
    }
}

bool stall_guard_step(stall_guard_t* guard,
                      float velocity_command, float applied_voltage,
                      float actual_velocity, float dt_s,
                      float minimum_command, float minimum_voltage,
                      float minimum_motion, float timeout_s)
{
    bool demanding_motion;
    bool motion_missing;

    if (guard == 0 || dt_s <= 0.0f || timeout_s <= 0.0f) {
        return false;
    }

    demanding_motion = fabsf(velocity_command) >= minimum_command
                     && fabsf(applied_voltage) >= minimum_voltage;
    motion_missing = fabsf(actual_velocity) < minimum_motion;
    if (!demanding_motion || !motion_missing) {
        guard->stalled_seconds = 0.0f;
        return false;
    }

    guard->stalled_seconds += dt_s;
    return guard->stalled_seconds >= (timeout_s - (0.5f * dt_s));
}
