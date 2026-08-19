#ifndef STALL_GUARD_H
#define STALL_GUARD_H

#include <stdbool.h>

typedef struct {
    float stalled_seconds;
} stall_guard_t;

void stall_guard_init(stall_guard_t* guard);
bool stall_guard_step(stall_guard_t* guard,
                      float velocity_command, float applied_voltage,
                      float actual_velocity, float dt_s,
                      float minimum_command, float minimum_voltage,
                      float minimum_motion, float timeout_s);

#endif /* STALL_GUARD_H */
