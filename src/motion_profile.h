#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdint.h>

typedef struct {
    int32_t target_position;
    float velocity_command;
    float velocity_limit;
    float acceleration;
    float deceleration;
} motion_profile_t;

void motion_profile_init(motion_profile_t* profile, float velocity_limit,
                         float acceleration, float deceleration);
void motion_profile_set_limits(motion_profile_t* profile, float velocity_limit,
                               float acceleration, float deceleration);
void motion_profile_set_target(motion_profile_t* profile, int32_t current_position,
                               int32_t target_position);
float motion_profile_step(motion_profile_t* profile, int32_t current_position, float dt_s);
void motion_profile_stop(motion_profile_t* profile);

#endif /* MOTION_PROFILE_H */
