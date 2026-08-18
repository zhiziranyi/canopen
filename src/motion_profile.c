#include "motion_profile.h"

#include <math.h>
#include <stddef.h>

#define MOTION_PROFILE_POSITION_WINDOW 4.0f
#define MOTION_PROFILE_STOP_SPEED 50.0f

static float positive_or(float value, float fallback)
{
    return (isfinite(value) && value > 0.0f) ? value : fallback;
}

void motion_profile_init(motion_profile_t* profile, float velocity_limit,
                         float acceleration, float deceleration)
{
    if (profile == NULL) {
        return;
    }

    profile->target_position = 0;
    profile->velocity_command = 0.0f;
    motion_profile_set_limits(profile, velocity_limit, acceleration, deceleration);
}

void motion_profile_set_limits(motion_profile_t* profile, float velocity_limit,
                               float acceleration, float deceleration)
{
    if (profile == NULL) {
        return;
    }

    profile->velocity_limit = positive_or(velocity_limit, 1.0f);
    profile->acceleration = positive_or(acceleration, 1.0f);
    profile->deceleration = positive_or(deceleration, 1.0f);
}

void motion_profile_set_target(motion_profile_t* profile, int32_t current_position,
                               int32_t target_position)
{
    if (profile == NULL) {
        return;
    }

    profile->target_position = target_position;
    if (current_position == target_position) {
        profile->velocity_command = 0.0f;
    }
}

float motion_profile_step(motion_profile_t* profile, int32_t current_position, float dt_s)
{
    int64_t distance_counts;
    float distance;
    float direction;
    float braking_velocity;
    float desired_velocity;
    float delta_velocity;
    float slew_rate;
    float maximum_delta;

    if (profile == NULL || !isfinite(dt_s) || dt_s <= 0.0f) {
        return 0.0f;
    }

    distance_counts = (int64_t)profile->target_position - (int64_t)current_position;
    distance = fabsf((float)distance_counts);
    if (distance <= MOTION_PROFILE_POSITION_WINDOW
        && fabsf(profile->velocity_command) <= MOTION_PROFILE_STOP_SPEED) {
        profile->velocity_command = 0.0f;
        return 0.0f;
    }

    direction = (distance_counts >= 0) ? 1.0f : -1.0f;
    braking_velocity = sqrtf(2.0f * profile->deceleration * distance);
    if (braking_velocity > profile->velocity_limit) {
        braking_velocity = profile->velocity_limit;
    }
    desired_velocity = direction * braking_velocity;

    delta_velocity = desired_velocity - profile->velocity_command;
    if ((profile->velocity_command * desired_velocity) < 0.0f
        || fabsf(desired_velocity) < fabsf(profile->velocity_command)) {
        slew_rate = profile->deceleration;
    } else {
        slew_rate = profile->acceleration;
    }
    maximum_delta = slew_rate * dt_s;
    if (delta_velocity > maximum_delta) {
        delta_velocity = maximum_delta;
    } else if (delta_velocity < -maximum_delta) {
        delta_velocity = -maximum_delta;
    }

    profile->velocity_command += delta_velocity;
    if (profile->velocity_command > profile->velocity_limit) {
        profile->velocity_command = profile->velocity_limit;
    } else if (profile->velocity_command < -profile->velocity_limit) {
        profile->velocity_command = -profile->velocity_limit;
    }

    return profile->velocity_command;
}

void motion_profile_stop(motion_profile_t* profile)
{
    if (profile != NULL) {
        profile->velocity_command = 0.0f;
    }
}
