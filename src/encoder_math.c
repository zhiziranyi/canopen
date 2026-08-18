#include "encoder_math.h"

#include <stddef.h>

#define ENCODER_MATH_CPR 4096
#define ENCODER_MATH_HALF_CPR (ENCODER_MATH_CPR / 2)
#define ENCODER_VELOCITY_OLD_WEIGHT 0.9f
#define ENCODER_VELOCITY_NEW_WEIGHT 0.1f

void encoder_math_init(encoder_math_t* state, uint16_t raw)
{
    if (state == NULL) {
        return;
    }

    state->previous_raw = raw & 0x0FFFu;
    state->position = 0;
    state->velocity = 0.0f;
    state->initialized = 1u;
}

void encoder_math_update(encoder_math_t* state, uint16_t raw, float dt_s)
{
    int32_t delta;
    float instant_velocity;

    if (state == NULL || dt_s <= 0.0f) {
        return;
    }
    raw &= 0x0FFFu;
    if (state->initialized == 0u) {
        encoder_math_init(state, raw);
        return;
    }

    delta = (int32_t)raw - (int32_t)state->previous_raw;
    if (delta > ENCODER_MATH_HALF_CPR) {
        delta -= ENCODER_MATH_CPR;
    } else if (delta < -ENCODER_MATH_HALF_CPR) {
        delta += ENCODER_MATH_CPR;
    }

    state->previous_raw = raw;
    state->position += delta;
    instant_velocity = (float)delta / dt_s;
    state->velocity = state->velocity * ENCODER_VELOCITY_OLD_WEIGHT
                    + instant_velocity * ENCODER_VELOCITY_NEW_WEIGHT;
    if (state->velocity < 1.0f && state->velocity > -1.0f) {
        state->velocity = 0.0f;
    }
}

int32_t encoder_math_position(const encoder_math_t* state)
{
    return (state != NULL) ? state->position : 0;
}

float encoder_math_velocity(const encoder_math_t* state)
{
    return (state != NULL) ? state->velocity : 0.0f;
}

int32_t encoder_math_directed_position(const encoder_math_t* state, int8_t direction)
{
    int32_t position = encoder_math_position(state);

    return (direction < 0) ? -position : position;
}

float encoder_math_directed_velocity(const encoder_math_t* state, int8_t direction)
{
    float velocity = encoder_math_velocity(state);

    return (direction < 0) ? -velocity : velocity;
}

void encoder_math_zero_position(encoder_math_t* state)
{
    if (state != NULL) {
        state->position = 0;
    }
}
