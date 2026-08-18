#ifndef ENCODER_MATH_H
#define ENCODER_MATH_H

#include <stdint.h>

typedef struct {
    uint16_t previous_raw;
    int32_t position;
    float velocity;
    uint8_t initialized;
} encoder_math_t;

void encoder_math_init(encoder_math_t* state, uint16_t raw);
void encoder_math_update(encoder_math_t* state, uint16_t raw, float dt_s);
int32_t encoder_math_position(const encoder_math_t* state);
float encoder_math_velocity(const encoder_math_t* state);
void encoder_math_zero_position(encoder_math_t* state);

#endif /* ENCODER_MATH_H */
