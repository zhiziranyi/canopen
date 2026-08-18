#include "voltage_limiter.h"

#include <math.h>

static float clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void voltage_limiter_init(voltage_limiter_t* state)
{
    if (state != 0) {
        state->output = 0.0f;
    }
}

float voltage_limiter_step(voltage_limiter_t* state, float requested_voltage,
                           float phase_current, float voltage_limit,
                           float soft_current_limit, float rise_step,
                           float fall_step)
{
    float target;
    float step;

    if (state == 0 || voltage_limit <= 0.0f) {
        return 0.0f;
    }

    target = clamp(requested_voltage, voltage_limit);
    if (fabsf(phase_current) >= soft_current_limit) {
        target = 0.0f;
    }

    step = (fabsf(target) < fabsf(state->output)) ? fall_step : rise_step;
    if (step < 0.0f) {
        step = 0.0f;
    }
    if (target > state->output + step) {
        state->output += step;
    } else if (target < state->output - step) {
        state->output -= step;
    } else {
        state->output = target;
    }

    state->output = clamp(state->output, voltage_limit);
    return state->output;
}
