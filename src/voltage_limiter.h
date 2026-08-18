#ifndef VOLTAGE_LIMITER_H
#define VOLTAGE_LIMITER_H

typedef struct {
    float output;
} voltage_limiter_t;

void voltage_limiter_init(voltage_limiter_t* state);
float voltage_limiter_step(voltage_limiter_t* state, float requested_voltage,
                           float phase_current, float voltage_limit,
                           float soft_current_limit, float rise_step,
                           float fall_step);

#endif /* VOLTAGE_LIMITER_H */
