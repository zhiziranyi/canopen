#include "fast_trig.h"

#define FAST_TRIG_PI       3.14159265f
#define FAST_TRIG_TWO_PI   6.28318531f
#define FAST_TRIG_B         1.27323954f
#define FAST_TRIG_C        -0.405284735f
#define FAST_TRIG_CORRECT   0.225f

static float fast_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float wrap_to_pi(float angle_rad)
{
    while (angle_rad > FAST_TRIG_PI) {
        angle_rad -= FAST_TRIG_TWO_PI;
    }
    while (angle_rad < -FAST_TRIG_PI) {
        angle_rad += FAST_TRIG_TWO_PI;
    }
    return angle_rad;
}

static float fast_sin(float angle_rad)
{
    float value = FAST_TRIG_B * angle_rad
                + FAST_TRIG_C * angle_rad * fast_abs(angle_rad);

    return value + FAST_TRIG_CORRECT * (value * fast_abs(value) - value);
}

void fast_sin_cos(float angle_rad, float* sin_value, float* cos_value)
{
    float wrapped = wrap_to_pi(angle_rad);

    if (sin_value != 0) {
        *sin_value = fast_sin(wrapped);
    }
    if (cos_value != 0) {
        *cos_value = fast_sin(wrap_to_pi(wrapped + (FAST_TRIG_PI * 0.5f)));
    }
}
