#include "cia402_sm.h"
#include "encoder_math.h"
#include "fast_trig.h"
#include "foc.h"
#include "motion_profile.h"
#include "pid.h"
#include "voltage_limiter.h"

#include <math.h>
#include <stdio.h>

static int failures = 0;

#define CHECK_TRUE(expr)                                                                          \
    do {                                                                                          \
        if (!(expr)) {                                                                            \
            (void)printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr);                         \
            failures++;                                                                           \
        }                                                                                         \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                                   \
    do {                                                                                          \
        float check_actual = (actual);                                                            \
        float check_expected = (expected);                                                        \
        if (fabsf(check_actual - check_expected) > (tolerance)) {                                \
            (void)printf("FAIL %s:%d: %.8f != %.8f\n", __FILE__, __LINE__,                      \
                         (double)check_actual, (double)check_expected);                            \
            failures++;                                                                           \
        }                                                                                         \
    } while (0)

static void test_svpwm_zero_vector_is_centered(void)
{
    float du = 0.0f;
    float dv = 0.0f;
    float dw = 0.0f;

    foc_inverse_park_svpwm(0.0f, 0.0f, 0.0f, 1.0f, 12.0f, &du, &dv, &dw);

    CHECK_NEAR(du, 0.5f, 1.0e-6f);
    CHECK_NEAR(dv, 0.5f, 1.0e-6f);
    CHECK_NEAR(dw, 0.5f, 1.0e-6f);
}

static void test_svpwm_outputs_are_bounded(void)
{
    float du = 0.0f;
    float dv = 0.0f;
    float dw = 0.0f;

    foc_inverse_park_svpwm(50.0f, -50.0f, 0.70710678f, 0.70710678f,
                           12.0f, &du, &dv, &dw);

    CHECK_TRUE(du >= 0.0f && du <= 1.0f);
    CHECK_TRUE(dv >= 0.0f && dv <= 1.0f);
    CHECK_TRUE(dw >= 0.0f && dw <= 1.0f);
}

static void test_clarke_park_d_axis(void)
{
    float id = 0.0f;
    float iq = 0.0f;

    foc_clarke_park(1.0f, -0.5f, -0.5f, 0.0f, 1.0f, &id, &iq);

    CHECK_NEAR(id, 1.0f, 1.0e-5f);
    CHECK_NEAR(iq, 0.0f, 1.0e-5f);
}

static void test_fast_sin_cos_tracks_unit_circle(void)
{
    static const float angles[] = {
        -6.28318531f, -4.71238898f, -3.14159265f, -1.57079633f,
        -0.78539816f, 0.0f, 0.78539816f, 1.57079633f,
        3.14159265f, 4.71238898f, 6.28318531f
    };
    size_t index;

    for (index = 0u; index < sizeof(angles) / sizeof(angles[0]); ++index) {
        float sin_value;
        float cos_value;

        fast_sin_cos(angles[index], &sin_value, &cos_value);
        CHECK_NEAR(sin_value, sinf(angles[index]), 0.002f);
        CHECK_NEAR(cos_value, cosf(angles[index]), 0.002f);
        CHECK_NEAR(sin_value * sin_value + cos_value * cos_value, 1.0f, 0.004f);
    }
}

static void test_voltage_limiter_ramps_and_soft_limits_current(void)
{
    voltage_limiter_t limiter;

    voltage_limiter_init(&limiter);
    CHECK_NEAR(voltage_limiter_step(&limiter, 2.0f, 0.0f,
                                    2.0f, 0.45f, 0.10f, 0.50f),
               0.10f, 1.0e-6f);
    CHECK_NEAR(voltage_limiter_step(&limiter, 2.0f, 0.0f,
                                    2.0f, 0.45f, 0.10f, 0.50f),
               0.20f, 1.0e-6f);
    CHECK_NEAR(voltage_limiter_step(&limiter, 2.0f, 0.50f,
                                    2.0f, 0.45f, 0.10f, 0.50f),
               0.0f, 1.0e-6f);
}

static void test_pid_saturates_and_resets(void)
{
    pid_t pid;

    pid_init(&pid, 2.0f, 1.0f, 0.0f, -1.0f, 1.0f, 0.001f);
    CHECK_NEAR(pid_update(&pid, 100.0f), 1.0f, 1.0e-6f);
    pid_reset(&pid);
    CHECK_NEAR(pid.integral, 0.0f, 1.0e-6f);
    CHECK_NEAR(pid.prev_error, 0.0f, 1.0e-6f);
}

static void test_encoder_wrap_is_one_count(void)
{
    encoder_math_t encoder;

    encoder_math_init(&encoder, 4095u);
    encoder_math_update(&encoder, 0u, 0.001f);
    CHECK_TRUE(encoder_math_position(&encoder) == 1);
    encoder_math_update(&encoder, 4095u, 0.001f);
    CHECK_TRUE(encoder_math_position(&encoder) == 0);
    CHECK_TRUE(fabsf(encoder_math_velocity(&encoder)) < 1000.0f);
}

static void test_encoder_accumulates_and_zeros_position(void)
{
    encoder_math_t encoder;

    encoder_math_init(&encoder, 100u);
    encoder_math_update(&encoder, 104u, 0.001f);
    CHECK_TRUE(encoder_math_position(&encoder) == 4);
    CHECK_TRUE(encoder_math_velocity(&encoder) > 0.0f);
    encoder_math_zero_position(&encoder);
    CHECK_TRUE(encoder_math_position(&encoder) == 0);
}

static void test_encoder_direction_matches_foc_alignment(void)
{
    encoder_math_t encoder;

    encoder_math_init(&encoder, 100u);
    encoder_math_update(&encoder, 104u, 0.001f);
    CHECK_TRUE(encoder_math_directed_position(&encoder, 1) == 4);
    CHECK_TRUE(encoder_math_directed_position(&encoder, -1) == -4);
    CHECK_TRUE(encoder_math_directed_velocity(&encoder, 1) > 0.0f);
    CHECK_TRUE(encoder_math_directed_velocity(&encoder, -1) < 0.0f);
}

static void test_cia402_standard_enable_sequence(void)
{
    cia402_sm_t sm;

    cia402_sm_init(&sm);
    CHECK_TRUE(cia402_sm_statusword(&sm) == 0x0240u);
    cia402_sm_step(&sm, 0x0006u, false);
    CHECK_TRUE(cia402_sm_statusword(&sm) == 0x0231u);
    cia402_sm_step(&sm, 0x0007u, false);
    CHECK_TRUE(cia402_sm_statusword(&sm) == 0x0233u);
    cia402_sm_step(&sm, 0x000Fu, false);
    CHECK_TRUE(cia402_sm_operation_enabled(&sm));
    CHECK_TRUE(cia402_sm_statusword(&sm) == 0x0237u);
    cia402_sm_step(&sm, 0x000Bu, false);
    CHECK_TRUE(!cia402_sm_operation_enabled(&sm));
    CHECK_TRUE(cia402_sm_statusword(&sm) == 0x0217u);
}

static void test_cia402_fault_requires_reset_edge(void)
{
    cia402_sm_t sm;

    cia402_sm_init(&sm);
    cia402_sm_step(&sm, 0x0000u, true);
    CHECK_TRUE(cia402_sm_statusword(&sm) == 0x0208u);
    cia402_sm_step(&sm, 0x0000u, false);
    CHECK_TRUE(cia402_sm_statusword(&sm) == 0x0208u);
    cia402_sm_step(&sm, 0x0080u, false);
    CHECK_TRUE(cia402_sm_statusword(&sm) == 0x0240u);
}

static void test_cia402_supported_modes_are_explicit(void)
{
    CHECK_TRUE(cia402_mode_supported(1));
    CHECK_TRUE(cia402_mode_supported(3));
    CHECK_TRUE(cia402_mode_supported(6));
    CHECK_TRUE(!cia402_mode_supported(0));
    CHECK_TRUE(!cia402_mode_supported(2));
    CHECK_TRUE(!cia402_mode_supported(4));
}

static void test_motion_profile_handles_forward_reverse_and_stop(void)
{
    motion_profile_t profile;
    float command;

    motion_profile_init(&profile, 2000.0f, 10000.0f, 10000.0f);
    motion_profile_set_target(&profile, 0, 1000);
    command = motion_profile_step(&profile, 0, 0.001f);
    CHECK_TRUE(command > 0.0f);
    CHECK_TRUE(command <= 10.0001f);

    motion_profile_init(&profile, 2000.0f, 10000.0f, 10000.0f);
    motion_profile_set_target(&profile, 2000, 1000);
    command = motion_profile_step(&profile, 2000, 0.001f);
    CHECK_TRUE(command < 0.0f);
    CHECK_TRUE(command >= -10.0001f);

    motion_profile_set_target(&profile, 100, 100);
    CHECK_NEAR(motion_profile_step(&profile, 100, 0.001f), 0.0f, 1.0e-6f);
}

static void test_motion_profile_brakes_near_target(void)
{
    motion_profile_t profile;
    float far_command;
    float near_command;

    motion_profile_init(&profile, 2000.0f, 1000000.0f, 10000.0f);
    motion_profile_set_target(&profile, 0, 10000);
    far_command = motion_profile_step(&profile, 0, 0.1f);
    near_command = motion_profile_step(&profile, 9990, 0.1f);

    CHECK_TRUE(far_command > 0.0f);
    CHECK_TRUE(near_command >= 0.0f);
    CHECK_TRUE(near_command < far_command);
}

int main(void)
{
    test_svpwm_zero_vector_is_centered();
    test_svpwm_outputs_are_bounded();
    test_clarke_park_d_axis();
    test_fast_sin_cos_tracks_unit_circle();
    test_voltage_limiter_ramps_and_soft_limits_current();
    test_pid_saturates_and_resets();
    test_encoder_wrap_is_one_count();
    test_encoder_accumulates_and_zeros_position();
    test_encoder_direction_matches_foc_alignment();
    test_cia402_standard_enable_sequence();
    test_cia402_fault_requires_reset_edge();
    test_cia402_supported_modes_are_explicit();
    test_motion_profile_handles_forward_reverse_and_stop();
    test_motion_profile_brakes_near_target();

    if (failures != 0) {
        (void)printf("native tests: FAIL (%d)\n", failures);
        return 1;
    }

    (void)printf("native tests: PASS\n");
    return 0;
}
