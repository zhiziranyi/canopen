#include "cia402_sm.h"
#include "encoder_math.h"
#include "foc.h"
#include "pid.h"

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

int main(void)
{
    test_svpwm_zero_vector_is_centered();
    test_svpwm_outputs_are_bounded();
    test_clarke_park_d_axis();
    test_pid_saturates_and_resets();
    test_encoder_wrap_is_one_count();
    test_encoder_accumulates_and_zeros_position();
    test_cia402_standard_enable_sequence();
    test_cia402_fault_requires_reset_edge();

    if (failures != 0) {
        (void)printf("native tests: FAIL (%d)\n", failures);
        return 1;
    }

    (void)printf("native tests: PASS\n");
    return 0;
}
