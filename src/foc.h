/**
 * @file    foc.h
 * @brief   FOC 数学核心：Clarke/Park/反 Park/SVPWM/相电流重构
 */
#ifndef FOC_H
#define FOC_H

#include <stdint.h>

/* 相电流重构：单相(IA) + 占空比分配 */
void foc_reconstruct_currents(float ia, float du, float dv, float dw,
                              float* ib, float* ic);

/* Clarke + Park */
void foc_clarke_park(float ia, float ib, float ic, float sin_e, float cos_e,
                     float* id, float* iq);

/* 反 Park + SVPWM（SimpleFOC 风格中点偏移），输出归一化占空比 0..1 */
void foc_inverse_park_svpwm(float vd, float vq, float sin_e, float cos_e,
                            float bus_v, float* du, float* dv, float* dw);

#endif /* FOC_H */
