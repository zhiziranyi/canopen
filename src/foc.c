#include "foc.h"
#include <math.h>

void foc_reconstruct_currents(float ia, float du, float dv, float dw,
                              float* ib, float* ic)
{
    float b, c, s;
    (void)dw;
    /* 依据 B/C 两相占空比按比例分配（单相内联采样的标准近似） */
    s = dv + dw;
    if (s > 0.01f) {
        b = -ia * (dv / s);
        c = -ia - b;
    } else {
        /* 低占空比退化情况：平均分配 */
        b = -ia * 0.5f;
        c = -ia * 0.5f;
    }
    *ib = b;
    *ic = c;
}

void foc_clarke_park(float ia, float ib, float ic, float sin_e, float cos_e,
                     float* id, float* iq)
{
    float i_alpha, i_beta;
    (void)ic;
    /* Clarke（等幅值变换，仅用 A/B 两相，C = -A-B） */
    i_alpha = ia;
    i_beta = (ia + 2.0f * ib) * 0.577350269f;   /* 1/sqrt(3) */

    /* Park */
    *id = i_alpha * cos_e + i_beta * sin_e;
    *iq = -i_alpha * sin_e + i_beta * cos_e;
}

void foc_inverse_park_svpwm(float vd, float vq, float sin_e, float cos_e,
                            float bus_v, float* du, float* dv, float* dw)
{
    float v_alpha, v_beta;
    float va, vb, vc;
    float vmax, vmin, vmid;
    float d;

    /* 反 Park */
    v_alpha = vd * cos_e - vq * sin_e;
    v_beta = vd * sin_e + vq * cos_e;

    /* 反 Clarke（等幅值） */
    va = v_alpha;
    vb = -0.5f * v_alpha + 0.866025404f * v_beta;
    vc = -0.5f * v_alpha - 0.866025404f * v_beta;

    /* 中点偏移（等效 SVPWM，注入零序分量） */
    vmax = va;
    if (vb > vmax) vmax = vb;
    if (vc > vmax) vmax = vc;
    vmin = va;
    if (vb < vmin) vmin = vb;
    if (vc < vmin) vmin = vc;
    vmid = (vmax + vmin) * 0.5f;

    if (bus_v < 0.01f) bus_v = 0.01f;
    d = 0.5f * bus_v;
    va = va - vmid + d;
    vb = vb - vmid + d;
    vc = vc - vmid + d;

    /* 归一化并限幅 */
    if (va > bus_v) va = bus_v;
    else if (va < 0.0f) va = 0.0f;
    if (vb > bus_v) vb = bus_v;
    else if (vb < 0.0f) vb = 0.0f;
    if (vc > bus_v) vc = bus_v;
    else if (vc < 0.0f) vc = 0.0f;

    *du = va / bus_v;
    *dv = vb / bus_v;
    *dw = vc / bus_v;
}
