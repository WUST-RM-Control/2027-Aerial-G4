#include "Filter.h"

#include "math.h"   /* sinf/cosf */

#ifndef PI
#define PI  3.14159265358979f
#endif

void Filter_LowPass(float input, float *output, float tau, float dt)
{
    float alpha = dt / (tau + dt);
    *output += (input - *output) * alpha;
}

void Filter_Average(float input, float *output, float *Buffer, uint16_t Buffer_Num)
{  
    float Sum;
    for(uint16_t i = 0; i < Buffer_Num - 1; i++)
    {
        Buffer[Buffer_Num - i - 1] = Buffer[Buffer_Num - i - 2];
        Sum += Buffer[Buffer_Num - i - 1];
    }
    Sum += input;
    *output = Sum/Buffer_Num;
}

/*==============================================================================
 *  二阶滤波器(Biquad, Direct Form 1)
 *  系数按 RBJ Audio-EQ-Cookbook 计算。采样率固定用标称值(不做变系数),
 *  不要求与真实控制周期严格相等, 只影响陷波中心频率的微小偏移。
 *============================================================================*/

/* 复位为“直通”(b0=1, 其它系数与状态全 0) */
void Biquad_Init(Biquad_t *F)
{
    F->b0 = 1.0f; F->b1 = 0.0f; F->b2 = 0.0f;
    F->a1 = 0.0f; F->a2 = 0.0f;
    F->x1 = 0.0f; F->x2 = 0.0f;
    F->y1 = 0.0f; F->y2 = 0.0f;
}

/* 每采样点调用一次: 输入 -> 输出 */
float Biquad_Update(Biquad_t *F, float In)
{
    float Out = F->b0 * In + F->b1 * F->x1 + F->b2 * F->x2
              - F->a1 * F->y1 - F->a2 * F->y2;

    F->x2 = F->x1; F->x1 = In;
    F->y2 = F->y1; F->y1 = Out;

    return Out;
}

/*
 * 陷波(Notch): 干掉 F0 附近的窄带振动
 *   w0 = 2πF0/Fs, alpha = sin(w0)/(2Q)
 *   b = [1, -2cos(w0), 1] / a0
 *   a = [a0, -2cos(w0), 1-alpha] / a0   (a0 = 1+alpha)
 * Q 越大陷波越窄(BW = F0/Q); 主峰不确定时取 Q = 1~2 宽一点更保险。
 */
void Biquad_Notch_Init(Biquad_t *F, float F0_Hz, float Fs_Hz, float Q)
{
    float w0, cw, sw, alpha, a0;

    if ((Fs_Hz <= 0.0f) || (F0_Hz <= 0.0f) || (F0_Hz >= Fs_Hz * 0.5f))
    {
        Biquad_Init(F);                       /* 参数越界 -> 直通 */
        return;
    }
    if (Q <= 0.0f) Q = 0.7071f;

    w0    = 2.0f * PI * F0_Hz / Fs_Hz;
    cw    = cosf(w0);
    sw    = sinf(w0);
    alpha = sw / (2.0f * Q);
    a0    = 1.0f + alpha;

    F->b0 =  1.0f / a0;
    F->b1 = -2.0f * cw / a0;
    F->b2 =  1.0f / a0;
    F->a1 = -2.0f * cw / a0;
    F->a2 = (1.0f - alpha) / a0;

    F->x1 = 0.0f; F->x2 = 0.0f; F->y1 = 0.0f; F->y2 = 0.0f;
}

/*
 * 二阶低通(Butterworth, Q=0.7071): 压一点陷波之后的宽带残余
 *   w0 = 2πFc/Fs, alpha = sin(w0)/(2Q), a0 = 1+alpha, k = (1-cos(w0))/2
 *   b = [k, 1-cos(w0), k] / a0
 */
void Biquad_LowPass_Init(Biquad_t *F, float Fc_Hz, float Fs_Hz, float Q)
{
    float w0, cw, sw, alpha, a0, k;

    if ((Fs_Hz <= 0.0f) || (Fc_Hz <= 0.0f) || (Fc_Hz >= Fs_Hz * 0.5f))
    {
        Biquad_Init(F);                       /* 参数越界 -> 直通 */
        return;
    }
    if (Q <= 0.0f) Q = 0.7071f;

    w0    = 2.0f * PI * Fc_Hz / Fs_Hz;
    cw    = cosf(w0);
    sw    = sinf(w0);
    alpha = sw / (2.0f * Q);
    a0    = 1.0f + alpha;
    k     = (1.0f - cw) * 0.5f;

    F->b0 =  k / a0;
    F->b1 = (1.0f - cw) / a0;
    F->b2 =  k / a0;
    F->a1 = -2.0f * cw / a0;
    F->a2 = (1.0f - alpha) / a0;

    F->x1 = 0.0f; F->x2 = 0.0f; F->y1 = 0.0f; F->y2 = 0.0f;
}