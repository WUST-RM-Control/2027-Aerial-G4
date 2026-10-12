#ifndef __Filter__
#define __Filter__

#include "main.h"

void Filter_LowPass(float input, float *output, float tau, float dt);
void Filter_Average(float input, float *output, float *Buffer, uint16_t Buffer_Num);

/*===| 二阶滤波器(Biquad): 用于陀螺振动抑制(陷波/低通) |===
 * 系数按 RBJ Audio-EQ-Cookbook 计算, 采样率用标称控制频率即可(不做变系数)。
 * 陷波用来干掉电机/桨的窄带振动(对环路延迟影响最小),
 * 低通用在陷波之后再压一点宽带噪声。
 * ------------------------------------------------------- */
typedef struct
{
    float b0, b1, b2;    /* 分子系数(a0 已归一化为 1) */
    float a1, a2;        /* 分母系数 */
    float x1, x2;        /* 输入历史 */
    float y1, y2;        /* 输出历史 */
} Biquad_t;

void  Biquad_Init(Biquad_t *F);                                                /* 复位为直通 */
void  Biquad_Notch_Init(Biquad_t *F, float F0_Hz, float Fs_Hz, float Q);       /* 陷波 */
void  Biquad_LowPass_Init(Biquad_t *F, float Fc_Hz, float Fs_Hz, float Q);     /* 二阶低通 */
float Biquad_Update(Biquad_t *F, float In);                                   /* 每采样点调用一次 */

#endif
