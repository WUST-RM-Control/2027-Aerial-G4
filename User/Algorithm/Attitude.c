/**
 * @file  Attitude.c
 * @brief 姿态数学: 欧拉角/四元数/旋转矩阵/姿态误差
 * @note  纯函数, 无全局状态, 与 Control / INS 解耦
 */
#include "Attitude.h"

#include <math.h>

/**
 * @brief 欧拉角 -> 四元数 (ZYX 次序)
 * @param Roll/Pitch/Yaw 单位 deg
 * @param q 输出 (w,x,y,z), 机体系->导航系
 */
void Attitude_Euler2Quat(float Roll, float Pitch, float Yaw, float q[4])
{
    float hr = 0.5f * Roll  * Angle2Rad;
    float hp = 0.5f * Pitch * Angle2Rad;
    float hy = 0.5f * Yaw   * Angle2Rad;

    float cr = cosf(hr), sr = sinf(hr);
    float cp = cosf(hp), sp = sinf(hp);
    float cy = cosf(hy), sy = sinf(hy);

    q[0] = cy * cp * cr + sy * sp * sr;     /* w */
    q[1] = cy * cp * sr - sy * sp * cr;     /* x: 横滚轴 */
    q[2] = cy * sp * cr + sy * cp * sr;     /* y: 俯仰轴 */
    q[3] = sy * cp * cr - cy * sp * sr;     /* z: 偏航轴 */
}

/**
 * @brief 四元数 -> 旋转矩阵(机体系->导航系)
 * @note  R[2][2] = cos(倾角); R[2][0..2] = 机体 Z 轴在导航系的投影(即 INS.zn)
 */
void Attitude_Quat2RotMatrix(const float q[4], float R[3][3])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    R[0][0] = 1.0f - 2.0f * (y * y + z * z);
    R[0][1] = 2.0f * (x * y - w * z);
    R[0][2] = 2.0f * (x * z + w * y);

    R[1][0] = 2.0f * (x * y + w * z);
    R[1][1] = 1.0f - 2.0f * (x * x + z * z);
    R[1][2] = 2.0f * (y * z - w * x);

    R[2][0] = 2.0f * (x * z - w * y);
    R[2][1] = 2.0f * (y * z + w * x);
    R[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

/**
 * @brief 姿态误差(当前 -> 目标)的旋转矢量, 无 +-90° 奇异
 * @param q_cur 当前姿态四元数
 * @param q_tgt 目标姿态四元数
 * @param Err   输出 [0]=Roll [1]=Pitch [2]=Yaw (deg)
 * @note  误差矢量表达在“当前姿态坐标系” = IMU 芯片坐标系里,
 *        下面再做一次芯片->机体的轴向映射(与 INS.c 的角速度映射完全一致):
 *          芯片 X 轴 = -机体俯仰轴   芯片 Y 轴 = -机体横滚轴
 *          芯片 Z 轴 = +机体偏航轴
 *        所以 Err[0] 取 -qe[2], Err[1] 取 -qe[1]。
 *        改 IMU 安装方向时, 这两行必须和 INS.c 的 Roll_Speed/Pitch_Speed 一起改。
 */
void Attitude_QuatError(const float q_cur[4], const float q_tgt[4], float Err[3])
{
    float qe[4];
    float s, k;

    const float *qc = q_cur;
    const float *qt = q_tgt;

    /* qe = q_cur^-1 (共轭) (x) q_tgt */
    qe[0] = qc[0] * qt[0] + qc[1] * qt[1] + qc[2] * qt[2] + qc[3] * qt[3];
    qe[1] = qc[0] * qt[1] - qc[1] * qt[0] - qc[2] * qt[3] + qc[3] * qt[2];
    qe[2] = qc[0] * qt[2] + qc[1] * qt[3] - qc[2] * qt[0] - qc[3] * qt[1];
    qe[3] = qc[0] * qt[3] - qc[1] * qt[2] + qc[2] * qt[1] - qc[3] * qt[0];

    /* 取最短旋转(qe[0] >= 0 ⇒ 转角属于 [0,180°]) */
    if (qe[0] < 0.0f)
    {
        qe[0] = -qe[0];
        qe[1] = -qe[1];
        qe[2] = -qe[2];
        qe[3] = -qe[3];
    }

    /* 旋转矢量 = 转轴 x 转角(deg) */
    s = sqrtf(qe[1] * qe[1] + qe[2] * qe[2] + qe[3] * qe[3]);
    k = (s > 1e-6f) ? (2.0f * atan2f(s, qe[0]) * Rad2Angle / s) : 0.0f;

    Err[0] = -qe[2] * k;        /* Roll : 绕机体X (= -绕芯片Y) */
    Err[1] = -qe[1] * k;        /* Pitch: 绕机体Y (= -绕芯片X, 与 INS.Pitch 同号) */
    Err[2] =  qe[3] * k;        /* Yaw  : 绕机体Z (= +绕芯片Z) */
}

/**
 * @brief cos(倾角) = R[2][2] = cos(Roll)*cos(Pitch)
 */
float Attitude_GetCosTilt(const float q[4])
{
    return 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
}
