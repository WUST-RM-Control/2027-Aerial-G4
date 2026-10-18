#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "main.h"

/* 欧拉角(deg, ZYX 次序: 先偏航-再俯仰-后横滚) -> 四元数 (w,x,y,z), 机体系->导航系 */
void Attitude_Euler2Quat(float Roll, float Pitch, float Yaw, float q[4]);

/* 四元数 -> 旋转矩阵 R[3][3] (机体系->导航系) */
void Attitude_Quat2RotMatrix(const float q[4], float R[3][3]);

/* 姿态误差: qe = q_cur^-1 (x) q_tgt, 取旋转矢量输出(deg)
 * 符号按本工程 INS 约定: [0]=Roll(+绕X) [1]=Pitch(-绕Y) [2]=Yaw(+绕Z) */
void Attitude_QuatError(const float q_cur[4], const float q_tgt[4], float Err[3]);

/* cos(倾角) = R[2][2]: 1=水平, 0=倾倒 90°, 用于倾斜油门补偿 */
float Attitude_GetCosTilt(const float q[4]);

#endif
