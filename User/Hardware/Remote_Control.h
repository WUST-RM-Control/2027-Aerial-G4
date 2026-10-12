#ifndef __Remote_Control__
#define __Remote_Control__

#include "main.h"

/*===| 遥控器数据结构体 |===*/
typedef struct
{
    uint8_t If_Remote_Connect;
    uint8_t If_Remote_Data_New;
    uint16_t Remote_GetData_Ticker;

    float Right_X;
    float Right_Y;
    float Left_X;
    float Left_Y;

    /* SBUS 旋钮/开关通道(连续值 -1~1, 通道号 = sbusVal 下标+1) */
    float LO;    /* 通道9 : 左手旋钮 */
    float RO;    /* 通道10: 右手旋钮 */
    float SA;    /* 通道5 : 开关/旋钮 A */
    float SB;    /* 通道6 : 开关/旋钮 B */
    float SC;    /* 通道7 : 开关/旋钮 C */
    float SD;    /* 通道8 : 开关/旋钮 D */
    float SE;    /* 通道11: 开关/旋钮 E */
    float SF;    /* 通道12: 开关/旋钮 F */
    float SG;    /* 通道13: 开关/旋钮 G */
    float SH;    /* 通道14: 开关/旋钮 H */
} Remote_StructTypeDef;

typedef __packed struct
{
  
}remote_rawdata_t;

extern Remote_StructTypeDef Remote;
extern Remote_StructTypeDef Remote_Last;

void Remote_Rx_CallBack(uint8_t *Remote_Control_RxDataBuff);


#endif
