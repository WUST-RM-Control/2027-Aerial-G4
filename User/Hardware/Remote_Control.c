//=====================================================================================
//                    Remote_Control[遥控器数据解包]
//      解析遥控器的数据，得到遥控器各个通道状态和键鼠信息，供RoboControl使用
//=====================================================================================

#include "Remote_Control.h"

/*===| 遥控器数据结构体 |===*/
remote_rawdata_t remote_rawdata;
Remote_StructTypeDef Remote;
Remote_StructTypeDef Remote_Last;

uint16_t Last_Data_Ticker;
void Remote_Task()
{
    for(;;)
    {
        if(Remote_Last.If_Remote_Data_New == 0) Last_Data_Ticker ++;
        if(Last_Data_Ticker > 50)   
        {
            memcpy(&Remote_Last, &Remote, sizeof(Remote_StructTypeDef));
            Remote_Last.If_Remote_Data_New = 1;
            Last_Data_Ticker = 0;
        }
            
        osDelay(1);
    }
}

/**
 * @brief 遥控串口接收中断回调函数
 *
 * @param 串口收到的数组
 */
void Remote_Rx_CallBack(uint8_t *data)
{
    //更新连接状态
    Remote.If_Remote_Connect = 1;
    Remote.Remote_GetData_Ticker = 0;
    Remote.If_Remote_Data_New = 1;

    /*===| SBUS 16 通道 11 位解包(data[1]~data[22] 共 176 位) |===*/
    uint16_t sbusVal[16];
    
    sbusVal[ 0] = ((uint16_t)data[ 1] >> 0 | ((uint16_t)data[ 2] << 8)) & 0x07FF;
    sbusVal[ 1] = ((uint16_t)data[ 2] >> 3 | ((uint16_t)data[ 3] << 5)) & 0x07FF;
    sbusVal[ 2] = ((uint16_t)data[ 3] >> 6 | ((uint16_t)data[ 4] << 2) | (uint16_t)data[ 5] << 10) & 0x07FF;
    sbusVal[ 3] = ((uint16_t)data[ 5] >> 1 | ((uint16_t)data[ 6] << 7)) & 0x07FF;
    sbusVal[ 4] = ((uint16_t)data[ 6] >> 4 | ((uint16_t)data[ 7] << 4)) & 0x07FF;
    sbusVal[ 5] = ((uint16_t)data[ 7] >> 7 | ((uint16_t)data[ 8] << 1) | (uint16_t)data[ 9] << 9) & 0x07FF;
    sbusVal[ 6] = ((uint16_t)data[ 9] >> 2 | ((uint16_t)data[10] << 6)) & 0x07FF;
    sbusVal[ 7] = ((uint16_t)data[10] >> 5 | ((uint16_t)data[11] << 3)) & 0x07FF;
    sbusVal[ 8] = ((uint16_t)data[12] << 0 | ((uint16_t)data[13] << 8)) & 0x07FF;
//    sbusVal[ 9] = ((uint16_t)data[13] >> 3 | ((uint16_t)data[14] << 5)) & 0x07FF;
//    sbusVal[10] = ((uint16_t)data[14] >> 6 | ((uint16_t)data[15] << 2) | (uint16_t)data[16] << 10) & 0x07FF;
//    sbusVal[11] = ((uint16_t)data[16] >> 1 | ((uint16_t)data[17] << 7)) & 0x07FF;
//    sbusVal[12] = ((uint16_t)data[17] >> 4 | ((uint16_t)data[18] << 4)) & 0x07FF;
//    sbusVal[13] = ((uint16_t)data[18] >> 7 | ((uint16_t)data[19] << 1) | (uint16_t)data[20] << 9) & 0x07FF;
//    sbusVal[14] = ((uint16_t)data[20] >> 2 | ((uint16_t)data[21] << 6)) & 0x07FF;
//    sbusVal[15] = ((uint16_t)data[21] >> 5 | ((uint16_t)data[22] << 3)) & 0x07FF;

    /*===| 摇杆四轴(通道1~4): 中位 1024, 满偏 ±671 → ±1 |===*/
    Remote.Right_X = (sbusVal[ 0] - 1000) / 800.0f;
    Remote.Right_Y = -(sbusVal[ 1] - 1000) / 800.0f;
    Remote.Left_Y  = (sbusVal[ 2] - 1000) / 800.0f;
    Remote.Left_X  = (sbusVal[ 3] - 1000) / 800.0f;

    /*===| 旋钮/开关通道(连续值 -1~1, 通道号 = sbusVal 下标+1) |===*/
    Remote.SA = (sbusVal[ 6] - 1000) / 800.0f;   /* 通道5  */
    Remote.SB = (sbusVal[ 4] - 1000) / 800.0f;   /* 通道6  */
    Remote.SC = (sbusVal[ 7] - 1000) / 800.0f;   /* 通道7  */
    Remote.SD = (sbusVal[ 5] - 1000) / 800.0f;   /* 通道8  */
//    Remote.LO = -(sbusVal[ 8] - 1024) / 800.0f;  /* 通道9  : 左手旋钮 */
//    Remote.RO = -(sbusVal[ 9] - 1024) / 671.0f;  /* 通道10 : 右手旋钮 */
//    Remote.SE = -(sbusVal[10] - 1024) / 671.0f;   /* 通道11 */
//    Remote.SF = (sbusVal[11] - 1024) / 671.0f;   /* 通道12 */
//    Remote.SG = -(sbusVal[12] - 1024) / 671.0f;   /* 通道13 */
//    Remote.SH = -(sbusVal[13] - 1024) / 671.0f;   /* 通道14 */

    if(data[23] == 0 && fabs((float)Remote.SA) <= 1 && fabs((float)Remote.SB) <= 1)
    {
        Remote.If_Remote_Connect = 1;
        Remote.Remote_GetData_Ticker = 0;
    }
    else
    {
        Remote.If_Remote_Connect = 0;
    }
}

