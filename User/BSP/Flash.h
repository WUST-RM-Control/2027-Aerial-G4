#ifndef __FLASH_H__
#define __FLASH_H__

#include "main.h"

/*==============================================================================
 *  简易配置存储（单文件式）
 *
 *  - 只占用 Flash 最后一页 **2 KB**：0x0801F800 ~ 0x0801FFFF（128 KB 器件的最后一页）
 *  - 记录格式：[16 B 头][88 B 数据]，头里带 Magic / 版本 / 长度 / 序号 / CRC32
 *  - 读取：直接内存映射读，校验 Magic + 版本 + 长度 + CRC，任一不符判为“无有效数据”
 *  - 写入：擦整页 -> 按 8 字节(双字)编程 -> 回读 memcmp 校验
 *
 *  ⚠ 擦写寿命 & 停顿：
 *    每次 Config_Save() 都会擦一次这页（G4 页擦约 20~40 ms，期间 CPU 会停顿），
 *    所以只在“人机交互 IMU 校准完成”时保存；**不要上电就写**。
 *
 *  ⚠ 该页在链接脚本(0x08000000 + 0x20000)范围内：当前固件 Code≈82 KB，
 *    离 0x0801F800 还有 ~40 KB 余量；若将来固件超过 126 KB，必须把这页移走
 *    （改 CONFIG_FLASH_BASE 并同步改 scatter 文件里的 IROM 大小）。
 *============================================================================*/

/* ---- 存储布局 ---- */
#define CONFIG_FLASH_BASE       0x0801F800u  /* 最后一页起始地址          */
#define CONFIG_FLASH_PAGE_SZ    0x800u       /* STM32G4 页大小 = 2 KB      */
#define CONFIG_BANK1_BASE       0x08000000u
#define CONFIG_BANK2_BASE       0x08010000u  /* 双 bank 时每个 bank 64 KB  */

#define CONFIG_MAGIC            0x47464331u  /* '1CFG' 小端 = "1CFG"      */
#define CONFIG_VERSION          1u

/* ---- 记录头(16 B) ---- */
typedef struct
{
    uint32_t Magic;      /* CONFIG_MAGIC                       */
    uint16_t Version;    /* CONFIG_VERSION                     */
    uint16_t Size;       /* 后面 payload 的字节数               */
    uint32_t Seq;        /* 第几次保存(调试用, 每次 +1)         */
    uint32_t Crc;        /* payload 的 CRC32                   */
} Config_Header_t;

/* ---- 校准参数(写进 Flash 的 payload) ----
 * ★ 改这个结构体必须同时改 CONFIG_VERSION, 否则老数据会被判为不兼容而丢弃 */
typedef struct
{
    float    Acc_Offset[3];   /* 加速度零偏   m/s²    */
    float    Acc_Scale[3];    /* 加速度比例           */
    float    Gyro_Offset[3];  /* 陀螺零漂     rad/s   */
    float    Frame_R[3][3];   /* 芯片->机体 旋转矩阵  */
    uint32_t Is_Calibrated;   /* 1 = 内容有效         */
    uint32_t Reserved[3];     /* 预留，同时保证结构体 8 字节对齐 */
} Config_StructTypedef;

extern Config_StructTypedef Config;

/* ---- 调试可读状态(调试器里直接看) ---- */
extern volatile uint8_t  Config_Load_Ok;   /* 1 = 上电读到了有效数据   */
extern volatile uint8_t  Config_Save_Ok;   /* 1 = 最近一次保存成功     */
extern volatile uint32_t Config_Seq;       /* Flash 里记录的保存次数   */

/* ---- 接口 ---- */
void    Flash_Init(void);          /* 上电调用一次: Flash -> Config(内存)      */
uint8_t Config_Load(void);         /* 1 = 读到有效数据; 0 = 无/损坏            */
uint8_t Config_Save(void);         /* 1 = 擦+写+回读校验全部成功               */
void    Config_Set_Default(void);  /* 内存里填默认值(只改 RAM, 不写 Flash)     */
void    Config_Read(void);         /* 兼容旧接口, 等价于 Config_Load()         */

#endif /* __FLASH_H__ */
