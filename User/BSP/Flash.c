#include "Flash.h"
#include <string.h>

/* ============================ 内部变量 ============================ */

Config_StructTypedef Config;

volatile uint8_t  Config_Load_Ok = 0;
volatile uint8_t  Config_Save_Ok = 0;
volatile uint32_t Config_Seq     = 0;

/* 先把记录拼在 RAM 的缓冲里, 再整段按双字写进去 */
#define CONFIG_PAYLOAD_SZ   ((uint16_t)sizeof(Config_StructTypedef))
#define CONFIG_RECORD_SZ    ((uint16_t)(sizeof(Config_Header_t) + CONFIG_PAYLOAD_SZ))

/* 双字编程要求 8 字节对齐, 缓冲区和长度都要凑成 8 的整数倍 */
#define CONFIG_BUF_SZ       ((uint32_t)((CONFIG_RECORD_SZ + 7u) & ~7u))

static uint8_t Config_Buf[CONFIG_BUF_SZ] __attribute__((aligned(8)));

/* ============================ CRC32 ============================ */

/* 位运算实现的 CRC32 (多项式 0xEDB88320, 与 zlib/crc32 一致)
 * 88 字节算一次约几十微秒, 不值得为它塞一张 1 KB 查表 */
static uint32_t Config_Crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i, b;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (b = 0; b < 8u; b++)
        {
            if (crc & 1u)
            {
                crc = (crc >> 1) ^ 0xEDB88320u;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

/* ============================ 地址换算 ============================ */

/* G4 的 Flash 可以配成 单 bank(128KB) 或 双 bank(每 bank 64KB)。
 * 同样的绝对地址 0x0801F800 对应的 bank / 页号不一样, 必须现读 OPTR 判断。 */
static void Config_Get_Erase_Info(uint32_t *bank, uint32_t *page)
{
    if ((FLASH->OPTR & FLASH_OPTR_DBANK) != 0u)
    {
        *bank = FLASH_BANK_2;
        *page = (CONFIG_FLASH_BASE - CONFIG_BANK2_BASE) / CONFIG_FLASH_PAGE_SZ; /* = 31 */
    }
    else
    {
        *bank = FLASH_BANK_1;
        *page = (CONFIG_FLASH_BASE - CONFIG_BANK1_BASE) / CONFIG_FLASH_PAGE_SZ; /* = 63 */
    }
}

/* ============================ 默认值 ============================ */

void Config_Set_Default(void)
{
    memset(&Config, 0, sizeof(Config));

    Config.Acc_Scale[0] = 1.0f;
    Config.Acc_Scale[1] = 1.0f;
    Config.Acc_Scale[2] = 1.0f;

    /* 安装方向: 单位阵(芯片坐标系 == 机体坐标系) */
    Config.Frame_R[0][0] = 1.0f;
    Config.Frame_R[1][1] = 1.0f;
    Config.Frame_R[2][2] = 1.0f;

    Config.Is_Calibrated = 0u;   /* RAM 里的默认值 ≠ Flash 里的有效校准 */
    Config_Seq           = 0u;
}

/* ============================ 读取 ============================ */

/* 返回 1 = Flash 里有有效数据且已放进 Config
 * 返回 0 = 没有 / 损坏 / 版本不符, 此时 Config 被填成默认值 */
uint8_t Config_Load(void)
{
    const Config_Header_t *hdr = (const Config_Header_t *)CONFIG_FLASH_BASE;
    const uint8_t         *pay = (const uint8_t *)(CONFIG_FLASH_BASE + sizeof(Config_Header_t));
    uint32_t crc;

    Config_Load_Ok = 0;

    if ((hdr->Magic   != CONFIG_MAGIC)       ||
        (hdr->Version != (uint16_t)CONFIG_VERSION) ||
        (hdr->Size    != CONFIG_PAYLOAD_SZ))
    {
        Config_Set_Default();
        return 0;
    }

    crc = Config_Crc32(pay, CONFIG_PAYLOAD_SZ);
    if (crc != hdr->Crc)
    {
        Config_Set_Default();
        return 0;
    }

    /* 直接内存映射拷贝: Flash 可以像普通 RAM 一样读, 不需要走 HAL */
    memcpy(&Config, pay, CONFIG_PAYLOAD_SZ);

    Config_Seq     = hdr->Seq;
    Config_Load_Ok = 1;
    return 1;
}

/* 兼容旧接口 */
void Config_Read(void)
{
    (void)Config_Load();
}

/* ============================ 保存 ============================ */

/* 返回 1 = 成功; 0 = 失败(擦除或编程出错, 回读不一致)
 * ⚠ 内部会擦整页, CPU 停顿 ~20~40 ms, 只允许在地面/解锁前调用 */
uint8_t Config_Save(void)
{
    Config_Header_t *hdr = (Config_Header_t *)Config_Buf;
    uint8_t         *pay = Config_Buf + sizeof(Config_Header_t);
    FLASH_EraseInitTypeDef erase;
    uint32_t bank, page, page_error = 0u;
    uint32_t offset;
    uint64_t dw;
    uint8_t  ok = 0u;

    Config_Save_Ok = 0;

    /* ---- 1. 拼记录 ---- */
    memset(Config_Buf, 0, sizeof(Config_Buf));
    memcpy(pay, &Config, CONFIG_PAYLOAD_SZ);

    hdr->Magic   = CONFIG_MAGIC;
    hdr->Version = (uint16_t)CONFIG_VERSION;
    hdr->Size    = CONFIG_PAYLOAD_SZ;
    hdr->Seq     = Config_Seq + 1u;
    hdr->Crc     = Config_Crc32(pay, CONFIG_PAYLOAD_SZ);

    /* ---- 2. 擦页 ---- */
    Config_Get_Erase_Info(&bank, &page);

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = bank;
    erase.Page      = page;
    erase.NbPages   = 1u;

    (void)HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 0u;
    }

    /* ---- 3. 按 8 字节双字编程 ---- */
    for (offset = 0u; offset < CONFIG_BUF_SZ; offset += 8u)
    {
        memcpy(&dw, &Config_Buf[offset], sizeof(dw));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              CONFIG_FLASH_BASE + offset, dw) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0u;
        }
    }

    HAL_FLASH_Lock();

    /* ---- 4. 回读校验 ---- */
    if (memcmp((const void *)CONFIG_FLASH_BASE, Config_Buf, CONFIG_BUF_SZ) == 0)
    {
        Config_Seq     = hdr->Seq;
        Config_Save_Ok = 1u;
        ok             = 1u;
    }

    return ok;
}

/* ============================ 上电初始化 ============================ */

void Flash_Init(void)
{
    (void)Config_Load();
}