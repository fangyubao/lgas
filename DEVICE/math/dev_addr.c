#include "dev_addr.h"
#include <string.h>
#include "stm32f1xx_hal.h"

#define CFG_PAGE        0x0803F800U
#define CFG_MAGIC       0x5A5ACFCFU

int dev_addr_save(uint32_t addr)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error;

    HAL_FLASH_Unlock();

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = CFG_PAGE;
    erase.NbPages     = 1U;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -1;
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CFG_PAGE, CFG_MAGIC) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -2;
    }

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CFG_PAGE + 4U, addr) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return -3;
    }

    HAL_FLASH_Lock();
    return 0;
}

uint32_t dev_addr_load(void)
{
    const uint32_t *p = (const uint32_t *)CFG_PAGE;

    if (p[0] != CFG_MAGIC)
        return 0U;

    return p[1];
}
