#include "flash_storage.h"
#include <string.h>
#include "stm32f1xx_hal.h"

#ifndef STORAGE_ADDR
#define STORAGE_ADDR    0x08036000U
#endif

#define PAGE_SIZE       2048U
#define PARAM_PAGES     1U
#define LOG_PAGES       9U
#define LOG_OFFSET      (PARAM_PAGES * PAGE_SIZE)

#define HEADER_MAGIC    4U
#define HEADER_LEN      4U
#define HEADER_SIZE     (HEADER_MAGIC + HEADER_LEN)
#define STORAGE_MAGIC   0xA5A55A5AU
#define LOG_ENTRY_MAGIC 0xA5A5A5A5U

#define PARAM_DATA_SIZE (PARAM_GROUP_CNT * PARAM_FLOATS_PER_GROUP * 4U)

#define LOG_TOTAL_SIZE  (LOG_PAGES * PAGE_SIZE)

static uint32_t s_addr = 0U;

/* compact buffer: 18KB, only used during log compaction */
static uint8_t s_compact_buf[LOG_TOTAL_SIZE];

static uint32_t storage_addr(void)
{
    return (s_addr != 0U) ? s_addr : STORAGE_ADDR;
}

static int flash_program_buffer(uint32_t addr, const void *data, uint32_t len)
{
    const uint8_t *src = (const uint8_t *)data;
    uint32_t offset = 0U;
    uint32_t aligned = (len + 3U) & ~3U;

    while (offset < aligned)
    {
        uint32_t word = 0xFFFFFFFFU;
        uint32_t chunk = (offset < len) ? (len - offset) : 0U;
        if (chunk > sizeof(word))
            chunk = sizeof(word);
        if (chunk > 0U)
            memcpy(&word, &src[offset], chunk);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + offset, word) != HAL_OK)
            return -6;
        offset += sizeof(word);
    }

    return 0;
}

void storage_set_addr(uint32_t addr)
{
    s_addr = addr;
}

/*****************************************************************************
 * Param helpers
 *****************************************************************************/
static int param_erase_write(const void *data, uint32_t len)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error;
    uint32_t aligned;
    uint32_t addr;

    if ((data == NULL) || (len == 0U))
        return -1;
    if (len > (PAGE_SIZE - HEADER_SIZE))
        return -2;

    aligned = (len + 3U) & ~3U;

    HAL_FLASH_Unlock();

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = storage_addr();
    erase.NbPages     = PARAM_PAGES;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
    { HAL_FLASH_Lock(); return -3; }

    addr = storage_addr();
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, STORAGE_MAGIC) != HAL_OK)
    { HAL_FLASH_Lock(); return -4; }
    addr += 4U;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, aligned) != HAL_OK)
    { HAL_FLASH_Lock(); return -5; }
    addr += 4U;

    if (flash_program_buffer(addr, data, len) != 0)
    { HAL_FLASH_Lock(); return -6; }

    HAL_FLASH_Lock();
    return 0;
}

static int param_read_raw(void *data, uint32_t len)
{
    const uint32_t *flash = (const uint32_t *)storage_addr();
    uint32_t stored, copy;

    if ((data == NULL) || (len == 0U))
        return -1;
    if (flash[0] != STORAGE_MAGIC)
        return -2;

    stored = flash[1];
    if ((stored == 0U) || (stored == 0xFFFFFFFFU) ||
        (stored > (PAGE_SIZE - HEADER_SIZE)))
        return -3;

    copy = (len > stored) ? stored : len;
    memcpy(data, (const void *)(storage_addr() + HEADER_SIZE), copy);
    if (copy < len)
        memset((uint8_t *)data + copy, 0, len - copy);
    return 0;
}

/*****************************************************************************
 * Public param API
 *****************************************************************************/
int storage_param_write(const void *data, uint32_t len)
{
    return param_erase_write(data, len);
}

int storage_param_read(void *data, uint32_t len)
{
    return param_read_raw(data, len);
}

int storage_param_write_group(uint8_t group, float val1, float val2)
{
    uint8_t buf[PARAM_DATA_SIZE];
    int ret;

    if ((group < 1U) || (group > PARAM_GROUP_CNT))
        return -7;

    ret = param_read_raw(buf, sizeof(buf));
    if ((ret != 0) && (ret != -2) && (ret != -3))
        return ret;
    if (ret != 0)
        memset(buf, 0, sizeof(buf));

    memcpy(&buf[(group - 1U) * 8U], &val1, sizeof(val1));
    memcpy(&buf[(group - 1U) * 8U + sizeof(val1)], &val2, sizeof(val2));

    return param_erase_write(buf, sizeof(buf));
}

int storage_param_read_all(float data[PARAM_GROUP_CNT][PARAM_FLOATS_PER_GROUP])
{
    const uint32_t *flash = (const uint32_t *)storage_addr();
    if (flash[1] != PARAM_DATA_SIZE)
        return -3;
    return param_read_raw(data, PARAM_DATA_SIZE);
}

/*****************************************************************************
 * Log helpers
 *****************************************************************************/
static uint32_t log_base(void)
{
    return storage_addr() + LOG_OFFSET;
}

static int32_t log_scan_write_offset(void)
{
    uint32_t off = 0U;

    while (off < LOG_TOTAL_SIZE)
    {
        const uint32_t *p = (const uint32_t *)(log_base() + off);

        if (p[0] != LOG_ENTRY_MAGIC)
            return (int32_t)off;

        uint32_t entry_len = p[1];
        if ((entry_len == 0U) || (entry_len == 0xFFFFFFFFU))
            return (int32_t)off;
        if (entry_len > (LOG_TOTAL_SIZE - HEADER_SIZE))
            return (int32_t)off;

        uint32_t aligned = (entry_len + 3U) & ~3U;
        uint32_t total = HEADER_SIZE + aligned;
        if (off + total > LOG_TOTAL_SIZE)
            return (int32_t)off;

        off += total;
    }
    return (int32_t)off;
}

static int log_write_entry(uint32_t offset, const void *data, uint32_t len)
{
    uint32_t addr = log_base() + offset;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, LOG_ENTRY_MAGIC) != HAL_OK)
        return -4;
    addr += 4U;

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, len) != HAL_OK)
        return -5;
    addr += 4U;

    return flash_program_buffer(addr, data, len);
}

static int log_erase_all(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error;

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = log_base();
    erase.NbPages     = LOG_PAGES;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
        return -3;
    return 0;
}

/* Scan log region, returning info needed for compaction:
 *   p_total   — total bytes consumed by all valid entries
 *   p_skip    — bytes to skip for oldest entry (0 if none)
 *   p_keep    — bytes to preserve (p_total - p_skip)
 */
static void log_scan_entries(uint32_t *p_total, uint32_t *p_skip, uint32_t *p_keep)
{
    uint32_t off = 0U;
    uint32_t first_total = 0U;
    uint32_t entry_count = 0U;

    while (off < LOG_TOTAL_SIZE)
    {
        const uint32_t *p = (const uint32_t *)(log_base() + off);
        if (p[0] != LOG_ENTRY_MAGIC)
            break;

        uint32_t entry_len = p[1];
        if ((entry_len == 0U) || (entry_len == 0xFFFFFFFFU))
            break;
        if (entry_len > (LOG_TOTAL_SIZE - HEADER_SIZE))
            break;

        uint32_t aligned = (entry_len + 3U) & ~3U;
        uint32_t total = HEADER_SIZE + aligned;
        if (off + total > LOG_TOTAL_SIZE)
            break;

        if (entry_count == 0U)
            first_total = total;

        off += total;
        entry_count++;
    }

    *p_total = off;
    *p_skip  = (entry_count > 0U) ? first_total : 0U;
    *p_keep  = (entry_count > 0U) ? (off - first_total) : 0U;
}

/* Compact: discard oldest entries, preserve rest, append new entry.
 * Uses s_compact_buf to buffer preserved data before erase.
 * On entry, skip = bytes consumed by oldest entry, keep = total - skip.
 * If more space needed, iteratively drop entries from skip forward.
 */
static int log_compact(const void *data, uint32_t len)
{
    uint32_t total, skip, keep;
    uint32_t aligned_new = (len + 3U) & ~3U;
    uint32_t needed_new = HEADER_SIZE + aligned_new;

    log_scan_entries(&total, &skip, &keep);

    if (keep == 0U)
        goto fresh_start;

    /* Drop entries from the front until remaining + new entry fit */
    while (keep + needed_new > LOG_TOTAL_SIZE)
    {
        uint32_t dropped;
        const uint32_t *p = (const uint32_t *)(log_base() + skip);
        if (p[0] != LOG_ENTRY_MAGIC)
            goto fresh_start;
        uint32_t entry_len = p[1];
        if ((entry_len == 0U) || (entry_len == 0xFFFFFFFFU))
            goto fresh_start;
        if (entry_len > (LOG_TOTAL_SIZE - HEADER_SIZE))
            goto fresh_start;
        uint32_t aligned = (entry_len + 3U) & ~3U;
        dropped = HEADER_SIZE + aligned;
        if (dropped >= keep)
            goto fresh_start;
        keep  -= dropped;
        skip  += dropped;
    }

    memcpy(s_compact_buf, (const void *)(log_base() + skip), keep);

    HAL_FLASH_Unlock();
    if (log_erase_all() != 0)
    { HAL_FLASH_Lock(); return -3; }

    {
        uint32_t addr = log_base();
        if (flash_program_buffer(addr, s_compact_buf, keep) != 0)
        { HAL_FLASH_Lock(); return -6; }
    }

    if (log_write_entry(keep, data, len) != 0)
    { HAL_FLASH_Lock(); return -6; }

    HAL_FLASH_Lock();
    return 0;

fresh_start:
    HAL_FLASH_Unlock();
    if (log_erase_all() != 0)
    { HAL_FLASH_Lock(); return -3; }
    {
        int ret = log_write_entry(0U, data, len);
        HAL_FLASH_Lock();
        return ret;
    }
}

/*****************************************************************************
 * Public log API
 *****************************************************************************/
int storage_log_write(const void *data, uint32_t len)
{
    int32_t woff;
    uint32_t remaining;
    uint32_t aligned;
    uint32_t needed;
    int ret;

    if ((data == NULL) || (len == 0U))
        return -1;

    aligned = (len + 3U) & ~3U;
    needed  = HEADER_SIZE + aligned;

    if (needed > LOG_TOTAL_SIZE)
        return -2;

    woff = log_scan_write_offset();
    if (woff < 0)
        woff = 0;

    remaining = LOG_TOTAL_SIZE - (uint32_t)woff;
    if (needed <= remaining)
    {
        HAL_FLASH_Unlock();
        ret = log_write_entry((uint32_t)woff, data, len);
        HAL_FLASH_Lock();
        return ret;
    }

    return log_compact(data, len);
}

int storage_log_read(void *data, uint32_t len)
{
    int32_t end;
    uint32_t copy;

    end = log_scan_write_offset();
    if (end <= 0)
        return (end < 0) ? end : 0;

    copy = ((uint32_t)end > len) ? len : (uint32_t)end;
    memcpy(data, (const void *)log_base(), copy);
    return (int)copy;
}

int storage_log_size(void)
{
    int32_t end = log_scan_write_offset();
    return (end < 0) ? 0 : (int)end;
}

int storage_log_erase(void)
{
    int ret;
    HAL_FLASH_Unlock();
    ret = log_erase_all();
    HAL_FLASH_Lock();
    return ret;
}

int storage_log_read_offset(uint32_t offset, void *data, uint32_t len)
{
    int32_t end = log_scan_write_offset();
    if (end <= 0)
        return (end < 0) ? end : 0;
    if (offset >= (uint32_t)end)
        return -4;

    uint32_t copy = (len > (uint32_t)end - offset) ? ((uint32_t)end - offset) : len;
    memcpy(data, (const void *)(log_base() + offset), copy);
    return (int)copy;
}
