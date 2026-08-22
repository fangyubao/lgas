#include "mem_mgr.h"
#include "tlsf.h"
#include "stdint.h"
#define APP_TLSF_POOL_BYTES 4096U

typedef union {
    uint64_t align;
    unsigned char bytes[APP_TLSF_POOL_BYTES];
} app_tlsf_pool_t;

static app_tlsf_pool_t s_pool;
static tlsf_t s_tlsf;

void app_mem_init(void)
{
    if (s_tlsf == NULL)
    {
        s_tlsf = tlsf_create_with_pool(s_pool.bytes, sizeof(s_pool.bytes));
    }
}

void *app_malloc(size_t size)
{
    if ((s_tlsf == NULL) || (size == 0U))
    {
        return NULL;
    }

    return tlsf_malloc(s_tlsf, size);
}

void *app_realloc(void *ptr, size_t size)
{
    if (s_tlsf == NULL)
    {
        return NULL;
    }

    return tlsf_realloc(s_tlsf, ptr, size);
}

void app_free(void *ptr)
{
    if ((s_tlsf != NULL) && (ptr != NULL))
    {
        tlsf_free(s_tlsf, ptr);
    }
}
