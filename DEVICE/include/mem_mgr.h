#ifndef __MEM_MGR_H
#define __MEM_MGR_H

#include <stddef.h>

void app_mem_init(void);
void *app_malloc(size_t size);
void *app_realloc(void *ptr, size_t size);
void app_free(void *ptr);

#endif
