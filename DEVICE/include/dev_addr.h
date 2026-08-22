#ifndef __DEV_ADDR_H
#define __DEV_ADDR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int dev_addr_save(uint32_t addr);
uint32_t dev_addr_load(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEV_ADDR_H */
