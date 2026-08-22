#ifndef __FLASH_STORAGE_H
#define __FLASH_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FIT_USE_BLOCK   0U
#define ANALY_BLOCK     1U

#define PARAM_GROUP_CNT  4U
#define PARAM_FLOATS_PER_GROUP 2U

int storage_param_write(const void *data, uint32_t len);
int storage_param_read(void *data, uint32_t len);
int storage_param_write_group(uint8_t group, float val1, float val2);
int storage_param_read_all(float data[PARAM_GROUP_CNT][PARAM_FLOATS_PER_GROUP]);

int storage_log_write(const void *data, uint32_t len);
int storage_log_read(void *data, uint32_t len);
int storage_log_size(void);
int storage_log_read_offset(uint32_t offset, void *data, uint32_t len);
int storage_log_erase(void);

void storage_set_addr(uint32_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_STORAGE_H */
