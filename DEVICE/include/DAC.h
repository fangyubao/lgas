#ifndef __DAC_APP_H
#define __DAC_APP_H

#include <stdint.h>

uint8_t DAC_OutputInit(void);
uint8_t DAC_OutputSetMv(uint16_t target_mv);

#endif /* __DAC_APP_H */
