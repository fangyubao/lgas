#ifndef __STATS_H
#define __STATS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

float stats_getMean(const float *data, uint16_t len);
float stats_getSlope(const float *data, uint16_t len);
float stats_getStability(const float *data, uint16_t len);
float stats_getFilteredMean(const float *data, uint16_t len, float stability_threshold, float *stddeva);
void stats_floatToBytes(float value, uint8_t *out);
uint8_t bcd2dec(uint8_t bcd);
#ifdef __cplusplus
}
#endif

#endif /* __STATS_H */
