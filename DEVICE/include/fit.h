#ifndef __FIT_H
#define __FIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float k;
    float b;
    float r2;
} fit_linear_result_t;

typedef struct
{
    float a;
    float b;
    float c;
    float r2;
} fit_quadratic_result_t;

uint8_t fit_linear(const float *x, const float *y, uint16_t len, fit_linear_result_t *result);
uint8_t fit_quadratic(const float *x, const float *y, uint16_t len, fit_quadratic_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* __FIT_H */
