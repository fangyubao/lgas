#ifndef __LOOP_PROFILER_H
#define __LOOP_PROFILER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void loop_profiler_init(uint32_t print_period_ms);
void loop_profiler_taskDelay(void);
void loop_profiler_begin(void);
void loop_profiler_end(void);

#ifdef __cplusplus
}
#endif

#endif /* __LOOP_PROFILER_H */
