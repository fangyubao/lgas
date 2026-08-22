#include "stats.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

uint8_t bcd2dec(uint8_t bcd) {
    return ((bcd >> 4) & 0x0F) * 10 + (bcd & 0x0F);
}

static float stats_getStdDev(const float *data, uint16_t len, float mean)
{
  double sum_sq = 0.0;
  uint16_t i;

  if ((data == 0) || (len == 0U))
  {
    return 0.0f;
  }

  for (i = 0U; i < len; i++)
  {
    double diff = (double)data[i] - (double)mean;
    sum_sq += diff * diff;
  }

  return (float)sqrt(sum_sq / (double)len);
}

float stats_getMean(const float *data, uint16_t len)
{
  double sum = 0.0;
  uint16_t i;

  if ((data == 0) || (len == 0U))
  {
    return 0.0f;
  }

  for (i = 0U; i < len; i++)
  {
    sum += data[i];
  }

  return (float)(sum / (double)len);
}

float stats_getSlope(const float *data, uint16_t len)
{
  double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
  uint16_t i;

  if ((data == 0) || (len < 2U))
  {
    return 0.0f;
  }

  for (i = 0U; i < len; i++)
  {
    double x = (double)i;
    double y = (double)data[i];
    sum_x  += x;
    sum_y  += y;
    sum_xy += x * y;
    sum_xx += x * x;
  }

  double denom = (double)len * sum_xx - sum_x * sum_x;
  if (denom <= (double)FLT_EPSILON)
  {
    return 0.0f;
  }

  return (float)(((double)len * sum_xy - sum_x * sum_y) / denom);
}

float stats_getStability(const float *data, uint16_t len)
{
  float mean;
  float stddev;

  if ((data == 0) || (len == 0U))
  {
    return 0.0f;
  }

  mean = stats_getMean(data, len);
  stddev = stats_getStdDev(data, len, mean);

  if (stddev <= FLT_EPSILON)
  {
    return FLT_MAX;
  }

  return fabsf(mean) / stddev;
}

float stats_getFilteredMean(const float *data, uint16_t len, float stability_threshold, float *stddeva)
{
  float mean;
  float raw_stddev;
  double filtered_sum = 0.0;
  uint16_t filtered_count = 0U;
  uint16_t i;

  if ((data == 0) || (len == 0U))
  {
    return 0.0f;
  }

  mean = stats_getMean(data, len);
  raw_stddev = stats_getStdDev(data, len, mean);

  if (stddeva != NULL)
  {
    float abs_mean = fabsf(mean);
    stddeva[0] = (abs_mean > FLT_EPSILON) ? (raw_stddev / abs_mean) : raw_stddev;  /* [0] = CV = σ/μ */
    stddeva[1] = stats_getSlope(data, len);                                        /* [1] = slope */
  }

  /* Internal SNR gate (|μ|/σ) */
  if (stats_getStability(data, len) >= stability_threshold)
  {
    return mean;
  }

  /* Outlier filter uses raw σ (correct) */
  if (raw_stddev <= FLT_EPSILON)
  {
    return mean;
  }

  for (i = 0U; i < len; i++)
  {
    if (fabsf(data[i] - mean) <= (2.0f * raw_stddev))
    {
      filtered_sum += data[i];
      filtered_count++;
    }
  }

  if (filtered_count == 0U)
  {
    return mean;
  }

  return (float)(filtered_sum / (double)filtered_count);
}

void stats_floatToBytes(float value, uint8_t *out)
{
  union
  {
    float f;
    uint8_t b[4];
  } conv;

  uint16_t i;

  if (out == 0)
  {
    return;
  }

  conv.f = value;
  for (i = 0U; i < 4U; i++)
  {
    out[i] = conv.b[3U - i];
  }
}
