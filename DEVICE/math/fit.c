#include "fit.h"

#include <math.h>

static float fit_calc_r2_linear(const float *x, const float *y, uint16_t len, float k, float b)
{
    double y_mean = 0.0;
    double ss_res = 0.0;
    double ss_tot = 0.0;
    uint16_t i;

    for (i = 0U; i < len; i++)
    {
        y_mean += y[i];
    }
    y_mean /= (double)len;

    for (i = 0U; i < len; i++)
    {
        double y_fit = (double)k * x[i] + (double)b;
        double res = (double)y[i] - y_fit;
        double tot = (double)y[i] - y_mean;
        ss_res += res * res;
        ss_tot += tot * tot;
    }

    if (ss_tot <= 1e-12)
    {
        return (ss_res <= 1e-12) ? 1.0f : 0.0f;
    }

    return (float)(1.0 - (ss_res / ss_tot));
}

static float fit_calc_r2_quadratic(const float *x, const float *y, uint16_t len, float a, float b, float c)
{
    double y_mean = 0.0;
    double ss_res = 0.0;
    double ss_tot = 0.0;
    uint16_t i;

    for (i = 0U; i < len; i++)
    {
        y_mean += y[i];
    }
    y_mean /= (double)len;

    for (i = 0U; i < len; i++)
    {
        double xi = x[i];
        double y_fit = (double)a * xi * xi + (double)b * xi + (double)c;
        double res = (double)y[i] - y_fit;
        double tot = (double)y[i] - y_mean;
        ss_res += res * res;
        ss_tot += tot * tot;
    }

    if (ss_tot <= 1e-12)
    {
        return (ss_res <= 1e-12) ? 1.0f : 0.0f;
    }

    return (float)(1.0 - (ss_res / ss_tot));
}

static uint8_t fit_solve_3x3(double a[3][4], double out[3])
{
    uint8_t i;
    uint8_t j;
    uint8_t k;

    for (i = 0U; i < 3U; i++)
    {
        uint8_t pivot = i;
        double max_abs = fabs(a[i][i]);

        for (j = (uint8_t)(i + 1U); j < 3U; j++)
        {
            double cur_abs = fabs(a[j][i]);
            if (cur_abs > max_abs)
            {
                max_abs = cur_abs;
                pivot = j;
            }
        }

        if (max_abs <= 1e-12)
        {
            return 0U;
        }

        if (pivot != i)
        {
            for (k = i; k < 4U; k++)
            {
                double tmp = a[i][k];
                a[i][k] = a[pivot][k];
                a[pivot][k] = tmp;
            }
        }

        for (j = (uint8_t)(i + 1U); j < 3U; j++)
        {
            double factor = a[j][i] / a[i][i];
            for (k = i; k < 4U; k++)
            {
                a[j][k] -= factor * a[i][k];
            }
        }
    }

    for (i = 3U; i > 0U; i--)
    {
        uint8_t row = (uint8_t)(i - 1U);
        double sum = a[row][3];
        for (j = (uint8_t)(row + 1U); j < 3U; j++)
        {
            sum -= a[row][j] * out[j];
        }
        if (fabs(a[row][row]) <= 1e-12)
        {
            return 0U;
        }
        out[row] = sum / a[row][row];
    }

    return 1U;
}

uint8_t fit_linear(const float *x, const float *y, uint16_t len, fit_linear_result_t *result)
{
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    double denom;
    uint16_t i;

    if ((x == 0) || (y == 0) || (result == 0) || (len < 2U))
    {
        return 0U;
    }

    for (i = 0U; i < len; i++)
    {
        sum_x += x[i];
        sum_y += y[i];
        sum_xx += (double)x[i] * x[i];
        sum_xy += (double)x[i] * y[i];
    }

    denom = (double)len * sum_xx - sum_x * sum_x;
    if (fabs(denom) <= 1e-12)
    {
        return 0U;
    }

    result->k = (float)(((double)len * sum_xy - sum_x * sum_y) / denom);
    result->b = (float)((sum_y - (double)result->k * sum_x) / (double)len);
    result->r2 = fit_calc_r2_linear(x, y, len, result->k, result->b);

    return 1U;
}

uint8_t fit_quadratic(const float *x, const float *y, uint16_t len, fit_quadratic_result_t *result)
{
    double sum_x = 0.0;
    double sum_x2 = 0.0;
    double sum_x3 = 0.0;
    double sum_x4 = 0.0;
    double sum_y = 0.0;
    double sum_xy = 0.0;
    double sum_x2y = 0.0;
    double mat[3][4];
    double coeff[3] = {0.0, 0.0, 0.0};
    uint16_t i;

    if ((x == 0) || (y == 0) || (result == 0) || (len < 3U))
    {
        return 0U;
    }

    for (i = 0U; i < len; i++)
    {
        double xi = x[i];
        double yi = y[i];
        double x2 = xi * xi;

        sum_x += xi;
        sum_x2 += x2;
        sum_x3 += x2 * xi;
        sum_x4 += x2 * x2;
        sum_y += yi;
        sum_xy += xi * yi;
        sum_x2y += x2 * yi;
    }

    mat[0][0] = sum_x4;
    mat[0][1] = sum_x3;
    mat[0][2] = sum_x2;
    mat[0][3] = sum_x2y;
    mat[1][0] = sum_x3;
    mat[1][1] = sum_x2;
    mat[1][2] = sum_x;
    mat[1][3] = sum_xy;
    mat[2][0] = sum_x2;
    mat[2][1] = sum_x;
    mat[2][2] = (double)len;
    mat[2][3] = sum_y;

    if (fit_solve_3x3(mat, coeff) == 0U)
    {
        return 0U;
    }

    result->a = (float)coeff[0];
    result->b = (float)coeff[1];
    result->c = (float)coeff[2];
    result->r2 = fit_calc_r2_quadratic(x, y, len, result->a, result->b, result->c);

    return 1U;
}
