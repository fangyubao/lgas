#include "rtc.h"

#include "main.h"
#include "usbd_cdc_if.h"

#include <stdio.h>

#define RTC_DEFAULT_YEAR           2024U
#define RTC_DEFAULT_MONTH          1U
#define RTC_DEFAULT_DATE           1U
#define RTC_DEFAULT_HOUR           0U
#define RTC_DEFAULT_MIN            0U
#define RTC_DEFAULT_SEC            0U

static uint8_t rtc_is_leap_year(uint16_t year)
{
    if (((year % 4U) == 0U && (year % 100U) != 0U) || ((year % 400U) == 0U))
    {
        return 1U;
    }

    return 0U;
}

static uint8_t rtc_get_week(uint16_t year, uint8_t month, uint8_t day)
{
    uint8_t week;

    if (month < 3U)
    {
        month = (uint8_t)(month + 12U);
        year--;
    }

    week = (uint8_t)((day + 1U + 2U * month + 3U * (month + 1U) / 5U + year + year / 4U - year / 100U + year / 400U) % 7U);
    return week;
}

static uint8_t rtc_datetime_is_valid(const rtc_datetime_t *datetime)
{
    static const uint8_t month_table[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
    uint8_t max_day;

    if (datetime == NULL)
    {
        return 0U;
    }

    if (datetime->year < 1970U || datetime->year > 2099U)
    {
        return 0U;
    }

    if (datetime->month < 1U || datetime->month > 12U)
    {
        return 0U;
    }

    max_day = month_table[datetime->month - 1U];
    if (datetime->month == 2U && rtc_is_leap_year(datetime->year) != 0U)
    {
        max_day = 29U;
    }

    if (datetime->date < 1U || datetime->date > max_day)
    {
        return 0U;
    }

    if (datetime->hour > 23U || datetime->min > 59U || datetime->sec > 59U)
    {
        return 0U;
    }

    return 1U;
}

#if (RTC_BACKEND_SELECT == RTC_BACKEND_DS1302)

#define DS1302_GPIO_PORT           GPIOC
#define DS1302_PIN_SCLK            GPIO_PIN_13
#define DS1302_PIN_IO              GPIO_PIN_14
#define DS1302_PIN_RST             GPIO_PIN_15

#define DS1302_CMD_SEC_W           0x80U
#define DS1302_CMD_SEC_R           0x81U
#define DS1302_CMD_MIN_W           0x82U
#define DS1302_CMD_MIN_R           0x83U
#define DS1302_CMD_HOUR_W          0x84U
#define DS1302_CMD_HOUR_R          0x85U
#define DS1302_CMD_DATE_W          0x86U
#define DS1302_CMD_DATE_R          0x87U
#define DS1302_CMD_MONTH_W         0x88U
#define DS1302_CMD_MONTH_R         0x89U
#define DS1302_CMD_DAY_W           0x8AU
#define DS1302_CMD_YEAR_W          0x8CU
#define DS1302_CMD_YEAR_R          0x8DU
#define DS1302_CMD_WP_W            0x8EU

static void rtc_debug_report(const char *reason,
                             uint8_t sec_raw,
                             uint8_t min_raw,
                             uint8_t hour_raw,
                             uint8_t date_raw,
                             uint8_t month_raw,
                             uint8_t year_raw)
{
    char buf[192];
    int len;

    len = snprintf(buf, sizeof(buf),
                   "RTC_DBG reason=%s raw=%02X,%02X,%02X,%02X,%02X,%02X PC_IDR=%04lX PC_ODR=%04lX PC_CRH=%08lX\r\n",
                   (reason != NULL) ? reason : "unknown",
                   (unsigned)sec_raw,
                   (unsigned)min_raw,
                   (unsigned)hour_raw,
                   (unsigned)date_raw,
                   (unsigned)month_raw,
                   (unsigned)year_raw,
                   (unsigned long)DS1302_GPIO_PORT->IDR,
                   (unsigned long)DS1302_GPIO_PORT->ODR,
                   (unsigned long)DS1302_GPIO_PORT->CRH);
    if (len > 0)
    {
        (void)usb_cdc_send_packet((uint8_t *)buf, (uint16_t)len);
    }
}

static uint8_t rtc_bin2bcd(uint8_t value)
{
    return (uint8_t)(((value / 10U) << 4) | (value % 10U));
}

static uint8_t rtc_bcd2bin(uint8_t value)
{
    return (uint8_t)(((value >> 4) * 10U) + (value & 0x0FU));
}

static void ds1302_delay(void)
{
    volatile uint8_t i;
    for (i = 0U; i < 6U; i++)
    {
        __NOP();
    }
}

static void ds1302_io_out(void)
{
    GPIO_InitTypeDef init = {0};
    init.Pin = DS1302_PIN_IO;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS1302_GPIO_PORT, &init);
}

static void ds1302_io_in(void)
{
    GPIO_InitTypeDef init = {0};
    init.Pin = DS1302_PIN_IO;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DS1302_GPIO_PORT, &init);
}

static void ds1302_write_byte(uint8_t value)
{
    uint8_t i;

    ds1302_io_out();
    for (i = 0U; i < 8U; i++)
    {
        HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SCLK, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_IO, ((value & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        ds1302_delay();
        HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SCLK, GPIO_PIN_SET);
        ds1302_delay();
        value >>= 1U;
    }
}

static uint8_t ds1302_read_byte(void)
{
    uint8_t i;
    uint8_t value = 0U;

    ds1302_io_in();
    for (i = 0U; i < 8U; i++)
    {
        HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SCLK, GPIO_PIN_RESET);
        ds1302_delay();
        if (HAL_GPIO_ReadPin(DS1302_GPIO_PORT, DS1302_PIN_IO) == GPIO_PIN_SET)
        {
            value |= (uint8_t)(1U << i);
        }
        HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SCLK, GPIO_PIN_SET);
        ds1302_delay();
    }

    return value;
}

static void ds1302_write_reg(uint8_t cmd, uint8_t data)
{
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SCLK, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_RST, GPIO_PIN_SET);
    ds1302_write_byte((uint8_t)(cmd & 0xFEU));
    ds1302_write_byte(data);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_RST, GPIO_PIN_RESET);
}

static uint8_t ds1302_read_reg(uint8_t cmd)
{
    uint8_t value;

    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SCLK, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_RST, GPIO_PIN_SET);
    ds1302_write_byte((uint8_t)(cmd | 0x01U));
    value = ds1302_read_byte();
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_RST, GPIO_PIN_RESET);
    return value;
}

static void ds1302_gpio_init(void)
{
    GPIO_InitTypeDef init = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    init.Pin = DS1302_PIN_SCLK | DS1302_PIN_RST;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DS1302_GPIO_PORT, &init);

    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_SCLK, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_RST, GPIO_PIN_RESET);

    ds1302_io_out();
    HAL_GPIO_WritePin(DS1302_GPIO_PORT, DS1302_PIN_IO, GPIO_PIN_RESET);
}

HAL_StatusTypeDef rtc_init(void)
{
    rtc_datetime_t current;
    rtc_datetime_t default_datetime;

    ds1302_gpio_init();
    ds1302_write_reg(DS1302_CMD_WP_W, 0x00U);

    if (rtc_get_datetime(&current) == HAL_OK)
    {
        return HAL_OK;
    }

    default_datetime.year = RTC_DEFAULT_YEAR;
    default_datetime.month = RTC_DEFAULT_MONTH;
    default_datetime.date = RTC_DEFAULT_DATE;
    default_datetime.hour = RTC_DEFAULT_HOUR;
    default_datetime.min = RTC_DEFAULT_MIN;
    default_datetime.sec = RTC_DEFAULT_SEC;

    return rtc_set_datetime(&default_datetime);
}

HAL_StatusTypeDef rtc_get_datetime(rtc_datetime_t *datetime)
{
    uint8_t sec_raw;
    uint8_t sec_bcd;
    uint8_t min_bcd;
    uint8_t hour_bcd;
    uint8_t date_bcd;
    uint8_t month_bcd;
    uint8_t year_bcd;
    uint16_t year;

    if (datetime == NULL)
    {
        return HAL_ERROR;
    }

    sec_raw = ds1302_read_reg(DS1302_CMD_SEC_R);
    if ((sec_raw & 0x80U) != 0U)
    {
        rtc_debug_report("CH", sec_raw, 0U, 0U, 0U, 0U, 0U);
        return HAL_ERROR;
    }
    sec_bcd = (uint8_t)(sec_raw & 0x7FU);
    min_bcd = ds1302_read_reg(DS1302_CMD_MIN_R);
    hour_bcd = ds1302_read_reg(DS1302_CMD_HOUR_R);
    date_bcd = ds1302_read_reg(DS1302_CMD_DATE_R);
    month_bcd = ds1302_read_reg(DS1302_CMD_MONTH_R);
    year_bcd = ds1302_read_reg(DS1302_CMD_YEAR_R);

    datetime->sec = rtc_bcd2bin(sec_bcd);
    datetime->min = rtc_bcd2bin((uint8_t)(min_bcd & 0x7FU));
    datetime->hour = rtc_bcd2bin((uint8_t)(hour_bcd & 0x3FU));
    datetime->date = rtc_bcd2bin((uint8_t)(date_bcd & 0x3FU));
    datetime->month = rtc_bcd2bin((uint8_t)(month_bcd & 0x1FU));

    year = rtc_bcd2bin(year_bcd);
    datetime->year = (year >= 70U) ? (uint16_t)(1900U + year) : (uint16_t)(2000U + year);

    if (rtc_datetime_is_valid(datetime) == 0U)
    {
        rtc_debug_report("INVALID", sec_raw, min_bcd, hour_bcd, date_bcd, month_bcd, year_bcd);
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef rtc_set_datetime(const rtc_datetime_t *datetime)
{
    uint8_t week;
    uint8_t year2;

    if (rtc_datetime_is_valid(datetime) == 0U)
    {
        return HAL_ERROR;
    }

    year2 = (uint8_t)(datetime->year % 100U);
    week = rtc_get_week(datetime->year, datetime->month, datetime->date);
    if (week == 0U)
    {
        week = 7U;
    }

    ds1302_write_reg(DS1302_CMD_WP_W, 0x00U);
    ds1302_write_reg(DS1302_CMD_SEC_W, (uint8_t)(rtc_bin2bcd(datetime->sec) & 0x7FU));
    ds1302_write_reg(DS1302_CMD_MIN_W, rtc_bin2bcd(datetime->min));
    ds1302_write_reg(DS1302_CMD_HOUR_W, rtc_bin2bcd(datetime->hour));
    ds1302_write_reg(DS1302_CMD_DATE_W, rtc_bin2bcd(datetime->date));
    ds1302_write_reg(DS1302_CMD_MONTH_W, rtc_bin2bcd(datetime->month));
    ds1302_write_reg(DS1302_CMD_DAY_W, rtc_bin2bcd(week));
    ds1302_write_reg(DS1302_CMD_YEAR_W, rtc_bin2bcd(year2));
    ds1302_write_reg(DS1302_CMD_WP_W, 0x80U);

    return HAL_OK;
}

#else

#define RTC_BKP_MAGIC              0x5050U

typedef struct
{
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint16_t year;
    uint8_t month;
    uint8_t date;
    uint8_t week;
} rtc_calendar_t;

extern RTC_HandleTypeDef hrtc;

static rtc_calendar_t s_calendar;

static uint16_t rtc_read_bkr(uint32_t bkr)
{
    return (uint16_t)HAL_RTCEx_BKUPRead(&hrtc, bkr);
}

static void rtc_write_bkr(uint32_t bkr, uint16_t data)
{
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, bkr, data);
}

static long rtc_date2sec(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
    uint32_t y;
    uint32_t m;
    uint32_t d;
    uint32_t x;
    uint32_t total;
    int8_t mon = (int8_t)month;

    if (0 >= (mon -= 2))
    {
        mon = (int8_t)(mon + 12);
        year--;
    }

    y = (uint32_t)(year - 1U) * 365U + year / 4U - year / 100U + year / 400U;
    m = (uint32_t)(367 * mon / 12 - 30 + 59);
    d = (uint32_t)day - 1U;
    x = y + m + d - 719162U;
    total = ((x * 24U + hour) * 60U + min) * 60U + sec;

    return (long)total;
}

static HAL_StatusTypeDef rtc_refresh_calendar(void)
{
    static const uint8_t month_table[12] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
    static uint32_t cached_day = 0xFFFFFFFFUL;
    uint32_t sec_count;
    uint32_t day_count;
    uint32_t temp;
    uint16_t year;
    uint8_t month;

    if (hrtc.Instance != RTC)
    {
        return HAL_ERROR;
    }

    sec_count = ((uint32_t)RTC->CNTH << 16) | RTC->CNTL;
    day_count = sec_count / 86400UL;

    if (cached_day != day_count)
    {
        cached_day = day_count;
        temp = day_count;
        year = 1970U;

        while (temp >= 365UL)
        {
            if (rtc_is_leap_year(year) != 0U)
            {
                if (temp < 366UL)
                {
                    break;
                }
                temp -= 366UL;
            }
            else
            {
                temp -= 365UL;
            }
            year++;
        }

        s_calendar.year = year;
        month = 0U;
        while (month < 12U)
        {
            uint8_t days = month_table[month];
            if (month == 1U && rtc_is_leap_year(year) != 0U)
            {
                days = 29U;
            }

            if (temp < days)
            {
                break;
            }

            temp -= days;
            month++;
        }

        s_calendar.month = (uint8_t)(month + 1U);
        s_calendar.date = (uint8_t)(temp + 1UL);
        s_calendar.week = rtc_get_week(s_calendar.year, s_calendar.month, s_calendar.date);
    }

    temp = sec_count % 86400UL;
    s_calendar.hour = (uint8_t)(temp / 3600UL);
    s_calendar.min = (uint8_t)((temp % 3600UL) / 60UL);
    s_calendar.sec = (uint8_t)((temp % 3600UL) % 60UL);

    return HAL_OK;
}

HAL_StatusTypeDef rtc_init(void)
{
    rtc_datetime_t default_datetime;

    if (hrtc.Instance != RTC)
    {
        return HAL_ERROR;
    }

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    if (rtc_read_bkr(RTC_BKP_DR1) != RTC_BKP_MAGIC)
    {
        default_datetime.year = RTC_DEFAULT_YEAR;
        default_datetime.month = RTC_DEFAULT_MONTH;
        default_datetime.date = RTC_DEFAULT_DATE;
        default_datetime.hour = RTC_DEFAULT_HOUR;
        default_datetime.min = RTC_DEFAULT_MIN;
        default_datetime.sec = RTC_DEFAULT_SEC;

        if (rtc_set_datetime(&default_datetime) != HAL_OK)
        {
            return HAL_ERROR;
        }

        rtc_write_bkr(RTC_BKP_DR1, RTC_BKP_MAGIC);
    }

    return rtc_refresh_calendar();
}

HAL_StatusTypeDef rtc_get_datetime(rtc_datetime_t *datetime)
{
    if (datetime == NULL)
    {
        return HAL_ERROR;
    }

    if (rtc_refresh_calendar() != HAL_OK)
    {
        return HAL_ERROR;
    }

    datetime->year = s_calendar.year;
    datetime->month = s_calendar.month;
    datetime->date = s_calendar.date;
    datetime->hour = s_calendar.hour;
    datetime->min = s_calendar.min;
    datetime->sec = s_calendar.sec;

    return HAL_OK;
}

HAL_StatusTypeDef rtc_set_datetime(const rtc_datetime_t *datetime)
{
    uint32_t sec_count;

    if (rtc_datetime_is_valid(datetime) == 0U)
    {
        return HAL_ERROR;
    }

    if (hrtc.Instance != RTC)
    {
        return HAL_ERROR;
    }

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    sec_count = (uint32_t)rtc_date2sec(datetime->year, datetime->month, datetime->date, datetime->hour, datetime->min, datetime->sec);

    RTC->CRL |= RTC_CRL_CNF;
    RTC->CNTL = (uint16_t)(sec_count & 0xFFFFU);
    RTC->CNTH = (uint16_t)(sec_count >> 16);
    RTC->CRL &= (uint16_t)~RTC_CRL_CNF;

    while (__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_RTOFF) == RESET)
    {
    }

    rtc_write_bkr(RTC_BKP_DR1, RTC_BKP_MAGIC);

    return rtc_refresh_calendar();
}

#endif
