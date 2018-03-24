
#include "stm32l4xx_hal.h"
#include "rtc.h"
#include "my_rtc.h"
void my_rtc_read(void)
{
    RTC_TimeTypeDef my_sTime;
    RTC_DateTypeDef my_sDate;
    HAL_RTC_GetTime(&hrtc,&my_sTime,FORMAT_BIN);
    printf("%d:%d:%d\n",my_sTime.Hours,my_sTime.Minutes,my_sTime.Seconds);
    HAL_RTC_GetDate(&hrtc,&my_sDate,FORMAT_BIN);
    printf("%d-%d-%d\n",my_sDate.Year,my_sDate.Month,my_sDate.Date);

}

void my_rtc_write_init_time(void)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;


    sTime.Hours = 1;
    sTime.Minutes = 20;
    sTime.Seconds = 14;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_APRIL;
    sDate.Date = 5;
    sDate.Year = 16;

    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

}
