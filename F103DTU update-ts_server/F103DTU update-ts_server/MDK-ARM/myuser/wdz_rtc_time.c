#include "wdz_rtc_time.h"
#include "rtc.h"


/*
利用数组形式写入时间戳，放到EEPROM中进行存储
u8 rtcbuffer[]={00,00,30,8,20,9,14};  //2014-9-20 8：30：00
*/

void my_RTC_TO_EEPROM(uint8_t *_pWriteBuf,uint8_t _address)
{

    my_RTCtime_to_array(_pWriteBuf);
    SPI_EE_BufferWrite2(_pWriteBuf,_address,7);
}

/*
读取eeprom中的时间戳，修改矫正当前时间，重启RTC进行计时
*/
void my_EEPROM_TO_RTC(uint8_t *_pWriteBuf,uint8_t _address)
{
    //从EEPROM中读取7个字节到数组中
    SPI_EE_BufferRead2(_pWriteBuf,_address,7); //第字节放低数组，毫秒、毫秒：分：小时、日-月-年

    //把数组值写入到RTC中

    my_array_to_RTCtime(_pWriteBuf);



}


/*
功能：利用RTC时间数组值进行系统校时，数组值的修改由串口获得
输入：RTC数组
输出：无
*/
void my_array_to_RTCtime(uint8_t *_pWriteBuf)
{   //struct rtc_time mytm;
    //u32 mycnt=0;
    uint16_t my_second=0;

    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef DateToUpdate;

    my_second=_pWriteBuf[1];
    my_second=((my_second<<8)+_pWriteBuf[0])/1000;
    //Time_Regulate(tm);  //输入当前的时间，年，月，日，小时，分钟，秒，放到结构体中
//	mytm.tm_year=2000+_pWriteBuf[6];
//	mytm.tm_mon=_pWriteBuf[5];
//	mytm.tm_mday=_pWriteBuf[4];
//	mytm.tm_hour=_pWriteBuf[3];
//	mytm.tm_min=_pWriteBuf[2];
//	mytm.tm_sec=my_second;

    /* ???? */
    /* ????:2015?10?4? ??? */
    DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
    DateToUpdate.Month = _pWriteBuf[5];
    DateToUpdate.Date = _pWriteBuf[4];
    DateToUpdate.Year = _pWriteBuf[6];
    //HAL_RTC_SetDate(&hrtc,&DateToUpdate,RTC_FORMAT_BCD);
    HAL_RTC_SetDate(&hrtc,&DateToUpdate,RTC_FORMAT_BIN);

    sTime.Hours =_pWriteBuf[3];
    sTime.Minutes = _pWriteBuf[2];
    sTime.Seconds = my_second;
    //HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F1);
}

/*
功能：读取当前时间并转换，存入到数组中
输入参数：RTC数组地址
输出：无
*/
void my_RTCtime_to_array(uint8_t *_pWriteBuf)
{   //uint32_t TimeVar;
    //struct rtc_time mytm;
    uint16_t my_second=0;
    RTC_DateTypeDef sdatestructureget;
    RTC_TimeTypeDef stimestructureget;

    /* 获得时间 */
    HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
    /* 获得日期 */
    HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);


    //TimeVar=RTC_GetCounter();
    //to_tm(TimeVar, &mytm);/*把定时器的值转换为unix时标,存在tm里*/

//	    _pWriteBuf[6]=mytm.tm_year-2000; s
//			_pWriteBuf[5]=mytm.tm_mon;
//			_pWriteBuf[4]=mytm.tm_mday;
//			_pWriteBuf[3]=mytm.tm_hour;
//			_pWriteBuf[2]=mytm.tm_min;
//
//			my_second=mytm.tm_sec*1000;
//			_pWriteBuf[1]=my_second/0xff;
//			_pWriteBuf[0]=my_second%0xff;

    _pWriteBuf[6]=sdatestructureget.Year-2000;
    _pWriteBuf[5]=sdatestructureget.Month;
    _pWriteBuf[4]=sdatestructureget.Date;
    _pWriteBuf[3]=stimestructureget.Hours;
    _pWriteBuf[2]=stimestructureget.Minutes;

    my_second=stimestructureget.Seconds*1000;
    _pWriteBuf[1]=my_second/0xff;
    _pWriteBuf[0]=my_second%0xff;


}
