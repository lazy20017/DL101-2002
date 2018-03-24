#include "stm32f1xx_hal.h"
//#include "bsp_i2c_ee.h"
#include "bsp_rtc.h"
#include "bsp_spi_flash.h"
#include "bsp_SysTick.h"



void my_array_to_RTCtime(uint8_t *_pWriteBuf);  //把数组的值写入到RTC中
void my_EEPROM_TO_RTC(uint8_t *_pWriteBuf,uint8_t _address);//读取EEPROM中的时标，写入RTC中

void my_RTCtime_to_array(uint8_t *_pWriteBuf);//把RTC中的值读出，并写入到数组中
void my_RTC_TO_EEPROM(uint8_t *_pWriteBuf,uint8_t _address);//把RTC当前的时标值，写入到EEPROM中

