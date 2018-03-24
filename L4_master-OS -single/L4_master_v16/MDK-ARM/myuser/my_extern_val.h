#include "stm32l4xx_hal.h"



extern uint16_t rsbuf2pt_write;
extern uint8_t  rsbuf2[];
extern uint16_t rsbuf2pt_read;

extern uint8_t my_CC1101_Sleep_status;
extern uint16_t my_tim6_count;
extern uint16_t my_cyc_time_count;
extern uint16_t my_dianliu_exit_add;
extern int16_t my_Time_Cyc_exit_add;

extern uint8_t my_CC1101_chip_address;
extern uint8_t  my_Current_exit_Status;
extern uint8_t  my_E_Field_exit_Status;
extern uint8_t  my_Time_Cyc_exit_Status;
extern uint16_t  my_cyc_delay;
extern uint8_t my_cc_Efied_count;
extern uint16_t  my_CC1101_all_step;
extern uint8_t my_sys_start_status;

extern uint8_t my_DTU_status;
extern uint16_t my_DTU_send_faile_count;
extern uint8_t my_DAC_cyc_time;
extern uint8_t  my_CC1101_all_count;

extern uint8_t my_zsq_ALarm_send_status;
extern uint8_t my_cyc_alarm_status;

extern RTC_DateTypeDef my_RTC_date;
extern RTC_TimeTypeDef my_RTC_time;
extern RTC_HandleTypeDef hrtc;
extern uint16_t my_ADC_Count;  //用来记录ADC的二级缓存计算次数
extern uint16_t my_ADC_Count_old;  //旧数据
extern uint8_t my_Line_short_status;
extern uint16_t my_Line_short_count;
extern uint8_t my_use_Jiedi_exit_status;








