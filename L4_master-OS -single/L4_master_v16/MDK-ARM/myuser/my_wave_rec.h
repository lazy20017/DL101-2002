#include "stm32l4xx_hal.h"
#include "my_gloabal_val.h"
#include "my_ADC.h"
#include "my_usart.h"
#include "math.h"









void my_adc2_convert2(uint8_t my_status);
void my_adc2_convert_dis(uint8_t);  //参数为1，表示带ADC采样及转换和显示，为0只显示，不进行采样和转换

//利用全波，计算12个周期的电流有效值
void fun_real_all_Current(void);
void fun_real_half_Current(void);
void fun_real_all_dianchang(void);
int fun_my_wave1_to_wave2(void);



void funT_display_wave2(void);

void CPU2_check_generate(uint8_t *buffer);
void my_CPU2_cyc_data(void);
void my_CPU2_cyc_data_wave(void);
void fun_wave2_to_wave3(void);
uint8_t fun_Judege_It_cache3(void);
uint8_t fun_Judege_It_cache2(void);
uint8_t fun_Judege_It_end(void);
void my_fun_wave_rec(void);

uint8_t my_fun_current_exit_just(void);
void my_fun_get_Line_stop_Efild(void);

void my_fun_wave1_to_wave2_old_data(void);

//void my_fun_query_Efild(void);

