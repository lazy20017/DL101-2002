#include "adc.h"
#include "my_usart.h"


//#define MY_ADC_NUM 960 //每个周期80个，12个周期需要960个数据


void filter_1(void); //数据计算，进行求平均值
void my_adc_1_convert(void);
void my_adc_1_convert_dis(uint8_t); ////参数为1，表示带ADC采样及转换和显示，为0只显示，不进行采样和转换


