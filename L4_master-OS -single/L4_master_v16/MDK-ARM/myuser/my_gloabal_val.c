#include "stm32l4xx_hal.h"
#include "my_gloabal_val.h"




//串口
uint8_t rsbuf2[rsbuf2_max];
uint8_t USART2_FRAME_status;
uint8_t USART2_my_frame[100];
uint16_t rsbuf2pt_write;
uint16_t rsbuf2pt_read;

uint16_t my_wave_record[WAVE_Channel_Num][WAVE_number]= {0}; //全波的一级缓存，利用定时器进行波形的存储，假定20ms采集一次，看能采集到多少个数据。
uint16_t my_wave_write_add=0;  //全波录波一级缓存指针

uint16_t my_wave_record_sec2[WAVE_Channel_Num][WAVE_number_sec2]= {0}; //全波的二级缓存
uint16_t my_wave_record_sec3[WAVE_Channel_Num][WAVE_number_sec3]= {0}; //全波的3级缓存
uint8_t  my_wave_re_status=0;  //录波状态，0表示无数据，1，2,3,4,5表示二级缓存有数据，10表示发送数据



uint16_t ADC2_GetValue[ADC2_ROW][ADC2_COLM]= {0}; //用于保存采集的值,M个通道,N次，原始数据
uint16_t ADC2_Filer_value_buf_1[ADC2_COLM][3]= {0}; //用于保存3个通道，12个周期测量的平均值,1行为电流，2行为电场，3行为半波
double ADC2_Filer_value_buf_2[ADC2_COLM][3]= {0};  //AD转换后的物理值，12个周期的平均值，有效值，最大值，第1行为全波电流，第2行为电场，第3行为半波电流
double ADC2_Filer_value_buf_3[ADC2_COLM][3]= {0};  //三级缓冲，故障瞬间的换成后的数据


uint16_t ADC1_GetValue[ADC1_ROW][ADC1_COLM]= {0}; //用于保存采集的值,M个通道,N次
uint16_t ADC1_GetValue_Aver[ADC1_COLM]= {0}; //用于保存平均值
double ADC1_Filer_value_buf[ADC1_COLM]= {0};  //用来存储ADC1采样到的的DC直流数据，共7个通道，真实值，转换后的
//1温度、2电源、3参考电压、4干电池、5线上电压、6太阳能、7锂电池,保留小数点后两位
//中断
uint16_t my_PA01_count=0;
uint16_t my_PA00_count=0;
uint16_t my_PC06_count=0;

uint16_t my_dianliu_exit_add=0; //此变量，用来计算中断或者定时产生时刻，查询录波1级缓冲区所用的地址
uint16_t my_Wave_It_add=0;  //记录中断时刻地址，共同步录波使用

uint8_t  my_Current_exit_Status=0; //表示电流产生中断
//uint16_t my_Current_Exit_add=0;

uint8_t  my_E_Field_exit_Status=0; //表示电场产生中断
uint16_t my_E_Field_exit_add=0;

uint8_t  my_Time_Cyc_exit_Status=0; //周期定时中断
uint16_t my_Time_Cyc_exit_add=0;

//
uint16_t my_send_3S_count=0;

uint8_t my_send_ALL_A_960_data_status=1;  //为1表示发送周期录波数据，0表示不发送
uint8_t my_send_ALL_E_960_data_status=0;
uint8_t my_send_HALF_A_960_data_status=0;

//串口3状态标识
uint8_t my_usart3_error_status=0;  //1标识出现了问题

//===============
uint8_t  my_temp8_buf1[2000];

uint8_t my_CC1101_Sleep_status=1;  //CC1101的SLEEP标识，为1标识CC1101睡眠

//TIM6
uint16_t my_tim6_count=0;
uint16_t my_cyc_time_count=313;//313;//347;  //周期发送数据时间，秒为单位
uint16_t  my_cyc_delay=0;

uint8_t my_cc_Efied_count=0;  //记录CC1101发送后，停止的时间，然后开启接地中断
uint16_t  my_CC1101_all_step = 0;

//CC1101
uint8_t my_DTU_status=1;  //1为在线，0为不在线。DTU在线状态，如果发送10次都不能联系上DTU，DTU不在线为0，为1表示在线。
uint16_t my_DTU_send_faile_count=0;  //给DTU发送数据，失败计数器

//DAC
uint8_t my_DAC_cyc_time=1;  //DAC设置时间,设定后变为17秒

//CC1101报警发送
uint8_t my_zsq_ALarm_send_status=0;  //报警状态是否发送出去，1为发送出去，0为没有发送出去。

uint8_t my_cyc_alarm_status=0;   //周期报警状态，1为周期产生的报警，0为没有周期报警

RTC_DateTypeDef my_RTC_date;
RTC_TimeTypeDef my_RTC_time;

uint16_t my_ADC_Count=0;  //用来记录ADC的二级缓存计算次数
uint16_t my_ADC_Count_old=0;  //旧数据
uint8_t my_Line_short_status=0; //用来标识，300A时充电线路的状态，默认为0，标识开，正在充电。为1表示放电。
uint16_t my_Line_short_count=0; //300A以上充电，和放电时间，10分钟，60*10=600秒

uint8_t my_use_Jiedi_exit_status=1;  //使用接地中断判断，为1使用中断，为0不使用

