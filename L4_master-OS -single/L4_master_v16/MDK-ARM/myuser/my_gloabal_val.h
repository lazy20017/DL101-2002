#include "stm32l4xx_hal.h"

//#define rsbuf_max 100
//#define rsbuf_min	100
#define rsbuf3_max    100//51K,  51*1024=52224  41K=41984
#define rsbuf2_max    200


#define ADC1_ROW 3 //N个数的平均值,行
#define ADC1_COLM 8  //M采集M个通道，列

#define ADC2_ROW 3 //N个数的平均值,行
#define ADC2_COLM 3  //M采集M个通道，列



#define WAVE_number 8000  //1个周期80个点，12个周期960个点，1s钟50个周期80*50=4000个点，2秒钟8000个点
#define WAVE_Channel_Num 3  //录波的信道数量，第1行为全波电流，第2行为全波电场，第3行为半波电流

#define WAVE_number_sec2 960  //二级缓存中的录波数据，960个点为12个周期的数据，12*80=960
#define WAVE_number_sec3 WAVE_number_sec2

//=======================
//常量部分
//********串口部分**************************

#define USART_DEBUG &huart2   //定义调试端口


//===STM FLASH
//=========条件编译使用的内容==========
//CC1101
#define Debug_Usart_use_433data 0  //1串口转发CC1101发送数据，0串口不转发CC1101的发送数据
#define Debug_Usart_out_CC1101_Get_cmd 0 //1显示CC1101收到的101命令
#define CC1101_Use 1       //利用cc1101发送数据，1发送，0不发送

//ADC2
#define Debug_Usart_out_chazhidata 0  //显示录波数据第1个周期，和最后1个周期的第一个数据差值
#define ADC_CYC_data 1 //CC1101 开启周期采样
#define ADC_interrupt_data 1 //CC1101 开启中断采样
//DAC
#define Debug_Usart_out_DAC_normal_data 1 //串口显示DAC，设置相关参数
#define DAC_auto_change_on 1 //DAC的参考电压自动变化，1表示开启，0关闭

//OS_CC1101
#define Debug_Usart_out_ADCdata  1 //为1，表示在CC1101的对话过程中，最后，显示ADC的采样数据
#define Debug_Usart_out_wavedata_960Data_2cach 0  //**2级缓冲**转发ADC的录波数据,1为电流全波，2为电场全波，3为电流半波。前提条件是Debug_Usart_out_ADCdata==1
#define Debug_Usart_OUT_WAVE_12T_CYC  0  //2级缓冲，1显示2级缓冲中录波的12T的有效值
#define Debug_Usart_OUT_WAVE_VALUE 0  //1中断中，显示12个周期的电流、电场、


//OS_CC1101_Test

#define OS_CC1101_auto_reveive_OK 0 //OS调试使用，CC1101发送数据后，模拟收到OK帧
#define OS_CC1101_ZSQ_Monidata 0  //指示器产生模拟数据给DTU
#define OS_heap_high_water_data 0 //OS调试串口显示，每个函数剩余的堆栈区数量@@@，1为显示，0为不显示
#define USE_CC1101_PWR_ON_STATUS  1  //1给CC1101供电，0为不供电

//CC1101发送录波数据
//#define CC1101_960data_Efield_STATUS 0  //1 CC1101发送录波电场数据，0为不发送电场，发送电流
#define CC1101_SEND_I_E_Simulation_data_status 0  //CC1101发送 电流、电场模拟数据



//录波数据显示
#define Debug_Usart_OUT_WAVE_12T_Interupt 1 // 3级缓冲，1为显示，故障录波时，4+8=12的中断故障录波的,12个周期的数据的有效值
#define Debug_Usart_OUT_WAVE_960Data_Interupt 0 //3级缓冲,1位全波电流故障录波数据，2为电场，3为半波电流
#define Debug_usart_out_wave_cmpare_data 0  //3级缓冲中，1显示，周期的首尾差值

#define Debug_Usart_OUT_WAVE_Last_12T_Interupt 0 //显示故障中断后，每隔200ms录波的12个周期的数据
#define Debug_Usart_OUT_WAVE_End_Just_Interupt  1 //输出最终的判断结果，及依据。


//显示录波数据的首位差值
#define Debug_Usart_OUT_WAVE_Chazhi 1 //显示录波数据  首末周期，的第一个数据的差值

//显示停电状态
#define Debug_Usart_OUT_LINE_STOP_STATUS 1  //1为显示，0为不显示

//显示电场的实时状态
#define Debug_Usart_OUT_LINE_Efield_STATUS 1  //串口，停电状态采集，1为显示，0为不显示




//显示DC直流数据
#define Debug_Usart_OUT_DC_DATA_status 1 //显示直流数据

#define CC1101_TX_Delay 0 //cc1101发送延时一下

//周期和DAC，计算利用历史数据
#define USE_olde_12T_data_cyc 1  //周期，DAC利用旧的12T数据，提高速度
#define USE_olde_12T_data_DAC 1  //DAC，DAC利用旧的12T数据，提高速度

//LED灯控制
#define USE_LED14_STATUS 1  //1为使用LED显示，0为不使用

//校正算法是否使用
#define USE_Adjust_suanfa 2  //1为使用最小二乘法校正算法，2为分段系数最小二乘法校正法，0位不使用校正算法

//ADC半波采样，去零飘的方法
#define USE_half_adjust_zero 1  //1为使用半波去零飘方法，半波采样到直流值，很小0.1左右，正定为0
