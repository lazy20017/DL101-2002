#include "stm32f1xx_hal.h"
#include "my_def_value.h"
//#include "my_OS.h"






//=====OLD部分
//串口数据部分
extern uint8_t rsbuf1[rsbuf_max];	 //串口接收缓冲区
extern uint8_t rsbuf2[rsbuf_max];
extern uint8_t rsbuf3[rsbuf_max];
extern uint8_t rsbuf4[rsbuf_max];
extern uint8_t rsbuf5[rsbuf_max];

extern uint8_t txbuf1[256];
extern uint8_t txbuf2[256];
extern uint8_t txbuf3[256];
extern uint8_t txbuf4[256];
extern uint8_t txbuf5[256];

extern uint16_t rsbuf1pt_write;
extern uint16_t rsbuf1pt_read;
extern uint16_t rsbuf1pt_TEMP_read;
extern uint16_t rsbuf1pt_COMM_read;


extern uint16_t rsbuf2pt_write;
extern uint16_t rsbuf2pt_read;
extern uint16_t rsbuf2pt_TEMP_read;
extern uint16_t rsbuf2pt_COMM_read;

extern uint16_t rsbuf3pt_write;
extern uint16_t rsbuf3pt_read;
extern uint16_t rsbuf3pt_TEMP_read;
extern uint16_t rsbuf3pt_COMM_read;

extern uint16_t rsbuf4pt_write;
extern uint16_t rsbuf4pt_read;
extern uint16_t rsbuf4pt_TEMP_read;
extern uint16_t rsbuf4pt_COMM_read;

extern uint16_t rsbuf5pt_write;
extern uint16_t rsbuf5pt_read;
extern uint16_t rsbuf5pt_TEMP_read;
extern uint16_t rsbuf5pt_COMM_read;

//HAL库新添加
extern uint8_t  my_usart3_re_buf[200];
extern uint16_t my_usart3_re_count;
//END

//串口完
extern uint8_t gprs_status;
extern uint8_t USART1_TRANSMINT_STATUS;
extern uint8_t USART2_TRANSMINT_STATUS;
extern uint8_t FLASH_DTU;
extern uint8_t my_tim3_dog_cnt;

//=====OLD END====


extern	uint8_t MY_433_Alarmdata_NOtimeBuf[256]; //存储 无时标 报警数据
extern	uint8_t MY_433_Alarmdata_TimeBuf[256];  //存储，有时标，报警数据
extern	uint8_t MY_433_ALarmdata_NOtime_status; //为1，表示收到无时标报警数据
extern	uint8_t MY_433_ALarmdata_Time_status;   //为1，表示收到有时标报警数据
extern	uint8_t MY_433_ALarmdata_number;  // 存储，报警信息体个数


extern  uint8_t MY_MCU_RsBuf[8];  //存储，周期性电池电压、太阳能电压、温度、湿度共4类8个字节的数据
extern  uint8_t MY_MCU_getdata_status; //为1表示，总召成功了，收到了周期新数

extern uint8_t MY_433_Call_Single_data_buf[];
extern uint8_t MY_433_Call_Analog_data_buf[];
extern uint8_t MY_433_Call_Time_data_buf[];
extern uint8_t MY_433_Call_Single_data_number;
extern uint8_t MY_433_Call_Analog_data_number;
extern uint8_t MY_433_Call_Status;

extern uint8_t RE_ALarmData_Status;
extern uint8_t RE_CycleData_Status;


extern uint8_t MY_GPRS_MCU_RsBuf[8];  //存储，周期性电池电压、太阳能电压、温度、湿度共4类8个字节的数据

extern uint8_t MY_GPRS_Call_Single_data_buf[40];
extern uint8_t MY_GPRS_Call_Analog_data_buf[110];
extern uint8_t MY_GPRS_Call_Time_data_buf[7];
extern uint8_t MY_GPRS_Call_Single_data_number;
extern uint8_t MY_GPRS_Call_Analog_data_number;

extern uint16_t MY_H_speed_cyc;  //周期10分钟
extern uint16_t MY_H_speed_heart;  //心跳5分钟
extern uint16_t MY_M_speed_cyc;  //周期15分钟
extern uint16_t MY_M_speed_heart;  //心跳9分钟
extern uint16_t MY_L_speed_cyc;  //周期30分钟
extern uint16_t MY_L_speed_heart; //心跳7分钟
extern float MY_Speed_H_Gate;
extern float MY_Speed_L_Gate;

//====EEPROM===
extern uint32_t MY_Table1_Alarmdata_StartAddress;
extern uint32_t MY_Table1_Alarmdata_EndAddress;
extern uint32_t MY_Table1_Alarmdata_WriteAddress;
extern uint32_t MY_Table1_Alarmdata_ReadAddress;

extern uint32_t MY_Table2_Cycledata_StartAddress;
extern uint32_t MY_Table2_Cycledata_EndAddress;
extern uint32_t MY_Table2_Cycledata_WriteAddress;
extern uint32_t MY_Table2_Cycledata_ReadAddress;

extern uint8_t MY_Table2_Newdata_status; //EEPROM中Table2总召产生新数据标识，为1表示有新数据写入，为0表示无数据写入
extern uint8_t MY_Table1_Newdata_status; //EEPROM中Table1报警新数据标识，为1表示有新数据写入，为0表示无数据写入
//====END

extern uint8_t MY_433_Call_Single_data_buf[256];
extern uint8_t MY_433_Call_Analog_data_buf[256];
extern uint8_t MY_433_Call_Time_data_buf[7];
extern uint8_t MY_433_Call_Single_data_number;
extern uint8_t MY_433_Call_Analog_data_number;
extern uint8_t MY_433_Call_Status;  //433模块总召数据状态，为1表示收到数据并存入到数组缓冲区中，0为没有数据

extern uint8_t MY_IP[4];
extern uint16_t MY_PORT;

//====TIM6
extern uint32_t TimingDelay;

extern uint16_t  my_GPRS_all_step;
extern uint8_t my_gprs_TX_status;

extern RTC_DateTypeDef my_RTC_date;
extern RTC_TimeTypeDef my_RTC_time;
extern RTC_HandleTypeDef hrtc;

extern uint16_t MY_ACT_CYC_DTU;
extern uint16_t MY_ACT_Heart_DTU;
extern uint8_t my_act_cyc_dtu_delay;
extern float  my_AD_value;
extern float  my_AD_SUN_value;



extern struct my_ZSQ_change_vale my_zsq_value;
extern uint8_t my_system_restart_status;


