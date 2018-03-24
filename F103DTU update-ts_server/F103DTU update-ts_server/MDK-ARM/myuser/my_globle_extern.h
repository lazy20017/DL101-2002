
//M35
#define USE_INIT_M35 1  //m35芯片初始化，1为使用，0为不使用


//OS
#define OS_heap_high_water_data 1  //OS调试串口显示，每个函数剩余的堆栈区数量@@@，1为显示，0为不显示


//Debug_usart
#define Debug_Uart_out_DC_AC_DATA_Status 1  //利用串口显示接收的直流和交流信号，包括遥信
#define Debug_Uart_out_AC12T_DATA_Status 0  //利用串口发送12T数据
#define Debug_Uart_out_960WAVE_DATA_Status 0  //利用串口显示接收到的960数据

//CC1101
#define Use_CC1101_receive_interrupt_status 1  //1启用CC1101接收中断，0为不启用中断
#define Use_CC1101_send_heat_data_status 0    //1启用CC101发送周期心跳包给指示器。测试使用，0为不发送

#define Debug_uart_out_CC1101_TX_Status 0  //显示CC1101发送完成后，设置接收态的状态

#define Debug_uart_out_cc1101_rx_data_status 0  //1串口显示CC1101接收的数据，0不显示
#define Debug_uart_out_cc1101_tx_data_status 0  //1串口显示CC1101发送的数据，0为不显示


//GPRS test
#define Use_GPRS_auto_re_ok  0 //GPRS模拟自动接收到OK帧，对话过程自动进行
#define Use_indicatour_cyc_test_satus 0//产生指示器的模拟数据
#define Use_DTU_huanjing_jisuan 1  //对DTU的环境数据进行直接计算，保留小数点1位

//报警模拟
#define Use_gprs_ALarm_simulation_word 0 //1使用GPRS直接发送报警模拟数据，0为不使用





struct indicator_class 
{
    uint8_t duanlu_data;
    uint8_t jiedi_data;
    uint8_t DC_data_buf[14]; //1温度，2电源，3参考电压，4干电池，5线上电压，6太阳能，7锂电池
    uint8_t AC_data_buf[6];  //全波电流，电场、半波电流
    uint8_t AC12T_ALL_Current_data_buf[24];
    uint8_t AC12T_ALL_dianchang_data_buf[24];
    uint8_t AC12T_HALF_Current_data_buf[24];
    uint8_t RTC_time_buf[7]; //RTC
    uint8_t data_type;  //01为周期，02为报警
    uint8_t count_time[2];  //全局同步值
    uint8_t xinhao_db; //信号强度
    uint8_t TX_status;  //数据发送状态，发送完为0，没有发送为1
		uint8_t Line_STOP;  //线路停电状态，1为正常，2为停电

};


struct indicator_class_parameter
{
    uint16_t P1_300A_mul; //5031
    uint16_t P2_Add_value;  //5032
    uint16_t P3_E_mul;  //5033
    uint16_t P4_E_mul2;  //5034
    uint16_t P5_I_deta;  //5035
    uint16_t P6_I_max;  //5036
    uint16_t P7_I_0min;  //5037
    uint16_t P8_E_down_baifenbi;  //5038
    uint16_t P9_E_0min;  //5039
    uint16_t P10_E_down_min;  //503A
    uint16_t P11_V_Libat;  //503B
    uint16_t P12_CYC_time_MIN;  //503C
    uint16_t P13_CYC_time_MAX;  //503D
    uint16_t P14_sleep_time;  //503E
    uint16_t P15_awake_time;  //503F
    uint16_t P16;  //503F
    uint16_t P17_reset_LED_time;  //5041
    uint16_t P18_reset_sys_time;  //5042

};


struct indicator_alarm_class
{
    uint8_t duanlu_data;
    uint8_t jiedi_data;
    uint8_t DC_data_buf[14]; //1温度，2电源，3参考电压，4干电池，5线上电压，6太阳能，7锂电池
    uint8_t AC_data_buf[6];  //全波电流，电场、半波电流
    uint8_t AC12T_ALL_Current_data_buf[24];
    uint8_t AC12T_ALL_dianchang_data_buf[24];
    uint8_t AC12T_HALF_Current_data_buf[24];
    uint8_t RTC_time_buf[7]; //RTC
    uint8_t data_type;  //01为周期，02为报警
    uint8_t count_time[2];  //全局同步值
    uint8_t xinhao_db; //信号强度
    uint8_t TX_status_duanlu;  //短路数据发送状态，发送完为0，没有发送为1
		uint8_t TX_status_jiedi;   //接地数据发送状态，发送完为0，没有发送为1
	  uint8_t Line_STOP;  //线路停电状态，1为正常，2为停电

};

#define my_record_count 1931
struct indicator_record_class
{
    uint8_t my_wave_record_I_buf[1931];
		uint8_t my_wave_record_E_buf[1931];
    //uint8_t my_wave_type; //1电流，2接地，0没有
    uint8_t my_wave_alam; //1周期，2报警，0没有
		uint16_t my_count_write;
		uint16_t my_count_read;
		uint8_t my_wave_tx_status_I;  //录波数据发送状态，1表示两个波形还没有发送出去，0表示发送完成
		uint8_t my_wave_tx_status_E;
};

struct my_ZSQ_change_vale  //指示器设置参数使用
{
    uint16_t my_inf_add;
    uint16_t zsq_add;
    uint8_t data_buf[8];
    uint8_t status;  //没有发送为1，发送完了变为0
};


