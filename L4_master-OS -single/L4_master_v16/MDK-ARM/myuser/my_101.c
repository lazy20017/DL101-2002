#include "my_101.h"
#include "my_extern_val.h"



//typedef enum { TX_MODE, RX_MODE } TRMODE;
//typedef enum { BROAD_ALL, BROAD_NO, BROAD_0, BROAD_0AND255 } ADDR_MODE;  //地址模式
//typedef enum { BROADCAST, ADDRESS_CHECK} TX_DATA_MODE;


extern uint8_t my_Fault_Current_End_Status;  //
extern uint8_t my_Fault_E_Fild_End_Status;
extern uint8_t my_PWR_all_count;

extern uint8_t my_CC1101_COM_Fram_buf[]; //指令缓冲区
extern uint8_t my_CC1101_Frame_status;

extern double ADC1_Filer_value_buf[ADC1_COLM];
extern uint16_t ADC1_GetValue_Aver[ADC1_COLM];
extern double ADC2_Filer_value_buf_2[ADC2_COLM][3];
extern double ADC2_Filer_value_buf_3[ADC2_COLM][3];
extern uint16_t ADC2_Filer_value_buf_1[ADC2_COLM][3];

//发送12个周期的有效值,共36个数据
extern double WAVE_half_ave_Current2[12][3]; //利用三列存，平均，有效，最大，12个周期的每个周期的值
extern double WAVE_all_ave_Current2[12][3]; //利用三列存，平均，有效，最大
extern double WAVE_all_ave_E_Field2[12][3]; //利用三列存，平均，有效，最大

extern double WAVE_half_ave_Current3[12][3]; //利用三列存，平均，有效，最大，12个周期的每个周期的值
extern double WAVE_all_ave_Current3[12][3]; //利用三列存，平均，有效，最大
extern double WAVE_all_ave_E_Field3[12][3]; //利用三列存，平均，有效，最大

//存12周期，每个周期采样值得，真实值
extern uint16_t WAVE_half_ave_Current1[12][3]; //利用三列存，平均，有效，最大，12个周期的每个周期的值
extern uint16_t WAVE_all_ave_Current1[12][3]; //利用三列存，平均，有效，最大
extern uint16_t WAVE_all_ave_E_Field1[12][3];

extern uint16_t my_wave_record_sec2[WAVE_Channel_Num][WAVE_number_sec2];
extern uint16_t my_wave_record_sec3[WAVE_Channel_Num][WAVE_number_sec2];
extern double MY_VDD;
extern double my_i_ratio_value;
extern uint16_t my_all_i_up_value;
extern uint8_t  my_temp8_buf1[];

extern uint8_t my_CC1101_dest_address;
extern double my_I_100A_Radio;
extern double my_E_ratio_value;

extern uint8_t my_CC1101_chip_address; //发送源地址
extern uint8_t my_CC1101_dest_address; //发送目标地址
extern uint8_t my_sys_start_status;


//extern uint8_t my_protocole1_status;  //表示解析帧的状态，1为表示收到1个完整的帧，0表示没有收到完整的帧
//extern uint8_t my_protobole1_buf1[];
#define my_protocole1_status my_CC1101_Frame_status
#define my_protobole1_buf1 my_CC1101_COM_Fram_buf

uint8_t my_point = 0;
uint16_t my_PWR_heart_count = 0;
uint8_t my_101_send_buf[1935] = {0X68};  //101串口协议发送缓冲区

/*

功能：利用串口，发送数据缓冲区中的数据
参数1：串口号
参数2：控制命令
参数3：发送数据
*/


void my_fun_101_send_short_data(UART_HandleTypeDef* USARTx, uint8_t control_bye, uint16_t data, TX_DATA_MODE mode, INT8U desc_address)
{


    my_101_send_buf[0] = 0x10; //帧头
    my_101_send_buf[1] = control_bye; //ID
    my_101_send_buf[2] = my_CC1101_chip_address; //源地址
    my_101_send_buf[3] = my_CC1101_dest_address; //目的地址
    my_101_send_buf[4] = data; //数据低字节
    my_101_send_buf[5] = (data >> 8) ; //数据高字节
    my_101_send_buf[6] = my_fun_101check_generate(my_101_send_buf, 1);
    my_101_send_buf[7] = 0x16;

    //USART_printf(USARTx,my_send_buf);
#if Debug_Usart_use_433data==1
    HAL_UART_Transmit(USARTx, my_101_send_buf, 8, 3000);
#endif

#if CC1101_Use==1
    my_fun_CC1101_send_long_data(my_101_send_buf, 8, mode, desc_address); //利用CC1101发送数据
#endif
}
/*

功能：发送长帧数据
参数1：串口号
参数2：控制命令
参数3：数据缓冲区首地址
参数4：发送数据数量
*/


void my_fun_101_send_long_data(UART_HandleTypeDef* USARTx, uint8_t control_bye, uint8_t *send_buf, uint16_t send_number, TX_DATA_MODE mode, INT8U desc_address)
{
    uint16_t ii = 0;



    my_101_send_buf[0] = 0x68; //帧头
    my_101_send_buf[1] = (uint8_t)((send_number + 3) & 0x00ff); //长度,从ID开始算的长度
    my_101_send_buf[2] = (uint8_t)(((send_number + 3) & 0xff00) >> 8); //长度，除帧头以外的长度

    my_101_send_buf[3] = my_101_send_buf[1];
    my_101_send_buf[4] = my_101_send_buf[2];

    my_101_send_buf[5] = 0x68; //帧头
    my_101_send_buf[6] = control_bye;
    my_101_send_buf[7] = my_CC1101_chip_address; //源地址
    my_101_send_buf[8] = my_CC1101_dest_address; //目的地址


    for(ii = 0; ii < send_number; ii++)
    {
        my_101_send_buf[9 + ii] = send_buf[ii];

    }
    my_101_send_buf[9 + ii] = my_fun_101check_generate(my_101_send_buf, 1);;
    my_101_send_buf[9 + ii + 1] = 0x16;


#if 	Debug_Usart_use_433data==1 || CC1101_Use==1
    uint16_t my_length = 0;
    my_length = send_number + 11;
#endif


#if Debug_Usart_use_433data==1
    HAL_UART_Transmit(USARTx, my_101_send_buf, my_length, 3000); //利用串口传输数据，调试使用
#endif

#if CC1101_Use==1
    my_fun_CC1101_send_long_data(my_101_send_buf, my_length, mode, desc_address); //利用CC1101发送数据

#endif


}




/*
功能：发送直流ADC采样数据，4001信息体，对应转换后的数据，比例放大，保留小数点后2位，放大100倍后传输
思路：调用函数之前，先利用其它函数完成ADC1的采样工作。

参数1：为0，表示只发转换后的结果，为1，表示先发转换后结果，后发采样值
参数2：为发送的控制码，可以为直流的周期，或者直流的报警
*/


void my_fun_101send_DC_data(UART_HandleTypeDef* USARTx, uint8_t my_status, uint8_t my_contorl_byte)
{
    uint8_t my_data[ADC1_COLM * 2] = {0};
    uint8_t ii = 0;
    uint16_t temp = 0;


//发送转换后的数据******
    for(ii = 0; ii < ADC1_COLM; ii++)  //1温度、2电源、3参考电压、4干电池、5线上电压、6太阳能、7锂电池
    {   
			if(my_sys_start_status==0)
			{
					temp = (uint16_t)(ADC1_Filer_value_buf[ii] * 10); //直流数据，转换后的物理量，放到100倍，保留小数点后2位;放大10倍，保留小数点后1位
			}
			else
				{
					temp=0x1111; //指示器重新启动标识						
				}
				my_data[2 * ii] = temp;
        my_data[2 * ii + 1] = (temp >> 8) & 0x00FF;
			

    }
		
		//模拟数据
		#if OS_CC1101_ZSQ_Monidata==1
		for(ii = 0; ii < ADC1_COLM; ii++)  //1温度、2电源、3参考电压、4干电池、5线上电压、6太阳能、7锂电池
    {   
			
				//my_data[2 * ii] = 2*ii;
        //my_data[2 * ii + 1] = 2*ii+1;
			

    }
		
		
		#endif
		
		
		my_sys_start_status=0;
    my_fun_101_send_long_data(USARTx, my_contorl_byte, my_data, ADC1_COLM * 2, ADDRESS_CHECK, my_CC1101_dest_address); //发送转换后的结果
}


//发送交流有效值

void my_fun_101send_AC_data(UART_HandleTypeDef* USARTx, uint8_t my_status, uint8_t my_contorl_byte)
{

    uint8_t my_data[ADC2_COLM * 2] = {0};
    uint8_t ii = 0;
    uint16_t temp = 0;
    //周期值

		
    //=============
    if(my_contorl_byte == 0X41)
    {
        for(ii = 0; ii < ADC2_COLM; ii++)
        {   temp = (uint16_t)(ADC2_Filer_value_buf_2[ii][1] * 10); //转换值，有效值===电流、电场，半波
            my_data[2 * ii] = temp;
            my_data[2 * ii + 1] = (temp >> 8) & 0x00FF;

        }
    }
    else if(my_contorl_byte == 0X51)
    {

        for(ii = 0; ii < ADC2_COLM; ii++)
        {   //temp = (uint16_t)(ADC2_Filer_value_buf_3[ii][1] * 10); //转换值，1有效值，0为平均值，2为最大值===电流、电场，半波
						temp = (uint16_t)(ADC2_Filer_value_buf_3[ii][2] * 10 ); 
						my_data[2 * ii] = temp;
            my_data[2 * ii + 1] = (temp >> 8) & 0x00FF;

        }
    }
    //模拟数据
		#if OS_CC1101_ZSQ_Monidata==1
		uint16_t xx=0;
		xx=(my_CC1101_chip_address+1)*25;
			for(ii = 0; ii < ADC2_COLM; ii++)
			{
						my_data[2 * ii] = ++xx;
            my_data[2 * ii + 1] = (xx>>8);	
				if(xx>=200)
					xx=(my_CC1101_chip_address+1)*25;
			}
		#endif


    my_fun_101_send_long_data(USARTx, my_contorl_byte, my_data, ADC2_COLM * 2, ADDRESS_CHECK, my_CC1101_dest_address); //发送转换后的结果

}


void my_fun_101send_AC12T_Cyc_data(UART_HandleTypeDef* USARTx, uint8_t my_status, uint8_t my_contorl_byte)
{

    uint8_t my_data[ADC2_COLM * 12 * 2] = {0};
    uint8_t ii = 0;
    uint16_t temp = 0;

    if(my_contorl_byte == 0X42) //周期
    {

        for(ii = 0; ii < 12; ii++)
        {   temp = (uint16_t)(WAVE_all_ave_Current2[ii][1] * 10); //电流全波
            my_data[2 * ii] = temp;
            my_data[2 * ii + 1] = (temp >> 8) & 0x00FF;

        }
        for(ii = 0; ii < 12; ii++)
        {   temp = (uint16_t)(WAVE_all_ave_E_Field2[ii][1] * 10);//电场全波
            my_data[2 * ii + 24] = temp;
            my_data[2 * ii + 1 + 24] = (temp >> 8) & 0x00FF;

        }
        for(ii = 0; ii < 12; ii++)
        {   temp = (uint16_t)(WAVE_half_ave_Current2[ii][1] * 10); //电流半波
            my_data[2 * ii + 48] = temp;
            my_data[2 * ii + 1 + 48] = (temp >> 8) & 0x00FF;

        }
    }
    else if(my_contorl_byte == 0X52)
    {

        for(ii = 0; ii < 12; ii++)
        {   temp = (uint16_t)(WAVE_all_ave_Current3[ii][1] * 10);
            my_data[2 * ii] = temp;
            my_data[2 * ii + 1] = (temp >> 8) & 0x00FF;

        }
        for(ii = 0; ii < 12; ii++)
        {   temp = (uint16_t)(WAVE_all_ave_E_Field3[ii][1] * 10);
            my_data[2 * ii + 24] = temp;
            my_data[2 * ii + 1 + 24] = (temp >> 8) & 0x00FF;

        }
        for(ii = 0; ii < 12; ii++)
        {   temp = (uint16_t)(WAVE_half_ave_Current3[ii][1] * 10);
            my_data[2 * ii + 48] = temp;
            my_data[2 * ii + 1 + 48] = (temp >> 8) & 0x00FF;

        }
    }
		
		
		 //模拟数据
		#if OS_CC1101_ZSQ_Monidata==1
		uint16_t xx=300;
		xx=(my_CC1101_chip_address+1)*300;
				for(ii = 0; ii < 12; ii++)
        {   //电流全波
            my_data[2 * ii] = ++xx;
            my_data[2 * ii + 1] = (xx>>8);

        }
        for(ii = 0; ii < 12; ii++)
        {   //电场全波
            my_data[2 * ii + 24] =++xx;
            my_data[2 * ii + 1 + 24] = (xx>>8);

        }
        for(ii = 0; ii < 12; ii++)
        {   //电流半波
            my_data[2 * ii + 48] = ++xx;
            my_data[2 * ii + 1 + 48] =(xx>>8);

        }
		#endif
		
		
    my_fun_101_send_long_data(USARTx, my_contorl_byte, my_data, ADC2_COLM * 12 * 2, ADDRESS_CHECK, my_CC1101_dest_address); //发送转换后的结果


}

/*
功能：//发送录波数据
*/

void my_fun_101send_AC_Rec_data(UART_HandleTypeDef* USARTx, uint8_t my_status, uint8_t my_contorl_byte)
{
    uint8_t *my_data = my_temp8_buf1;
    uint8_t my_row = 0;
    uint16_t ii = 0;
    //uint16_t temp = 0; //有符号数据
		int temp=0;
		uint16_t xx=0;
    int my_cc=0;



    if(my_contorl_byte == 0X43 ) //周期电流全波
    {
        my_row = 0;
    }

    else if(my_contorl_byte == 0X44) //周期电流电场
    {
        my_row = 1;
    }
    else if(my_contorl_byte == 0X53) //报警电流全波
    {
        my_row = 10;
    }

    else if(my_contorl_byte == 0X54) //报警电流电场
    {
        my_row = 11;
    }
   
//--------------------
    

     if(my_row == 10 ) //电流
    {
        my_row = my_row - 10;
			 xx=15*(my_CC1101_chip_address+1);
			  //printf("my_all_i_up_value=%d\n",my_all_i_up_value);
				//printf("\n===960===start==\n");
        for(ii = 0; ii < WAVE_number_sec3; ii++)
        {
            //temp = (my_wave_record_sec3[my_row][ii] - my_all_i_up_value);
						temp = my_wave_record_sec3[my_row][ii];
			

            //temp=my_wave_record_sec2[my_row][ii];
            temp = temp * MY_VDD / 4096 * (my_i_ratio_value * my_I_100A_Radio) * 10; //保留小数点后1位
            my_data[2 * ii] = temp;
            my_data[2 * ii + 1] =(temp >> 8);
						
						//printf("%d\n",temp);
					  
					
					 #if	CC1101_SEND_I_E_Simulation_data_status==1
							
							my_data[2 * ii] = xx;
							my_data[2 * ii + 1] = (xx>>8);
							xx++;
							if(xx>=65535)
								xx=(my_CC1101_chip_address+1)*15;
							
					#endif
        }
				//printf("\n===960===end==\n");

    }
    else if(my_row == 11 )  //电场
    {
        my_row = my_row - 10;
				xx=2000*(my_CC1101_chip_address+1);
        for(ii = 0; ii < WAVE_number_sec3; ii++)
        {

            temp=my_wave_record_sec3[my_row][ii];
            temp = temp * MY_VDD / 4096 * (my_E_ratio_value ) * 10; //保留小数点后1位
            my_data[2 * ii] = temp;
            my_data[2 * ii + 1] = (temp >> 8) & 0x00FF;
					
					 #if	CC1101_SEND_I_E_Simulation_data_status==1
							my_data[2 * ii] =xx;
							my_data[2 * ii + 1] =(xx>>8);
							xx--;
							if(xx<=0)
								xx=2000*(my_CC1101_chip_address+1);
					#endif
        }

    }
		

    //测试使用
#if Debug_Usart_OUT_WAVE_Chazhi==1
    my_cc = my_wave_record_sec2[my_row][0] - my_wave_record_sec2[my_row][880];
    printf("error chazhi=%d\n", my_cc);
#endif


    my_fun_101_send_long_data(USARTx, my_contorl_byte, my_data, WAVE_number_sec2 * 2, ADDRESS_CHECK, my_CC1101_dest_address); //发送转换后的结果



}

/*

功能：程序控制,与从MCU的对话流程
*/


extern uint16_t my_pc01_count;
extern uint16_t my_pc02_count;

extern uint16_t my_dianliu_exit_add; //此变量，用来计算中断或者定时产生时刻，查询录波1级缓冲区所用的地址
extern uint16_t my_Wave_It_add;  //记录中断时刻地址，共同步录波使用

extern uint8_t  my_Current_exit_Status; //表示电流产生中断
//extern uint16_t my_Current_Exit_add;

extern uint8_t  my_E_Field_exit_Status; //表示电场产生中断
extern uint16_t my_E_Field_exit_add;

extern uint8_t  my_Time_Cyc_exit_Status; //表示电场产生中断
//extern uint16_t my_Time_Cyc_exit_add;

extern uint16_t my_wave_write_add;

uint8_t my_DC_AC_status = 0; //发送数据的类型，0为只发送转换后的数据，1为先发送转换后的数据，后发送ADC的采样值。
uint16_t my_pro_step = 0;
uint8_t my_step_send_count = 0;

uint16_t my_step_3S_start_count = 0;
uint16_t my_step_3S_end_count = 0;

extern uint16_t my_send_3S_count;
extern uint8_t my_send_ALL_A_960_data_status;  //发送录波数据标志，1为表示发送，0表示不发送
extern uint8_t my_send_ALL_E_960_data_status;
extern uint8_t my_send_HALF_A_960_data_status;


/*
功能：发送遥信帧
*/
extern uint16_t  my_cyc_delay;
uint8_t my_test_value=0;
void my_fun_101send_Alarm_status_data(UART_HandleTypeDef* USARTx, uint8_t my_status, uint8_t my_contorl_byte)
{
    uint8_t my_data[4] = {0}; //添加一个计时数

    my_data[0] = my_Fault_Current_End_Status;
    my_data[1] = my_tim6_count;  //计时低字节
    my_data[2] = my_Fault_E_Fild_End_Status;
    my_data[3] = (my_tim6_count>>8);  //计时高字节
		
		if(my_cyc_alarm_status==1 && my_contorl_byte==0x02)
		{
			
			my_data[0]=0XF0;  //周期发送录波数据数据
			my_cyc_alarm_status=0;
		}
		
		//my_sys_start_status=0;
		#if OS_CC1101_ZSQ_Monidata==1
		if(my_contorl_byte==0x02)
		//my_data[0] = ++my_test_value;
		;
		else
			;//my_data[0]=0XFF;	
		
    //my_data[1] = (my_CC1101_chip_address+1);  //计时低字节
		if(my_contorl_byte==0x02)
    //my_data[2] = ++my_test_value;
		;
		else
			//my_data[2]=0xFF;
		;
    //my_data[3] = (my_CC1101_chip_address+1);  //计时高字节	
		//printf("---------my_test_value=%d\n-----",my_test_value);
		#endif

    my_fun_101_send_long_data(USARTx, my_contorl_byte, my_data, 4, ADDRESS_CHECK, my_CC1101_dest_address); //发送转换后的结果，利用串口进行发送



}





//功能：发现PWR心跳帧
void my_fun_101send_PWR_heart_data(UART_HandleTypeDef* USARTx, uint8_t my_contorl_byte)
{

    uint8_t my_data[8] = {0};
    if(my_PWR_all_count == 1)
        my_PWR_heart_count++;

    my_data[0] = 0X10;
    my_data[1] = 0X1F;
    my_data[2] = 0X01;
    my_data[3] = 0X00;
    my_data[4] = my_PWR_heart_count;
    my_data[5] = (my_PWR_heart_count >> 8);
    my_data[6] = my_fun_101check_generate(my_data, 1);
    my_data[7] = 0X16;

    HAL_UART_Transmit(USARTx, my_data, 8, 3000);


}
void my_fun_101send_PWR_OK_data(UART_HandleTypeDef* USARTx, uint8_t my_contorl_byte)
{

    uint8_t my_data[8] = {0};


    my_data[0] = 0X10;
    my_data[1] = 0X20;
    my_data[2] = 0X01;
    my_data[3] = 0X00;
    my_data[4] = my_PWR_heart_count;
    my_data[5] = (my_PWR_heart_count >> 8);
    my_data[6] = my_fun_101check_generate(my_data, 1);
    my_data[7] = 0X16;

    HAL_UART_Transmit(USARTx, my_data, 8, 3000);
}



//==========
/*
功能：利用CC1101发送短帧数据，8个字节
参数1：控制ID
参数2：发送的16bit数据
参数3：发送模式，广播或者地址检查
参数4：目标地址，CC1101的地址
*/

void my_fun_101_send_short_data_CC1101(uint8_t control_bye, uint16_t data, TX_DATA_MODE mode, INT8U desc_address)
{


    my_101_send_buf[0] = 0x10; //帧头
    my_101_send_buf[1] = control_bye; //ID
    my_101_send_buf[2] = 01; //ADD_L
    my_101_send_buf[3] = 02; //ADD_H
    my_101_send_buf[4] = data; //数据低字节
    my_101_send_buf[5] = (data >> 8) ; //数据高字节
    my_101_send_buf[6] = my_fun_101check_generate(my_101_send_buf, 1);
    my_101_send_buf[7] = 0x16;

    my_fun_CC1101_send_long_data(my_101_send_buf, 8, mode, desc_address); //利用CC1101发送数据
}
/*

功能：发送长帧数据
*/


void my_fun_101_send_long_data_CC1101(uint8_t control_bye, uint8_t *send_buf, uint16_t send_number, TX_DATA_MODE mode, INT8U desc_address)
{
    uint16_t ii = 0;
    uint16_t my_length = 0;


    my_101_send_buf[0] = 0x68; //帧头
    my_101_send_buf[1] = (uint8_t)((send_number + 3) & 0x00ff); //长度,从ID开始算的长度
    my_101_send_buf[2] = (uint8_t)(((send_number + 3) & 0xff00) >> 8); //长度，除帧头以外的长度

    my_101_send_buf[3] = my_101_send_buf[1];
    my_101_send_buf[4] = my_101_send_buf[2];

    my_101_send_buf[5] = 0x68; //帧头
    my_101_send_buf[6] = control_bye;
    my_101_send_buf[7] = 01; //ADD_L
    my_101_send_buf[8] = 02; //ADD_H


    for(ii = 0; ii < send_number; ii++)
    {
        my_101_send_buf[9 + ii] = send_buf[ii];

    }
    my_101_send_buf[9 + ii] = my_fun_101check_generate(my_101_send_buf, 1);;
    my_101_send_buf[9 + ii + 1] = 0x16;
    my_length = send_number + 11;

    my_fun_CC1101_send_long_data(my_101_send_buf, my_length, mode, desc_address); //利用CC1101发送数据

}
