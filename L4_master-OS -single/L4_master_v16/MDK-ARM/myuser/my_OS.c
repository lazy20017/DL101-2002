#include "my_OS.h"
#include "my_usart.h"
#include "my_ADC.h"
#include "my_wave_rec.h"
#include "my_101.h"
#include "my_extern_val.h"

extern EventGroupHandle_t xCreatedEventGroup;
extern osMessageQId myQueue01Handle;
extern osMessageQId myQueue02Handle;
extern osMutexId myMutex01Handle;

extern osThreadId defaultTaskHandle;
extern osThreadId myTask02Handle;
extern osThreadId myTask03Handle;
extern osThreadId myTask04Handle;

extern osThreadId myTask08Handle;



extern uint8_t my_CC1101_dest_address;
extern uint8_t my_cc1101_tx_buf[];
extern uint8_t my_CC1101_COM_Fram_buf[];


extern uint16_t my_wave_write_add;
extern uint16_t my_dianliu_exit_add;
//extern uint16_t my_Time_Cyc_exit_add;
extern uint8_t my_DC_AC_status;
extern uint8_t my_CC1101_Frame_status;
extern uint8_t my_CC1101_COM_Fram_buf[];
extern uint16_t  my_CC1101_all_step;


extern uint8_t my_UART1_Status;
extern uint8_t my_UART2_Status;
extern uint8_t my_UART3_Status;

extern uint8_t my_use_alarm_rec_data_status_Efild;
extern double ADC2_Filer_value_buf_2[ADC2_COLM][3];

extern uint8_t my_Fault_Current_End_Status ; //
extern uint8_t my_Fault_E_Fild_End_Status;

uint16_t  my_GPRS_all_step = 0;
uint8_t my_GPRS_all_count = 0;

uint8_t  my_CC1101_all_count = 0; //重复发送计数
uint16_t my_PWR_all_step = 0;
uint8_t my_PWR_all_count = 0;

uint8_t temp8 = 0;

void my_fun_give_Queue(osMessageQId *my_QHL, uint16_t temp_step)
{
    BaseType_t pt = NULL;
    BaseType_t xResult;
    xResult = xQueueSendFromISR(*my_QHL, &temp_step, &pt);

    if(xResult != pdFAIL)
    {
        //printf(" send QH OK--[%XH]\r\n",temp_step);
        portYIELD_FROM_ISR(pt);

    }
    else
    {
        printf(" send QH ERROR--[%XH],xRseult=[%XH]--\r\n", temp_step, xResult);
        
        HAL_NVIC_SystemReset();  //系统重启，向指令队列放数据导致拥塞，
				



    }
    //printf("queue3 is %d\r\n",temp8);
}


//=====CC1101发送使用

//============
void my_fun_CC1101_time_dialog_tx2(
    uint16_t my_get_step,
    uint16_t my_before_step,
    uint16_t my_now_step,
    uint8_t end_status,
    void (*ptfun)(void),
		uint8_t re_send_count
)
{
    //===判断部分
    if(my_get_step == my_now_step && my_before_step == 0X00)
    {
        //printf("CC1101 TX--step = [%XH]\r\n",my_now_step);
        my_CC1101_all_count = 0;
        my_CC1101_all_step = my_now_step;
        my_get_step = 0;
    }
    else if(my_get_step == my_now_step && my_before_step == my_CC1101_all_step)
    {
        // printf("CC1101 TX--step = [%XH]\r\n",my_now_step);
        my_CC1101_all_count = 0;
        my_CC1101_all_step = my_now_step;
        my_get_step = 0;
    }

    //===重复发送部分
    if( my_now_step == my_CC1101_all_step && my_CC1101_all_count < re_send_count )
    {
        my_CC1101_all_count++;
        printf("CC1101 TX-Fun= [%XH]--%d\r\n", my_now_step, my_CC1101_all_count);
        //osDelay(1);
        //HAL_Delay(1);//CC1101发送数据，要延时一下，因为操作系统任务切换需要1ms
        ptfun();		//调用对应的函数
    }
    else if(my_CC1101_all_count >= re_send_count)
    {

        my_CC1101_all_count = 0;
        my_CC1101_all_step = 0x00;
        my_Time_Cyc_exit_Status = 0;


        if(my_CC1101_Sleep_status == 1 || my_DTU_send_faile_count>=re_send_count-1)
        {
            CC1101SetSleep();
        }
        //DTU失败次数标识
        my_DTU_send_faile_count++;
        if(my_DTU_send_faile_count >= 0XFFFF)
            my_DTU_send_faile_count = re_send_count;

    }

    //====只发送一次就结束
    if(end_status == 1 && my_CC1101_all_count > 0 && my_CC1101_all_step == my_now_step )
    {
        my_CC1101_all_count = 0;
        my_CC1101_all_step = 0x00;

        //my_Time_Cyc_exit_Status = 0;



        if(my_CC1101_Sleep_status == 1)
        {
            CC1101SetSleep();
        }

    }

}


//==========
void my_fun_CC1101_time_dialog_rx2(
    osMessageQId *QHL_send,
    uint16_t my_get_step,   //
    uint16_t my_before_step,
    uint16_t my_now_step,
    uint16_t my_next_step,
    uint8_t end_status,
    uint8_t (*ptfun)(void)
)
{
    uint8_t my_status = 0;
    uint8_t my_temp = 0;

    //=====0====
    if(my_get_step == my_now_step && my_before_step == 0x00) //无条件处理接收到的数据
    {
        my_status = 1;
        my_DTU_send_faile_count = 0;
    }
    else if(my_get_step == my_now_step && my_before_step == my_CC1101_all_step) //条件绑定，有前提条件
    {
        my_status = 1;
        my_DTU_send_faile_count = 0; //DTU在发送数据失败计数
    }
    else
    {
        return;

    }
    //======1=====
    if(my_status == 1 && end_status == 0)
    {
        //printf("CC1101 RX-step = [%XH]\r\n",my_now_step);


        my_temp = ptfun();
        if(my_temp == 1)
        {
            my_CC1101_all_step = my_now_step;	//当前状态
            xQueueSend(*QHL_send, &my_next_step, 100);	//标识下一个状态
        }
        else
            printf("接收数据错误，不进行状态转换\r\n");
    }
    else if(my_status == 1 && end_status == 1)
    {
        // printf("CC1101 RX-step = [%XH]\r\n",my_now_step);
        ptfun();
        my_CC1101_all_step = 0X00; //结束状态

        my_Time_Cyc_exit_Status = 0;

        if(my_CC1101_Sleep_status == 1)
        {
            CC1101SetSleep();
        }

    }

    //

}


//=========
//#define com_long_10  "\x10\x20\x3\x4\x5\x6\x7\x16"
//#define com_short_68 "\x68\x04\x04\x68\x1\x2\x3\x4\x5\x16"
//#define com_long_68 "\x68\x05\x00\x05\x00\x68\x1\x2\x3\x4\x5\x6\x16"
//uint8_t my_buf2[]=com_long_10;
//uint8_t my_buf3[]=com_short_68;
//uint8_t my_buf4[]=com_long_68;
//uint8_t my_cc1101_tx_buf[64]={0x10,0x20,0x13,0x14,0x15,0x16};


uint8_t my_heart_count = 0;
void my_fun_CC1101_test2(void)
{
    uint8_t temp_status = 0;
    if(my_CC1101_COM_Fram_buf[0] == 0x10)
    {
        temp_status = my_CC1101_COM_Fram_buf[1];
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0);
    }
    else if (my_CC1101_COM_Fram_buf[0] == 0x68)
    {
        temp_status = my_CC1101_COM_Fram_buf[6];
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0);
    }

    if(temp_status == 0x75)
        printf("====CC heart count %d=======\r\n", my_heart_count++);
    else if(temp_status == 0xF0)
    {
        printf("接收升级数据开始  F0\r\n");
    }
    else if(temp_status == 0xF2)
    {
        printf("接收升级数据开始  block=%d   F2\r\n", my_CC1101_COM_Fram_buf[7]);
    }
    else if(temp_status == 0xF4)
    {
        printf("接收最后的数据包   F4\r\n");
    }
    else if(temp_status == 0xF6)
    {
        printf("接收最后的数据包   block=%d  F6\r\n", my_CC1101_COM_Fram_buf[7]);
    }
    else if(temp_status == 0xF8)
    {
        printf("升级数据传输结束   F8\r\n");
    }


}
//=====================
//指示器模拟程序部分，用来模拟指示器的数据发送给DTU
//1A--1,1B--2,1C--3    2A--1,2B--2,2C--3    3A--1,3B--2,3C--3
void my_fun_indicator_heart_awaken_send(void)
{
    //10-71-LL-YY-01-00-CRC-16
    static uint8_t count = 0;
    uint8_t indicator_ABC = 1;
    count++;
    my_cc1101_tx_buf[0] = 0x10;
    my_cc1101_tx_buf[1] = 0X71;
    my_cc1101_tx_buf[2] = indicator_ABC;
    my_cc1101_tx_buf[3] = 00;
    my_cc1101_tx_buf[4] = count;
    my_cc1101_tx_buf[5] = 00;
    my_cc1101_tx_buf[6] = count;
    my_cc1101_tx_buf[7] = 0X16;

    CC1101SendPacket_add(my_cc1101_tx_buf, 8, ADDRESS_CHECK, my_CC1101_dest_address);
    my_fun_display_buf_16(my_cc1101_tx_buf, 8, 1); //测试使用
}
//1A--1,1B--2,1C--3    2A--1,2B--2,2C--3    3A--1,3B--2,3C--3
void my_fun_indicator_heart_data_send(void)
{
    //10-71-LL-YY-01-00-CRC-16
    static uint8_t count = 0;
    uint8_t indicator_ABC = 1;
    count++;

    my_cc1101_tx_buf[0] = 0x10;
    my_cc1101_tx_buf[1] = 0X73;
    my_cc1101_tx_buf[2] = indicator_ABC;
    my_cc1101_tx_buf[3] = 00;
    my_cc1101_tx_buf[4] = count;
    my_cc1101_tx_buf[5] = 00;
    my_cc1101_tx_buf[6] = count;
    my_cc1101_tx_buf[7] = 0X16;

    CC1101SendPacket_add(my_cc1101_tx_buf, 8, ADDRESS_CHECK, my_CC1101_dest_address);
    my_fun_display_buf_16(my_cc1101_tx_buf, 8, 1); //测试使用
}

void my_fun_indicator_heart_data_send2(void)
{
    //10-71-LL-YY-01-00-CRC-16
    static uint8_t count = 0;
    uint8_t indicator_ABC = 1;
    count++;

    my_cc1101_tx_buf[0] = 0x10;
    my_cc1101_tx_buf[1] = 0X75;
    my_cc1101_tx_buf[2] = indicator_ABC;
    my_cc1101_tx_buf[3] = 00;
    my_cc1101_tx_buf[4] = count;
    my_cc1101_tx_buf[5] = 00;
    my_cc1101_tx_buf[6] = count;
    my_cc1101_tx_buf[7] = 0X16;

    CC1101SendPacket_add(my_cc1101_tx_buf, 8, ADDRESS_CHECK, my_CC1101_dest_address);
    my_fun_display_buf_16(my_cc1101_tx_buf, 8, 1); //测试使用
}


//=========接收处理函数==========
uint8_t my_fun_GPRS_RX_test1(void) //此函数为结束函数，收到OK帧后，结束对话过程
{

    printf("GPRS dialog is Finish!---[%XH]\r\n", my_GPRS_all_step);
    return 1;

}

//遥信接收到OK帧
uint8_t my_fun_GPRS_RX_test2(void)  //接收到过程帧，不做处理
{

    printf("GPRS dialog get OK frame---[%XH]\r\n", my_GPRS_all_step);
    return 1;
}
//===CC1101,发送处理函数
void my_fun_TX_CC1101_test0(void)  //遥信
{

    //=====0 发送遥信数据包
    if(my_CC1101_all_step == 0x0001)
    {
        //==周期
        my_fun_101send_Alarm_status_data(&huart2, my_DC_AC_status, 0X01);
    }
    else if(my_CC1101_all_step == 0x0002)
    {
        //==报警
        my_fun_101send_Alarm_status_data(&huart2, my_DC_AC_status, 0X02);
    }



#if OS_CC1101_auto_reveive_OK==1
    my_fun_give_Queue(&myQueue02Handle, 0x2000); //@@@@@
#endif


}

void my_fun_TX_CC1101_test1(void)  //遥测 直流
{
    if(my_CC1101_all_step == 0x0040) //周期
    {
        //=====1 发送直流数据包
        my_fun_101send_DC_data(&huart2, my_DC_AC_status, 0X40); //发送直流数据
    }
    else if(my_CC1101_all_step == 0x0050)
    {
        //====报警
        my_fun_101send_DC_data(&huart2, my_DC_AC_status, 0X50); //发送直流数据

    }

#if Debug_Usart_OUT_DC_DATA_status==1

    my_adc_1_convert_dis(0);
#endif

#if OS_CC1101_auto_reveive_OK==1
    my_fun_give_Queue(&myQueue02Handle, 0x2000); //@@@@@
#endif




}
void my_fun_TX_CC1101_test2(void)  //遥测 交流有效值
{

    if(my_CC1101_all_step == 0x0041) //交流有效值
    {
        //===2发送AC
        my_fun_101send_AC_data(&huart2, my_DC_AC_status, 0X41); //
    }
    else if(my_CC1101_all_step == 0x0051)
    {
        //报警
        my_fun_101send_AC_data(&huart2, my_DC_AC_status, 0X51); //
    }
    //====	2 发送交流数据包
		
#if Debug_Usart_out_ADCdata==1
		//@@@ 发送显示数据AC，到调试串口
    printf("2All_A:AVR=%.2f, RMS=%.2f, MAX=%.2f, \n", ADC2_Filer_value_buf_2[0][0], ADC2_Filer_value_buf_2[0][1], ADC2_Filer_value_buf_2[0][2]);
    printf("2ALL_E:AVR=%.2f, RMS=%.2f, MAX=%.2f, \n", ADC2_Filer_value_buf_2[1][0], ADC2_Filer_value_buf_2[1][1], ADC2_Filer_value_buf_2[1][2]);
    printf("2Hal_A:AVR=%.2f, RMS=%.2f, MAX=%.2f, \n", ADC2_Filer_value_buf_2[2][0], ADC2_Filer_value_buf_2[2][1], ADC2_Filer_value_buf_2[2][2]); 
		printf("A_status=%X, E_status=%X\n",my_Fault_Current_End_Status,my_Fault_E_Fild_End_Status);
#endif


#if OS_CC1101_auto_reveive_OK==1
    my_fun_give_Queue(&myQueue02Handle, 0x2000); //@@@@@
#endif

}

void my_fun_TX_CC1101_test3(void)
{


    if(my_CC1101_all_step == 0x0042)
    {
        //=====3  发送12个周期的有效值12TAC
        my_fun_101send_AC12T_Cyc_data(&huart2, my_DC_AC_status, 0X42); //
    }
    else if(my_CC1101_all_step == 0x0052)
    {
        //==报警
        my_fun_101send_AC12T_Cyc_data(&huart2, my_DC_AC_status, 0X52); //

    }


#if OS_CC1101_auto_reveive_OK==1
    my_fun_give_Queue(&myQueue02Handle, 0x2000); //@@@@@
#endif

}
uint8_t CC1101_960data_Efield_STATUS = 0;
void my_fun_TX_CC1101_test4(void)
{
    if(my_CC1101_all_step == 0x0043)
    {
        //=====4  发送录波数据，周期
        if (CC1101_960data_Efield_STATUS == 1) //发送波形的选择，1为电场，0为电流
            my_fun_101send_AC_Rec_data(&huart2, my_DC_AC_status, 0X44); //960电场
        else
            my_fun_101send_AC_Rec_data(&huart2, my_DC_AC_status, 0X43); //960全波电流

    }
    else if(my_CC1101_all_step == 0x0053)
    {
        //====录波数据，报警
        if (CC1101_960data_Efield_STATUS == 1)
            my_fun_101send_AC_Rec_data(&huart2, my_DC_AC_status, 0X54); //电场
        else
            my_fun_101send_AC_Rec_data(&huart2, my_DC_AC_status, 0X53); //电流
    }

#if CC1101_TX_Delay==1
    HAL_Delay(2000);
#endif

#if Debug_Usart_out_ADCdata==1
    //my_adc2_convert_dis(0); //@@@ 发送显示数据AC，到调试串口
#endif
#if OS_CC1101_auto_reveive_OK==1
    my_fun_give_Queue(&myQueue02Handle, 0x2000); //@@@@@
#endif

}
//电场
void my_fun_TX_CC1101_test5(void)
{
    if(my_CC1101_all_step == 0x0044)
    {
        //=====4  发送录波数据，周期
       
            my_fun_101send_AC_Rec_data(&huart2, my_DC_AC_status, 0X44); //960电场
        

    }
    else if(my_CC1101_all_step == 0x0054)
    {
        //====录波数据，报警
        
            my_fun_101send_AC_Rec_data(&huart2, my_DC_AC_status, 0X54); //电场
        
    }

#if CC1101_TX_Delay==1
    HAL_Delay(2000);
#endif

#if Debug_Usart_out_ADCdata==1
    //my_adc2_convert_dis(0); //@@@ 发送显示数据AC，到调试串口
#endif
#if OS_CC1101_auto_reveive_OK==1
    my_fun_give_Queue(&myQueue02Handle, 0x2000); //@@@@@
#endif

}

//电场 over



void my_fun_CC1101_test1(void)
{
    uint8_t *pt;
    uint8_t my_step = 0;
    uint8_t indicator_ABC = 1;
    if(my_step == 0)
    {
        my_cc1101_tx_buf[0] = 0x10;
        my_cc1101_tx_buf[1] = 0X20;
        my_cc1101_tx_buf[2] = indicator_ABC;
        my_cc1101_tx_buf[3] = 00;
        my_cc1101_tx_buf[4] = 00;
        my_cc1101_tx_buf[5] = 00;
        my_cc1101_tx_buf[6] = 00;
        my_cc1101_tx_buf[7] = 0X16;
        pt = my_cc1101_tx_buf;
        pt[6] = my_fun_101check_generate(pt, 1);
        HAL_Delay(100);
        CC1101SendPacket_add( pt, 8,  ADDRESS_CHECK, my_CC1101_dest_address);
        my_fun_display_buf_16(pt, 8, 1); //测试使用

    }


}

//===CC1101  接收对话
extern uint16_t my_que1_wait_time;
uint8_t my_fun_RX_CC1101_text0_RX_OK(void)
{
    uint16_t my_temp = 0;
		uint16_t my_inf_add=0;
		uint8_t my_inf_len=0;
		uint16_t my_temp16=0;
		//校正timer
    if(my_CC1101_COM_Fram_buf[1] == 0x20 && my_CC1101_COM_Fram_buf[4]!=0XFF && my_CC1101_COM_Fram_buf[5] !=0XFF) //进行tim6的校时
    {
        my_temp = my_CC1101_COM_Fram_buf[5];
        my_temp = (my_temp << 8) + my_CC1101_COM_Fram_buf[4];
        my_tim6_count = my_temp;
    }
		
		//各种确认命令分析
    if(my_CC1101_COM_Fram_buf[1] == 0x20 && my_CC1101_all_step == 0x0042)
    {
				my_Time_Cyc_exit_Status = 0;
				if(my_CC1101_Sleep_status==1)
					CC1101SetSleep();
				
        printf("====CC1101 CYC TIME FINISH!!===\n\n");
    }
    else if(my_CC1101_COM_Fram_buf[1] == 0x20 && my_CC1101_all_step == 0x00E0)
    {
				my_Time_Cyc_exit_Status = 0;
				if(my_CC1101_Sleep_status==1)
					CC1101SetSleep();
        printf("====CC1101 Heart TIME FINISH!!===\n\n");
    }
		else if(my_CC1101_COM_Fram_buf[1] == 0x20 && my_CC1101_all_step == 0x0053)
    {  
			if(my_use_alarm_rec_data_status_Efild==0)
			{
				//my_zsq_ALarm_send_status=0;
				if(my_CC1101_Sleep_status==1)
				 	CC1101SetSleep();
			}
        printf("====@@@@ CC1101 ALarm TIME FINISH!!==dianliu=\n\n");
    }
		
		else if(my_CC1101_COM_Fram_buf[1] == 0x20 && my_CC1101_all_step == 0x0054)
    {
				my_zsq_ALarm_send_status=0;
				if(my_CC1101_Sleep_status==1)
					CC1101SetSleep();
        printf("====@@@@ CC1101 ALarm TIME FINISH!!==jiedi=\n\n");
    }
		//参数设置命令
		if( my_CC1101_all_step == 0x00E1 && my_CC1101_COM_Fram_buf[0]==0x68 && my_CC1101_COM_Fram_buf[5]==0x68 && my_CC1101_COM_Fram_buf[6] == 0x3F)
		{
			
			
			printf("===get config parameter!!===,len=%d\n", my_CC1101_COM_Fram_buf[1]);//len>5
			my_inf_add=my_CC1101_COM_Fram_buf[10];
			my_inf_add=(my_inf_add<<8)+my_CC1101_COM_Fram_buf[9];
			my_inf_len=my_CC1101_COM_Fram_buf[1];
			
			//RTC
			if(my_inf_len>5)
				my_inf_len=my_inf_len-3;
			
			if(my_inf_add==0X4001)
			{
				
			my_temp16=my_CC1101_COM_Fram_buf[12];
			my_temp16=(my_temp16<<8)+my_CC1101_COM_Fram_buf[11];
				
			HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN);
			HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);
			printf("OLD---RTC  %d-%d-%d %d:%d:%d===\n",my_RTC_date.Year,my_RTC_date.Month,my_RTC_date.Date,my_RTC_time.Hours,my_RTC_time.Minutes,my_RTC_time.Seconds);	
			
			my_RTC_time.Seconds=my_CC1101_COM_Fram_buf[13];
			my_RTC_time.Minutes=my_CC1101_COM_Fram_buf[15];
			my_RTC_time.Hours=	my_CC1101_COM_Fram_buf[16];
			my_RTC_date.Date=my_CC1101_COM_Fram_buf[17];
			my_RTC_date.Month=my_CC1101_COM_Fram_buf[18];
			my_RTC_date.Year=my_CC1101_COM_Fram_buf[19];			
				
			HAL_RTC_SetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN);
			HAL_RTC_SetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);
				
			HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN);
			HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);
			printf("NEW---RTC  %d-%d-%d %d:%d:%d===\n",my_RTC_date.Year,my_RTC_date.Month,my_RTC_date.Date,my_RTC_time.Hours,my_RTC_time.Minutes,my_RTC_time.Seconds);	
			
				
			}
			
		}
		
		
		
		//任务延时时间设定
		if(my_CC1101_COM_Fram_buf[1] == 0x20 && my_CC1101_all_step == 0x0052)
			my_que1_wait_time=10000;
		else if(my_CC1101_COM_Fram_buf[1] == 0x20 && my_CC1101_all_step == 0x0053)
			my_que1_wait_time=10000;
		else
			my_que1_wait_time=2000;
    return 1;
}






void my_fun_PWR_time_dialog_tx2(
    uint16_t my_get_step,
    uint16_t my_before_step,
    uint16_t my_now_step,
    uint8_t end_status,
    void (*ptfun)(void)
)
{
    //===判断部分
    if(my_get_step == my_now_step && my_before_step == 0X00)
    {
        //printf("CC1101 TX--step = [%XH]\r\n",my_now_step);
        my_PWR_all_count = 0;
        my_PWR_all_step = my_now_step;
        my_get_step = 0;
    }
    else if(my_get_step == my_now_step && my_before_step == my_PWR_all_step)
    {
        // printf("CC1101 TX--step = [%XH]\r\n",my_now_step);
        my_PWR_all_count = 0;
        my_PWR_all_step = my_now_step;
        my_get_step = 0;
    }

    //===重复发送部分
    if( my_now_step == my_PWR_all_step && my_PWR_all_count < 3 && my_get_step == 0)
    {
        my_PWR_all_count++;
        printf("PWR TX-Fun= [%XH]--%d\r\n", my_now_step, my_PWR_all_count);
        //osDelay(1);
        //HAL_Delay(1);//CC1101发送数据，要延时一下，因为操作系统任务切换需要1ms
        ptfun();		//调用对应的函数
    }
    else if(my_PWR_all_count >= 3)
    {
        my_PWR_all_count = 0;
        my_PWR_all_step = 0x00;
    }

    //====只发送一次就结束
    if(end_status == 1 && my_PWR_all_count > 0 && my_PWR_all_step == my_now_step && my_get_step == 0)
    {
        my_CC1101_all_count = 0;
        my_PWR_all_step = 0x00;
    }

}


//==========
void my_fun_PWR_time_dialog_rx2(
    osMessageQId *QHL_send,
    uint16_t my_get_step,   //
    uint16_t my_before_step,
    uint16_t my_now_step,
    uint16_t my_next_step,
    uint8_t end_status,
    uint8_t (*ptfun)(void)
)
{
    uint8_t my_status = 0;
    uint8_t my_temp = 0;
    //=====0====
    if(my_get_step == my_now_step && my_before_step == 0x00) //无条件处理接收到的数据
    {
        my_status = 1;
    }
    else if(my_get_step == my_now_step && my_before_step == my_PWR_all_step) //条件绑定，有前提条件
    {
        my_status = 1;
    }
    else
    {
        return;
    }
    //======1=====
    if(my_status == 1 && end_status == 0)
    {
        //printf("CC1101 RX-step = [%XH]\r\n",my_now_step);
        my_temp = ptfun();
        if(my_temp == 1)
        {
            my_PWR_all_step = my_now_step;	//当前状态
            xQueueSend(*QHL_send, &my_next_step, 100);	//标识下一个状态
        }
        else
            printf("接收数据错误，不进行状态转换\r\n");
    }
    else if(my_status == 1 && end_status == 1)
    {
        // printf("CC1101 RX-step = [%XH]\r\n",my_now_step);
        ptfun();
        my_PWR_all_step = 0X00; //结束状态

    }

    //

}

/*
功能：CC1101重新初始化
*/
void my_fun_CC1101_init_resume(void)
{
    uint8_t my_status = 0;
    uint8_t my_rx_count = 0;
    //xSemaphoreTake(myMutex01Handle,1000);
    CC1101SetIdle();
    HAL_Delay(10);
    my_status = CC1101ReadStatus(CC1101_MARCSTATE);
    my_rx_count = CC1101GetRXCnt();
    //xSemaphoreGive(myMutex01Handle);

    if(my_status == 0x01 && my_rx_count > 0)
    {
        printf("------ CC1101 status=[%XH] RXBUF=%d \n", my_status, my_rx_count);
        my_fun_CC1101_init_reum();
    }

    if(my_status != 0x01 && my_status != 0x0D &&  my_status != 0x13  ) //0X01空闲，0X0D接收，0X13发送,0x11接收溢出
    {
        printf("--error CC_status=[%XH] \n", my_status);
        //my_fun_CC1101_init_reum();
        printf("--inint after CC_status=[%XH] \n", my_status);
    }

}

void my_fun_usart_init_resume(void)
{

    if(my_UART2_Status == 0X01)
    {
        MX_USART2_UART_Init();
        my_UART2_Status = 0;
        HAL_UART_Receive_IT(&huart2, &rsbuf2[rsbuf2pt_write], 1); //开启接收USART3函数
    }
}

void my_fun_task_heap_value(void)
{

    portBASE_TYPE uxHighWaterMark;

    uxHighWaterMark = uxTaskGetStackHighWaterMark( defaultTaskHandle );
    printf("task01 heap value=%d\r\n", uxHighWaterMark);

    uxHighWaterMark = uxTaskGetStackHighWaterMark( myTask02Handle );
    printf("task02 heap value=%d\r\n", uxHighWaterMark);
    uxHighWaterMark = uxTaskGetStackHighWaterMark( myTask03Handle );
    printf("task03 heap value=%d\r\n", uxHighWaterMark);
    uxHighWaterMark = uxTaskGetStackHighWaterMark( myTask04Handle );
    printf("task04 heap value=%d\r\n", uxHighWaterMark);
    //uxHighWaterMark = uxTaskGetStackHighWaterMark( myTask05Handle );
    //printf("task05 heap value=%d\r\n", uxHighWaterMark);
    //uxHighWaterMark = uxTaskGetStackHighWaterMark( myTask06Handle );
    //printf("task06 heap value=%d\r\n", uxHighWaterMark);
    //uxHighWaterMark = uxTaskGetStackHighWaterMark( myTask07Handle );
    //printf("task07 heap value=%d\r\n", uxHighWaterMark);
    uxHighWaterMark = uxTaskGetStackHighWaterMark( myTask08Handle );
    printf("task08 heap value=%d\r\n", uxHighWaterMark);


}



void my_fun_TX_CC1101_heart(void)  //心跳
{

    my_fun_101_send_short_data(&huart2, 0x1F, my_tim6_count, ADDRESS_CHECK, my_CC1101_dest_address); //发送短帧



#if OS_CC1101_auto_reveive_OK==1
    my_fun_give_Queue(&myQueue02Handle, 0x2000); //@@@@@
#endif


}


//请求参数设置
void my_fun_TX_CC1101_config(void)  //参数设置
{

    my_fun_101_send_short_data(&huart2, 0x2F, my_tim6_count, ADDRESS_CHECK, my_CC1101_dest_address); //发送短帧



#if OS_CC1101_auto_reveive_OK==1
    my_fun_give_Queue(&myQueue02Handle, 0xE100); //@@@@@
#endif


}

void my_fun_TX_CC1101_config2(void)  //参数设置
{

    my_fun_101_send_short_data(&huart2, 0x4F, my_tim6_count, ADDRESS_CHECK, my_CC1101_dest_address); //发送短帧
		printf("===config parameter is finish!!!\n");

}
