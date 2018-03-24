#include "gpio.h"
#include "tim.h"
#include "my_led.h"
#include "my_usart.h"
#include "my_extern_val.h"
#include "my_gloabal_val.h"
#include "cmsis_os.h"
#include "my_OS.h"

#include "my_time.h"
#include "my_ADC.h"
#include "my_wave_rec.h"
#include "my_DAC.h"

extern EventGroupHandle_t xCreatedEventGroup2;
extern osMessageQId myQueue01Handle;

extern uint16_t my_wave_record[WAVE_Channel_Num][WAVE_number]; //全波的一级缓存，利用定时器进行波形的存储，假定20ms采集一次，看能采集到多少个数据。
extern uint16_t my_wave_write_add;  //全波录波一级缓存指针
extern uint16_t ADC2_GetValue[ADC2_ROW][ADC2_COLM];

uint8_t my_IT_status = 0; //为0表示，没有中断，为1表示，产生了中断，并且把2及缓存数据已经移动至1级缓存中
uint8_t my_IT_Count = 0;  //中断次数记录
uint8_t my_reset_count=0;

extern uint16_t  my_CC1101_all_step;
extern uint8_t my_Fault_Current_End_Status; //
extern uint8_t my_Fault_E_Fild_End_Status; //

uint16_t my_LED_ON_Count_gate=3600*8;  //用了设定LED报警后，闪烁的实际，1秒变化一次，最长3600*8,8个小时
uint16_t my_LED_ON_Count=0; //
uint16_t my_CC1101_RSSI_count=0; //cc1101的信号强度控制

extern int my_CC1101_RSSI;
extern double ADC1_Filer_value_buf[];


void HAL_TIM_PeriodElapsedCallback2(TIM_HandleTypeDef *htim)
{


    if(htim == &htim7)
    {

        //    //全波录波方案，不断的录波250us取一个点，
        my_wave_record[0][my_wave_write_add] = (ADC2_GetValue[0][0]+ 	ADC2_GetValue[1][0]+ ADC2_GetValue[2][0])/3; //全波电流
        my_wave_record[1][my_wave_write_add] = (ADC2_GetValue[0][1]+		ADC2_GetValue[1][1]+	ADC2_GetValue[2][1])/3; //全波电场
        my_wave_record[2][my_wave_write_add] = (ADC2_GetValue[0][2]+		ADC2_GetValue[1][2]+	ADC2_GetValue[2][2])/3; //全波电场

        my_wave_write_add++;
        if(my_wave_write_add >= WAVE_number)
            my_wave_write_add = 0;

        //自定义处理部分，需要把系统自动生成的下面的代码屏蔽掉

        __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
        return ;

    }


    //============
    else if(htim == &htim6) //TIM6为1s一次，为了响应中断
    {
        my_tim6_count++;
				my_Line_short_count++;
			 //喂狗信号
			  //LED3_TOGGLE;
			  WDI1_TOGGLE;
        
        BaseType_t xResult;
        BaseType_t xHigherPriorityTaskWoken=pdFAIL;

        if(my_Current_exit_Status==1 || my_E_Field_exit_Status==1 || my_sys_start_status==1)
				{
          printf("==Current_exit_status=%d,e_filed_exit_status=%d,my_sys_start_status=%d  time6=%d,reset_count=%d\n"
					,my_Current_exit_Status,my_E_Field_exit_Status,my_sys_start_status,my_tim6_count,my_reset_count);  
					
					my_reset_count++;
					if(my_reset_count>250)
						HAL_NVIC_SystemReset();  //系统重启
						
					return;
				}
				my_reset_count=0;

       
				//开启周期采样
				 if(my_tim6_count%my_cyc_time_count==0 && my_tim6_count!=0) 
				{
					  my_Time_Cyc_exit_Status=1;
            my_dianliu_exit_add=my_wave_write_add; //当前录波地址
            my_Time_Cyc_exit_add = my_wave_write_add;
					
							switch(my_CC1101_chip_address)
						{
						case 0x01:
								my_cyc_delay=my_tim6_count+3;
								break;
						case 0x02:
								my_cyc_delay=my_tim6_count+10;
								break;
						case 0x03:
								my_cyc_delay=my_tim6_count+17;
								break;
						default:
								my_cyc_delay=0;
						}
					
					
            xResult=	xEventGroupSetBitsFromISR(xCreatedEventGroup2, 0X02,&xHigherPriorityTaskWoken);
            if(xResult!=pdFAIL)
            {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
					
				}

        


        //开启周期发送
        if(my_tim6_count==my_cyc_delay )
        {
          printf("\n------my_tim6_count6=%d -----\n",my_tim6_count);
					my_fun_give_Queue(&myQueue01Handle, 0X0001); //发送周期数据
        }

			//设置参数请求
			if(my_tim6_count % (123+(my_CC1101_chip_address)*2) ==0 && my_CC1101_all_step==0x00 && my_tim6_count!=my_cyc_delay)
			{
					printf("==send config parameter，my_tim6_count=%d !!!--2\n",my_tim6_count);
					my_fun_give_Queue(&myQueue01Handle, 0X00E1); //发送报警
			}
       
				
				//CC1101重新初始化
					if(my_tim6_count%3600==0 && my_tim6_count!=0 && my_tim6_count!=my_cyc_delay)	
					{
						my_fun_CC1101_init_reum();
						
					}
					
					 //定时一段时间进行录波计算，获得线上的电流和电场值
        
        if(my_Time_Cyc_exit_Status==0 && my_tim6_count%17==0 && my_tim6_count!=0 && my_tim6_count%my_cyc_time_count!=0 && my_CC1101_all_step==0) //
        {
         
            my_dianliu_exit_add=my_wave_write_add; //当前录波地址
            my_Time_Cyc_exit_add = my_wave_write_add;								
						my_ADC_Count++;
            xResult=	xEventGroupSetBitsFromISR(xCreatedEventGroup2, 0X02,&xHigherPriorityTaskWoken);
            if(xResult!=pdFAIL)
            {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }																	
        }
				
				
				//LED灯闪烁控制--有报警状态====================
				//my_LED_ON_Count_gate=60*10;  //灯闪烁时间的阀值，单位秒
				
				if(((my_Fault_Current_End_Status & 0X01)==1 || (my_Fault_E_Fild_End_Status & 0X01)==1) && my_LED_ON_Count<my_LED_ON_Count_gate)
				{
					LED2_TOGGLE;
					LED4_TOGGLE;
					my_LED_ON_Count++;
					
				}
				else if((my_Fault_Current_End_Status & 0X01)==0 && (my_Fault_E_Fild_End_Status & 0X01)==0  && my_LED_ON_Count>0) //线路恢复正常
				{
					LED2_OFF;
					LED4_OFF;
					my_LED_ON_Count=0;
					
//				fun_wave2_to_wave3();     
//        printf("==time6 return normal--1 A=%d  E=%d\n",my_Fault_Current_End_Status,my_Fault_E_Fild_End_Status);
//				my_Fault_Current_End_Status = 00;
//        my_Fault_E_Fild_End_Status = 00;
//        my_zsq_ALarm_send_status = 1;
//        uint16_t my_step = 0x0002; //发送报警，消息任务
//        //xQueueSend(myQueue01Handle, &my_step, 100);
//				my_fun_give_Queue(&myQueue01Handle, my_step); //发送报警
					
					
				}
				else if(my_LED_ON_Count>=my_LED_ON_Count_gate) //报警时间已过
				{
					LED2_OFF;
					LED4_OFF;
					my_Fault_Current_End_Status=0;
					my_Fault_E_Fild_End_Status=0;
					my_LED_ON_Count=0;
					
				}
				
				//CC1101发送控制，如果my_CC1101_RSSI信号强度低于-70就关闭
				if(my_tim6_count%(1*61)==0)   //1分钟检测一次
				{
					if(my_CC1101_RSSI<-70)
					{
						my_CC1101_RSSI_count++;//信号过弱				
					}
					else
					{
						my_CC1101_RSSI_count=0;  //信号正常
					}
					
					//==================
					
					if(my_CC1101_RSSI_count>60 && my_CC1101_RSSI_count<=75)
					{      //短时间内开启一段时间
						    
								if(my_CC1101_RSSI_count==61)
								{
									CC1101_PWR_ON; //CC1101上电    
									CC1101Init();
	
								}
						
					}
					else if(my_CC1101_RSSI_count>75)
					{
						CC1101_PWR_OFF;//CC1101断电
						my_CC1101_RSSI_count=1;      //继续关闭
					}
					else if(my_CC1101_RSSI_count>10 && my_CC1101_RSSI_count<59)
					{
						CC1101_PWR_OFF;//CC1101断电
					}
					
				
				
				//=======电池直流电源的控制====CC1101，如果电压过低停止CC1101============
					if(ADC1_Filer_value_buf[6] <3.6)  //停止CC1101
					{
						CC1101_PWR_OFF;//CC1101断电
						
					}
					else if (ADC1_Filer_value_buf[6] <3.8) //开启CC1101
					{
						
						CC1101_PWR_ON; //CC1101上电 
					}
					
					
					
			}
			
					
					
				
				
					

    }


}





