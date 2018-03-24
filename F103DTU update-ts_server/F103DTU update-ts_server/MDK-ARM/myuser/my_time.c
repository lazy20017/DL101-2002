#include "gpio.h"

#include "my_time.h"
#include "bsp_led.h"
#include "my_def_value.h"
#include "my_extrn_value.h"
#include "my_usart.h"
#include "my_OS.h"





uint16_t my_tim6_count = 0;
uint16_t my_tim6_old_count = 0;
extern uint16_t my_os_count1;
extern osMessageQId myQueue01Handle;
extern uint16_t  my_CC1101_all_step;
extern 
void HAL_TIM_PeriodElapsedCallback2(TIM_HandleTypeDef *htim)
{


    my_tim6_count++; //1秒钟的计时器
    if(my_tim6_count % 17 == 0 && my_tim6_count != 0)
    {
        my_tim6_old_count = my_os_count1;
    }
    else if(my_tim6_count % 37 == 0 && my_tim6_count != 0 && my_tim6_count % 7 != 0)
    {
        if(my_tim6_old_count == my_os_count1)
        {
            printf("\n  OS is stop \n");
        }

    }

    //GPRS 周期发送
		MY_ACT_CYC_DTU=487;//487; //@@@@快速测试使用
    if(my_tim6_count % (MY_ACT_CYC_DTU+my_act_cyc_dtu_delay) == 0 &&my_tim6_count != 0 )			
    {  if( my_GPRS_all_step == 0 && my_gprs_TX_status == 0 && my_system_restart_status==0 )
			{
			  if((my_CC1101_all_step>=0X5000 && my_CC1101_all_step<=0X5400) ||
							(my_CC1101_all_step>=0X0050 && my_CC1101_all_step<=0X0054)  ||
						   my_CC1101_all_step==0X0002 || my_CC1101_all_step==0X0200
						)
				return;
				my_act_cyc_dtu_delay=0;
				printf("====GPRS CYC time START=%d\r\n", my_os_count1);
        my_gprs_TX_status = 1;
        my_fun_give_Queue(&myQueue01Handle, 0XB100);
			}
			else
			{
				my_act_cyc_dtu_delay=30;
			}
        
    }
    //GPRS主动发送数据，心跳服务器，发送到01号队列，对应04号任务
		MY_ACT_Heart_DTU=187;//187;//@@@快速测试
    if(my_tim6_count % (MY_ACT_Heart_DTU) == 0 && my_tim6_count != 0  && my_GPRS_all_step == 0 && my_gprs_TX_status == 0 && my_system_restart_status==0)
    {
			 if((my_CC1101_all_step>=0X5000 && my_CC1101_all_step<=0X5400) ||
							(my_CC1101_all_step>=0X0050 && my_CC1101_all_step<=0X0054)  ||
						   my_CC1101_all_step==0X0002 || my_CC1101_all_step==0X0200
						)
				return;
			  
				printf("====GPRS Heart time =%d\r\n", my_os_count1);
        my_gprs_TX_status = 1;
        my_fun_give_Queue(&myQueue01Handle, 0X1F00);
       
    }

    LED6_TOGGLE; //表示TIM6硬件定时器活着，1秒1次
}


//********WDZ 开定时，开始计时tme时间




