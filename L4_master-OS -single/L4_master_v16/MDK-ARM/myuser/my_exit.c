#include "gpio.h"
#include "my_usart.h"
#include "my_rtc.h"
#include "my_adc.h"
#include "my_time.h"
#include "my_wave_rec.h"
#include "my_gloabal_val.h"
//#include "my_led.h"
#include "my_cc1101.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "my_extern_val.h"

extern EventGroupHandle_t xCreatedEventGroup;
extern EventGroupHandle_t xCreatedEventGroup2;

extern uint16_t ADC2_GetValue[ADC2_ROW][ADC2_COLM];//用于保存采集的值,M个通道,N次
extern uint16_t my_wave_record[2][WAVE_number];
extern uint16_t my_wave_write_add;  //全波录波一级缓存

extern uint16_t my_PA01_count;
extern uint16_t my_PA00_count;
extern uint16_t my_PC06_count;

extern uint16_t my_dianliu_exit_add; //此变量，用来计算中断或者定时产生时刻，查询录波1级缓冲区所用的地址
extern uint16_t my_Wave_It_add;  //记录中断时刻地址，共同步录波使用

extern uint8_t  my_Current_exit_Status; //表示电流产生中断
//extern uint16_t my_Current_Exit_add;

extern uint8_t  my_E_Field_exit_Status; //表示电场产生中断
extern uint16_t my_E_Field_exit_add;

extern uint8_t  my_Time_Cyc_exit_Status; //表示周期产生中断



uint8_t my_get_count=0; //获得的字符数量

void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin)
{
    //int xi=0;
    //int xj=0;
    //PA1=电流短路中断，PA2=接地中断
    uint8_t temp_status=0;
    BaseType_t xResult;
    BaseType_t xHigherPriorityTaskWoken=pdFAIL;
    int my_add1=0;


    if(GPIO_Pin==EXIT_dianliu_Pin)
    {
#if ADC_interrupt_data==1
        my_PA00_count++; //记录中断次数
			  if(my_sys_start_status==1) return;
      
        if(my_Current_exit_Status==0) //如果已经进入过1次中断，在处理完成之前，防止重复进入
        {
            my_Current_exit_Status=1;  //电流中断
            my_dianliu_exit_add=my_wave_write_add; //当前录波地址
						my_E_Field_exit_add=my_wave_write_add;
            //计算故障点

            my_add1=my_dianliu_exit_add-40;
            if(my_add1<0)
            {
                my_add1=WAVE_number+my_add1;
            }
            my_dianliu_exit_add=my_add1;
						my_dianliu_exit_add=my_add1;


            printf("\n ==电流中断==exit PA0,I_interrup=%d====\n",my_PA00_count);
						printf("dianliu_add=%d,E_fild_add=%d",my_dianliu_exit_add,my_E_Field_exit_add);

            //关闭电场中断					
						HAL_NVIC_DisableIRQ(EXIT_jiedi_EXTI_IRQn);
						HAL_NVIC_DisableIRQ(EXIT_dianliu_EXTI_IRQn);
						
						
            //发送状态标识0X01，到状态标识组中,PA1
            xResult=	xEventGroupSetBitsFromISR(xCreatedEventGroup2, 0X01,&xHigherPriorityTaskWoken);
            if(xResult!=pdFAIL)
            {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }



        }
#endif
        //__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1); //这个可以清除外部中断，每个中断的清除函数都不一样，需要分别调用


    }

    else if(GPIO_Pin==EXIT_jiedi_Pin)
    {

        //=============
#if ADC_interrupt_data==1
        my_PA01_count++; //记录中断次数
			  if(my_sys_start_status==1) return;
			  printf("==enter E_Interrupt==%d\n",my_PA01_count);
				printf("==efild exit==CC1101_all_step=[%XH],A_EXIT_status=%d,E_exit_status=%d,cyc_exit_status=%d\n",
			  my_CC1101_all_step,my_E_Field_exit_Status,my_Current_exit_Status,my_Time_Cyc_exit_Status);
			 
        if(my_E_Field_exit_Status==0) //如果已经进入过1次中断，在处理完成之前，防止重复进入
        {
            my_E_Field_exit_Status=1;  //电流中断
            my_E_Field_exit_add=my_wave_write_add;
            my_dianliu_exit_add=my_wave_write_add; //当前录波地址
            //计算故障点

            //my_add1=my_dianliu_exit_add-40;
						my_add1=my_dianliu_exit_add-40-320;
            if(my_add1<0)
            {
                my_add1=WAVE_number+my_add1;
            }
            my_dianliu_exit_add=my_add1;
            my_E_Field_exit_add=my_dianliu_exit_add;

            printf("\n ==电场中断==exit PA1,E_interrup=%d====\n",my_PA01_count);
						printf("dianliu_add=%d,E_fild_add=%d",my_dianliu_exit_add,my_E_Field_exit_add);
						
						 //关闭电场中断				
						HAL_NVIC_DisableIRQ(EXIT_jiedi_EXTI_IRQn);
						HAL_NVIC_DisableIRQ(EXIT_dianliu_EXTI_IRQn);

            //发送状态标识0X01，到状态标识组中,PA1
            xResult=	xEventGroupSetBitsFromISR(xCreatedEventGroup2, 0X01,&xHigherPriorityTaskWoken);
            if(xResult!=pdFAIL)
            {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }



        }
#endif




    }

    //CC1101

    //--------   CC1101中断 被动接收数据--


    else 	if(GPIO_Pin==PIN_CC_IRQ)  //PC6
    {   uint32_t my_temp32=0;
        while (CC_IRQ_READ() == 1)
				{
					my_temp32++;
					if(my_temp32>=0x003FFFFFF)
					{
						printf("\n*****EXIT CC_IRQ_READ() == 1 ******\n");
						CC1101ClrRXBuff( );
						return;
					}
					
				}
        // printf("before rx_count=%d\n",	my_get_count);//测试使用，接收到的字符数量
        temp_status=CC1101GetRXCnt();  //接收到的字符数量
        my_get_count=temp_status;
        if(temp_status>0 && temp_status<64 )
        {
            //发送状态标识0X08，到状态标识组中
            xResult=	xEventGroupSetBitsFromISR(xCreatedEventGroup, 0X08,&xHigherPriorityTaskWoken);
            if(xResult!=pdFAIL)
            {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            my_PC06_count++;

        }
        else
        {
            //my_pc06_count++;

        }
        //printf("CC_IRQ=%d -- rx_count=%d\r\n",my_pb0_count,temp_status);//显示接收到的数据量


    }

}
