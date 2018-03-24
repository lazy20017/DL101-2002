#include "gpio.h"
#include "my_usart.h"
//#include "my_cc_TX_RX.h"
#include "my_OS.h"

//extern uint32_t ADC_ConvertedValue[N][M];//用于保存采集的值,M个通道,N次
uint16_t my_pb1_count=0;
uint16_t my_pb0_count=0;
uint16_t my_pc13_count=0;
extern uint8_t Cmd_CC1101_RX[];
extern uint8_t my_CC1101_chip_address;  //芯片地址，默认为0x05
extern uint8_t MY_TX_RX_STEP;   //命令的步骤
extern EventGroupHandle_t xCreatedEventGroup ;

uint8_t my_get_count=0; //获得的字符数量
void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin)
{

    uint8_t temp_status=0;
    //--------   CC1101中断 被动接收数据--
    BaseType_t xResult;
    BaseType_t xHigherPriorityTaskWoken=pdFAIL;

    if(GPIO_Pin==PIN_CC_IRQ)
    {

        uint32_t my_temp32=0;
        while (CC_IRQ_READ() == 1)
        {
            my_temp32++;
            if(my_temp32>=0x003FFFFF)
            {
                printf("\n*****EXIT CC_IRQ_READ() == 1 ******\n");
                CC1101ClrRXBuff( );
                my_fun_CC1101_init_reum();
                return;
            }

        }



        // printf("before rx_count=%d\n",	my_get_count);//测试使用，接收到的字符数量
        temp_status=CC1101GetRXCnt();
        my_get_count=temp_status;
        if(temp_status>0)
        {
            //发送状态标识0X08，到状态标识组中
            xResult=	xEventGroupSetBitsFromISR(xCreatedEventGroup, 0X08,&xHigherPriorityTaskWoken);
            if(xResult!=pdFAIL)
            {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            my_pb0_count++;

        }
        else
        {
            my_pb0_count++;

        }
        //printf("CC_IRQ=%d -- rx_count=%d\r\n",my_pb0_count,temp_status);//显示接收到的数据量


    }


    if(GPIO_Pin==GPIO_PIN_1)
    {
        my_pb1_count++;
        printf("PC7 INT=%d-\n",my_pb1_count);

    }


}
