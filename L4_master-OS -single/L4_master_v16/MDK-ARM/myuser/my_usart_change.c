#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "event_groups.h"

#include "my_usart_change.h"
#include "my_extern_val.h"
#include "my_gloabal_val.h"

/*
串口接收数据，存入到缓冲区中
*/
extern EventGroupHandle_t xCreatedEventGroup;
uint8_t my_UART2_Status=0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    uint8_t ret = HAL_OK;
    uint16_t KK=0;
    uint8_t temp=0;
    BaseType_t xResult;
    BaseType_t xHigherPriorityTaskWoken=pdFAIL;

    if(UartHandle==&huart2)
    {


        temp=rsbuf2[rsbuf2pt_write];

        rsbuf2pt_write++;
        if(rsbuf2pt_write>=rsbuf2_max)
            rsbuf2pt_write=0;
        __HAL_UART_CLEAR_OREFLAG(&huart2);

        if(temp==0X16)
        {
            xResult=	xEventGroupSetBitsFromISR(xCreatedEventGroup, 0X02,&xHigherPriorityTaskWoken);
            if(xResult!=pdFAIL)
            {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }

        do
        {   KK++;
            if(KK>=0xFFF)
            {
                my_UART2_Status=1;
                break;
            }
            ret = 	HAL_UART_Receive_IT(&huart2,&rsbuf2[rsbuf2pt_write],1);  //开启接收USART4函数，等待接收下一个字符
        } while(ret != HAL_OK);

    }

}
