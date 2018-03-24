#include "stdio.h"
#include "my_usart.h"
#include "my_cc1101.h"


extern INT8U   COM_TxNeed;
extern INT8U   COM_TimeOut;
extern INT8U   COM_RxCounter;
extern INT8U   COM_TxCounter;
extern INT8U   COM_RxBuffer[];
extern INT8U   COM_TxBuffer[];



/*
利用串口2接收数据，并把此数据利用CC1101发送
*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    uint8_t ret = HAL_OK;


    COM_RxBuffer[COM_RxCounter++]=my_usart2_re_buf[my_usart2_re_count];  //进入中断就已经接收到了一个字符，保存此字符
    if (COM_RxCounter > 64)     {
        COM_RxCounter = 0;    //判断是否越界，RX接收区域
    }
    COM_TimeOut = 5;  //同时开始计时，等待接收完成，TIM3定时器，单位为1ms，5个单位为5毫秒



    my_usart2_re_count++;    //
    if(my_usart2_re_count>=200) my_usart2_re_count=0; //接收缓冲区控制

    do
    {
        ret = 	HAL_UART_Receive_IT(&huart2,&my_usart2_re_buf[my_usart2_re_count],1);  //开启接收USART2函数，等待接收下一个字符
    } while(ret != HAL_OK);




}