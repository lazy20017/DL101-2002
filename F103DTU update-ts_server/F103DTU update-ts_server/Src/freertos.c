/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
//#include "my_led.h"
#include "bsp_led.h"
#include "wdz_m35.h"
#include "my_OS.h"
#include "bsp_iap.h"
#include "my_globle_extern.h"
#include "my_extrn_value.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;
osThreadId myTask07Handle;
osThreadId myTask08Handle;
osMessageQId myQueue01Handle;
osMessageQId myQueue02Handle;
osMessageQId myQueue03Handle;
osMessageQId myQueue04Handle;
osTimerId myTimer01Handle;
osMutexId myMutex01Handle;
osSemaphoreId myBinarySem01Handle;
osSemaphoreId myBinarySem02Handle;

/* USER CODE BEGIN Variables */

EventGroupHandle_t xCreatedEventGroup = NULL;
EventGroupHandle_t xCreatedEventGroup2 = NULL;

extern	uint8_t my_usart1_tx_buf1[10];
extern uint8_t my_UART1_Status;
extern uint8_t USART1_my_frame[256];
extern uint8_t USART3_my_frame[256];
extern uint8_t my_UART3_Status;



extern uint8_t my_CC1101_COM_Fram_buf[];
extern uint8_t my_CC1101_Frame_status;

extern uint16_t  my_CC1101_all_step;
extern uint16_t my_GPRS_all_step;

extern  TIM_HandleTypeDef htim6;
extern uint8_t GPRS_Heartdata_error_count;
extern uint16_t my_CC1101_re_buf_pt_write ;
extern uint16_t my_CC1101_re_buf_pt_read;
extern uint8_t NET_Server_status;
extern uint16_t MY_Bat_value;  //2016-06-12  修改 读取电池电压值
extern uint16_t MY_Sun_value;  //2016-08-30  修改 读取SUN电压值
extern uint16_t MY_Temperature_value; //2016-08-30
extern uint16_t MY_Humidity_value; //2016-08-30
extern uint8_t my_system_restart_status;
extern uint8_t my_query_index;
extern uint8_t GPRS_Status;
extern struct indicator_alarm_class my_indicator_alarm_data[];
extern struct indicator_record_class my_indicator_record_data[];
extern uint8_t my_indicator_tx_index;
extern uint16_t my_gprs_count_time;

extern struct indicator_class my_indicator_data[];



uint16_t my_os_count1 = 0;
uint8_t my_cc_count = 0;
uint8_t my_gprs_TX_status = 0; //1表示gprs正在进行发送环节，0表示结束
uint16_t my_cc1101_tx_wait_time = 2000;
uint8_t my_cc1101_record_Efild_status = 1; //录波数据，发送的第2组数据为电场数据
uint8_t my_CC1101_ERROR_count = 0; //CC1101故障记录数
uint16_t my_CC1101_ERROR_OLD = 0;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);
void StartTask06(void const * argument);
void StartTask07(void const * argument);
void StartTask08(void const * argument);
void Callback01(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Create the mutex(es) */
    /* definition and creation of myMutex01 */
    osMutexDef(myMutex01);
    myMutex01Handle = osMutexCreate(osMutex(myMutex01));

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* definition and creation of myBinarySem01 */
    osSemaphoreDef(myBinarySem01);
    myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

    /* definition and creation of myBinarySem02 */
    osSemaphoreDef(myBinarySem02);
    myBinarySem02Handle = osSemaphoreCreate(osSemaphore(myBinarySem02), 1);

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* Create the timer(s) */
    /* definition and creation of myTimer01 */
    osTimerDef(myTimer01, Callback01);
    myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    osTimerStart(myTimer01Handle, 1000);

    /* USER CODE END RTOS_TIMERS */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* definition and creation of myTask02 */
    osThreadDef(myTask02, StartTask02, osPriorityBelowNormal, 0, 512);
    myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

    /* definition and creation of myTask03 */
    osThreadDef(myTask03, StartTask03, osPriorityNormal, 0, 512);
    myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

    /* definition and creation of myTask04 */
    osThreadDef(myTask04, StartTask04, osPriorityNormal, 0, 512);
    myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

    /* definition and creation of myTask05 */
    osThreadDef(myTask05, StartTask05, osPriorityNormal, 0, 512);
    myTask05Handle = osThreadCreate(osThread(myTask05), NULL);

    /* definition and creation of myTask06 */
    osThreadDef(myTask06, StartTask06, osPriorityNormal, 0, 256);
    myTask06Handle = osThreadCreate(osThread(myTask06), NULL);

    /* definition and creation of myTask07 */
    osThreadDef(myTask07, StartTask07, osPriorityNormal, 0, 256);
    myTask07Handle = osThreadCreate(osThread(myTask07), NULL);

    /* definition and creation of myTask08 */
    osThreadDef(myTask08, StartTask08, osPriorityRealtime, 0, 256);
    myTask08Handle = osThreadCreate(osThread(myTask08), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* Create the queue(s) */
    /* definition and creation of myQueue01 */
    osMessageQDef(myQueue01, 3, uint16_t);
    myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

    /* definition and creation of myQueue02 */
    osMessageQDef(myQueue02, 3, uint16_t);
    myQueue02Handle = osMessageCreate(osMessageQ(myQueue02), NULL);

    /* definition and creation of myQueue03 */
    osMessageQDef(myQueue03, 3, uint16_t);
    myQueue03Handle = osMessageCreate(osMessageQ(myQueue03), NULL);

    /* definition and creation of myQueue04 */
    osMessageQDef(myQueue04, 3, uint16_t);
    myQueue04Handle = osMessageCreate(osMessageQ(myQueue04), NULL);

    /* USER CODE BEGIN RTOS_QUEUES */


    /* add queues, ... */

    xCreatedEventGroup = xEventGroupCreate();
    xCreatedEventGroup2 = xEventGroupCreate();

    //RTOS初始化完成后，开启各种中断
    HAL_TIM_Base_Start_IT(&htim6); //定时中断，用来进行堵塞方式中，3秒的延时
#if Use_CC1101_receive_interrupt_status==1
    HAL_NVIC_EnableIRQ(EXIT_CC_IRQ_GD0); //开启CC1101中断，接收/发送数据产生
#endif
    /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}

/* StartTask02 function */
void StartTask02(void const * argument)
{
    /* USER CODE BEGIN StartTask02 */



    /* Infinite loop */
    for(;;)
    {
        if(my_CC1101_all_step == 0 && my_GPRS_all_step == 0)
        {
            my_fun_CC1101_init_resume();
            my_fun_usart_init_resume();
#if Use_CC1101_receive_interrupt_status==1
            HAL_NVIC_EnableIRQ(EXIT_CC_IRQ_GD0); //开启CC1101中断，接收/发送数据产生
#endif

            osDelay(10000);
        }
    }
    /* USER CODE END StartTask02 */
}

/* StartTask03 function */
void StartTask03(void const * argument)
{
    /* USER CODE BEGIN StartTask03 */


    //===========GRPS   接收协议解析
    extern uint8_t USART1_FRAME_status;
    extern uint8_t USART3_FRAME_status;
    uint8_t my_status = 0;
    uint8_t temp8 = 0, temp8_ti = 0, temp8_cot = 0, temp8_QOI = 0;
    uint16_t temp16_inf_add = 0;
    uint16_t my_step = 0;
    /* Infinite loop */
    for(;;)
    {
        if(my_UART1_Status == 0X01)
        {
            MX_USART1_UART_Init();
            my_UART1_Status = 0;
            HAL_UART_Receive_IT(&huart1, &rsbuf3[rsbuf1pt_write], 1); //开启接收USART1函数
        }


        EventBits_t	uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* 事件标志组句柄 */
                             0x01,            /* 等待bit0和bit1被设置 */
                             pdTRUE,             /* 退出前bit0和bit1被清除，这里是bit0和bit1都被设置才表示“退出”*/
                             pdTRUE,             /* 设置为pdTRUE表示等待bit1和bit0都被设置*/
                             3000); 	 /* 等待延迟时间 */

        if((uxBits & 0x01) == 0x01)
        {
            my_status = 1; //通过标志组，获得串口接收到数据的标志
            //printf("Usart1 receiv  data 0X16h !\r\n");
            LED4_TOGGLE;
        }
        else
            my_status = 0;


        if(my_status == 1)
        {

            //my_usart_GPRS_101frame(3);  //协议解析，获得一帧数据
            my_101frame_analyse(1, 0, my_GPRS_CRC_check); //第2个参数，0为单地址
            if(USART1_FRAME_status > 0) //分析出来数据
            {
                my_fun_display_fram_16(1, 8); //测试使用，显示接收到的数据
                USART1_FRAME_status = 0;

                if(USART1_my_frame[0] == 0x10)
                {
                    my_status = 0x10;
                    temp8 = USART1_my_frame[1]; //控制域功能码
                }
                else if (USART1_my_frame[0] == 0x68)
                {
                    my_status = 0x68;
                    temp8 = USART1_my_frame[4]; //控制域功能码
                    temp8_ti = USART1_my_frame[7]; //帧类型 ,[8]长度
                    temp8_cot = USART1_my_frame[9]; //传输原因,为2个字节，9和10,后面的11和12位域地址
                    //temp16_inf_add = USART1_my_frame[13]; //信息体高字节
                    //temp16_inf_add = (temp16_inf_add << 8) + USART1_my_frame[12]; //信息体低字节
                    temp16_inf_add = USART1_my_frame[14]; //信息体高字节
                    temp16_inf_add = (temp16_inf_add << 8) + USART1_my_frame[13]; //信息体低字节
                    temp8_QOI = USART1_my_frame[15];

                }
                else
                {
                    my_status = 0;
                    temp8 = 0;
                }

            }
            //===
            if(temp8 != 0x00)
            {
                printf("GPRS now_step1=[%XH], get_step= [%XH]\r\n", my_GPRS_all_step, temp8); //@@@@当前接收到的状态

            }
            //====开始对话过程=====
            if( temp8 == 0x20) //周期
            {
                my_step = 0X2000;
                xQueueSend(myQueue02Handle, &my_step, 100);		 //队列2对应gprs接收队列	,0X20为OK帧
            }
            else if( temp8 == 0x10) //报警
            {
                my_step = 0X1000;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }


            //=======自定义标准101协议 20180324
            //A版程序
            //Server主动发起链路建立对话，TCP握手
            else if( (temp8 & 0X0F) == 0x09 && my_status == 0X10)
            {
                my_step = 0X0044;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0X0F) == 0x00 && my_status == 0X10  &&  my_GPRS_all_step == 0X4400)
            {
                my_step = 0X0045;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0X0F) == 0x0B && my_status == 0X10  &&  my_GPRS_all_step == 0X4500)
            {
                my_step = 0X0046;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0X0F) == 0x00 && my_status == 0X10 &&  my_GPRS_all_step == 0X4600)
            {
                my_step = 0X0047;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0X0F) == 0x00 && my_status == 0X10 &&  my_GPRS_all_step == 0X4700)
            {
                my_step = 0X0048;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }




            //=======TCP握手 E1  DTU主动发起
            else if( ((temp8 & 0X0F) == 0x0B || temp8 == 0x0B) && my_status == 0X10 &&  my_GPRS_all_step == 0xE100)
            {
                my_step = 0X00E1;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( ((temp8 & 0X0F) == 0x800 || temp8 == 0x00) &&  my_status == 0X10 &&  my_GPRS_all_step == 0XE200)
            {
                my_step = 0X00E2;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( ((temp8 & 0X0F) == 0x00) && my_status == 0X10 &&  my_GPRS_all_step == 0XE300)
            {
                my_step = 0X00E3;

                xQueueSend(myQueue02Handle, &my_step, 100);
            }

            //========================

            //总召周期数据C1
            else if((temp8 & 0x0F) == 0X03 && temp8_ti == 0X64 && temp8_cot == 0x06 && temp8_QOI == 20) //总召
            {
                my_step = 0X00C1;
                xQueueSend(myQueue02Handle, &my_step, 100);

            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XC100 ) //总召确认
            {
                my_step = 0X00C2;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XC200 ) //总召遥信 收到确认
            {
                my_step = 0X00C3;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XC300 ) //总召遥测4011 收到确认，指示器
            {
                my_step = 0X00C4;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XC400 ) //总召遥测4001 收到确认，DTU
            {
                my_step = 0X00C5;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XC500 ) //确认总召结束
            {
                my_step = 0X00C6;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }

            //时钟同步 D1
            else if((temp8 & 0x0F) == 0X03 && temp8_ti == 103 && temp8_cot == 0x06 )
            {
                my_step = 0X00D1;
                xQueueSend(myQueue02Handle, &my_step, 100);

            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XD100 )
            {
                my_step = 0X00D2;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }

            //服务器发起读取时间

            else if((temp8 & 0x0F) == 0X03 && temp8_ti == 103 && temp8_cot == 0x05 )
            {
                my_step = 0X00D3;
                xQueueSend(myQueue02Handle, &my_step, 100);

            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XD300 )
            {
                my_step = 0X00D4;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }

            //链路测试命令

            else if((temp8 & 0x0F) == 0X03 && temp8_ti == 104 && temp8_cot == 0x06 )
            {
                my_step = 0X00D5;
                xQueueSend(myQueue02Handle, &my_step, 100);

            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XD500 )
            {
                my_step = 0X00D6;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }


            //服务器发起心跳命令

            else if((temp8 & 0x0F) == 0X02 )
            {
                my_step = 0X00D7;
                xQueueSend(myQueue02Handle, &my_step, 100);

            }

            //周期遥测

            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XB100 )
            {
                my_step = 0X00B1;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0x0F) == 0X000 && my_GPRS_all_step == 0XB200 )
            {
                my_step = 0X00B2;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XB300 )
            {
                my_step = 0X00B3;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            //进程复位A1
            else if((temp8 & 0x0F) == 0X03 && temp8_ti == 105 && temp8_cot == 6)
            {
                my_step = 0X00A1;
                xQueueSend(myQueue02Handle, &my_step, 100);

            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XA100 )
            {
                my_step = 0X00A2;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }

            //报警
            else if( (temp8 & 0x0F) == 0X0 && my_GPRS_all_step == 0X9100 )
            {
                my_step = 0X0091;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }
            else if( (temp8 & 0x0F) == 0X0 && my_GPRS_all_step == 0X9200 )
            {
                my_step = 0X0092;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }


            //召唤目录
            else if( (temp8 & 0x0F) == 0X3 && temp8_ti == 122 && temp8_cot == 5 && temp8_QOI == 2)
            {
                my_step = 0X0051;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }
            else if( (temp8 & 0x0F) == 0X0 && my_GPRS_all_step == 0X5100 )
            {
                my_step = 0X0052;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }



            //读取文件激活

						 else if( (temp8 & 0x0F) == 0X03 && temp8_ti == 210 && temp8_cot == 6 && temp8_QOI == 3) //读文件激活
            {
                my_step = 0X0053;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }
            else if( (temp8 & 0x0F) == 0X0 && my_GPRS_all_step == 0X5300 )  //确认读文件激活
            {
                my_step = 0X0054;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }
						
						//读取文件，发送数据
						 else if( (temp8 & 0x0F) == 0X3 && temp8_ti == 210 && temp8_cot == 6 && temp8_QOI == 6 )  //确定收到文件段
            {
                my_step = 0X0056;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }











            //===========******************
            //C版程序
            //心跳包 1F
            else if( temp8 == 0x80 && my_GPRS_all_step == 0X1F00 )
            {
                my_step = 0X001F;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }

            //参数设定81
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X30 && temp8_cot == 0x06 && (temp16_inf_add >= 0x5001 && temp16_inf_add <= 0X5012))
            {
                my_step = 0X0081;
                xQueueSend(myQueue02Handle, &my_step, 100);

            }
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X64 && temp8_cot == 0x6E)
            {
                my_step = 0X0081;
                xQueueSend(myQueue02Handle, &my_step, 100);

            }

            //遥控命令61
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X2D && temp8_cot == 0x06)
            {
                my_step = 0X0061;
                xQueueSend(myQueue02Handle, &my_step, 100);

            }


            //查询指令71-1 dtu参数
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X30 && temp8_cot == 0x06 && temp16_inf_add == 0X5020 )
            {
                my_step = 0X0071;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( temp8 == 0x80 && my_GPRS_all_step == 0X7100 )
            {
                my_step = 0X0072;
                xQueueSend(myQueue02Handle, &my_step, 100); //中间重复过程
            }
            else if( temp8 == 0x80 && my_GPRS_all_step == 0X7200 )
            {
                my_step = 0X0073;
                xQueueSend(myQueue02Handle, &my_step, 100); //结束
            }
            //查询指令71-2 ZSQ参数
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X30 && temp8_cot == 0x06 && temp16_inf_add == 0X5021 )
            {

                my_query_index = (temp16_inf_add & 0x000F);
                my_step = 0X0077;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( temp8 == 0x80 && my_GPRS_all_step == 0X7700 )
            {
                my_step = 0X0078;
                xQueueSend(myQueue02Handle, &my_step, 100); //中间重复过程
            }
            else if( temp8 == 0x80 && my_GPRS_all_step == 0X7800 )
            {
                my_step = 0X0079;
                xQueueSend(myQueue02Handle, &my_step, 100); //结束
            }

            //GRPS信号强度41
            //
            else if( temp8 == 0x80 && my_GPRS_all_step == 0X4100 )
            {
                my_step = 0X0041;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }


            //查询计数同步值和RTC  31
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X09 && temp8_cot == 0x70 && temp16_inf_add == 0X5070 )
            {
                my_step = 0X0031;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }




            //总召录波数据51
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X64 && temp8_cot == 0x6D)
            {
                my_step = 0X0051;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }

            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X64 && temp8_cot == 0x73 && my_GPRS_all_step == 0x5100 )
            {
                my_step = 0X0052;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X64 && temp8_cot == 0x73 && my_GPRS_all_step == 0x5200)
            {
                my_step = 0X0053;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X64 && temp8_cot == 0x73 && my_GPRS_all_step == 0x5300)
            {
                my_step = 0X0054;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }

        }




        //osDelay(1);
        ///
    }
    /* USER CODE END StartTask03 */
}

/* StartTask04 function */
void StartTask04(void const * argument)
{
    /* USER CODE BEGIN StartTask04 */
    //===GPRS 发送
    xEventGroupSetBits(xCreatedEventGroup, 0x10);
    uint16_t my_step = 0;
    BaseType_t my_result = 0;

    /* Infinite loop */
    for(;;)
    {
        EventBits_t	uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* 事件标志组句柄 */
                             0x10,            /* 等待bit0和bit1被设置 */
                             pdTRUE,             /* 退出前bit0和bit1被清除，这里是bit0和bit1都被设置才表示“退出”*/
                             pdTRUE,             /* 设置为pdTRUE表示等待bit1和bit0都被设置*/
                             1); 	 /* 等待延迟时间 */

        if((uxBits & 0x10) == 0x10)
        {
            if(GPRS_Heartdata_error_count >= 5)
            {
                AT_M35_Reset();
            }

            else if(GPRS_Heartdata_error_count > 3)
            {
                my_init_m35();
                printf("===M35 init finish!===\r\n");
            }
        }

        //=========GPRS 发送====数据对话===


        my_result = xQueueReceive(myQueue01Handle, &my_step, 3000); //xQueuePeek,01队列，对应M35的发送队列
        if(my_result == pdPASS)
        {
            printf("GPRS T_QH01 = [%XH]\r\n", my_step);
            //LED5_TOGGLE;
        }
        else
        {
            my_step = 0X00;
            //printf("M35 not receive step = %d\r\n",my_step);

        }
        //A版程序
        //====GRPS 建立链路（1）--Server发起===========
        my_fun_gprs_time_dialog_tx(my_step, 0X0044, 0x4400, 0, my_fun_GPRS_TX_start1_server);
        my_fun_gprs_time_dialog_tx(my_step, 0X0045, 0x4500, 0, my_fun_GPRS_TX_start2_server);
        my_fun_gprs_time_dialog_tx(my_step, 0X0046, 0x4600, 0, my_fun_GPRS_TX_start3_server);
        my_fun_gprs_time_dialog_tx(my_step, 0X0047, 0x4700, 0, my_fun_GPRS_TX_start4_server);



        //====DTU-GPRS 建立连接---DTU发起
        my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0xE100, 0, my_fun_GPRS_TX_start1); //获得状态，前一状态，当前准备状态，结束状态，调用函数
        my_fun_gprs_time_dialog_tx(my_step, 0x00E1, 0xE200, 0, my_fun_GPRS_TX_start2);
        my_fun_gprs_time_dialog_tx(my_step, 0x00E2, 0xE300, 0, my_fun_GPRS_TX_start3);


        //=====总召
        my_fun_gprs_time_dialog_tx(my_step, 0X00C1, 0xC100, 0, my_fun_GPRS_TX_Call_0); //OK帧
        my_fun_gprs_time_dialog_tx(my_step, 0X00C2, 0xC200, 0, my_fun_GPRS_TX_CYC2); //遥信，无时标
        my_fun_gprs_time_dialog_tx(my_step, 0X00C3, 0xC300, 0, my_fun_GPRS_TX_CYC3); //遥测(4011,指示器数据)
        my_fun_gprs_time_dialog_tx(my_step, 0X00C4, 0xC400, 0, my_fun_GPRS_TX_CYC4); //遥测(4001,DTU数据)
        my_fun_gprs_time_dialog_tx(my_step, 0X00C5, 0xC500, 0, my_fun_GPRS_TX_CYC5);//总召结束


        //RTC时钟同步
        my_fun_gprs_time_dialog_tx(my_step, 0X00D1, 0xD100, 0, my_fun_GPRS_TX_RTC_data);

        //RTC时钟读取
        my_fun_gprs_time_dialog_tx(my_step, 0X00D3, 0xD300, 0, my_fun_GPRS_TX_RTC_data_read);

        //链路测试命令
        my_fun_gprs_time_dialog_tx(my_step, 0X00D5, 0xD500, 0, my_fun_GPRS_TX_test_data);

        //心跳命令 服务器发起
        my_fun_gprs_time_dialog_tx(my_step, 0X00D7, 0xD700, 1, my_fun_GPRS_TX_heart_toserver_data);

        //周期主动上传遥测信息
        my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0xB100, 0, my_fun_GPRS_TX_CYC2_B); //遥信，有时标
        my_fun_gprs_time_dialog_tx(my_step, 0X00B1, 0xB200, 0, my_fun_GPRS_TX_CYC3_B); //遥测(4011,指示器数据),无时标
        my_fun_gprs_time_dialog_tx(my_step, 0X00B2, 0xB300, 0, my_fun_GPRS_TX_CYC4_B); //遥测(4001,DTU数据)

        //复位进程
        my_fun_gprs_time_dialog_tx(my_step, 0X00A1, 0xA100, 0, my_fun_GPRS_TX_RESET); //

        //报警（遥信有时标，遥测无时标）
        //my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0x9100, 0, my_fun_GPRS_TX_ALarm_data_yaoxin); //遥信，有时标,分遥信，和遥测进行数据传输
        //my_fun_gprs_time_dialog_tx(my_step, 0X0091, 0x9200, 0, my_fun_GPRS_TX_ALarm_data_yaoce); //遥测，无时标my_fun_GPRS_TX_ALarm_data_yaoxin_yaoce
        my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0x9100, 0, my_fun_GPRS_TX_ALarm_data_yaoxin_yaoce); //遥信和遥测合并在一起发生，遥信有时标，遥测没有时标

        //召唤目录
				my_fun_gprs_time_dialog_tx(my_step, 0X0051, 0x5100, 0, my_fun_GPRS_TX_catalog); //


        //召唤文件
				my_fun_gprs_time_dialog_tx(my_step, 0X0053, 0x5300, 0, my_fun_GPRS_TX_file_data_1); //读文件激活、激活确认
				
							
				my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0x5500, 0, my_fun_GPRS_TX_file_data_2); //文件发送
				my_fun_gprs_time_dialog_tx(my_step, 0X0056, 0x5600, 1, my_fun_GPRS_TX_file_data_3);  //文件数据发送，确定。同时启动发送启动文件传输


        
				
				
				
				//C版程序
        //=====DTU==GPRS 主动发送心跳包---
        my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0x1F00, 0, my_fun_GPRS_TX_test1);
        //===DTU参数设置部分
        my_fun_gprs_time_dialog_tx(my_step, 0X0081, 0x8100, 1, my_fun_GPRS_TX_changeparameter); //
        //===DTU 控制指示器翻牌
        my_fun_gprs_time_dialog_tx(my_step, 0X0061, 0x6100, 1, my_fun_GPRS_TX_TurnLED); //
        //===DTU查询参数
        my_fun_gprs_time_dialog_tx(my_step, 0X0071, 0x7100, 0, my_fun_GPRS_TX_query_data);
        my_fun_gprs_time_dialog_tx(my_step, 0X0072, 0x7200, 0, my_fun_GPRS_TX_query_data);
        my_fun_gprs_time_dialog_tx(my_step, 0X0073, 0x7300, 1, my_fun_GPRS_TX_query_data);


        my_fun_gprs_time_dialog_tx(my_step, 0X0077, 0x7700, 0, my_fun_GPRS_TX_query_data2);
        my_fun_gprs_time_dialog_tx(my_step, 0X0078, 0x7800, 0, my_fun_GPRS_TX_query_data2);
        my_fun_gprs_time_dialog_tx(my_step, 0X0078, 0x7900, 1, my_fun_GPRS_TX_query_data2);

        //--发送信号强度 (DTU--指示器1/2/3，每个强度为一个自己0-99,0为最好
        my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0x4100, 0, my_fun_GPRS_TX_xinhaoqiangdu);
        //--SERVER查询DTU计数值和RTC时间
        my_fun_gprs_time_dialog_tx(my_step, 0X0031, 0x3100, 1, my_fun_GPRS_TX_TIME_RTC); //


        //总召录波
        //my_fun_gprs_time_dialog_tx(my_step, 0X0051, 0x5100, 0, my_fun_GPRS_TX_rec_data); //激活帧,文件第一段
       // my_fun_gprs_time_dialog_tx(my_step, 0X0052, 0x5200, 0, my_fun_GPRS_TX_rec_data); //
        //my_fun_gprs_time_dialog_tx(my_step, 0X0053, 0x5300, 0, my_fun_GPRS_TX_rec_data); //文件最后一段









        //osDelay(1);
    }
    /* USER CODE END StartTask04 */
}

/* StartTask05 function */
void StartTask05(void const * argument)
{
    /* USER CODE BEGIN StartTask05 */

    //====GPRS  接收
    //=========GPRS 接收数据对话过程
    BaseType_t my_result;
    uint16_t my_step;
    /* Infinite loop */
    for(;;)
    {

        //my_fun_take_group();
        //my_fun_take_BinarySem();
        //my_fun_take_Queue();

        //=========GPRS 接收数据对话过程
        my_result = xQueueReceive(myQueue02Handle, &my_step, 5000); //xQueuePeek,队列2对应，02队列，M35接收队列
        if(my_result == pdPASS)
        {
            printf("GPRS R_QH02 = [%XH]\r\n", my_step);
            //LED6_TOGGLE;
        }
        else
        {
            //printf("M35 not receive step = %d\r\n",my_step);
            my_step = 0X00;
        }

        //A版程序
        //==GPRS==链路建立 server发起
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0x0044, 0X4400, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x4400, 0x0045, 0X4500, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x4500, 0x0046, 0X4600, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x4600, 0x0047, 0X4700, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x4700, 0x0048, 0X0000, 1, my_fun_GPRS_RX_test1);



        //===GPRS start--tcp握手--DTU主动发起
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xE100, 0x00E1, 0XE200, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xE200, 0x00E2, 0XE300, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xE300, 0x00E3, 0X0000, 1, my_fun_GPRS_RX_test1);

        //=======周期总召
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X00C1, 0XC100, 0, my_fun_GPRS_RX_test1); //总召激活命令
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xC100, 0x00C2, 0XC200, 0, my_fun_GPRS_RX_test1); //总召确认激活OK
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0XC200, 0x00C3, 0XC300, 0, my_fun_GPRS_RX_test1); //遥信OK
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0XC300, 0x00C4, 0XC400, 0, my_fun_GPRS_RX_test1); //遥测4011开始的OK，指示器
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0XC400, 0x00C5, 0XC500, 0, my_fun_GPRS_RX_test1); //遥测4001开始的OK，DTU环境
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0XC500, 0x00C6, 0X0000, 1, my_fun_GPRS_RX_test1); //总召结束OK

        //====RTC时钟同步
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X00D1, 0XD100, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xD100, 0X00D2, 0X0000, 1, my_fun_GPRS_RX_test1);
        //====RTC时钟读取
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X00D3, 0XD300, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xD300, 0X00D4, 0X0000, 1, my_fun_GPRS_RX_test1);
        //====链路测试命令
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X00D5, 0XD500, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xD500, 0X00D6, 0X0000, 1, my_fun_GPRS_RX_test1);

        //=====心跳  服务器发起

        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X00D7, 0xD700, 0, my_fun_GPRS_RX_test1);

        //==周期遥测
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xB100, 0x00B1, 0XB200, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xB200, 0x00B2, 0XB300, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xB300, 0x00B3, 0XB400, 1, my_fun_GPRS_RX_test1);

        //DTU进程复位
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X00A1, 0XA100, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xA100, 0X00A2, 0X0000, 1, my_fun_GPRS_RX_test1);

        //故障报警（遥信+时标，遥测+时标）
        //my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9100, 0X0091, 0X9200, 0, my_fun_GPRS_RX_test1); //遥信有时标
        //my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9200, 0X0092, 0X0000, 1, my_fun_GPRS_RX_test1); //遥信有时标
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9100, 0X0091, 0X0000, 1, my_fun_GPRS_RX_test1); //遥信和遥测合并，一次完事


        //召唤目录
				my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0051, 0X5100, 0, my_fun_GPRS_RX_test1);
				my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x5100, 0X0052, 0X0000, 1, my_fun_GPRS_RX_test1);


        //召唤文件
				my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0053, 0X5300, 0, my_fun_GPRS_RX_test1); //读文件激活
				my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x5300, 0X0054, 0X0000, 1, my_fun_GPRS_RX_test1); //读文件激活确认，启动文件传输 把my_GPRS_all_step=0
				
				my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x5500, 0X0056, 0X5600, 0, my_fun_GPRS_RX_test1); //读文数据传输确定，开启下一次传输





        //C版程序
        //===计数值同步
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0x00E5, 0XE600, 0, my_fun_GPRS_RX_test1);
        //========GPRS心跳
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x1F00, 0X001F, 0X0000, 1, my_fun_GPRS_RX_test1); //心跳包接收到OK帧







        //DTU参数设置
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0081, 0X8100, 0, my_fun_GPRS_RX_change_parameter);

        //指示器翻牌
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0061, 0X6100, 0, my_fun_GPRS_RX_turn_led);

        //查询参数
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0071, 0X7100, 0, my_fun_GPRS_RX_query_data);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x7100, 0X0072, 0X7200, 0, my_fun_GPRS_RX_query_data);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x7200, 0X0073, 0X7300, 0, my_fun_GPRS_RX_query_data);

        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0077, 0X7700, 0, my_fun_GPRS_RX_query_data2);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x7700, 0X0078, 0X7800, 0, my_fun_GPRS_RX_query_data2);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x7800, 0X0079, 0X7A00, 0, my_fun_GPRS_RX_query_data2);
        //发送信号强度
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x4100, 0X0041, 0X0000, 1, my_fun_GPRS_RX_test1);
        //查询计数值和RTC
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0031, 0X3100, 0, my_fun_GPRS_RX_test1);

        //报警
//        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9100, 0X0091, 0X9200, 0, my_fun_GPRS_RX_test1); //遥信无时标
//        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9200, 0X0092, 0X9300, 0, my_fun_GPRS_RX_test1); //遥信有时标
//        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9300, 0X0093, 0X9400, 0, my_fun_GPRS_RX_test1); //遥测AC
//        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9400, 0X0094, 0X9500, 0, my_fun_GPRS_RX_test1); //遥测12T
//        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9500, 0X0095, 0X9600, 1, my_fun_GPRS_RX_test1); //计数及RTC
        //总召录波数据
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0051, 0X5100, 0, my_fun_GPRS_RX_Rec_data); //激活指令
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x5100, 0X0052, 0X5200, 0, my_fun_GPRS_RX_Rec_data);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x5200, 0X0053, 0X5300, 0, my_fun_GPRS_RX_Rec_data);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x5300, 0X0054, 0X5400, 1, my_fun_GPRS_RX_Rec_data);



        //osDelay(1);
    }
    /* USER CODE END StartTask05 */
}

/* StartTask06 function */
void StartTask06(void const * argument)
{
    /* USER CODE BEGIN StartTask06 */
    //=====CC1101发送数据对话
    extern	uint8_t  my_cc1101_tx_buf[];
    uint16_t my_step = 0;
    BaseType_t my_result = 0;
    extern uint8_t my_cc1101_dest_address;
    /* Infinite loop */
    for(;;)
    {


        //=====CC1101发送数据对话
        //my_result = xQueueReceive(myQueue03Handle, &my_step, my_cc1101_tx_wait_time); //xQueuePeek
        my_result = xQueueReceive(myQueue03Handle, &my_step, 10000); //xQueuePeek
        if(my_result == pdPASS)
        {
            printf("CC1101 T_QU03 IS send=[%XH]\r\n", my_step);
            LED2_TOGGLE;
        }
        else
        {
            my_step = 0X00;
            //printf("M35 not receive step = %d\r\n",my_step);

        }

        //====DTU 在线升级--发送部分
        my_fun_CC1101_time_dialog_tx3(my_step, 0xF000, 0xF100, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0xF200, 0xF300, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0xF400, 0xF500, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0xF600, 0xF700, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0xF800, 0xF900, 1, my_fun_CC1101_test1);
        //===========心跳回复OK
        my_fun_CC1101_time_dialog_tx3(my_step, 0xE000, 0x00E0, 1, my_fun_CC1101_TX_OK);
        //===参数设置
        my_fun_CC1101_time_dialog_tx3(my_step, 0xE100, 0x00E1, 0, my_fun_CC1101_TX_config_parmeter);

        //======CC1101 周期====
        my_fun_CC1101_time_dialog_tx3(my_step, 0x0100, 0x0001, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0x4000, 0x0040, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0x4100, 0x0041, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0x4200, 0x0042, 1, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0x4300, 0x0043, 1, my_fun_CC1101_test1);


        //======CC1101 报警
        my_fun_CC1101_time_dialog_tx3(my_step, 0x0200, 0x0002, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0x5000, 0x0050, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0x5100, 0x0051, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0x5200, 0x0052, 0, my_fun_CC1101_test1);
        if(my_cc1101_record_Efild_status == 0)
        {
            my_fun_CC1101_time_dialog_tx3(my_step, 0x5300, 0x0053, 1, my_fun_CC1101_test1);
        }
        else
        {
            my_fun_CC1101_time_dialog_tx3(my_step, 0x5300, 0x0053, 0, my_fun_CC1101_test1);
            my_fun_CC1101_time_dialog_tx3(my_step, 0x5400, 0x0054, 1, my_fun_CC1101_test1);
        }

        //osDelay(1);


    }
    /* USER CODE END StartTask06 */
}

/* StartTask07 function */
void StartTask07(void const * argument)
{
    /* USER CODE BEGIN StartTask07 */
    //=========CC1101接收数据对话过程
    BaseType_t my_result;
    uint16_t my_step;
    /* Infinite loop */
    for(;;)
    {
        //=========CC1101接收数据对话过程
        my_result = xQueueReceive(myQueue04Handle, &my_step, 2000); //xQueuePeek,队列2对应，M35接收队列
        if(my_result == pdPASS)
        {
            printf("CC1101 R_QU04 receive=[ %XH]\r\n", my_step);
            LED3_TOGGLE;
        }
        else
        {
            //printf("CC1101 not receive step = %d\r\n",my_step);
            my_step = 0X0000;
        }
        //====DTU在线升级--接收部分--

        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xF000, 0xF100, 0, my_fun_write_update_data_to_FLASH);
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xF200, 0xF300, 0, my_fun_write_update_data_to_FLASH);
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xF400, 0xF500, 0, my_fun_write_update_data_to_FLASH);
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xF600, 0xF700, 0, my_fun_write_update_data_to_FLASH);
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xF800, 0xF900, 0, my_fun_write_update_data_to_FLASH);

        //=====ZSQ发送心跳
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xE000, 0x00E0, 0, my_fun_dialog_CC1101_RX_0);
        //=====参数设置
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xE100, 0x00E1, 0, my_fun_dialog_CC1101_RX_0);
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xE200, 0x0000, 1, my_fun_dialog_CC1101_RX_0);

        //====CC1101 周期=====
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x0100, 0x0001, 0, my_fun_dialog_CC1101_RX_1);//遥信
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x4000, 0x0040, 0, my_fun_dialog_CC1101_RX_1);//遥测DC
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x4100, 0x0041, 0, my_fun_dialog_CC1101_RX_1);//遥测AC
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x4200, 0x0042, 0, my_fun_dialog_CC1101_RX_1);	//AC12T

        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x4300, 0x0043, 0, my_fun_dialog_CC1101_RX_1); //录波数据

        //=====CC1101报警==
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x0200, 0x0002, 0, my_fun_dialog_CC1101_RX_1); //遥信
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x5000, 0x0050, 0, my_fun_dialog_CC1101_RX_1);//遥测DC
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x5100, 0x0051, 0, my_fun_dialog_CC1101_RX_1);//遥测AC
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x5200, 0x0052, 0, my_fun_dialog_CC1101_RX_1);//AC12T
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x5300, 0x0053, 0, my_fun_dialog_CC1101_RX_1); //电流录波数据
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x5400, 0x0054, 0, my_fun_dialog_CC1101_RX_1);//电场



        //osDelay(1);
    }
    /* USER CODE END StartTask07 */
}

/* StartTask08 function */
void StartTask08(void const * argument)
{
    /* USER CODE BEGIN StartTask08 */
    //CC1101接收到数据的协议解析
    uint8_t my_status = 0;
    uint8_t temp8 = 0;
    uint16_t my_step = 0;

    /* Infinite loop */
    for(;;)
    {



        EventBits_t	uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* 事件标志组句柄 */
                             0x08,            /* 等待bit0和bit1被设置 */
                             pdTRUE,             /* 退出前bit0和bit1被清除，这里是bit0和bit1都被设置才表示“退出”*/
                             pdTRUE,             /* 设置为pdTRUE表示等待bit1和bit0都被设置*/
                             3000); 	 /* 等待延迟时间 */

        if((uxBits & 0x08) == 0x08)
        {
            my_status = 1; //通过标志组，获得接收到数据的标志
            //printf("cc1101 receive data !\r\n");
        }
        else
            my_status = 0;
        //把CC101接收到的数据移动到缓冲区中
        if(my_status == 1)
        {
            if(RF_RecvHandler_intrrupt_get_data_to_buf() > 0)
                my_status = 1;
            else
                my_status = 0;

        }

        //对缓冲区进行帧分析
        if(my_status == 1)
        {

            //协议解析，获得一帧数据
            my_101frame_analyse(4, 1, CC1101_CRC_check); //CC1101协议解析
            //============
            if(my_CC1101_Frame_status > 0) //分析出来数据
            {
#if Debug_uart_out_cc1101_rx_data_status==1
                my_fun_display_fram_16(4, 8); //测试使用，显示接收到的数据
#endif

                my_CC1101_Frame_status = 0;

                if(my_CC1101_COM_Fram_buf[0] == 0x10)
                {
                    my_status = 0x10;
                    temp8 = my_CC1101_COM_Fram_buf[1]; //功能码为0X20，代表OK帧
                }
                else if (my_CC1101_COM_Fram_buf[0] == 0x68)
                {
                    my_status = 0x68;
                    temp8 = my_CC1101_COM_Fram_buf[6]; //功能码为0X20，代表OK帧
                }
                else
                {
                    my_status = 0;
                    temp8 = 0;
                }
            }



            if(temp8 != 0x00)
            {
                printf("CC1101 now_step=[%XH], get_step= [%XH]\r\n", my_CC1101_all_step, temp8);

            }

            //===DTU  在线升级====
            if( temp8 == 0xF0 && my_GPRS_all_step == 0)
            {
                //my_CC1101_all_step=0XF000; //对话过程的开始状态
                my_step = 0XF000;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0xF2)
            {
                my_step = 0XF200;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0xF4)
            {
                my_CC1101_all_step = 0XF400;
                my_step = 0XF400;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0xF6)
            {
                my_step = 0XF600;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0xF8)
            {
                my_step = 0XF800;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            //=========CC1101心跳=====
            else if( temp8 == 0x1F && my_GPRS_all_step == 0) //指示器发送心跳帧
            {

                my_step = 0XE000;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }

            //===========CC1101周期====

            else if( temp8 == 0x01 && my_GPRS_all_step == 0)
            {
                my_step = 0X0100;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }

            else if( temp8 == 0x40)
            {
                my_step = 0X4000;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0x41)
            {
                my_step = 0X4100;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0x42)
            {
                my_step = 0X4200;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0x43 )
            {
                my_step = 0X4300;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0x44)
            {
                my_step = 0X4400;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }

            //=============CC1101报警========
            else if( temp8 == 0x02 && my_GPRS_all_step == 0)
            {
                my_step = 0X0200;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }

            else if( temp8 == 0x50)
            {
                my_step = 0X5000;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0x51)
            {
                my_step = 0X5100;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0x52)
            {
                my_step = 0X5200;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0x53)
            {
                my_step = 0X5300;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0x54)
            {
                my_step = 0X5400;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            //参数设置
            else if( temp8 == 0x2F)
            {
                my_step = 0XE100;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }
            else if( temp8 == 0x4F)
            {
                my_step = 0XE200;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }


            //===========
            temp8 = 0;
            my_CC1101_Frame_status = 0;
            my_status = 0;





        }

        //osDelay(1);
    }
    /* USER CODE END StartTask08 */
}

/* Callback01 function */
void Callback01(void const * argument)
{
    /* USER CODE BEGIN Callback01 */


    my_os_count1++;

    if(my_os_count1 % 5 == 0)
    {
        if(GPRS_Heartdata_error_count == 0) //GPRS状态标识，off为正常，on为断开
        {
            LED1_OFF;
        }
        else
        {
            LED1_ON;
        }
    }

    //#########################################
    if(my_os_count1 % (13) == 0 && my_os_count1 != 0 && my_GPRS_all_step == 0x00 && my_CC1101_all_step == 0x00)
    {   /*
        发送报警，通过检测到ZSQ的报警状态进行转发。
        */
        if(
            (my_indicator_alarm_data[0].TX_status_duanlu == 1 || my_indicator_alarm_data[0].TX_status_jiedi == 1 || my_indicator_alarm_data[0].TX_status_Line_STOP == 1)
            || ( my_indicator_alarm_data[1].TX_status_duanlu == 1 || my_indicator_alarm_data[1].TX_status_jiedi == 1 || my_indicator_alarm_data[0].TX_status_Line_STOP == 1)
            || (my_indicator_alarm_data[2].TX_status_duanlu == 1 || my_indicator_alarm_data[2].TX_status_jiedi == 1 || my_indicator_alarm_data[0].TX_status_Line_STOP == 1)
        )
        {
            if((my_CC1101_all_step >= 0X5000 && my_CC1101_all_step <= 0X5400) ||
                    (my_CC1101_all_step >= 0X0050 && my_CC1101_all_step <= 0X0054)  ||
                    my_CC1101_all_step == 0X0002 || my_CC1101_all_step == 0X0200
              )
                return ;
            printf("\n@@=GPRS ALarm time start=== my_os_count1=%d\n", my_os_count1);
            my_indicator_tx_index = 0;
            if(my_indicator_alarm_data[0].TX_status_duanlu == 1 || my_indicator_alarm_data[0].TX_status_jiedi == 1)
                my_indicator_tx_index = 0X01;
            else if(my_indicator_alarm_data[1].TX_status_duanlu == 1 || my_indicator_alarm_data[1].TX_status_jiedi == 1)
                my_indicator_tx_index = my_indicator_tx_index | 0x02;
            else if(my_indicator_alarm_data[2].TX_status_duanlu == 1 || my_indicator_alarm_data[2].TX_status_jiedi == 1)
                my_indicator_tx_index = my_indicator_tx_index | 0x04;
            else
                my_indicator_tx_index = 0;

            printf("GPRS ALarm report index=%d\n", my_indicator_tx_index);

            my_fun_give_Queue(&myQueue01Handle, 0X9100); // GPRS主动发送报警数据  @@@@测试使用


        }

    }
    //#############################
    if(my_os_count1 % (3667) == 0 && my_os_count1 != 0 && my_CC1101_all_step == 0 && my_GPRS_all_step == 0)
    {
        printf("\n====GPRS Radio PW== my_os_count1=%d\r\n", my_os_count1);
        //C版 程序,主动发送信号强度
        //my_fun_give_Queue(&myQueue01Handle, 0X4100); // GPRS主动发送信号强度，ZSQ信号强度1小时左右一次

    }
    if(my_os_count1 % (60) == 0 && my_os_count1 != 0)
    {
        my_fun_M35_resume_init();//M35重新初始化，发送标志信息号

        if( my_CC1101_ERROR_OLD == my_gprs_count_time)  //全局标志对比
        {
            my_CC1101_ERROR_count++;  //计数加一
            printf("not change my_gprs_count_time=%d,ERROR_count=%d\n", my_gprs_count_time, my_CC1101_ERROR_count);
        }
        else
        {
            my_CC1101_ERROR_OLD = my_gprs_count_time;
            my_CC1101_ERROR_count = 0;
        }
        if(my_CC1101_ERROR_count >= 35)  //35分钟左右
        {
            my_CC1101_ERROR_count = 0;
            HAL_NVIC_SystemReset();
        }

    }


    //#######################################################################3
    //#########################################################################
    //========GPRS================

    // GPRS主动发送 TCP握手指令 DTU主动发起
    if(NET_Server_status == 0  && GPRS_Status == 1 && my_os_count1 % 13 == 0 && my_CC1101_all_step == 0)
    {
        if(NET_Server_status == 1)
            return;

        my_gprs_TX_status = 1;                       //A 版，DTU主动发起链路建立命令，
        //my_fun_give_Queue(&myQueue01Handle, 0XE100); //计数13秒后开始进行 TCP链路建立20180324，定义一个全局变量进行链路状态标识
        printf("====GPRS TCP server Start=%d\r\n", my_os_count1);
        //NET_Server_status = 1;

    }

    //GPRS主动发送数据，周期数据，发送到01号队列，对应04号任务,系统重新启动后，发送一次数据
    if(my_system_restart_status == 1 && my_GPRS_all_step == 0 && my_gprs_TX_status == 0 && my_CC1101_all_step == 0 && my_os_count1 > 15)
    {
        printf("\n\n====Restart GPRS CYC time==my_os_count1=%d\r\n", my_os_count1);
        my_system_restart_status = 0;
        my_init_m35();
        my_gprs_TX_status = 1;
        //my_fun_give_Queue(&myQueue01Handle, 0XB100);@@@@@20180328  C版 重启初始化

    }

//    //M35重新初始化
//    if(my_os_count1 % (60) == 0 && my_os_count1 != 0)
//    {
//        my_fun_M35_resume_init();//M35重新初始化，发送标志信息号
//    }
    //==========GPRS end=============================


    //=====OS 堆栈区检测====
#if OS_heap_high_water_data==1
    if(my_os_count1 % (577) == 0 && my_os_count1 != 0)
    {
        //if(my_CC1101_all_step == 0 && my_GPRS_all_step == 0)
        {
            my_fun_task_heap_value();  //检测任务堆栈剩余量
        }

    }
#endif

    //=====OS 堆栈区检测==end==


    //====处理CC1101接收缓冲区，数据丢失，带来的追赶问题
    if(my_os_count1 % (2) == 0 && my_os_count1 != 0)
    {
        if(my_CC1101_re_buf_pt_write != my_CC1101_re_buf_pt_read)
        {
            my_cc_count++;
        }
        else
            my_cc_count = 0;
        if(my_cc_count > 10)
            my_CC1101_re_buf_pt_read = my_CC1101_re_buf_pt_write;

    }
    //=======
    //=====CC1101发送心跳包
    if(my_os_count1 % (5) == 0 && my_os_count1 != 0 && my_GPRS_all_step == 0)
    {

#if		Use_CC1101_send_heat_data_status==1
        my_fun_CC1101_send_heart_data();
#endif

    }
    //=====END==CC1101心跳包===


    //====从CPU STM32F0
    //======主CPU给从CPU发送指令,获得DTU的电压和温湿度
    //总召
    if(my_os_count1 % (45) == 0 && my_CC1101_all_step == 0 && my_GPRS_all_step == 0)
    {
        uint8_t my_txbuf[18] = TX101_calldata;
        uint8_t start_length = 14 + 6;
        uint8_t ii = 0;
        //发送总召命令
        USART_printf(&huart2, my_txbuf);
        osDelay(2000);
        //数据处理

        for(ii = 0; ii < 8; ii++)
        {
            MY_MCU_RsBuf[ii] = rsbuf2[start_length + ii];
        }
        my_buf1_to_buf2(MY_MCU_RsBuf, 0, MY_GPRS_MCU_RsBuf, 0, 8); //环境参数


        MY_Bat_value = MY_MCU_RsBuf[1];
        MY_Bat_value = (MY_Bat_value << 8) + MY_MCU_RsBuf[0];
        //2016-08-30 取太阳能，温度、湿度
        MY_Sun_value = MY_MCU_RsBuf[3];
        MY_Sun_value = (MY_Sun_value << 8) + MY_MCU_RsBuf[2];

        my_AD_value = (float)(MY_Bat_value * 3.3) / 4096 * 11;
        my_AD_SUN_value = (float)(MY_Sun_value * 3.3) / 4096 * 11;

        MY_Temperature_value = MY_MCU_RsBuf[5];
        MY_Temperature_value = (MY_Temperature_value << 8) + MY_MCU_RsBuf[4];

        MY_Humidity_value = MY_MCU_RsBuf[7];
        MY_Humidity_value = (MY_Humidity_value << 8) + MY_MCU_RsBuf[6];


        //环境参数进行处理，直接处理成实际值
#if Use_DTU_huanjing_jisuan==1
        MY_Bat_value = (float)(MY_Bat_value * 3.3) / 4096 * 11 * 10;
        MY_Sun_value = (float)(MY_Sun_value * 3.3) / 4096 * 11 * 10;
        MY_Temperature_value = ((float)(MY_Temperature_value) / 65536 * 165.00 - 40) * 10;
        MY_Humidity_value = ((float)(MY_Humidity_value) / 65536 * 100.00) * 10;



        MY_MCU_RsBuf[0] = MY_Bat_value;
        MY_MCU_RsBuf[1] = (MY_Bat_value >> 8);
        MY_MCU_RsBuf[2] = MY_Sun_value;
        MY_MCU_RsBuf[3] = (MY_Sun_value >> 8);
        MY_MCU_RsBuf[4] = MY_Temperature_value;
        MY_MCU_RsBuf[5] = (MY_Temperature_value >> 8);
        MY_MCU_RsBuf[6] = MY_Humidity_value;
        MY_MCU_RsBuf[7] = (MY_Humidity_value >> 8);

        my_buf1_to_buf2(MY_MCU_RsBuf, 0, MY_GPRS_MCU_RsBuf, 0, 8); //环境参数
#endif

        //回复OK帧
        my_UART4_printf(&huart2, TX101_OKdata);
        //USART_printf(&huart2,TX101_OKdata);
        rsbuf2pt_write = 0;
        //输出DTU的环境参数
        printf("DTU LIBAT=%.1f,SUNbat=%.1f,mytemperature=%.1f,myshidu=%.1f\n", MY_Bat_value / 10.0, MY_Sun_value / 10.0, MY_Temperature_value / 10.0, MY_Humidity_value / 10.0);

        //=========设定周期时间============
        if(my_AD_value >= MY_Speed_H_Gate && my_AD_value <= 13) //高速 周期10分钟，心跳5分钟
        {
            MY_ACT_Heart_DTU = MY_H_speed_heart;
            MY_ACT_CYC_DTU = MY_H_speed_cyc;

        }
        else if(my_AD_value >= MY_Speed_L_Gate && my_AD_value < MY_Speed_H_Gate) //中速，周期15分钟，心跳10分钟,系统默认时间，GPRS指令修改心跳和周期值为此值
        {
            MY_ACT_Heart_DTU = MY_M_speed_heart;
            MY_ACT_CYC_DTU = MY_M_speed_cyc;
        }
        else //低速，周期30分钟，心跳10分钟
        {
            MY_ACT_Heart_DTU = MY_L_speed_heart;
            MY_ACT_CYC_DTU = MY_L_speed_cyc;
        }


        //=============
    }
    //心跳
    else if(my_os_count1 % (57) == 0) //给小CPU
    {
        uint8_t my_txbuf[18] = TX101_heartdata;
        USART_printf(&huart2, my_txbuf);
        osDelay(2000);
        rsbuf2pt_write = 0;




    }
    if(my_os_count1 % 13 == 0 && my_CC1101_all_step == 0 && my_GPRS_all_step == 0) //RTC
    {
        if((my_CC1101_all_step >= 0X5000 && my_CC1101_all_step <= 0X5400) ||
                (my_CC1101_all_step >= 0X0050 && my_CC1101_all_step <= 0X0054)  ||
                my_CC1101_all_step == 0X0002 || my_CC1101_all_step == 0X0200
          )
            return;

        HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN); //读取RTC时间
        HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);
        printf("@@RTC= %d/%d/%d %d:%d:%d== RSSI=[%d]-[%d]-[%d]\n",
               my_RTC_date.Year, my_RTC_date.Month, my_RTC_date.Date, my_RTC_time.Hours, my_RTC_time.Minutes, my_RTC_time.Seconds,
               my_indicator_data[0].xinhao_db, my_indicator_data[1].xinhao_db, my_indicator_data[2].xinhao_db
              );

    }
    //====从CPU==END===
    //显示当前数据的值
    if(my_os_count1 % 69 == 0)
    {
        printf("==display my_os_count1=%d", my_os_count1);
        //my_fun_display_ZSQ_data();

    }

    /*
    模拟报警中断，短路和接地报警中断
    */
#if Use_gprs_ALarm_simulation_word==1
    if(my_os_count1 % 37 == 0)
    {
        my_indicator_tx_index = 2;
        if(my_indicator_alarm_data[my_indicator_tx_index].duanlu_data == 0X21)
        {
            my_indicator_alarm_data[my_indicator_tx_index].duanlu_data = 0X41;
            my_indicator_alarm_data[my_indicator_tx_index].jiedi_data = 0X51;
            my_indicator_alarm_data[my_indicator_tx_index].TX_status_jiedi = 1;
        }
        else
        {
            my_indicator_alarm_data[my_indicator_tx_index].duanlu_data = 0X21;
            my_indicator_alarm_data[my_indicator_tx_index].jiedi_data = 0X31;
            my_indicator_alarm_data[my_indicator_tx_index].TX_status_duanlu = 1;

        }
    }
#endif

    LED2_TOGGLE; //led2翻转表示，OS系统活着，1秒1次，软定时器
    /* USER CODE END Callback01 */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
