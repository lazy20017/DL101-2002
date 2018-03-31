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
extern uint16_t MY_Bat_value;  //2016-06-12  �޸� ��ȡ��ص�ѹֵ
extern uint16_t MY_Sun_value;  //2016-08-30  �޸� ��ȡSUN��ѹֵ
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
uint8_t my_gprs_TX_status = 0; //1��ʾgprs���ڽ��з��ͻ��ڣ�0��ʾ����
uint16_t my_cc1101_tx_wait_time = 2000;
uint8_t my_cc1101_record_Efild_status = 1; //¼�����ݣ����͵ĵ�2������Ϊ�糡����
uint8_t my_CC1101_ERROR_count = 0; //CC1101���ϼ�¼��
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

    //RTOS��ʼ����ɺ󣬿��������ж�
    HAL_TIM_Base_Start_IT(&htim6); //��ʱ�жϣ��������ж�����ʽ�У�3�����ʱ
#if Use_CC1101_receive_interrupt_status==1
    HAL_NVIC_EnableIRQ(EXIT_CC_IRQ_GD0); //����CC1101�жϣ�����/�������ݲ���
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
            HAL_NVIC_EnableIRQ(EXIT_CC_IRQ_GD0); //����CC1101�жϣ�����/�������ݲ���
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


    //===========GRPS   ����Э�����
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
            HAL_UART_Receive_IT(&huart1, &rsbuf3[rsbuf1pt_write], 1); //��������USART1����
        }


        EventBits_t	uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* �¼���־���� */
                             0x01,            /* �ȴ�bit0��bit1������ */
                             pdTRUE,             /* �˳�ǰbit0��bit1�������������bit0��bit1�������òű�ʾ���˳���*/
                             pdTRUE,             /* ����ΪpdTRUE��ʾ�ȴ�bit1��bit0��������*/
                             3000); 	 /* �ȴ��ӳ�ʱ�� */

        if((uxBits & 0x01) == 0x01)
        {
            my_status = 1; //ͨ����־�飬��ô��ڽ��յ����ݵı�־
            //printf("Usart1 receiv  data 0X16h !\r\n");
            LED4_TOGGLE;
        }
        else
            my_status = 0;


        if(my_status == 1)
        {

            //my_usart_GPRS_101frame(3);  //Э����������һ֡����
            my_101frame_analyse(1, 0, my_GPRS_CRC_check); //��2��������0Ϊ����ַ
            if(USART1_FRAME_status > 0) //������������
            {
                my_fun_display_fram_16(1, 8); //����ʹ�ã���ʾ���յ�������
                USART1_FRAME_status = 0;

                if(USART1_my_frame[0] == 0x10)
                {
                    my_status = 0x10;
                    temp8 = USART1_my_frame[1]; //����������
                }
                else if (USART1_my_frame[0] == 0x68)
                {
                    my_status = 0x68;
                    temp8 = USART1_my_frame[4]; //����������
                    temp8_ti = USART1_my_frame[7]; //֡���� ,[8]����
                    temp8_cot = USART1_my_frame[9]; //����ԭ��,Ϊ2���ֽڣ�9��10,�����11��12λ���ַ
                    //temp16_inf_add = USART1_my_frame[13]; //��Ϣ����ֽ�
                    //temp16_inf_add = (temp16_inf_add << 8) + USART1_my_frame[12]; //��Ϣ����ֽ�
                    temp16_inf_add = USART1_my_frame[14]; //��Ϣ����ֽ�
                    temp16_inf_add = (temp16_inf_add << 8) + USART1_my_frame[13]; //��Ϣ����ֽ�
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
                printf("GPRS now_step1=[%XH], get_step= [%XH]\r\n", my_GPRS_all_step, temp8); //@@@@��ǰ���յ���״̬

            }
            //====��ʼ�Ի�����=====
            if( temp8 == 0x20) //����
            {
                my_step = 0X2000;
                xQueueSend(myQueue02Handle, &my_step, 100);		 //����2��Ӧgprs���ն���	,0X20ΪOK֡
            }
            else if( temp8 == 0x10) //����
            {
                my_step = 0X1000;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }


            //=======�Զ����׼101Э�� 20180324
            //A�����
            //Server����������·�����Ի���TCP����
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




            //=======TCP���� E1  DTU��������
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

            //������������C1
            else if((temp8 & 0x0F) == 0X03 && temp8_ti == 0X64 && temp8_cot == 0x06 && temp8_QOI == 20) //����
            {
                my_step = 0X00C1;
                xQueueSend(myQueue02Handle, &my_step, 100);

            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XC100 ) //����ȷ��
            {
                my_step = 0X00C2;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XC200 ) //����ң�� �յ�ȷ��
            {
                my_step = 0X00C3;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XC300 ) //����ң��4011 �յ�ȷ�ϣ�ָʾ��
            {
                my_step = 0X00C4;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XC400 ) //����ң��4001 �յ�ȷ�ϣ�DTU
            {
                my_step = 0X00C5;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( (temp8 & 0x0F) == 0X00 && my_GPRS_all_step == 0XC500 ) //ȷ�����ٽ���
            {
                my_step = 0X00C6;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }

            //ʱ��ͬ�� D1
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

            //�����������ȡʱ��

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

            //��·��������

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


            //������������������

            else if((temp8 & 0x0F) == 0X02 )
            {
                my_step = 0X00D7;
                xQueueSend(myQueue02Handle, &my_step, 100);

            }

            //����ң��

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
            //���̸�λA1
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

            //����
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


            //�ٻ�Ŀ¼
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



            //��ȡ�ļ�����

						 else if( (temp8 & 0x0F) == 0X03 && temp8_ti == 210 && temp8_cot == 6 && temp8_QOI == 3) //���ļ�����
            {
                my_step = 0X0053;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }
            else if( (temp8 & 0x0F) == 0X0 && my_GPRS_all_step == 0X5300 )  //ȷ�϶��ļ�����
            {
                my_step = 0X0054;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }
						
						//��ȡ�ļ�����������
						 else if( (temp8 & 0x0F) == 0X3 && temp8_ti == 210 && temp8_cot == 6 && temp8_QOI == 6 )  //ȷ���յ��ļ���
            {
                my_step = 0X0056;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }











            //===========******************
            //C�����
            //������ 1F
            else if( temp8 == 0x80 && my_GPRS_all_step == 0X1F00 )
            {
                my_step = 0X001F;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }

            //�����趨81
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

            //ң������61
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X2D && temp8_cot == 0x06)
            {
                my_step = 0X0061;
                xQueueSend(myQueue02Handle, &my_step, 100);

            }


            //��ѯָ��71-1 dtu����
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X30 && temp8_cot == 0x06 && temp16_inf_add == 0X5020 )
            {
                my_step = 0X0071;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( temp8 == 0x80 && my_GPRS_all_step == 0X7100 )
            {
                my_step = 0X0072;
                xQueueSend(myQueue02Handle, &my_step, 100); //�м��ظ�����
            }
            else if( temp8 == 0x80 && my_GPRS_all_step == 0X7200 )
            {
                my_step = 0X0073;
                xQueueSend(myQueue02Handle, &my_step, 100); //����
            }
            //��ѯָ��71-2 ZSQ����
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X30 && temp8_cot == 0x06 && temp16_inf_add == 0X5021 )
            {

                my_query_index = (temp16_inf_add & 0x000F);
                my_step = 0X0077;
                xQueueSend(myQueue02Handle, &my_step, 100);
            }
            else if( temp8 == 0x80 && my_GPRS_all_step == 0X7700 )
            {
                my_step = 0X0078;
                xQueueSend(myQueue02Handle, &my_step, 100); //�м��ظ�����
            }
            else if( temp8 == 0x80 && my_GPRS_all_step == 0X7800 )
            {
                my_step = 0X0079;
                xQueueSend(myQueue02Handle, &my_step, 100); //����
            }

            //GRPS�ź�ǿ��41
            //
            else if( temp8 == 0x80 && my_GPRS_all_step == 0X4100 )
            {
                my_step = 0X0041;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }


            //��ѯ����ͬ��ֵ��RTC  31
            else if((temp8 == 0x73 || temp8 == 0x53) && temp8_ti == 0X09 && temp8_cot == 0x70 && temp16_inf_add == 0X5070 )
            {
                my_step = 0X0031;
                xQueueSend(myQueue02Handle, &my_step, 100); //
            }




            //����¼������51
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
    //===GPRS ����
    xEventGroupSetBits(xCreatedEventGroup, 0x10);
    uint16_t my_step = 0;
    BaseType_t my_result = 0;

    /* Infinite loop */
    for(;;)
    {
        EventBits_t	uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* �¼���־���� */
                             0x10,            /* �ȴ�bit0��bit1������ */
                             pdTRUE,             /* �˳�ǰbit0��bit1�������������bit0��bit1�������òű�ʾ���˳���*/
                             pdTRUE,             /* ����ΪpdTRUE��ʾ�ȴ�bit1��bit0��������*/
                             1); 	 /* �ȴ��ӳ�ʱ�� */

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

        //=========GPRS ����====���ݶԻ�===


        my_result = xQueueReceive(myQueue01Handle, &my_step, 3000); //xQueuePeek,01���У���ӦM35�ķ��Ͷ���
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
        //A�����
        //====GRPS ������·��1��--Server����===========
        my_fun_gprs_time_dialog_tx(my_step, 0X0044, 0x4400, 0, my_fun_GPRS_TX_start1_server);
        my_fun_gprs_time_dialog_tx(my_step, 0X0045, 0x4500, 0, my_fun_GPRS_TX_start2_server);
        my_fun_gprs_time_dialog_tx(my_step, 0X0046, 0x4600, 0, my_fun_GPRS_TX_start3_server);
        my_fun_gprs_time_dialog_tx(my_step, 0X0047, 0x4700, 0, my_fun_GPRS_TX_start4_server);



        //====DTU-GPRS ��������---DTU����
        my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0xE100, 0, my_fun_GPRS_TX_start1); //���״̬��ǰһ״̬����ǰ׼��״̬������״̬�����ú���
        my_fun_gprs_time_dialog_tx(my_step, 0x00E1, 0xE200, 0, my_fun_GPRS_TX_start2);
        my_fun_gprs_time_dialog_tx(my_step, 0x00E2, 0xE300, 0, my_fun_GPRS_TX_start3);


        //=====����
        my_fun_gprs_time_dialog_tx(my_step, 0X00C1, 0xC100, 0, my_fun_GPRS_TX_Call_0); //OK֡
        my_fun_gprs_time_dialog_tx(my_step, 0X00C2, 0xC200, 0, my_fun_GPRS_TX_CYC2); //ң�ţ���ʱ��
        my_fun_gprs_time_dialog_tx(my_step, 0X00C3, 0xC300, 0, my_fun_GPRS_TX_CYC3); //ң��(4011,ָʾ������)
        my_fun_gprs_time_dialog_tx(my_step, 0X00C4, 0xC400, 0, my_fun_GPRS_TX_CYC4); //ң��(4001,DTU����)
        my_fun_gprs_time_dialog_tx(my_step, 0X00C5, 0xC500, 0, my_fun_GPRS_TX_CYC5);//���ٽ���


        //RTCʱ��ͬ��
        my_fun_gprs_time_dialog_tx(my_step, 0X00D1, 0xD100, 0, my_fun_GPRS_TX_RTC_data);

        //RTCʱ�Ӷ�ȡ
        my_fun_gprs_time_dialog_tx(my_step, 0X00D3, 0xD300, 0, my_fun_GPRS_TX_RTC_data_read);

        //��·��������
        my_fun_gprs_time_dialog_tx(my_step, 0X00D5, 0xD500, 0, my_fun_GPRS_TX_test_data);

        //�������� ����������
        my_fun_gprs_time_dialog_tx(my_step, 0X00D7, 0xD700, 1, my_fun_GPRS_TX_heart_toserver_data);

        //���������ϴ�ң����Ϣ
        my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0xB100, 0, my_fun_GPRS_TX_CYC2_B); //ң�ţ���ʱ��
        my_fun_gprs_time_dialog_tx(my_step, 0X00B1, 0xB200, 0, my_fun_GPRS_TX_CYC3_B); //ң��(4011,ָʾ������),��ʱ��
        my_fun_gprs_time_dialog_tx(my_step, 0X00B2, 0xB300, 0, my_fun_GPRS_TX_CYC4_B); //ң��(4001,DTU����)

        //��λ����
        my_fun_gprs_time_dialog_tx(my_step, 0X00A1, 0xA100, 0, my_fun_GPRS_TX_RESET); //

        //������ң����ʱ�꣬ң����ʱ�꣩
        //my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0x9100, 0, my_fun_GPRS_TX_ALarm_data_yaoxin); //ң�ţ���ʱ��,��ң�ţ���ң��������ݴ���
        //my_fun_gprs_time_dialog_tx(my_step, 0X0091, 0x9200, 0, my_fun_GPRS_TX_ALarm_data_yaoce); //ң�⣬��ʱ��my_fun_GPRS_TX_ALarm_data_yaoxin_yaoce
        my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0x9100, 0, my_fun_GPRS_TX_ALarm_data_yaoxin_yaoce); //ң�ź�ң��ϲ���һ������ң����ʱ�꣬ң��û��ʱ��

        //�ٻ�Ŀ¼
				my_fun_gprs_time_dialog_tx(my_step, 0X0051, 0x5100, 0, my_fun_GPRS_TX_catalog); //


        //�ٻ��ļ�
				my_fun_gprs_time_dialog_tx(my_step, 0X0053, 0x5300, 0, my_fun_GPRS_TX_file_data_1); //���ļ��������ȷ��
				
							
				my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0x5500, 0, my_fun_GPRS_TX_file_data_2); //�ļ�����
				my_fun_gprs_time_dialog_tx(my_step, 0X0056, 0x5600, 1, my_fun_GPRS_TX_file_data_3);  //�ļ����ݷ��ͣ�okȷ����ͬʱ�������������ļ�����


        
				
				
				
				//C�����
        //=====DTU==GPRS ��������������---
        my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0x1F00, 0, my_fun_GPRS_TX_test1);
        //===DTU�������ò���
        my_fun_gprs_time_dialog_tx(my_step, 0X0081, 0x8100, 1, my_fun_GPRS_TX_changeparameter); //
        //===DTU ����ָʾ������
        my_fun_gprs_time_dialog_tx(my_step, 0X0061, 0x6100, 1, my_fun_GPRS_TX_TurnLED); //
        //===DTU��ѯ����
        my_fun_gprs_time_dialog_tx(my_step, 0X0071, 0x7100, 0, my_fun_GPRS_TX_query_data);
        my_fun_gprs_time_dialog_tx(my_step, 0X0072, 0x7200, 0, my_fun_GPRS_TX_query_data);
        my_fun_gprs_time_dialog_tx(my_step, 0X0073, 0x7300, 1, my_fun_GPRS_TX_query_data);


        my_fun_gprs_time_dialog_tx(my_step, 0X0077, 0x7700, 0, my_fun_GPRS_TX_query_data2);
        my_fun_gprs_time_dialog_tx(my_step, 0X0078, 0x7800, 0, my_fun_GPRS_TX_query_data2);
        my_fun_gprs_time_dialog_tx(my_step, 0X0078, 0x7900, 1, my_fun_GPRS_TX_query_data2);

        //--�����ź�ǿ�� (DTU--ָʾ��1/2/3��ÿ��ǿ��Ϊһ���Լ�0-99,0Ϊ���
        my_fun_gprs_time_dialog_tx(my_step, 0X0000, 0x4100, 0, my_fun_GPRS_TX_xinhaoqiangdu);
        //--SERVER��ѯDTU����ֵ��RTCʱ��
        my_fun_gprs_time_dialog_tx(my_step, 0X0031, 0x3100, 1, my_fun_GPRS_TX_TIME_RTC); //


        //����¼��
        //my_fun_gprs_time_dialog_tx(my_step, 0X0051, 0x5100, 0, my_fun_GPRS_TX_rec_data); //����֡,�ļ���һ��
       // my_fun_gprs_time_dialog_tx(my_step, 0X0052, 0x5200, 0, my_fun_GPRS_TX_rec_data); //
        //my_fun_gprs_time_dialog_tx(my_step, 0X0053, 0x5300, 0, my_fun_GPRS_TX_rec_data); //�ļ����һ��









        //osDelay(1);
    }
    /* USER CODE END StartTask04 */
}

/* StartTask05 function */
void StartTask05(void const * argument)
{
    /* USER CODE BEGIN StartTask05 */

    //====GPRS  ����
    //=========GPRS �������ݶԻ�����
    BaseType_t my_result;
    uint16_t my_step;
    /* Infinite loop */
    for(;;)
    {

        //my_fun_take_group();
        //my_fun_take_BinarySem();
        //my_fun_take_Queue();

        //=========GPRS �������ݶԻ�����
        my_result = xQueueReceive(myQueue02Handle, &my_step, 5000); //xQueuePeek,����2��Ӧ��02���У�M35���ն���
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

        //A�����
        //==GPRS==��·���� server����
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0x0044, 0X4400, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x4400, 0x0045, 0X4500, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x4500, 0x0046, 0X4600, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x4600, 0x0047, 0X4700, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x4700, 0x0048, 0X0000, 1, my_fun_GPRS_RX_test1);



        //===GPRS start--tcp����--DTU��������
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xE100, 0x00E1, 0XE200, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xE200, 0x00E2, 0XE300, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xE300, 0x00E3, 0X0000, 1, my_fun_GPRS_RX_test1);

        //=======��������
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X00C1, 0XC100, 0, my_fun_GPRS_RX_test1); //���ټ�������
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xC100, 0x00C2, 0XC200, 0, my_fun_GPRS_RX_test1); //����ȷ�ϼ���OK
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0XC200, 0x00C3, 0XC300, 0, my_fun_GPRS_RX_test1); //ң��OK
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0XC300, 0x00C4, 0XC400, 0, my_fun_GPRS_RX_test1); //ң��4011��ʼ��OK��ָʾ��
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0XC400, 0x00C5, 0XC500, 0, my_fun_GPRS_RX_test1); //ң��4001��ʼ��OK��DTU����
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0XC500, 0x00C6, 0X0000, 1, my_fun_GPRS_RX_test1); //���ٽ���OK

        //====RTCʱ��ͬ��
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X00D1, 0XD100, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xD100, 0X00D2, 0X0000, 1, my_fun_GPRS_RX_test1);
        //====RTCʱ�Ӷ�ȡ
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X00D3, 0XD300, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xD300, 0X00D4, 0X0000, 1, my_fun_GPRS_RX_test1);
        //====��·��������
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X00D5, 0XD500, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xD500, 0X00D6, 0X0000, 1, my_fun_GPRS_RX_test1);

        //=====����  ����������

        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X00D7, 0xD700, 0, my_fun_GPRS_RX_test1);

        //==����ң��
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xB100, 0x00B1, 0XB200, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xB200, 0x00B2, 0XB300, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xB300, 0x00B3, 0XB400, 1, my_fun_GPRS_RX_test1);

        //DTU���̸�λ
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X00A1, 0XA100, 0, my_fun_GPRS_RX_test1);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0xA100, 0X00A2, 0X0000, 1, my_fun_GPRS_RX_test1);

        //���ϱ�����ң��+ʱ�꣬ң��+ʱ�꣩
        //my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9100, 0X0091, 0X9200, 0, my_fun_GPRS_RX_test1); //ң����ʱ��
        //my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9200, 0X0092, 0X0000, 1, my_fun_GPRS_RX_test1); //ң����ʱ��
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9100, 0X0091, 0X0000, 1, my_fun_GPRS_RX_test1); //ң�ź�ң��ϲ���һ������


        //�ٻ�Ŀ¼
				my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0051, 0X5100, 0, my_fun_GPRS_RX_test1);
				my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x5100, 0X0052, 0X0000, 1, my_fun_GPRS_RX_test1);


        //�ٻ��ļ�
				my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0053, 0X5300, 0, my_fun_GPRS_RX_test1); //���ļ�����
				my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x5300, 0X0054, 0X0000, 1, my_fun_GPRS_RX_test1); //���ļ�����ȷ�ϣ������ļ����� ��my_GPRS_all_step=0
				
				my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x5500, 0X0056, 0X5600, 0, my_fun_GPRS_RX_test1); //�������ݴ���ȷ����������һ�δ���





        
				
				
				
				
				
				
				
				
				
				//C�����
        //===����ֵͬ��
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0x00E5, 0XE600, 0, my_fun_GPRS_RX_test1);
        //========GPRS����
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x1F00, 0X001F, 0X0000, 1, my_fun_GPRS_RX_test1); //���������յ�OK֡

        //DTU��������
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0081, 0X8100, 0, my_fun_GPRS_RX_change_parameter);

        //ָʾ������
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0061, 0X6100, 0, my_fun_GPRS_RX_turn_led);

        //��ѯ����
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0071, 0X7100, 0, my_fun_GPRS_RX_query_data);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x7100, 0X0072, 0X7200, 0, my_fun_GPRS_RX_query_data);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x7200, 0X0073, 0X7300, 0, my_fun_GPRS_RX_query_data);

        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0077, 0X7700, 0, my_fun_GPRS_RX_query_data2);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x7700, 0X0078, 0X7800, 0, my_fun_GPRS_RX_query_data2);
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x7800, 0X0079, 0X7A00, 0, my_fun_GPRS_RX_query_data2);
        //�����ź�ǿ��
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x4100, 0X0041, 0X0000, 1, my_fun_GPRS_RX_test1);
        //��ѯ����ֵ��RTC
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0031, 0X3100, 0, my_fun_GPRS_RX_test1);

        //����
//        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9100, 0X0091, 0X9200, 0, my_fun_GPRS_RX_test1); //ң����ʱ��
//        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9200, 0X0092, 0X9300, 0, my_fun_GPRS_RX_test1); //ң����ʱ��
//        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9300, 0X0093, 0X9400, 0, my_fun_GPRS_RX_test1); //ң��AC
//        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9400, 0X0094, 0X9500, 0, my_fun_GPRS_RX_test1); //ң��12T
//        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x9500, 0X0095, 0X9600, 1, my_fun_GPRS_RX_test1); //������RTC
        //����¼������
        my_fun_gprs_time_dialog_rx(&myQueue01Handle, my_step, 0x0000, 0X0051, 0X5100, 0, my_fun_GPRS_RX_Rec_data); //����ָ��
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
    //=====CC1101�������ݶԻ�
    extern	uint8_t  my_cc1101_tx_buf[];
    uint16_t my_step = 0;
    BaseType_t my_result = 0;
    extern uint8_t my_cc1101_dest_address;
    /* Infinite loop */
    for(;;)
    {


        //=====CC1101�������ݶԻ�
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

        //====DTU ��������--���Ͳ���
        my_fun_CC1101_time_dialog_tx3(my_step, 0xF000, 0xF100, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0xF200, 0xF300, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0xF400, 0xF500, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0xF600, 0xF700, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0xF800, 0xF900, 1, my_fun_CC1101_test1);
        //===========�����ظ�OK
        my_fun_CC1101_time_dialog_tx3(my_step, 0xE000, 0x00E0, 1, my_fun_CC1101_TX_OK);
        //===��������
        my_fun_CC1101_time_dialog_tx3(my_step, 0xE100, 0x00E1, 0, my_fun_CC1101_TX_config_parmeter);

        //======CC1101 ����====
        my_fun_CC1101_time_dialog_tx3(my_step, 0x0100, 0x0001, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0x4000, 0x0040, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0x4100, 0x0041, 0, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0x4200, 0x0042, 1, my_fun_CC1101_test1);
        my_fun_CC1101_time_dialog_tx3(my_step, 0x4300, 0x0043, 1, my_fun_CC1101_test1);


        //======CC1101 ����
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
    //=========CC1101�������ݶԻ�����
    BaseType_t my_result;
    uint16_t my_step;
    /* Infinite loop */
    for(;;)
    {
        //=========CC1101�������ݶԻ�����
        my_result = xQueueReceive(myQueue04Handle, &my_step, 2000); //xQueuePeek,����2��Ӧ��M35���ն���
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
        //====DTU��������--���ղ���--

        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xF000, 0xF100, 0, my_fun_write_update_data_to_FLASH);
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xF200, 0xF300, 0, my_fun_write_update_data_to_FLASH);
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xF400, 0xF500, 0, my_fun_write_update_data_to_FLASH);
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xF600, 0xF700, 0, my_fun_write_update_data_to_FLASH);
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xF800, 0xF900, 0, my_fun_write_update_data_to_FLASH);

        //=====ZSQ��������
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xE000, 0x00E0, 0, my_fun_dialog_CC1101_RX_0);
        //=====��������
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xE100, 0x00E1, 0, my_fun_dialog_CC1101_RX_0);
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0xE200, 0x0000, 1, my_fun_dialog_CC1101_RX_0);

        //====CC1101 ����=====
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x0100, 0x0001, 0, my_fun_dialog_CC1101_RX_1);//ң��
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x4000, 0x0040, 0, my_fun_dialog_CC1101_RX_1);//ң��DC
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x4100, 0x0041, 0, my_fun_dialog_CC1101_RX_1);//ң��AC
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x4200, 0x0042, 0, my_fun_dialog_CC1101_RX_1);	//AC12T

        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x4300, 0x0043, 0, my_fun_dialog_CC1101_RX_1); //¼������

        //=====CC1101����==
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x0200, 0x0002, 0, my_fun_dialog_CC1101_RX_1); //ң��
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x5000, 0x0050, 0, my_fun_dialog_CC1101_RX_1);//ң��DC
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x5100, 0x0051, 0, my_fun_dialog_CC1101_RX_1);//ң��AC
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x5200, 0x0052, 0, my_fun_dialog_CC1101_RX_1);//AC12T
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x5300, 0x0053, 0, my_fun_dialog_CC1101_RX_1); //����¼������
        my_fun_CC1101_time_dialog_rx2(&myQueue03Handle, my_step, 0x0000, 0x5400, 0x0054, 0, my_fun_dialog_CC1101_RX_1);//�糡



        //osDelay(1);
    }
    /* USER CODE END StartTask07 */
}

/* StartTask08 function */
void StartTask08(void const * argument)
{
    /* USER CODE BEGIN StartTask08 */
    //CC1101���յ����ݵ�Э�����
    uint8_t my_status = 0;
    uint8_t temp8 = 0;
    uint16_t my_step = 0;

    /* Infinite loop */
    for(;;)
    {



        EventBits_t	uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* �¼���־���� */
                             0x08,            /* �ȴ�bit0��bit1������ */
                             pdTRUE,             /* �˳�ǰbit0��bit1�������������bit0��bit1�������òű�ʾ���˳���*/
                             pdTRUE,             /* ����ΪpdTRUE��ʾ�ȴ�bit1��bit0��������*/
                             3000); 	 /* �ȴ��ӳ�ʱ�� */

        if((uxBits & 0x08) == 0x08)
        {
            my_status = 1; //ͨ����־�飬��ý��յ����ݵı�־
            //printf("cc1101 receive data !\r\n");
        }
        else
            my_status = 0;
        //��CC101���յ��������ƶ�����������
        if(my_status == 1)
        {
            if(RF_RecvHandler_intrrupt_get_data_to_buf() > 0)
                my_status = 1;
            else
                my_status = 0;

        }

        //�Ի���������֡����
        if(my_status == 1)
        {

            //Э����������һ֡����
            my_101frame_analyse(4, 1, CC1101_CRC_check); //CC1101Э�����
            //============
            if(my_CC1101_Frame_status > 0) //������������
            {
#if Debug_uart_out_cc1101_rx_data_status==1
                my_fun_display_fram_16(4, 8); //����ʹ�ã���ʾ���յ�������
#endif

                my_CC1101_Frame_status = 0;

                if(my_CC1101_COM_Fram_buf[0] == 0x10)
                {
                    my_status = 0x10;
                    temp8 = my_CC1101_COM_Fram_buf[1]; //������Ϊ0X20������OK֡
                }
                else if (my_CC1101_COM_Fram_buf[0] == 0x68)
                {
                    my_status = 0x68;
                    temp8 = my_CC1101_COM_Fram_buf[6]; //������Ϊ0X20������OK֡
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

            //===DTU  ��������====
            if( temp8 == 0xF0 && my_GPRS_all_step == 0)
            {
                //my_CC1101_all_step=0XF000; //�Ի����̵Ŀ�ʼ״̬
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
            //=========CC1101����=====
            else if( temp8 == 0x1F && my_GPRS_all_step == 0) //ָʾ����������֡
            {

                my_step = 0XE000;
                xQueueSend(myQueue04Handle, &my_step, 100);
            }

            //===========CC1101����====

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

            //=============CC1101����========
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
            //��������
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
        if(GPRS_Heartdata_error_count == 0) //GPRS״̬��ʶ��offΪ������onΪ�Ͽ�
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
        ���ͱ�����ͨ����⵽ZSQ�ı���״̬����ת����
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

            my_fun_give_Queue(&myQueue01Handle, 0X9100); // GPRS�������ͱ�������  @@@@����ʹ��


        }

    }
    //#############################
    if(my_os_count1 % (3667) == 0 && my_os_count1 != 0 && my_CC1101_all_step == 0 && my_GPRS_all_step == 0)
    {
        printf("\n====GPRS Radio PW== my_os_count1=%d\r\n", my_os_count1);
        //C�� ����,���������ź�ǿ��
        //my_fun_give_Queue(&myQueue01Handle, 0X4100); // GPRS���������ź�ǿ�ȣ�ZSQ�ź�ǿ��1Сʱ����һ��

    }
    if(my_os_count1 % (60) == 0 && my_os_count1 != 0)
    {
        my_fun_M35_resume_init();//M35���³�ʼ�������ͱ�־��Ϣ��

        if( my_CC1101_ERROR_OLD == my_gprs_count_time)  //ȫ�ֱ�־�Ա�
        {
            my_CC1101_ERROR_count++;  //������һ
            printf("not change my_gprs_count_time=%d,ERROR_count=%d\n", my_gprs_count_time, my_CC1101_ERROR_count);
        }
        else
        {
            my_CC1101_ERROR_OLD = my_gprs_count_time;
            my_CC1101_ERROR_count = 0;
        }
        if(my_CC1101_ERROR_count >= 35)  //35��������
        {
            my_CC1101_ERROR_count = 0;
            HAL_NVIC_SystemReset();
        }

    }


    //#######################################################################3
    //#########################################################################
    //========GPRS================

    // GPRS�������� TCP����ָ�� DTU��������
    if(NET_Server_status == 0  && GPRS_Status == 1 && my_os_count1 % 13 == 0 && my_CC1101_all_step == 0)
    {
        if(NET_Server_status == 1)
            return;

        my_gprs_TX_status = 1;                       //A �棬DTU����������·�������
        //my_fun_give_Queue(&myQueue01Handle, 0XE100); //����13���ʼ���� TCP��·����20180324������һ��ȫ�ֱ���������·״̬��ʶ
        printf("====GPRS TCP server Start=%d\r\n", my_os_count1);
        //NET_Server_status = 1;

    }

    //GPRS�����������ݣ��������ݣ����͵�01�Ŷ��У���Ӧ04������,ϵͳ���������󣬷���һ������
    if(my_system_restart_status == 1 && my_GPRS_all_step == 0 && my_gprs_TX_status == 0 && my_CC1101_all_step == 0 && my_os_count1 > 15)
    {
        printf("\n\n====Restart GPRS CYC time==my_os_count1=%d\r\n", my_os_count1);
        my_system_restart_status = 0;
        my_init_m35();
        my_gprs_TX_status = 1;
        //my_fun_give_Queue(&myQueue01Handle, 0XB100);@@@@@20180328  C�� ������ʼ��

    }

//    //M35���³�ʼ��
//    if(my_os_count1 % (60) == 0 && my_os_count1 != 0)
//    {
//        my_fun_M35_resume_init();//M35���³�ʼ�������ͱ�־��Ϣ��
//    }
    //==========GPRS end=============================


    //=====OS ��ջ�����====
#if OS_heap_high_water_data==1
    if(my_os_count1 % (577) == 0 && my_os_count1 != 0)
    {
        //if(my_CC1101_all_step == 0 && my_GPRS_all_step == 0)
        {
            my_fun_task_heap_value();  //��������ջʣ����
        }

    }
#endif

    //=====OS ��ջ�����==end==


    //====����CC1101���ջ����������ݶ�ʧ��������׷������
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
    //=====CC1101����������
    if(my_os_count1 % (5) == 0 && my_os_count1 != 0 && my_GPRS_all_step == 0)
    {

#if		Use_CC1101_send_heat_data_status==1
        my_fun_CC1101_send_heart_data();
#endif

    }
    //=====END==CC1101������===


    //====��CPU STM32F0
    //======��CPU����CPU����ָ��,���DTU�ĵ�ѹ����ʪ��
    //����
    if(my_os_count1 % (45) == 0 && my_CC1101_all_step == 0 && my_GPRS_all_step == 0)
    {
        uint8_t my_txbuf[18] = TX101_calldata;
        uint8_t start_length = 14 + 6;
        uint8_t ii = 0;
        //������������
        USART_printf(&huart2, my_txbuf);
        osDelay(2000);
        //���ݴ���

        for(ii = 0; ii < 8; ii++)
        {
            MY_MCU_RsBuf[ii] = rsbuf2[start_length + ii];
        }
        my_buf1_to_buf2(MY_MCU_RsBuf, 0, MY_GPRS_MCU_RsBuf, 0, 8); //��������


        MY_Bat_value = MY_MCU_RsBuf[1];
        MY_Bat_value = (MY_Bat_value << 8) + MY_MCU_RsBuf[0];
        //2016-08-30 ȡ̫���ܣ��¶ȡ�ʪ��
        MY_Sun_value = MY_MCU_RsBuf[3];
        MY_Sun_value = (MY_Sun_value << 8) + MY_MCU_RsBuf[2];

        my_AD_value = (float)(MY_Bat_value * 3.3) / 4096 * 11;
        my_AD_SUN_value = (float)(MY_Sun_value * 3.3) / 4096 * 11;

        MY_Temperature_value = MY_MCU_RsBuf[5];
        MY_Temperature_value = (MY_Temperature_value << 8) + MY_MCU_RsBuf[4];

        MY_Humidity_value = MY_MCU_RsBuf[7];
        MY_Humidity_value = (MY_Humidity_value << 8) + MY_MCU_RsBuf[6];


        //�����������д���ֱ�Ӵ����ʵ��ֵ
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

        my_buf1_to_buf2(MY_MCU_RsBuf, 0, MY_GPRS_MCU_RsBuf, 0, 8); //��������
#endif

        //�ظ�OK֡
        my_UART4_printf(&huart2, TX101_OKdata);
        //USART_printf(&huart2,TX101_OKdata);
        rsbuf2pt_write = 0;
        //���DTU�Ļ�������
        printf("DTU LIBAT=%.1f,SUNbat=%.1f,mytemperature=%.1f,myshidu=%.1f\n", MY_Bat_value / 10.0, MY_Sun_value / 10.0, MY_Temperature_value / 10.0, MY_Humidity_value / 10.0);

        //=========�趨����ʱ��============
        if(my_AD_value >= MY_Speed_H_Gate && my_AD_value <= 13) //���� ����10���ӣ�����5����
        {
            MY_ACT_Heart_DTU = MY_H_speed_heart;
            MY_ACT_CYC_DTU = MY_H_speed_cyc;

        }
        else if(my_AD_value >= MY_Speed_L_Gate && my_AD_value < MY_Speed_H_Gate) //���٣�����15���ӣ�����10����,ϵͳĬ��ʱ�䣬GPRSָ���޸�����������ֵΪ��ֵ
        {
            MY_ACT_Heart_DTU = MY_M_speed_heart;
            MY_ACT_CYC_DTU = MY_M_speed_cyc;
        }
        else //���٣�����30���ӣ�����10����
        {
            MY_ACT_Heart_DTU = MY_L_speed_heart;
            MY_ACT_CYC_DTU = MY_L_speed_cyc;
        }


        //=============
    }
    //����
    else if(my_os_count1 % (57) == 0) //��СCPU
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

        HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN); //��ȡRTCʱ��
        HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);
        printf("@@RTC= %d/%d/%d %d:%d:%d== RSSI=[%d]-[%d]-[%d]\n",
               my_RTC_date.Year, my_RTC_date.Month, my_RTC_date.Date, my_RTC_time.Hours, my_RTC_time.Minutes, my_RTC_time.Seconds,
               my_indicator_data[0].xinhao_db, my_indicator_data[1].xinhao_db, my_indicator_data[2].xinhao_db
              );

    }
    //====��CPU==END===
    //��ʾ��ǰ���ݵ�ֵ
    if(my_os_count1 % 69 == 0)
    {
        printf("==display my_os_count1=%d", my_os_count1);
        //my_fun_display_ZSQ_data();

    }

    /*
    ģ�ⱨ���жϣ���·�ͽӵر����ж�
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

    LED2_TOGGLE; //led2��ת��ʾ��OSϵͳ���ţ�1��1�Σ���ʱ��
    /* USER CODE END Callback01 */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
