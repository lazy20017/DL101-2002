#include "my_OS.h"
#include "math.h"
#include  "my_globle_extern.h"
#include "string.h"




extern EventGroupHandle_t xCreatedEventGroup;
extern osMessageQId myQueue01Handle;
extern osMessageQId myQueue03Handle;
extern osMutexId myMutex01Handle;

extern osThreadId defaultTaskHandle;
extern osThreadId myTask02Handle;
extern osThreadId myTask03Handle;
extern osThreadId myTask04Handle;
extern osThreadId myTask05Handle;
extern osThreadId myTask06Handle;
extern osThreadId myTask07Handle;
extern osThreadId myTask08Handle;
extern osThreadId myTimer01Handle;
extern osMessageQId myQueue02Handle;

extern uint8_t my_usart1_tx_buf1[];
extern uint8_t my_cc1101_dest_address;
extern uint8_t GPRS_Heartdata_error_count;
extern uint8_t my_CC1101_COM_Fram_buf[];

extern uint8_t my_UART1_Status;
extern uint8_t my_UART2_Status;
extern uint8_t my_UART3_Status;
extern uint16_t DTU_ADDRESS;
extern uint8_t my_CC1101_chip_address;
extern uint8_t my_gprs_TX_status;
extern uint8_t my_system_restart_status;
extern uint8_t USART1_my_frame[];
extern uint8_t NET_Server_status;
extern uint16_t my_cc1101_tx_wait_time;
extern uint8_t my_RSSI_dbm_all;
extern uint16_t MY_Bat_value;


//=================
//GPRS��·״̬��ʶ��0Ϊû�н�����·��1Ϊ������·��
uint8_t my_101_DIR = 0X80; //101��Լ��,�������DIRλ ���λD8
uint8_t my_101_PRM = 0X40; //101��Լ�У��������PRM  D7
uint8_t my_101_FCB = 0; //������D6
uint8_t my_101_FCV = 0; //������D5

uint8_t my_101_FC = 0; //�����򣬹�����
uint8_t my_101_add_low = 0;
uint8_t my_101_add_high = 0;
uint8_t my_101_TI = 0;
uint8_t my_101_VSQ_1_7 = 0;
uint8_t my_101_VSQ_8_SQ = 0;
uint8_t my_101_COT_low = 0;
uint8_t my_101_COT_high = 0;

//==========




#define my_pi  3.1415926

uint8_t my_cc1101_tx_buf[64] = {0x10, 0x20, 0x13, 0x14, 0x15, 0x16};

uint16_t  my_GPRS_all_step = 0;
uint8_t my_GPRS_all_count = 0;
uint16_t  my_CC1101_all_step = 0;
uint8_t  my_CC1101_all_count = 0;




uint8_t temp8 = 0;
uint16_t query_data = 0X00; //��ѯ���͵�����
uint16_t query_data2 = 0X00;
#define my_indicator_count 3 //ϵͳ��ָʾ������
uint8_t my_tx_rec_count_all = 9; //�����ܵĶ���
uint8_t my_tx_rec_count_finish = 0; //�Ѿ����͵Ķ���

//��������ʹ��
struct indicator_class my_indicator_data[my_indicator_count];
struct indicator_class_parameter my_indicator_parameter_data[3];
//��������ʹ��
struct indicator_alarm_class my_indicator_alarm_data[my_indicator_count];
struct indicator_record_class my_indicator_record_data[3];

uint8_t my_indicator_tx_index = 99; //׼�����͵ı�������ָʾ���ı��0��1,2,3,4����5,6��ע�⣬��Ŵ�0��ʼ


uint8_t my_CC1101_record_wave_last_index = 0; //��¼��õ����µ�¼����ָʾ����ţ�01��02,03
uint8_t my_CC1101_receive_cmd_ID = 0;

uint16_t my_gprs_count_time = 0; //GPRSͨ�ţ��������ݣ����ݸ�SERVER��DTU�յ���zsq�ļ���ֵ
uint8_t  my_gprs_RTC_buf[7] = {0};
uint8_t  my_get_data_buf[2] = {0};

char my_file_catalog_buf[15] = {0};
uint8_t my_file_catalog_status = 0; //0Ŀ¼�������ļ���1����ʱ����ļ�
uint8_t my_file_RTC_start_time[7] = {0}; //��ѯ��ʼʱ��
uint8_t my_file_RTC_end_time[7] = {0}; //��ѯ����ʱ��


uint8_t my_file_class_ID=0;  //��ȡ�ļ����͵�ID��1ΪSOE�¼���2Ϊ�����¼���ݣ�3Ϊ��־
uint8_t my_file_name_buf[31]={0};  //�ļ����ƻ�����������·�������ơ���չ��
uint8_t my_file_name_count=0;      
uint16_t my_file_part_count = 0; //�ļ�׼�����͵� �� ������0��ʾû�з��͵ĶΣ����65535,��������ʽ��1�����һ����
uint32_t my_file_data_count=0;  //�ļ��е��ֽ�����
uint8_t my_file_part_data_count_aver=0;  //�����һ���⣬ÿ�ε��ֽ�����
uint8_t my_file_part_data_count_end=0;  //���һ�Σ��ֽ�����

union MY_float
{
    float value;
    unsigned char byte[4];
};


//=====================��������
void my_fun_set_group(void)
{

    BaseType_t xResult;
    BaseType_t xHigherPriorityTaskWoken = pdFAIL;
    //xEventGroupSetBits(xCreatedEventGroup,0X01);
    xResult =	xEventGroupSetBitsFromISR(xCreatedEventGroup, 0X01, &xHigherPriorityTaskWoken);
    if(xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
void my_fun_take_group(void)
{
    EventBits_t	uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* �¼���־���� */
                         0X01,            /* �ȴ�bit0��bit1������ */
                         pdTRUE,             /* �˳�ǰbit0��bit1�������������bit0��bit1�������òű�ʾ���˳���*/
                         pdTRUE,             /* ����ΪpdTRUE��ʾ�ȴ�bit1��bit0��������*/
                         200); 	 /* �ȴ��ӳ�ʱ�� */

    if((uxBits & 0x01) == 0x01)
    {
        /* ���յ�bit1��bit0�������õ���Ϣ */
        printf("group bit0 is ok!\r\n");
    }

}



extern osSemaphoreId myBinarySem01Handle;
void my_fun_give_BinarySem(void)
{

    BaseType_t xResult;
    BaseType_t xHigherPriorityTaskWoken = NULL;
    //xEventGroupSetBits(xCreatedEventGroup,0X01);
    // xResult=	xSemaphoreGiveFromISR(myBinarySem01Handle,&xHigherPriorityTaskWoken);
    BaseType_t pxHigherPriorityTaskWoken = NULL;
    xSemaphoreGiveFromISR(myBinarySem01Handle, &pxHigherPriorityTaskWoken);

    if(xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
void my_fun_take_BinarySem(void)
{
    BaseType_t xResult = xSemaphoreTake(myBinarySem01Handle, 200);

    if(xResult == pdTRUE)
    {
        /* ���յ�bit1��bit0�������õ���Ϣ */
        printf("BinarySem01 take ok��\r\n");
    }

}

void my_fun_give_Queue1(void)
{
    temp8 = 0;
    temp8++;

    my_GPRS_all_step = temp8;
    BaseType_t pt = NULL;
    BaseType_t xResult;
    xResult = xQueueSendFromISR(myQueue01Handle, &temp8, &pt); //����1��Ӧ��M35�ķ���״̬����

    if(xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(pt);
    }
}


void my_fun_give_Queue31(void)
{
    uint16_t temp_step = 0;
    temp_step = 0X0071;
    my_CC1101_all_step = 0x0071;
    BaseType_t pt = NULL;
    BaseType_t xResult;
    xResult = xQueueSendFromISR(myQueue03Handle, &temp_step, &pt); //����1��Ӧ��M35�ķ���״̬����

    if(xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(pt);
    }
    //printf("queue3 is %d\r\n",temp8);
}
/*

���ܣ�����з�������
����1������ָ�룬����2�����͵�����16λ
*/

void my_fun_give_Queue(osMessageQId *my_QHL, uint16_t temp_step)
{
    BaseType_t pt = NULL;
    BaseType_t xResult;
    xResult = xQueueSendFromISR(*my_QHL, &temp_step, &pt); //����1��Ӧ��M35�ķ���״̬����

    if(xResult != pdFAIL)
    {
        //printf(" send QH OK--[%XH]\r\n",temp_step);
        portYIELD_FROM_ISR(pt);

    }
    else
    {
        printf(" send QH ERROR--[%XH]\r\n", temp_step);
        NVIC_SystemReset();

    }
    //printf("queue3 is %d\r\n",temp8);
}



void my_fun_take_Queue(void)
{
    uint8_t xx;
    BaseType_t xResult = xQueueReceive(myQueue01Handle, &xx, 100);

    if(xResult == pdPASS)
    {
        /* ���յ�bit1��bit0�������õ���Ϣ */
        printf("queue is =%d\r\n", xx);
    }

}


//=======================

//=====GPRS�Ի����� ����ʹ��==
void my_fun_gprs_time_dialog_rx(
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
    uint16_t my_old_step = 0;
    //=====0====
    //������������յ�������
    if(my_get_step == my_now_step && my_before_step == 0x00)
    {
        my_status = 1;
    }
    //�����󶨣���ǰ������
    else if(my_get_step == my_now_step && my_before_step == my_GPRS_all_step)
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
        //printf("GPRS RX-step = [%XH]\r\n",my_now_step);
        my_old_step = my_GPRS_all_step;
        my_GPRS_all_step = my_now_step;	//��ǰ״̬
        my_temp = ptfun(); //���̴����õģ����մ�����
        GPRS_Heartdata_error_count = 0;
        if(my_temp == 1)
        {
            //my_GPRS_all_step = my_now_step;	//��ǰ״̬
            xQueueSend(*QHL_send, &my_next_step, 100);	//��ʶ��һ��״̬
        }
        else
        {   my_GPRS_all_step = my_old_step;
            printf("GPRS�������ݴ��󣬲�����״̬ת��\r\n");
        }
    }
    else if(my_status == 1 && end_status == 1)
    {
        // printf("CC1101 RX-step = [%XH]\r\n",my_now_step);
        my_GPRS_all_step = my_now_step;
        ptfun();	 //�����õģ����մ�����
        my_GPRS_all_step = 0X00; //����״̬
        GPRS_Heartdata_error_count = 0;
        my_gprs_TX_status = 0;

    }

    //

}

//=====GPRS�Ի����� ����ʹ��==
void my_fun_gprs_time_dialog_tx(
    uint16_t my_get_step,  //���״̬
    uint16_t my_before_step,  //ǰһ��״̬
    uint16_t my_now_step,   //��ǰ׼������״̬
    uint8_t end_status,   //����״̬
    void (*ptfun)(void)
)
{

    //===�жϲ���
    if(my_get_step == my_now_step && my_before_step == 0XFF00) //����ǰһ��״̬Ϊ0XFF��������ж�֮ǰ�ĶԻ��������¶Ի�������ֻ�ܵȴ�ǰ�ζԻ�����
    {
        //printf("GPRS TX--step = [%XH]\r\n",my_now_step);
        my_GPRS_all_count = 0;
        my_GPRS_all_step = my_now_step;
        my_get_step = 0;
    }
    else if(my_get_step == my_now_step && my_before_step == my_GPRS_all_step)
    {
        // printf("GPRS TX--step = [%XH]\r\n",my_now_step);
        my_GPRS_all_count = 0;
        my_GPRS_all_step = my_now_step;
        my_get_step = 0;
    }

    //===�ظ����Ͳ���
    if( my_now_step == my_GPRS_all_step && my_GPRS_all_count < 3)
    {
        my_GPRS_all_count++;
        printf("GPRS TX-Fun= [%XH]--%d\r\n", my_now_step, my_GPRS_all_count);
        //osDelay(1);
        //HAL_Delay(1);//GPRS�������ݣ�Ҫ��ʱһ�£���Ϊ����ϵͳ�����л���Ҫ1ms
        ptfun();		//���ö�Ӧ�ĺ���
    }
    else if(my_GPRS_all_count >= 3)
    {
        my_GPRS_all_count = 0;
        my_GPRS_all_step = 0x00;
        GPRS_Heartdata_error_count++;
        my_gprs_TX_status = 0;
    }

    //====ֻ����һ�ξͽ���
    if(end_status == 1 && my_GPRS_all_count > 0 && my_GPRS_all_step == my_now_step)
    {
        my_GPRS_all_count = 0;
        my_GPRS_all_step = 0x00;
        my_gprs_TX_status = 0;
    }

}
//======end=




//-----------------
void my_fun_CC1101_time_dialog_tx3(
    uint16_t my_get_step,
    uint16_t my_before_step,
    uint16_t my_now_step,
    //uint16_t my_next_step,
    uint8_t end_status,
    void (*ptfun)(void)
)
{
    //===�жϲ���
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

    //===�ظ����Ͳ���
    if( my_now_step == my_CC1101_all_step && my_CC1101_all_count < 3 &&  my_now_step != 0)
    {
        my_CC1101_all_count++;
        printf("CC1101 TX-Fun[%d]= [%XH]--%d\r\n", my_cc1101_dest_address, my_now_step, my_CC1101_all_count);
        //osDelay(1);
        //HAL_Delay(1);//CC1101�������ݣ�Ҫ��ʱһ�£���Ϊ����ϵͳ�����л���Ҫ1ms
        ptfun();		//���ö�Ӧ�ĺ���
    }
    else if(my_CC1101_all_count >= 3)
    {
        my_CC1101_all_count = 0;
        my_CC1101_all_step = 0x00;
    }

    //====ֻ����һ�ξͽ���
    if(end_status == 1 && my_CC1101_all_count > 0 && my_CC1101_all_step == my_now_step)
    {
        my_CC1101_all_count = 0;
        my_CC1101_all_step = 0x00;
    }

}

//=====CC1101�Ի� ����ʹ��
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
    uint16_t my_old_setp = 0;
    //=====0====
    if(my_get_step == my_now_step && my_before_step == 0x00) //������������յ�������
    {
        my_status = 1;
    }
    else if(my_get_step == my_now_step && my_before_step == my_CC1101_all_step) //�����󶨣���ǰ������
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
        my_old_setp = my_CC1101_all_step;
        my_CC1101_all_step = my_now_step;
        my_temp = ptfun();
        if(my_temp == 1)
        {
            my_CC1101_all_step = my_now_step;	//��ǰ״̬
            xQueueSend(*QHL_send, &my_next_step, 100);	//��ʶ��һ��״̬
        }
        else
        {   my_CC1101_all_step = my_old_setp;
            printf("�������ݴ��󣬲�����״̬ת��\r\n");
        }
    }
    else if(my_status == 1 && end_status == 1)
    {

        ptfun();
        my_CC1101_all_step = 0X00; //����״̬

    }

    //

}


//=========

//=====GPRS ���ʹ�����======
/*
����:GPRS����������
*/
uint8_t my_GPRS_heart_count = 0;
extern uint16_t DTU_ADDRESS;
uint8_t my_cc1101_error_count = 0;



/*
���ܣ�CC1101���³�ʼ��
*/
void my_fun_CC1101_init_resume(void)
{
    uint8_t my_status = 0;
    uint8_t my_rx_count = 0;
    //xSemaphoreTake(myMutex01Handle,1000);
    my_status = CC1101ReadStatus(CC1101_MARCSTATE);
    my_rx_count = CC1101GetRXCnt();

    //xSemaphoreGive(myMutex01Handle);



    if(my_status == 0x01 && my_rx_count > 0)
    {
        printf("------ CC1101 status=[%XH] RXBUF=%d \n", my_status, my_rx_count);
        my_cc1101_error_count++;
        if(my_cc1101_error_count >= 300)
            HAL_NVIC_SystemReset();

        my_fun_CC1101_init_reum();
    }
    if(my_status != 0x01 && my_status != 0x0D &&  my_status != 0x13  ) //0x11,�������,//0X01���У�0X0D���գ�0X13����
    {
        printf("--error CC_status=[%XH] \n", my_status);
        my_cc1101_error_count++;
        if(my_cc1101_error_count >= 300)
            HAL_NVIC_SystemReset();
        my_fun_CC1101_init_reum();
        printf("--inint after CC_status=[%XH] \n", my_status);

        if(my_status == 0xff)
            HAL_NVIC_SystemReset();

    }
    else
    {
        my_cc1101_error_count = 0;
    }
}

/*
���ܣ��������³�ʼ������
*/

void my_fun_usart_init_resume(void)
{
    if(my_UART3_Status == 0X01)
    {
        MX_USART3_UART_Init();
        my_UART3_Status = 0;
        HAL_UART_Receive_IT(&huart3, &rsbuf3[rsbuf3pt_write], 1); //��������USART3����
    }
    if(my_UART1_Status == 0X01)
    {
        MX_USART1_UART_Init();
        my_UART1_Status = 0;
        HAL_UART_Receive_IT(&huart1, &rsbuf1[rsbuf1pt_write], 1); //��������USART1����
    }


}

/*
���ܣ���ʾ����ϵͳÿ������ʣ��Ķ�ջ��
*/
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
    uxHighWaterMark = uxTaskGetStackHighWaterMark( myTask05Handle );
    printf("task05 heap value=%d\r\n", uxHighWaterMark);
    uxHighWaterMark = uxTaskGetStackHighWaterMark( myTask06Handle );
    printf("task06 heap value=%d\r\n", uxHighWaterMark);
    uxHighWaterMark = uxTaskGetStackHighWaterMark( myTask07Handle );
    printf("task07 heap value=%d\r\n", uxHighWaterMark);
    uxHighWaterMark = uxTaskGetStackHighWaterMark( myTask08Handle );
    printf("task08 heap value=%d\r\n", uxHighWaterMark);
    //uxHighWaterMark=uxTaskGetStackHighWaterMark( myTimer01Handle );
    //printf("Callback01 heap value=%d\r\n",uxHighWaterMark);

}


/*
���ܣ�CC1101���ͣ�������������OK֡
*/
extern uint16_t my_tim6_count;
void my_fun_CC1101_test1(void)
{
    uint8_t *pt;
    uint16_t my_temp16 = my_tim6_count;
    pt = my_cc1101_tx_buf;
    pt[0] = 0x10;
    pt[1] = 0x20; //IDΪ0X20����ʾOK֡
    pt[2] = my_CC1101_chip_address; //����DTU��CC1101��ַ��Ϊ0XFE,��0XFDΪ�������ĵ�ַ��01,02,03---Ϊָʾ����ַ
    pt[3] = my_cc1101_dest_address; //Ŀ���ַ

    pt[4] = my_temp16;                   //my_CC1101_receive_cmd_ID; //�յ���ָ���IDֵ
    pt[5] = (my_temp16 >> 8);


    pt[6] = my_fun_101check_generate(pt, 1);
    pt[7] = 0x16;

    CC1101SendPacket_add( pt, 8,  ADDRESS_CHECK, my_cc1101_dest_address);
    printf("after CC TX my_CC1101_all_step=[%XH]\n", my_CC1101_all_step);

    if(my_CC1101_all_step == 0X0042)
    {
        printf("========CC1101 CYC Time is finish!====\n\n");

    }
    else if(my_CC1101_all_step == 0X0053)
    {

        printf("========CC1101 ALarm is finish!!!!==dianliu==\n\n");

    }
    else if(my_CC1101_all_step == 0X0054)
    {

        printf("========CC1101 ALarm is finish!!!!==jiedi==\n\n");

    }

#if Debug_uart_out_cc1101_tx_data_status==1
    my_fun_display_buf_16(pt, 8, 1); //����ʹ��
#endif

}

/*
���ܣ�CC1101�Ľ��գ����������������ݣ��������ݴ���
*/
uint16_t my_ZSQ_time_count[3] = {0}; //ָʾ��ң�����ݣ���Ӧ��ȫ�ּ���ֵ

uint8_t my_fun_dialog_CC1101_RX_1(void)
{
    uint8_t temp_status = 0;
    uint16_t my_length = 0;
    uint8_t my_address = 0;
//    uint8_t my_address_dest = 0;
    uint8_t my_indicator_index = 0;
    uint16_t ii = 0;
    uint8_t my_re = 0;

    if(my_CC1101_COM_Fram_buf[0] == 0x10) //10֡��
    {
        //���ָ��ID
        temp_status = my_CC1101_COM_Fram_buf[1];
        my_CC1101_receive_cmd_ID = temp_status;
        //��õ�ַ--����Դ
        my_address = my_CC1101_COM_Fram_buf[2]; //֡�еķ���Դ��ַ
        my_cc1101_dest_address = my_address; //�޸�CC1101��Ŀ�ĵ�ַ��Ϊ������׼��ʹ��

//        my_address_dest = my_CC1101_COM_Fram_buf[3]; //����Ŀ�ĵ�ַ
#if Debug_uart_out_cc1101_rx_data_status==1
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //����ʹ�ã���ʾ���յ�������8���ֽ�
#endif
    }
    else if (my_CC1101_COM_Fram_buf[0] == 0x68) //68��֡
    {
        //���ָ��ID
        temp_status = my_CC1101_COM_Fram_buf[6];
        printf("===============control word==[%X]\n", temp_status);
        my_CC1101_receive_cmd_ID = temp_status;

        my_length = my_CC1101_COM_Fram_buf[2];
        my_length = (my_length << 8) + my_CC1101_COM_Fram_buf[1] - 3; //��ó���

        //��õ�ַ
        my_address = my_CC1101_COM_Fram_buf[7];
        my_cc1101_dest_address = my_address; //�޸�CC1101��Ŀ�ĵ�ַ��Ϊ������׼��ʹ��
//        my_address_dest = my_CC1101_COM_Fram_buf[8];

#if Debug_uart_out_cc1101_rx_data_status==1
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //����ʹ�ã���ʾ���յ�������8���ֽ�
#endif
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //@@@@����ʹ�ã���ʾ���յ�������8���ֽ�
    }

    //ָʾ����ַ�ж�
    if(my_address == 0X01)
    {
        my_indicator_index = 0;
    }
    else if(my_address == 0X02)
    {
        my_indicator_index = 1;
    }
    else if(my_address == 0X03)
    {
        my_indicator_index = 2;
    }
    else
        my_indicator_index = 0X10;
    //֡���������ж�
    if(temp_status == 0x01)
    {
        my_indicator_data[my_indicator_index].data_type = 0x01; //����
    }
    else if(temp_status == 0x02)
    {
        my_indicator_data[my_indicator_index].data_type = 0x02; //ң��

    }
    my_cc1101_tx_wait_time = 2000;
//==========���ڡ�����������===========
    if(temp_status == 0x01 || temp_status == 0x02) //ң��--01Ϊ���ڣ�02Ϊ����
    {
        my_indicator_data[my_indicator_index].duanlu_data = my_CC1101_COM_Fram_buf[9]; //��·
        my_indicator_data[my_indicator_index].jiedi_data =	my_CC1101_COM_Fram_buf[11]; //�ӵ�
        //timer����ֵ
        my_ZSQ_time_count[my_indicator_index] = my_CC1101_COM_Fram_buf[12]; //����ֵ���ֽ�
        my_ZSQ_time_count[my_indicator_index] = (my_ZSQ_time_count[my_indicator_index] << 8) + my_CC1101_COM_Fram_buf[10];//����ֵ���ֽ�
        my_indicator_data[my_indicator_index].count_time[0] = my_CC1101_COM_Fram_buf[10]; //����ֵ���ֽ�
        my_indicator_data[my_indicator_index].count_time[1] = my_CC1101_COM_Fram_buf[12]; //����ֵ���ֽ�
        //RTCʱ��
        HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN);
        HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);
        my_indicator_data[my_indicator_index].RTC_time_buf[0] = my_RTC_time.Seconds; //RTCʱ��
        my_indicator_data[my_indicator_index].RTC_time_buf[1] = 0; //RTCʱ��
        my_indicator_data[my_indicator_index].RTC_time_buf[2] = my_RTC_time.Minutes; //RTCʱ��
        my_indicator_data[my_indicator_index].RTC_time_buf[3] = my_RTC_time.Hours; //RTCʱ��
        my_indicator_data[my_indicator_index].RTC_time_buf[4] = my_RTC_date.Date; //RTCʱ��
        my_indicator_data[my_indicator_index].RTC_time_buf[5] = my_RTC_date.Month; //RTCʱ��
        my_indicator_data[my_indicator_index].RTC_time_buf[6] = my_RTC_date.Year; //RTCʱ��

        //�ź�ǿ��
        my_indicator_data[my_indicator_index].xinhao_db = my_RSSI_dbm_all;

        //

        my_gprs_count_time = my_indicator_data[my_indicator_index].count_time[1];
        my_gprs_count_time = (my_gprs_count_time << 8) + my_indicator_data[my_indicator_index].count_time[0];
        for(ii = 0; ii < 7; ii++)
        {
            my_gprs_RTC_buf[ii] = my_indicator_data[my_indicator_index].RTC_time_buf[ii];

        }


        //����
        if(temp_status == 0x02)
        {


            //timer����ֵ

            my_indicator_alarm_data[my_indicator_index].count_time[0] = my_CC1101_COM_Fram_buf[10]; //����ֵ���ֽ�
            my_indicator_alarm_data[my_indicator_index].count_time[1] = my_CC1101_COM_Fram_buf[12]; //����ֵ���ֽ�
            //RTCʱ��

            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[0] = my_RTC_time.Seconds; //RTCʱ��
            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[1] = 0; //RTCʱ��
            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[2] = my_RTC_time.Minutes; //RTCʱ��
            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[3] = my_RTC_time.Hours; //RTCʱ��
            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[4] = my_RTC_date.Date; //RTCʱ��
            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[5] = my_RTC_date.Month; //RTCʱ��
            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[6] = my_RTC_date.Year; //RTCʱ��




            //�������ݷ���״̬��¼
            if(my_indicator_alarm_data[my_indicator_index].duanlu_data != 0 && my_CC1101_COM_Fram_buf[9] == 0)
                my_indicator_alarm_data[my_indicator_index].TX_status_duanlu = 0x01; //��ʾ�б������ݲ�������û�з��͡�����������ˣ�������ֽ�����
            else if( my_CC1101_COM_Fram_buf[9] != 0)
                my_indicator_alarm_data[my_indicator_index].TX_status_duanlu = 0x01;
            else
                my_indicator_alarm_data[my_indicator_index].TX_status_duanlu = 0x00;

            if(my_indicator_alarm_data[my_indicator_index].jiedi_data != 0 && my_CC1101_COM_Fram_buf[11] == 0)
                my_indicator_alarm_data[my_indicator_index].TX_status_jiedi = 0x01;
            else if( my_CC1101_COM_Fram_buf[11] != 0)
                my_indicator_alarm_data[my_indicator_index].TX_status_jiedi = 0x01;
            else
                my_indicator_alarm_data[my_indicator_index].TX_status_jiedi = 0x00;
            //���±���״̬
            my_indicator_alarm_data[my_indicator_index].duanlu_data = my_CC1101_COM_Fram_buf[9]; //��·
            my_indicator_alarm_data[my_indicator_index].jiedi_data =	my_CC1101_COM_Fram_buf[11]; //�ӵ�


        }



        my_re = 1;
    }
    else if(temp_status == 0x40 || temp_status == 0x50)	//����DC��1�¶ȣ�2��Դ��3�ο���ѹ��4�ɵ�أ�5���ϵ�ѹ��6̫���ܣ�7﮵��
    {
        for(ii = 0; ii < my_length; ii++)
        {
            my_indicator_data[my_indicator_index].DC_data_buf[ii] = my_CC1101_COM_Fram_buf[9 + ii];

        }
        //�ж�ָʾ���Ƿ�������ָʾ��0X11����״̬
        if(					my_indicator_data[my_indicator_index].DC_data_buf[0] == 0X11
                            && 	my_indicator_data[my_indicator_index].DC_data_buf[1] == 0X11
                            &&	my_indicator_data[my_indicator_index].DC_data_buf[2] == 0X11
                            &&	my_indicator_data[my_indicator_index].DC_data_buf[3] == 0X11
          )
        {
            //����CC1101���ڷ�ʽ��������
            //GPRS�����������ݣ��������ݣ����͵�01�Ŷ��У���Ӧ04������
            if(my_GPRS_all_step == 0 && my_gprs_TX_status == 0)
            {
                printf("====GPRS CYC time ZSQ[%d] reset========\n", my_indicator_index);
                my_gprs_TX_status = 1;
                my_fun_give_Queue(&myQueue01Handle, 0XB100);

            }

        }

        //��������
        if(temp_status == 0x50)
        {

            for(ii = 0; ii < my_length; ii++)
            {
                my_indicator_alarm_data[my_indicator_index].DC_data_buf[ii] = my_CC1101_COM_Fram_buf[9 + ii];
            }


        }


        my_re = 1;
    }
    else if(temp_status == 0x41 || temp_status == 0x51) //AC��Чֵ(ȫ���������糡���벨����)
    {

        for(ii = 0; ii < my_length; ii++)
        {
            my_indicator_data[my_indicator_index].AC_data_buf[ii] = my_CC1101_COM_Fram_buf[9 + ii];

            if(my_indicator_data[my_indicator_index].AC_data_buf[0] == 0X0C)
            {
                printf("======ZSQ[%d]====1.2\n", my_indicator_index);

                my_fun_display_fram_16(4, 22);
            }

        }
        //��������
        if(temp_status == 0x51)
        {

            for(ii = 0; ii < my_length; ii++)
            {
                my_indicator_alarm_data[my_indicator_index].AC_data_buf[ii] = my_CC1101_COM_Fram_buf[9 + ii];

            }


        }
        my_re = 1;
        my_cc1101_tx_wait_time = 3000;
    }
    else if(temp_status == 0x42 || temp_status == 0x52) //AC12T���������糡���벨������
    {
        for(ii = 0; ii < 24; ii++)
        {
            my_indicator_data[my_indicator_index].AC12T_ALL_Current_data_buf[ii] = my_CC1101_COM_Fram_buf[9 + ii];

        }
        for(ii = 0; ii < 24; ii++)
        {
            my_indicator_data[my_indicator_index].AC12T_ALL_dianchang_data_buf[ii] = my_CC1101_COM_Fram_buf[33 + ii];

        }
        for(ii = 0; ii < 24; ii++)
        {
            my_indicator_data[my_indicator_index].AC12T_HALF_Current_data_buf[ii] = my_CC1101_COM_Fram_buf[57 + ii];

        }

        //��������
        if(temp_status == 0x52)
        {
            for(ii = 0; ii < 24; ii++)
            {
                my_indicator_alarm_data[my_indicator_index].AC12T_ALL_Current_data_buf[ii] = my_CC1101_COM_Fram_buf[9 + ii];

            }
            for(ii = 0; ii < 24; ii++)
            {
                my_indicator_alarm_data[my_indicator_index].AC12T_ALL_dianchang_data_buf[ii] = my_CC1101_COM_Fram_buf[33 + ii];

            }
            for(ii = 0; ii < 24; ii++)
            {
                my_indicator_alarm_data[my_indicator_index].AC12T_HALF_Current_data_buf[ii] = my_CC1101_COM_Fram_buf[57 + ii];

            }


        }
        my_cc1101_tx_wait_time = 5000;
        my_re = 1;
    }
    else if(temp_status == 0x53 ) //AC_record
    {
        //my_indicator_record_data[my_indicator_index].my_wave_type = 0x01; //����ȫ��
        my_indicator_record_data[my_indicator_index].my_wave_alam = 0x02; //����
        my_indicator_record_data[my_indicator_index].my_wave_tx_status_I = 1;
        my_CC1101_record_wave_last_index = my_indicator_index; //��¼������µ�¼������ָʾ�����

        my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[0] = my_indicator_index + 1;
        my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[1] = 0;
        my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[2] = my_indicator_alarm_data[my_indicator_index].count_time[0];
        my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[3] = my_indicator_alarm_data[my_indicator_index].count_time[1];
        my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[4] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[0];
        my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[5] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[1];
        my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[6] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[2];
        my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[7] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[3];
        my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[8] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[4];
        my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[9] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[5];
        my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[10] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[6];



        for(ii = 0; ii < my_length; ii++)
            my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[ii + 11] = my_CC1101_COM_Fram_buf[9 + ii];

        my_cc1101_tx_wait_time = 5000;
        my_re = 1;
    }
    else if(temp_status == 0x54) //AC_record
    {
        //my_indicator_record_data[my_indicator_index].my_wave_type = 0x02; //�糡ȫ��
        my_indicator_record_data[my_indicator_index].my_wave_alam = 0x02; //����
        my_indicator_record_data[my_indicator_index].my_wave_tx_status_E = 1;
        my_CC1101_record_wave_last_index = my_indicator_index; //��¼������µ�¼������ָʾ�����

        my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[0] = my_indicator_index + 1;
        my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[1] = 0;
        my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[2] = my_indicator_alarm_data[my_indicator_index].count_time[0];
        my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[3] = my_indicator_alarm_data[my_indicator_index].count_time[1];
        my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[4] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[0];
        my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[5] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[1];
        my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[6] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[2];
        my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[7] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[3];
        my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[8] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[4];
        my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[9] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[5];
        my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[10] = my_indicator_alarm_data[my_indicator_index].RTC_time_buf[6];



        for(ii = 0; ii < my_length; ii++)
            my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[ii + 11] = my_CC1101_COM_Fram_buf[9 + ii];
        my_re = 1;
    }
//====���� ���� �������===========

//���ô�����ʾ����
#if Debug_Uart_out_DC_AC_DATA_Status==1

    double yy[7] = {0};
    //if(temp_status == 0x41 || temp_status == 0x51)
    if(temp_status == 0x42 || temp_status == 0x52)
    {
        //����״̬��2������·�ͽӵ�
        printf("========START DC2============\n");
        printf("---ZSQ=[%d]--TIME=[%d]-[%d]-[%d]--local time=[%d]\n", my_address, my_ZSQ_time_count[0], my_ZSQ_time_count[1], my_ZSQ_time_count[2], my_tim6_count);
        printf("ALARM:duanlu=[%XH],jiedi=[%XH]\n", my_indicator_data[my_indicator_index].duanlu_data, my_indicator_data[my_indicator_index].jiedi_data);
        printf("RSSI=%d\n", my_indicator_data[my_indicator_index].xinhao_db);
        //ֱ������7��
        for(ii = 0; ii < 7; ii++)
        {
            yy[ii] = (my_indicator_data[my_indicator_index].DC_data_buf[2 * ii] +
                      (my_indicator_data[my_indicator_index].DC_data_buf[2 * ii + 1] << 8)) / 10.0;
        }
        printf(" DC:Temp=%.2f,vbat=%.2f,vref=%.2f\n", yy[0], yy[1], yy[2]);
        printf(" DC:GANbat=%.2f,Zaixian=%.2f,sunbat=%.2f,Libat=%.2f\n", yy[3], yy[4], yy[5], yy[6]);
        //���Ͻ�������3��������ȫ�����糡�������벨
        for(ii = 0; ii < 3; ii++)
        {
            yy[ii] = (my_indicator_data[my_indicator_index].AC_data_buf[2 * ii] +
                      (my_indicator_data[my_indicator_index].AC_data_buf[2 * ii + 1] << 8)) / 10.0;
        }
        printf("AC:A=%.1f,E=%.1f,HA=%.1f\n", yy[0], yy[1], yy[2]);
        printf("========END DC============\n\n");
    }


#endif


#if Debug_Uart_out_AC12T_DATA_Status==1
    double xx2 = 0, xx3 = 0, xx4 = 0;
    if(temp_status == 0x42 || temp_status == 0x52)
    {
        printf("\n***AC12T data start*****\n");
        for(ii = 0; ii < 12; ii++)
        {
            xx2 = (my_indicator_data[my_indicator_index].AC12T_ALL_Current_data_buf[2 * ii] +
                   (my_indicator_data[my_indicator_index].AC12T_ALL_Current_data_buf[2 * ii + 1] << 8)) / 10.0;
            xx3 = (my_indicator_data[my_indicator_index].AC12T_ALL_dianchang_data_buf[2 * ii] +
                   (my_indicator_data[my_indicator_index].AC12T_ALL_dianchang_data_buf[2 * ii + 1] << 8)) / 10.0;
            xx4 = (my_indicator_data[my_indicator_index].AC12T_HALF_Current_data_buf[2 * ii] +
                   (my_indicator_data[my_indicator_index].AC12T_HALF_Current_data_buf[2 * ii + 1] << 8)) / 10.0;

            printf("\n A=%.2f,E=%.2f,HA=%.2f", xx2, xx3, xx4);
        }
        printf("\n***AC12T data END*****\n");
    }

#endif

#if Debug_Uart_out_960WAVE_DATA_Status==1

    double xx1 = 0;
    if(temp_status == 0x53) //����AC_record
    {
        printf("\n***960 data start��I*****\n");
        printf("my_indicator_index=%d\n", my_indicator_index + 1);
        for(ii = 0; ii < 960; ii++)
        {
            xx1 = (my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[2 * ii + 11]
                   + ((my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[2 * ii + 1 + 11]) << 8)) / 10.0;
            printf("\n %.1f", xx1);

        }
        printf("\n***960 data end---I*****\n");
    }
    if(temp_status == 0x54 ) //����AC_record
    {
        printf("\n***960 data start��E*****\n");
        printf("my_indicator_index=%d\n", my_indicator_index + 1);
        for(ii = 0; ii < 960; ii++)
        {
            xx1 = (my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[2 * ii + 11]
                   + ((my_indicator_record_data[my_indicator_index].my_wave_record_E_buf[2 * ii + 1 + 11]) << 8)) / 10.0;
            printf("\n %.1f", xx1);

        }
        printf("\n***960 data end---E*****\n");
    }
#endif


//===��ʾ���ֽ���======
    return my_re;


}


//CC1101��������֡
void my_fun_CC1101_send_heart_data(void)
{
    uint8_t *pt;
    uint16_t my_temp16_Broadaddress = 0XFF; //�㲥��ַ
    pt = my_cc1101_tx_buf;
    pt[0] = 0x10;
    pt[1] = 0x20; //IDΪ0X1F����ʾ����֡
    pt[2] = my_CC1101_chip_address; //����DTU��CC1101��ַ��Ϊ0XFE,��0XFDΪ�������ĵ�ַ��01,02,03---Ϊָʾ����ַ
    pt[3] = my_temp16_Broadaddress;

    pt[4] = my_tim6_count; //�յ���ָ���IDֵ
    pt[5] = (my_tim6_count >> 8);
    pt[6] = my_fun_101check_generate(pt, 1);
    pt[7] = 0x16;



    CC1101SendPacket_add( pt, 8,  BROADCAST, my_temp16_Broadaddress);
#if Debug_uart_out_cc1101_tx_data_status==1
    my_fun_display_buf_16(pt, 8, 1); //����ʹ��
#endif

}


//========================
//==================GPRS �Ի�����=============

void  my_fun_GPRS_TX_start1_server(void)
{

    //���ֽ�
    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;

    my_101_DIR = 0X80;
    my_101_PRM = 0X00;
    my_101_FC = 0X0B;

    my_usart1_tx_buf1[1] = (my_101_DIR | my_101_PRM | my_101_FC);
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //

    printf("my_GPRS send start1:");
    my_fun_display_buf_16(my_usart1_tx_buf1, 6, 1);

#if Use_GPRS_auto_re_ok==1
    uint8_t my_step = 0X00E1;
    xQueueSend(myQueue02Handle, &my_step, 100);
#endif
}

void  my_fun_GPRS_TX_start2_server(void)
{

    //���ֽ�
    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;

    my_101_DIR = 0X80;
    my_101_PRM = 0X00;
    my_101_FC = 0X00;

    my_usart1_tx_buf1[1] = (my_101_DIR | my_101_PRM | my_101_FC);
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //

    printf("my_GPRS send start1:");
    my_fun_display_buf_16(my_usart1_tx_buf1, 6, 1);

    //==========
    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    my_101_FC = 0X09;

    my_usart1_tx_buf1[1] = (my_101_DIR | my_101_PRM | my_101_FC);
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //
    printf("my_GPRS send start1:");
    my_fun_display_buf_16(my_usart1_tx_buf1, 6, 1);

#if Use_GPRS_auto_re_ok==1
    uint8_t my_step = 0X00E1;
    xQueueSend(myQueue02Handle, &my_step, 100);
#endif
}

void  my_fun_GPRS_TX_start3_server(void)
{

    //���ֽ�
    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;

    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    my_101_FC = 0X00;

    my_usart1_tx_buf1[1] = (my_101_DIR | my_101_PRM | my_101_FC);
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //

    printf("my_GPRS send start1:");
    my_fun_display_buf_16(my_usart1_tx_buf1, 6, 1);

#if Use_GPRS_auto_re_ok==1
    uint8_t my_step = 0X00E1;
    xQueueSend(myQueue02Handle, &my_step, 100);
#endif
}

void  my_fun_GPRS_TX_start4_server(void)
{



    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    my_101_FCB = 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;



    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = 0x0C; //
    my_usart1_tx_buf1[2] = 0x0C;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 0X46;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x04;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI; //���ͱ�ʶ����ʱ���ң��������Ϣ��
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //��Ϣ�����
    my_usart1_tx_buf1[9] = my_101_COT_low; //����ԭ��
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x00; //ң����Ϣ���׵�ַ
    my_usart1_tx_buf1[14] = 0x00;

    my_usart1_tx_buf1[15] = 0X00;



    my_usart1_tx_buf1[16] = my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_usart1_tx_buf1[17] = 0x16;

    my_at_senddata(my_usart1_tx_buf1); //

    printf("my_GPRS send start1:");
    my_fun_display_buf_16(my_usart1_tx_buf1, 6, 1);

#if Use_GPRS_auto_re_ok==1
    uint8_t my_step = 0X00E1;
    xQueueSend(myQueue02Handle, &my_step, 100);
#endif
}






//=======================

void  my_fun_GPRS_TX_start1(void)
{

    //���ֽ�
    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;
    my_101_DIR = 0X00;
    my_101_PRM = 0X40;
    my_101_FC = 0X09;

    my_usart1_tx_buf1[1] = (my_101_DIR | my_101_PRM | my_101_FC);
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //

    printf("my_GPRS send start1:");
    my_fun_display_buf_16(my_usart1_tx_buf1, 6, 1);

#if Use_GPRS_auto_re_ok==1
    uint8_t my_step = 0X00E1;
    xQueueSend(myQueue02Handle, &my_step, 100);
#endif
}
void  my_fun_GPRS_TX_start2(void)
{

    //���ֽ�
    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;

    my_101_DIR = 0X00;
    my_101_PRM = 0X40;
    my_101_FC = 0X00;

    my_usart1_tx_buf1[1] = (my_101_DIR | my_101_PRM | my_101_FC);
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //

    printf("my_GPRS send start2:");
    my_fun_display_buf_16(my_usart1_tx_buf1, 6, 1);
#if Use_GPRS_auto_re_ok==1
    uint8_t my_step = 0X00E2;
    xQueueSend(myQueue02Handle, &my_step, 100);
#endif

}
void  my_fun_GPRS_TX_start3(void)
{


    my_101_DIR = 0X00;
    my_101_PRM = 0X40;
    my_101_FCB = 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;



    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = 0x0C; //0X24 ��36����Ϣ�壬ÿ����Ϣ��3���ֽ�
    my_usart1_tx_buf1[2] = 0x0C;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 0X46;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0X00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x04;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI; //���ͱ�ʶ����ʱ���ң��������Ϣ��
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //��Ϣ�����
    my_usart1_tx_buf1[9] = my_101_COT_low; //����ԭ��
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x00; //ң����Ϣ���׵�ַ
    my_usart1_tx_buf1[14] = 0x00;

    my_usart1_tx_buf1[15] = 0X00;


    my_usart1_tx_buf1[16] = my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_usart1_tx_buf1[17] = 0x16;

    my_at_senddata(my_usart1_tx_buf1); //

    printf("my_GPRS send start1:");
    my_fun_display_buf_16(my_usart1_tx_buf1, 6, 1);

#if Use_GPRS_auto_re_ok==1
    uint8_t my_step = 0X00E1;
    xQueueSend(myQueue02Handle, &my_step, 100);
#endif
}


//=============
void  my_fun_GPRS_TX_OK_80(void)
{

    //���ֽ�
    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;

    my_usart1_tx_buf1[1] = 0x80;
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //

    printf("my_GPRS send ok:");
    my_fun_display_buf_16(my_usart1_tx_buf1, 6, 1);


}



//==============

void  my_fun_GPRS_TX_OK(void)
{

    //���ֽ�
    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;

    my_usart1_tx_buf1[1] = 0x00;
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //

    printf("my_GPRS send ok:");
    my_fun_display_buf_16(my_usart1_tx_buf1, 6, 1);

#if Use_GPRS_auto_re_ok==1
    uint8_t my_step = 0;
    if(my_GPRS_all_step == 0XE400)
    {
        my_step = 0X00E4;
        xQueueSend(myQueue02Handle, &my_step, 100);
    }
    if(my_GPRS_all_step == 0XE500)
    {

        my_step = 0X00E5;
        xQueueSend(myQueue02Handle, &my_step, 100);
    }
#endif
}

void  my_fun_GPRS_TX_test1(void)
{


    my_GPRS_heart_count++;

    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;

    my_usart1_tx_buf1[1] = 0xD2;
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);


    my_at_senddata(my_usart1_tx_buf1); //

    printf("my_GPRS send heart data:");
    my_fun_display_buf_16(my_usart1_tx_buf1, 6, 1);

#if Use_GPRS_auto_re_ok==1
    uint8_t my_step = 0X001F;
    xQueueSend(myQueue02Handle, &my_step, 100);
#endif

}


//===
void  my_fun_GPRS_TX_RTC_data(void)
{
    //���ֽ�
    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;
    my_101_DIR = 0X80;
    my_101_PRM = 0X00;
    my_101_FC = 0X00;

    my_usart1_tx_buf1[1] = (my_101_DIR | my_101_PRM | my_101_FC);
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //

    //==========================

    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;



    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = 0x12; //0X24 ��36����Ϣ�壬ÿ����Ϣ��3���ֽ�
    my_usart1_tx_buf1[2] = 0x12;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 103;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x07;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI;
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //��Ϣ�����
    my_usart1_tx_buf1[9] = my_101_COT_low; //����ԭ��
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x00; //ң����Ϣ���׵�ַ
    my_usart1_tx_buf1[14] = 0x00;

    HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN); //��ȡRTCʱ��
    HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);

    my_usart1_tx_buf1[15] =  (my_RTC_time.Seconds * 1000); //���ݲ���
    my_usart1_tx_buf1[16] = ((my_RTC_time.Seconds * 1000) >> 8);
    my_usart1_tx_buf1[17] = my_RTC_time.Minutes;
    my_usart1_tx_buf1[18] = my_RTC_time.Hours;
    my_usart1_tx_buf1[19] = (my_RTC_date.Date + (my_RTC_date.WeekDay << 5));
    my_usart1_tx_buf1[20] = my_RTC_date.Month;
    my_usart1_tx_buf1[21] = my_RTC_date.Year;



    my_usart1_tx_buf1[22] = my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_usart1_tx_buf1[23] = 0x16;

    my_at_senddata(my_usart1_tx_buf1); //

    //=================

}

void  my_fun_GPRS_TX_RTC_data_read(void)
{
    //���ֽ�
    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;
    my_101_DIR = 0X80;
    my_101_PRM = 0X00;
    my_101_FC = 0X00;

    my_usart1_tx_buf1[1] = (my_101_DIR | my_101_PRM | my_101_FC);
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //

    //==========================

    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;



    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = 0x12; //
    my_usart1_tx_buf1[2] = 0x12;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 103;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x05;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI;
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //��Ϣ�����
    my_usart1_tx_buf1[9] = my_101_COT_low; //����ԭ��
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

    my_usart1_tx_buf1[13] = 0x00; //ң����Ϣ���׵�ַ
    my_usart1_tx_buf1[14] = 0x00;


    HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN); //��ȡRTCʱ��
    HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);

    my_usart1_tx_buf1[15] =  (my_RTC_time.Seconds * 1000); //���ݲ���
    my_usart1_tx_buf1[16] = ((my_RTC_time.Seconds * 1000) >> 8);
    my_usart1_tx_buf1[17] = my_RTC_time.Minutes;
    my_usart1_tx_buf1[18] = my_RTC_time.Hours;
    my_usart1_tx_buf1[19] = (my_RTC_date.Date + (my_RTC_date.WeekDay << 5));
    my_usart1_tx_buf1[20] = my_RTC_date.Month;
    my_usart1_tx_buf1[21] = my_RTC_date.Year;



    my_usart1_tx_buf1[22] = my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_usart1_tx_buf1[23] = 0x16;

    my_at_senddata(my_usart1_tx_buf1); //

    //=================

}



/*
���ܣ�GPRS������������
*/


void  my_fun_GPRS_TX_CYC1(void)  //���������ź�
{
#if Use_indicatour_cyc_test_satus==1
    //����ָʾ��ģ���ź�
    uint8_t i = 0, jj = 0, xx = 1;;
    for(i = 0; i < my_indicator_count; i++)
    {
        my_indicator_data[i].duanlu_data = 0x21;
        my_indicator_data[i].jiedi_data = 0x31;
        for(jj = 0; jj < 14; jj++)
        {
            my_indicator_data[i].DC_data_buf[jj] = xx++;
        }
        for(jj = 0; jj < 6; jj++)
        {
            my_indicator_data[i].AC_data_buf[jj] = xx++;
        }
        for(jj = 0; jj < 24; jj++)
        {
            my_indicator_data[i].AC12T_ALL_Current_data_buf[jj] = xx++;
        }
        for(jj = 0; jj < 24; jj++)
        {
            my_indicator_data[i].AC12T_ALL_dianchang_data_buf[jj] = xx++;
        }
        for(jj = 0; jj < 24; jj++)
        {
            my_indicator_data[i].AC12T_HALF_Current_data_buf[jj] = xx++;
        }
        for(jj = 0; jj < 7; jj++)
        {
            my_indicator_data[i].RTC_time_buf[jj] = xx++;
        }

        my_indicator_data[i].data_type = xx++;

        for(jj = 0; jj < 2; jj++)
        {
            my_indicator_data[i].count_time[jj] = xx++;
        }

        my_indicator_data[i].xinhao_db = xx++;
        my_indicator_data[i].TX_status = xx++;

    }
#endif


    //====END==============
    wdz_GPRS_string_to_array(TX_GPRS_101_testdata, my_usart1_tx_buf1); //����ָ��

    my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
    my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1);

    printf("\n\nmy_GPRS send CYC data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

#if Use_GPRS_auto_re_ok==1
    uint8_t my_step = 0X00B1;
    xQueueSend(myQueue02Handle, &my_step, 100);
#endif

}



void  my_fun_GPRS_TX_CYC2(void)  //����ң����,���٣���ʱ��
{
    uint8_t ii = 0, kk = 0, my_shibiao_time = 0;
    my_gprs_generate_101single_data(1, my_usart1_tx_buf1, my_shibiao_time, 20); //�ܹ���56����Ϣ���ַ
    for(ii = 1; ii <= MY_GPRS_Call_Single_data_number; ii++)
    {
        my_usart1_tx_buf1[14 + ii] = 0X00; //ң��˫����Ϣ 01Ϊ������0X02Ϊ����

    }

    my_usart1_tx_buf1[14 + 1] = MY_yaoxin_status_OK;
    my_usart1_tx_buf1[14 + 2] = MY_yaoxin_status_OK;
    //A��״̬

    for(kk = 0; kk < 3; kk++)
    {
        //ͨ�Ż���
        if(my_indicator_data[kk].xinhao_db > 75)
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 1] = MY_yaoxin_status_ERROR; //�ɼ���Ԫͨ��״̬
        else
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 1] = MY_yaoxin_status_OK; //�ɼ���Ԫͨ��״̬

        //��·״̬
        if(my_indicator_data[kk].duanlu_data == 0X01)
        {
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 2] = MY_yaoxin_status_OK; //˲ʱ�Զ�·
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 3] = MY_yaoxin_status_ERROR; 	//�����Զ�·

        }
        else if(my_indicator_data[kk].duanlu_data == 0X41 || my_indicator_data[kk].duanlu_data == 0X81)
        {
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 2] = MY_yaoxin_status_ERROR; //˲ʱ�Զ�·
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 3] = MY_yaoxin_status_OK; 	//�����Զ�·

        }
        else
        {
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 2] = MY_yaoxin_status_OK; //˲ʱ�Զ�·
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 3] = MY_yaoxin_status_OK; 	//�����Զ�·

        }
        //�ӵ�״̬
        if(my_indicator_data[kk].jiedi_data == 0X01)
        {
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 4] = MY_yaoxin_status_ERROR; //�ӵ�
        }
        else
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 4] = MY_yaoxin_status_OK;

        my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 5] = MY_yaoxin_status_OK; //�¶ȳ���
        my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 6] = MY_yaoxin_status_OK; //��������

        //���״̬
        float yy = 0;
        yy = (my_indicator_data[kk].DC_data_buf[12] + (my_indicator_data[kk].DC_data_buf[13] << 8)) / 10.0;
        if(yy < 3.8)
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 7] = MY_yaoxin_status_ERROR;
        else
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 7] = MY_yaoxin_status_OK; //���Ƿѹ����


        //��·ͣ��״̬
        if(my_indicator_data[kk].Line_STOP == 2)
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 8] = MY_yaoxin_status_ERROR; //��·�е�,ͣ����
        else
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 8] = MY_yaoxin_status_OK; //�������е�

    }


    //ʱ��
    if(my_shibiao_time == 7)
    {
        my_usart1_tx_buf1[14 + MY_GPRS_Call_Single_data_number + 1] = (my_RTC_time.Seconds * 1000);
        my_usart1_tx_buf1[14 + MY_GPRS_Call_Single_data_number + 2] = ((my_RTC_time.Seconds * 1000) >> 8);
        my_usart1_tx_buf1[14 + MY_GPRS_Call_Single_data_number + 3] = my_RTC_time.Minutes;
        my_usart1_tx_buf1[14 + MY_GPRS_Call_Single_data_number + 4] = my_RTC_time.Hours;
        my_usart1_tx_buf1[14 + MY_GPRS_Call_Single_data_number + 5] = my_RTC_date.Date || (my_RTC_date.WeekDay << 5);
        my_usart1_tx_buf1[14 + MY_GPRS_Call_Single_data_number + 6] = my_RTC_date.Month;
        my_usart1_tx_buf1[14 + MY_GPRS_Call_Single_data_number + 7] = my_RTC_date.Year;
    }


    wdz_GPRS_101check_generate(my_usart1_tx_buf1);
    my_at_senddata(my_usart1_tx_buf1);

    printf("my_GPRS send CYC data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);
}

void  my_fun_GPRS_TX_CYC3(void)  //ң�⣬����,��ʱ��
{
    uint8_t jj = 0, shibiao = 0; //ʱ�곤��
    union MY_float my_AC_data, my_vol_data, my_temperature_data;
    my_gprs_generate_101analog_data(1, my_usart1_tx_buf1, shibiao, 20);

    //
    for(jj = 0; jj < MY_GPRS_Call_Analog_data_number * 5; jj++)
    {
        my_usart1_tx_buf1[15 + jj] = 0;
    }

    for(jj = 0; jj < 3; jj++)
    {
        my_AC_data.value = (my_indicator_data[jj].AC_data_buf[0] + (my_indicator_data[jj].AC_data_buf[1] << 8)) / 10.0;
        my_vol_data.value = (my_indicator_data[jj].DC_data_buf[12] + (my_indicator_data[jj].DC_data_buf[13] << 8)) / 10.0;
        my_temperature_data.value = (my_indicator_data[jj].DC_data_buf[0] + (my_indicator_data[jj].DC_data_buf[1] << 8)) / 10.0;

        my_usart1_tx_buf1[15 + jj * 5 + 0] = my_AC_data.byte[0];
        my_usart1_tx_buf1[15 + jj * 5 + 1] = my_AC_data.byte[1];
        my_usart1_tx_buf1[15 + jj * 5 + 2] = my_AC_data.byte[2];
        my_usart1_tx_buf1[15 + jj * 5 + 3] = my_AC_data.byte[3];
        my_usart1_tx_buf1[15 + jj * 5 + 4] = 0;

        my_usart1_tx_buf1[15 + 20 + jj * 5 + 0] = my_temperature_data.byte[0];
        my_usart1_tx_buf1[15 + 20 + jj * 5 + 1] = my_temperature_data.byte[1];
        my_usart1_tx_buf1[15 + 20 + jj * 5 + 2] = my_temperature_data.byte[2];
        my_usart1_tx_buf1[15 + 20 + jj * 5 + 3] = my_temperature_data.byte[3];
        my_usart1_tx_buf1[15 + 20 + jj * 5 + 4] = 0;

        my_usart1_tx_buf1[15 + 40 + jj * 5 + 0] = my_vol_data.byte[0];
        my_usart1_tx_buf1[15 + 40 + jj * 5 + 1] = my_vol_data.byte[1];
        my_usart1_tx_buf1[15 + 40 + jj * 5 + 2] = my_vol_data.byte[2];
        my_usart1_tx_buf1[15 + 40 + jj * 5 + 3] = my_vol_data.byte[3];
        my_usart1_tx_buf1[15 + 40 + jj * 5 + 4] = 0;


    }

//        printf("ZSQ[%d]-A=%.1f,E=%.1f\n", jj + 1,
//               (my_indicator_data[jj].AC_data_buf[1] * 256 + my_indicator_data[jj].AC_data_buf[0]) / 10.0,
//               (my_indicator_data[jj].AC_data_buf[3] * 256 + my_indicator_data[jj].AC_data_buf[2]) / 10.0
//              );

    //ϵͳ�����������������ݴ���  0XFB��ָʾ������Ϊ0X11
    if(my_system_restart_status == 1)
    {
    }
    my_system_restart_status = 0;


    wdz_GPRS_101check_generate(my_usart1_tx_buf1);
    my_at_senddata(my_usart1_tx_buf1);

    printf("my_GPRS send CYC data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);


}


void  my_fun_GPRS_TX_CYC4(void)  //  ����
{
    union MY_float my_DTU_vol_data;
    uint8_t my_cot = 0;
    my_cot = 20;

    my_gprs_generate_101MCU_data(1, my_usart1_tx_buf1, my_cot);

    my_DTU_vol_data.value = MY_Bat_value / 10.0;
    my_usart1_tx_buf1[15 + 0] = my_DTU_vol_data.byte[0];
    my_usart1_tx_buf1[15 + 1] = my_DTU_vol_data.byte[1];
    my_usart1_tx_buf1[15 + 2] = my_DTU_vol_data.byte[2];
    my_usart1_tx_buf1[15 + 3] = my_DTU_vol_data.byte[3];
    my_usart1_tx_buf1[15 + 4] = 0;


    wdz_GPRS_101check_generate(my_usart1_tx_buf1);

    my_at_senddata(my_usart1_tx_buf1);

    printf("my_GPRS send CYC data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

}

void  my_fun_GPRS_TX_CYC5(void)  //  ���ٽ���
{
    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;

    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = 0x0C; //
    my_usart1_tx_buf1[2] = 0x0C;
    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73
    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 0X46;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 10;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI; //���ͱ�ʶ����ʱ���ң��������Ϣ��
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //��Ϣ�����
    my_usart1_tx_buf1[9] = my_101_COT_low; //����ԭ��
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x00; //ң����Ϣ���׵�ַ
    my_usart1_tx_buf1[14] = 0x00;

    my_usart1_tx_buf1[15] = 20;

    my_usart1_tx_buf1[16] = my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_usart1_tx_buf1[17] = 0x16;

    my_at_senddata(my_usart1_tx_buf1); //
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

}








/*

*/
void  my_fun_GPRS_TX_RESET(void)  //  DTU����
{
    //���ֽ�
    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;
    my_101_DIR = 0X80;
    my_101_PRM = 0X00;
    my_101_FC = 0X00;

    my_usart1_tx_buf1[1] = (my_101_DIR | my_101_PRM | my_101_FC);
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //

    //==========================

    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;



    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = 0x0C; //1����Ϣ�壬��Ϣֵ1���ֽ�
    my_usart1_tx_buf1[2] = 0x0C;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 104;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x07;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI;
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //��Ϣ�����
    my_usart1_tx_buf1[9] = my_101_COT_low; //����ԭ��
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

    my_usart1_tx_buf1[13] = 0x00; //ң����Ϣ���׵�ַ
    my_usart1_tx_buf1[14] = 0x00;

    my_usart1_tx_buf1[15] = 1;

    my_usart1_tx_buf1[16] = my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_usart1_tx_buf1[17] = 0x16;

    my_at_senddata(my_usart1_tx_buf1); //


}

void  my_fun_GPRS_TX_changeparameter(void)  //  DTU����
{
    //68 0B 0B 68 53 01 00 69 01 07 01 00 00 00 01 C7 16
    uint16_t my_temp16 = 0;
    my_temp16 = USART1_my_frame[13];
    my_temp16 = (my_temp16 << 8) + USART1_my_frame[12];
    if(my_GPRS_all_step == 0X8100 && (my_temp16 == 0x5001 || my_temp16 == 0x5002 || my_temp16 == 0x5003))
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_changeparameter_ACK_2byte_data, my_usart1_tx_buf1); //
        if(my_temp16 == 0x5001)
        {
            my_usart1_tx_buf1[12] = 0x01;
            my_usart1_tx_buf1[14] = (uint8_t)(DTU_ADDRESS);
            my_usart1_tx_buf1[15] = (uint8_t)(DTU_ADDRESS >> 8);

        }
        else if (my_temp16 == 0x5002)
        {
            my_usart1_tx_buf1[12] = 0x02;
            my_usart1_tx_buf1[14] = (uint8_t)(MY_M_speed_heart);
            my_usart1_tx_buf1[15] = (uint8_t)(MY_M_speed_heart >> 8);
        }
        else if (my_temp16 == 0x5003)
        {
            my_usart1_tx_buf1[12] = 0x03;
            my_usart1_tx_buf1[14] = (uint8_t)(MY_M_speed_cyc);
            my_usart1_tx_buf1[15] = (uint8_t)(MY_M_speed_cyc >> 8);
        }

        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send setparameter_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }
    else if(my_GPRS_all_step == 0X8100 && my_temp16 == 0x5004 )
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_changeparameter_ACK_6byte_data, my_usart1_tx_buf1); //
        my_usart1_tx_buf1[12] = 0x04;

        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send reset_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }
    else if(my_GPRS_all_step == 0X8100 &&  (my_temp16 == 0x5010 || my_temp16 == 0x5011 || my_temp16 == 0x5012) )
    {
        my_usart1_tx_buf1[0] = 0x10;
        my_usart1_tx_buf1[5] = 0x16;

        my_usart1_tx_buf1[1] = 0x00;
        my_usart1_tx_buf1[2] = DTU_ADDRESS;
        my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
        my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_at_senddata(my_usart1_tx_buf1); //
    }
    else if(my_GPRS_all_step == 0X8100 &&  (my_temp16 >= 0x5031 && my_temp16 <= 0x5050) )
    {
        my_usart1_tx_buf1[0] = 0x10;
        my_usart1_tx_buf1[5] = 0x16;

        my_usart1_tx_buf1[1] = 0x00;
        my_usart1_tx_buf1[2] = DTU_ADDRESS;
        my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
        my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_at_senddata(my_usart1_tx_buf1); //
    }









    //===============================================
    if(my_temp16 == 0X5001 || my_temp16 == 0X5004)
    {
        HAL_NVIC_SystemReset();
    }

}

void  my_fun_GPRS_TX_TurnLED(void)  //  ����
{
    //68 0B 0B 68 53 01 00 69 01 07 01 00 00 00 01 C7 16
    uint16_t my_temp16 = 0, my_temp162 = 0;
    uint8_t my_temp8 = 0;
    my_temp16 = USART1_my_frame[13];
    my_temp16 = (my_temp16 << 8) + USART1_my_frame[12];
    my_temp8 =	USART1_my_frame[14];
    my_temp162 = USART1_my_frame[16];
    my_temp162 = (my_temp162 << 8) + USART1_my_frame[15];

    if(my_GPRS_all_step == 0X6100 && my_temp16 == 0x6001 && (my_temp162 >= 0x6002))
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_turn_lend_data, my_usart1_tx_buf1); //

        my_usart1_tx_buf1[12] = 0x01;
        my_usart1_tx_buf1[13] = 0x60;
        my_usart1_tx_buf1[14] = my_temp8;
        my_usart1_tx_buf1[15] = my_temp162;
        my_usart1_tx_buf1[16] = (uint8_t)(my_temp162 >> 8);
        my_usart1_tx_buf1[17] = 0X00;



        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }
    //===============================================
    if(my_temp16 == 0X6004)
    {
        HAL_NVIC_SystemReset();
    }

}
//��ѯ����
void  my_fun_GPRS_TX_query_data(void)  //  ��ѯ����
{
    //68 0B 0B 68 53 01 00 69 01 07 01 00 00 00 01 C7 16
    uint16_t my_temp16 = 0, my_temp162 = 0;



    my_temp16 = USART1_my_frame[13];
    my_temp16 = (my_temp16 << 8) + USART1_my_frame[12];


    if(my_GPRS_all_step == 0X7100 &&  query_data == 1)
    {
        my_fun_GPRS_TX_OK();
        wdz_GPRS_string_to_array(TX_GPRS_101_Chaxun_data, my_usart1_tx_buf1); //
        my_temp162 = DTU_ADDRESS;      //DTU��ַ
        my_usart1_tx_buf1[12] = 0x01;
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = my_temp162;
        my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);;
        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }

    else if(my_GPRS_all_step == 0X7200 && query_data == 2)
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_Chaxun_data, my_usart1_tx_buf1); //
        my_temp162 = MY_M_speed_heart; //����֡���
        my_usart1_tx_buf1[12] = 0x02;
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = my_temp162;
        my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);;
        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_GPRS_all_step = 0X7100;

        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }

    else if(my_GPRS_all_step == 0X7200 && query_data == 3)
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_Chaxun_data, my_usart1_tx_buf1); //
        my_temp162 = MY_M_speed_cyc; //���ڼ��
        my_usart1_tx_buf1[12] = 0x03;
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = my_temp162;
        my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);
        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_GPRS_all_step = 0X7100;

        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }

    else if(my_GPRS_all_step == 0X7200 && query_data == 4)
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_Chaxun_6bye_data, my_usart1_tx_buf1); //

        my_usart1_tx_buf1[12] = 0x04;
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = MY_IP[0];
        my_usart1_tx_buf1[15] = MY_IP[1];
        my_usart1_tx_buf1[16] = MY_IP[2];
        my_usart1_tx_buf1[17] = MY_IP[3];
        my_usart1_tx_buf1[18] = MY_PORT;
        my_usart1_tx_buf1[19] = (uint8_t)(MY_PORT >> 8);
        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_GPRS_all_step = 0X7100;

        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }
    else if(my_GPRS_all_step == 0X7200 && query_data == 5)
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_Chaxun_7bye_data, my_usart1_tx_buf1); //
        //DTU RTCʱ��
        my_usart1_tx_buf1[12] = 0x19;
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = my_RTC_time.Seconds;
        my_usart1_tx_buf1[15] = 00;
        my_usart1_tx_buf1[16] = my_RTC_time.Minutes;
        my_usart1_tx_buf1[17] = my_RTC_time.Hours;
        my_usart1_tx_buf1[18] = my_RTC_date.Date;
        my_usart1_tx_buf1[19] = my_RTC_date.Month;
        my_usart1_tx_buf1[20] = my_RTC_date.Year;

        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }
    else if(my_GPRS_all_step == 0X7300 && query_data > 5) //����
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_Chaxun_data, my_usart1_tx_buf1); //

        my_usart1_tx_buf1[12] = 0x20;  //����
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = 00;
        my_usart1_tx_buf1[15] = 00;
        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }

    //===============================================


}

uint8_t my_query_index = 0;
void  my_fun_GPRS_TX_query_data2(void)  //  ��ѯ����
{
    //68 0B 0B 68 53 01 00 69 01 07 01 00 00 00 01 C7 16
    uint16_t my_temp16 = 0, my_temp162 = 0;

    my_temp16 = USART1_my_frame[13];
    my_temp16 = (my_temp16 << 8) + USART1_my_frame[12];



    if(my_GPRS_all_step == 0X7700 &&  query_data2 == 1)
    {
        my_fun_GPRS_TX_OK();
        wdz_GPRS_string_to_array(TX_GPRS_101_Chaxun_data, my_usart1_tx_buf1); //

        my_temp162 = my_indicator_parameter_data[my_query_index].P1_300A_mul;
        my_usart1_tx_buf1[12] = 0x31;
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = my_temp162;
        my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);;
        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }

    else if(my_GPRS_all_step == 0X7800 )
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_Chaxun_data, my_usart1_tx_buf1); //
        if(query_data2 == 2)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P2_Add_value; //����֡���
            my_usart1_tx_buf1[12] = 0x32;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);
        }
        else if(query_data2 == 3)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P3_E_mul; //����֡���
            my_usart1_tx_buf1[12] = 0x33;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 4)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P4_E_mul2; //����֡���
            my_usart1_tx_buf1[12] = 0x34;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 5)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P5_I_deta; //����֡���
            my_usart1_tx_buf1[12] = 0x35;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 6)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P6_I_max; //����֡���
            my_usart1_tx_buf1[12] = 0x36;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 7)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P7_I_0min; //����֡���
            my_usart1_tx_buf1[12] = 0x37;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 8)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P8_E_down_baifenbi; //����֡���
            my_usart1_tx_buf1[12] = 0x38;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 9)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P9_E_0min; //����֡���
            my_usart1_tx_buf1[12] = 0x39;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 10)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P10_E_down_min; //����֡���
            my_usart1_tx_buf1[12] = 0x3A;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 11)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P11_V_Libat; //����֡���
            my_usart1_tx_buf1[12] = 0x3B;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 12)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P12_CYC_time_MIN; //����֡���
            my_usart1_tx_buf1[12] = 0x3C;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 13)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P13_CYC_time_MAX; //����֡���
            my_usart1_tx_buf1[12] = 0x3D;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 14)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P14_sleep_time; //����֡���
            my_usart1_tx_buf1[12] = 0x3E;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 15)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P15_awake_time; //����֡���
            my_usart1_tx_buf1[12] = 0x3F;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 16)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P16; //����֡���
            my_usart1_tx_buf1[12] = 0x40;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 17)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P17_reset_LED_time; //����֡���
            my_usart1_tx_buf1[12] = 0x41;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 18)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P18_reset_sys_time; //����֡���
            my_usart1_tx_buf1[12] = 0x42;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }


        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        if(query_data2 != 18)
        {
            my_GPRS_all_step = 0X7700;
        }

        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }


    else if(my_GPRS_all_step == 0X7300 ) //����
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_Chaxun_data, my_usart1_tx_buf1); //

        my_usart1_tx_buf1[12] = 0x21;  //����
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = 00;
        my_usart1_tx_buf1[15] = 00;
        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }

    //===============================================


}


//�����ź�ǿ��
extern uint8_t MY_AT_CSQ_Value;
void  my_fun_GPRS_TX_xinhaoqiangdu(void)  //
{
    //68 0B 0B 68 53 01 00 69 01 07 01 00 00 00 01 C7 16

    wdz_GPRS_string_to_array(TX_GPRS_101_xinhaoqiangdu_data, my_usart1_tx_buf1); //


    my_usart1_tx_buf1[12] = 0x60;
    my_usart1_tx_buf1[13] = 0x50;
    my_usart1_tx_buf1[14] = MY_AT_CSQ_Value;

    my_usart1_tx_buf1[15] = 0x61;
    my_usart1_tx_buf1[16]	= 0x50;
    my_usart1_tx_buf1[17] = my_indicator_data[0].xinhao_db; //CC1101�ź�ǿ��
    my_usart1_tx_buf1[18] = my_indicator_data[1].xinhao_db;;
    my_usart1_tx_buf1[19] = my_indicator_data[2].xinhao_db;;

    //�޸�֡ DTU��ַ����Ϣ�������
    my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
    my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_at_senddata(my_usart1_tx_buf1);
    printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);
}


//���ͼ���ֵ��RTC
void  my_fun_GPRS_TX_TIME_RTC(void)  //
{
    if(my_GPRS_all_step == 0X3100 )
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_count_time_data, my_usart1_tx_buf1); //

        my_usart1_tx_buf1[12] = 0x70;
        my_usart1_tx_buf1[13] = 0x50;//my_RTC_date
        my_usart1_tx_buf1[14] = my_tim6_count;
        my_usart1_tx_buf1[15] = (uint8_t)(my_tim6_count >> 8);
        my_usart1_tx_buf1[16] = my_RTC_time.Seconds;
        my_usart1_tx_buf1[17]	= 0X00;
        my_usart1_tx_buf1[18] = my_RTC_time.Minutes;
        my_usart1_tx_buf1[19] = my_RTC_time.Hours;
        my_usart1_tx_buf1[20] = my_RTC_date.Date;
        my_usart1_tx_buf1[21] = my_RTC_date.Month;
        my_usart1_tx_buf1[22] = my_RTC_date.Year;

        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send RTC_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }
    //===============================================

}

/*

���ܣ��������ݷ���
*/
void  my_fun_GPRS_TX_ALarm_data(void)
{
    uint8_t ii = 0;
    uint8_t inf_add = 0;
    uint8_t alarm_data = 00;
    //��������ģ������
#if Use_indicatour_cyc_test_satus==1
    uint8_t i = 0, jj = 0, xx = 1;;
    for(i = 0; i < my_indicator_count; i++)
    {
        my_indicator_alarm_data[i].duanlu_data = 0x21;
        my_indicator_alarm_data[i].jiedi_data = 0x31;
        for(jj = 0; jj < 14; jj++)
        {
            my_indicator_alarm_data[i].DC_data_buf[jj] = xx++;
        }
        for(jj = 0; jj < 6; jj++)
        {
            my_indicator_alarm_data[i].AC_data_buf[jj] = xx++;
        }
        for(jj = 0; jj < 24; jj++)
        {
            my_indicator_alarm_data[i].AC12T_ALL_Current_data_buf[jj] = xx++;
        }
        for(jj = 0; jj < 24; jj++)
        {
            my_indicator_alarm_data[i].AC12T_ALL_dianchang_data_buf[jj] = xx++;
        }
        for(jj = 0; jj < 24; jj++)
        {
            my_indicator_alarm_data[i].AC12T_HALF_Current_data_buf[jj] = xx++;
        }
        for(jj = 0; jj < 7; jj++)
        {
            my_indicator_alarm_data[i].RTC_time_buf[jj] = xx++;
        }

        my_indicator_alarm_data[i].data_type = xx++;

        for(jj = 0; jj < 2; jj++)
        {
            my_indicator_alarm_data[i].count_time[jj] = xx++;
        }

        my_indicator_alarm_data[i].xinhao_db = xx++;
        my_indicator_alarm_data[i].TX_status = xx++;

    }

#endif

    //ģ������OK


    if(my_GPRS_all_step == 0X9100 ) //ң��-��ʱ��
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_ALarm_single_notime_data, my_usart1_tx_buf1); //

        if(my_indicator_tx_index >= my_indicator_count)  //my_indicator_tx_index��0��ʼ
        {
            //my_GPRS_all_step=0;
            return;
        }
        ii = my_indicator_tx_index;
        if(my_indicator_alarm_data[ii].TX_status_duanlu >= 0X01)
        {
            inf_add = ii * 2; //��·ң�ŵ�ַ��
            alarm_data = my_indicator_alarm_data[ii].duanlu_data;
        }
        else if(my_indicator_alarm_data[ii].TX_status_jiedi >= 0x01)
        {
            inf_add = ii * 2 + 1; //�ӵ�ң�ŵ�ַ��
            alarm_data = my_indicator_alarm_data[ii].jiedi_data;

        }
        my_usart1_tx_buf1[12] = inf_add;
        my_usart1_tx_buf1[13] = 0x00;//
        my_usart1_tx_buf1[14] = alarm_data;




        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);


    }
    else if(my_GPRS_all_step == 0X9200) //ң��--��ʱ��
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_ALarm_single_with_time_data, my_usart1_tx_buf1); //
        ii = my_indicator_tx_index;
        if(ii < my_indicator_count)
        {

            if(my_indicator_alarm_data[ii].TX_status_duanlu >= 0X01)
            {
                inf_add = ii * 2;
                alarm_data = my_indicator_alarm_data[ii].duanlu_data;
            }
            else if(my_indicator_alarm_data[ii].TX_status_jiedi >= 0x01)
            {
                inf_add = ii * 2 + 1;
                alarm_data = my_indicator_alarm_data[ii].jiedi_data;

            }
            my_usart1_tx_buf1[12] = inf_add;
            my_usart1_tx_buf1[13] = 0x00;//
            my_usart1_tx_buf1[14] = alarm_data;

            my_usart1_tx_buf1[15] = my_RTC_time.Seconds;
            my_usart1_tx_buf1[16]	= 0X00;
            my_usart1_tx_buf1[17] = my_RTC_time.Minutes;
            my_usart1_tx_buf1[18] = my_RTC_time.Hours;
            my_usart1_tx_buf1[19] = my_RTC_date.Date;
            my_usart1_tx_buf1[20] = my_RTC_date.Month;
            my_usart1_tx_buf1[21] = my_RTC_date.Year;

            //�޸�֡ DTU��ַ����Ϣ�������
            my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
            my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
            my_fun_101check_generate(my_usart1_tx_buf1, 0);

            my_at_senddata(my_usart1_tx_buf1);
            printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
            my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

            return;

        }

    }

    else if(my_GPRS_all_step == 0X9300)  // ң��-���Ͻ���ֵ
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_ALarm_single_AC_data, my_usart1_tx_buf1); //
        if((int)my_indicator_tx_index >= 0 && my_indicator_tx_index < 10)
        {
            my_usart1_tx_buf1[12] = my_indicator_tx_index + 1;
            my_usart1_tx_buf1[13] = 0X44;

        }
        for(ii = 0; ii < 3; ii++)
        {
            if(ii == my_indicator_tx_index)  //�����������
            {

                my_usart1_tx_buf1[14 + ii * 6] = my_indicator_alarm_data[ii].AC_data_buf[0];
                my_usart1_tx_buf1[15 + ii * 6] = my_indicator_alarm_data[ii].AC_data_buf[1];
                my_usart1_tx_buf1[16 + ii * 6] = my_indicator_alarm_data[ii].AC_data_buf[2];
                my_usart1_tx_buf1[17 + ii * 6] = my_indicator_alarm_data[ii].AC_data_buf[3];
                my_usart1_tx_buf1[18 + ii * 6] = my_indicator_alarm_data[ii].AC_data_buf[4];
                my_usart1_tx_buf1[19 + ii * 6] = my_indicator_alarm_data[ii].AC_data_buf[5];
            }
            else  //���������������������
            {
                my_usart1_tx_buf1[14 + ii * 6] = my_indicator_data[ii].AC_data_buf[0];
                my_usart1_tx_buf1[15 + ii * 6] = my_indicator_data[ii].AC_data_buf[1];
                my_usart1_tx_buf1[16 + ii * 6] = my_indicator_data[ii].AC_data_buf[2];
                my_usart1_tx_buf1[17 + ii * 6] = my_indicator_data[ii].AC_data_buf[3];
                my_usart1_tx_buf1[18 + ii * 6] = my_indicator_data[ii].AC_data_buf[4];
                my_usart1_tx_buf1[19 + ii * 6] = my_indicator_data[ii].AC_data_buf[5];


            }


        }
        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

        return;



    }

    else if(my_GPRS_all_step == 0X9400) //ң��--����AC12T
    {
        my_fun_gprs_generate_12T_data(my_usart1_tx_buf1);//����12T����
        if((int)my_indicator_tx_index >= 0 && my_indicator_tx_index < 10)
        {
            my_usart1_tx_buf1[12] = my_indicator_tx_index + 1;
            my_usart1_tx_buf1[12] = 1;
            my_usart1_tx_buf1[13] = 0X43;

        }

        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;

        my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

        return;
    }

    else if(my_GPRS_all_step == 0X9500)
    {   ii = my_indicator_tx_index;
        wdz_GPRS_string_to_array(TX_GPRS_101_ALarm_single_countRTC_dadta, my_usart1_tx_buf1); //
        my_usart1_tx_buf1[12] = 0x01;
        my_usart1_tx_buf1[13] = 0X4F;

        my_usart1_tx_buf1[14] = my_indicator_alarm_data[ii].count_time[0];
        my_usart1_tx_buf1[15] = my_indicator_alarm_data[ii].count_time[1];
        my_usart1_tx_buf1[16] = my_indicator_alarm_data[ii].RTC_time_buf[0];
        my_usart1_tx_buf1[17] = my_indicator_alarm_data[ii].RTC_time_buf[1];
        my_usart1_tx_buf1[18] = my_indicator_alarm_data[ii].RTC_time_buf[2];
        my_usart1_tx_buf1[19] = my_indicator_alarm_data[ii].RTC_time_buf[3];
        my_usart1_tx_buf1[20] = my_indicator_alarm_data[ii].RTC_time_buf[4];
        my_usart1_tx_buf1[21] = my_indicator_alarm_data[ii].RTC_time_buf[5];
        my_usart1_tx_buf1[22] = my_indicator_alarm_data[ii].RTC_time_buf[6];

        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);


    }





}

/*
���ܣ�����¼������
*/
void  my_fun_GPRS_TX_rec_data(void)  //
{
    if(my_GPRS_all_step == 0X5100)
    {
        my_usart1_tx_buf1[0] = 0x10;
        my_usart1_tx_buf1[5] = 0x16;
        my_usart1_tx_buf1[1] = 0x00;
        my_usart1_tx_buf1[2] = DTU_ADDRESS;
        my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
        my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1); //
        printf("my_GPRS send ok:");
        my_fun_display_buf_16(my_usart1_tx_buf1, 6, 1);

        my_tx_rec_count_all = 0;
        my_tx_rec_count_finish = 0;

        //========================
        my_tx_rec_count_all = 9;
        if(my_GPRS_all_count == 1)
            my_tx_rec_count_finish++;

        my_fun_GPRS_101_genert_record_data(my_usart1_tx_buf1);

        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }
    else if(my_GPRS_all_step == 0X5200)
    {
        if(my_GPRS_all_count == 1)
            my_tx_rec_count_finish++;

        my_fun_GPRS_101_genert_record_data( my_usart1_tx_buf1);

        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);
        if(my_tx_rec_count_finish < my_tx_rec_count_all - 1) //**�ܹ�N���Σ�ǰN-1���Σ����磬�ܹ�9�Σ���8�ξ�Ӧ���˳�
            my_GPRS_all_step = 0X5100;

    }

    else if(my_GPRS_all_step == 0X5300)
    {

        if(my_GPRS_all_count == 1)
            my_tx_rec_count_finish++;

        my_fun_GPRS_101_genert_record_data(my_usart1_tx_buf1);

        //�޸�֡ DTU��ַ����Ϣ�������
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }



}




//=====================================================================================================================
//=====================================================================================================================
//=====================================================================================================================
//GPRS  ���մ�����==========
uint8_t my_fun_GPRS_RX_test1(void) //�˺���Ϊ�����������յ�OK֡�󣬽����Ի�����
{

    //A�����
    //���ֽ���

    if(my_GPRS_all_step == 0X00E3 || my_GPRS_all_step == 0X0048)
    {
        printf("GPRS==TCP==start time is over-1!!\n\n");
        NET_Server_status = 1;

    }

    //RTCͬ��
    else if(my_GPRS_all_step == 0X00D1)
    {
        //RTCʱ�䴦��
        uint16_t my_temp16 = 0;
        my_RTC_date.Date =  (USART1_my_frame[19] & 0x1F);
        my_RTC_date.WeekDay = ((USART1_my_frame[19] >> 5) & 0x07);
        my_RTC_date.Month = USART1_my_frame[20];
        my_RTC_date.Year = USART1_my_frame[21];
        HAL_RTC_SetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN);

        my_temp16 = USART1_my_frame[16];
        my_temp16 = (my_temp16 << 8) + USART1_my_frame[15];
        my_RTC_time.Seconds = my_temp16 / 1000;
        my_RTC_time.Minutes = USART1_my_frame[17];
        my_RTC_time.Hours = USART1_my_frame[18];
        HAL_RTC_SetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);
        //
        printf("GPRS RX==RTC time =====!!\n");
    }

    else if(my_GPRS_all_step == 0X00D5)
    {
        my_get_data_buf[0] = USART1_my_frame[15];
        my_get_data_buf[1] = USART1_my_frame[16];


    }
    //dtu���̸�λ
    else if(my_GPRS_all_step == 0X00A2)
    {

        printf("GPRS==DTU Reset==  is over-7!!\n\n");

        HAL_NVIC_SystemReset();
    }
    else if(my_GPRS_all_step == 0X0051)
    {
        uint8_t ii = 0;
        uint8_t  length = 0;

        //Ŀ¼����
        length = USART1_my_frame[22];
        if(length == 0) //Ĭ��Ŀ¼
        {
            strcpy((char *)(my_file_catalog_buf), "HISTORY/SOE");
        }
        else if(length == 11)
        {
            for(ii = 0; ii < 11; ii++)
            {
                my_file_catalog_buf[ii] = USART1_my_frame[22 + ii];

            }

        }
        else if(length == 13)
        {
            for(ii = 0; ii < 13; ii++)
            {
                my_file_catalog_buf[ii] = USART1_my_frame[22 + ii];

            }

        }
        else if(length == 12)
        {
            for(ii = 0; ii < 12; ii++)
            {
                my_file_catalog_buf[ii] = USART1_my_frame[22 + ii];

            }

        }
        else
        {
            for(ii = 0; ii < 11; ii++)
            {
                my_file_catalog_buf[ii] = USART1_my_frame[22 + ii];

            }
        }
        my_file_catalog_buf[ii] = 0;

        //��ѯĿ¼״̬
        my_file_catalog_status = USART1_my_frame[22 + length + 1];
        if(my_file_catalog_status == 1)
        {
            for(ii = 0; ii < 7; ii++)
            {
                my_file_RTC_start_time[ii] = USART1_my_frame[22 + length + 2 + ii];
                my_file_RTC_end_time[ii] = USART1_my_frame[22 + length + 9 + ii];;
            }
        }
        else
        {
            for(ii = 0; ii < 7; ii++)
            {
                my_file_RTC_start_time[ii] = 0;
                my_file_RTC_end_time[ii] = 0;
            }
        }

        //��ѯĿ¼��ֹʱ��


    }
		
		else if(my_GPRS_all_step == 0X0053)
		{
			 uint8_t kk=0;
			 my_file_name_count=USART1_my_frame[4 +9 + 3 + 2];//�ļ�������
			 if(my_file_name_count>30)
			 {
				 for(kk=0;kk<30;kk++)
				 {
					 
					 my_file_name_buf[kk]=USART1_my_frame[4 +9 + 3 + 3+kk]; //�ļ���
					 		 
				 }
			 }
			 else
			 {
				 for(kk=0;kk<my_file_name_count;kk++)
				 {
					 
					 my_file_name_buf[kk]=USART1_my_frame[4 +9 + 3 + 3+kk]; //�ļ���
					 		 
				 }
				 
			 }
				 
			
			
			
		}
		

    //��ȡ�ļ�����ȷ��
    else if(my_GPRS_all_step == 0X0054)
    {
        //�ļ��������
			
			
			//������Ҫ�����Ǹ��ļ��������ܵ��ֽ��������ֶ�������ÿ�ε������������һ�ε���������ָ��λ�ÿ�ʼλ��
			//˼·���ж��ļ����ͣ�ȡ���ݣ�����ֶ�����
			my_file_class_ID=1;
			my_file_data_count=270;
			my_file_part_count=3;
			my_file_part_data_count_aver=100;
			my_file_part_data_count_end=70;
			
			

        //�����ļ���������Է��ͣ����뷢��״̬ 
        my_GPRS_all_step = 0;
        uint16_t my_step = 0X5500;
        xQueueSend(myQueue01Handle, &my_step, 100);	//��ʶ��һ��״̬,�����ļ�����



    }

		 else if(my_GPRS_all_step == 0X0056)
    {
        //�ļ��������
			
			
			



    }



    //C�����
    //����ֵͬ��ָ��
    else if(my_GPRS_all_step == 0X00E5)
    {
        uint16_t my_temp16 = 0;
        my_temp16 = USART1_my_frame[15];
        my_temp16 = (my_temp16 << 8) + USART1_my_frame[14];
        //ȫ�ּ���ʱ��ͬ����tim6count
        //my_tim6_count=my_temp16;
        printf("==TCP_start Count_syn=%d===\n", my_temp16);
        printf("GPRS==TCP count_syn==time is over-2!!\n\n");
    }
    //��������
    else if(my_GPRS_all_step == 0X001F)
    {
        printf("GPRS RX==heart==time is over-3!!\n\n");
    }

    else if(my_GPRS_all_step == 0X00D2)
    {
        printf("GPRS RX==RTC time FINISH===!!\n\n");
    }

    //���ڽ���
    else if(my_GPRS_all_step == 0X00B4)
    {
        printf("GPRS==old CYC== time is over-4!!\n\n");
    }
    else if(my_GPRS_all_step == 0X00B7)
    {

        printf("GPRS==new CYC== time is over-5!!\n\n");
    }







    else if(my_GPRS_all_step == 0X0041)
    {
        printf("GPRS==GPRS dbp==%d-ZSQ=[%d]-[%d]-[%d]!!\r\n", MY_AT_CSQ_Value, my_indicator_data[0].xinhao_db, my_indicator_data[1].xinhao_db, my_indicator_data[2].xinhao_db);
    }
    else if(my_GPRS_all_step == 0X0031)
    {
        printf("GPRS==Get RTC -Count time!!\r\n");
    }






    else
        printf("GPRS RX my_GPRS_ALL_STEP=[%XH]\r\n", my_GPRS_all_step);

    //=============




    return 1;

}

//GPRS ң�Ž��յ�OK֡
uint8_t my_fun_GPRS_RX_test2(void)  //���յ�����֡����������
{

    printf("GPRS dialog get OK frame---[%XH]\r\n", my_GPRS_all_step);
    return 1;
}



/*
���ܣ�DTU��������
*/



struct my_ZSQ_change_vale my_zsq_value; //ָʾ�����ò���ʹ��
uint8_t my_fun_GPRS_RX_change_parameter(void) //
{
    uint16_t my_temp16 = 0;
    uint16_t mydata16 = 0;
    uint8_t mybuf[8] = {0};

    my_temp16 = USART1_my_frame[13];
    my_temp16 = (my_temp16 << 8) + USART1_my_frame[12]; //������ݵ���Ϣ���ַ


    //DTU��������
    if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X5001)
    {
        mybuf[0] = USART1_my_frame[14];
        mybuf[1] = USART1_my_frame[15];
        mydata16 = mybuf[1];
        mydata16 = (mydata16 << 8) + mybuf[0];

        if(mydata16 != 0x0000)
        {
            DTU_ADDRESS = mydata16;
            AT25_WriteByte(mybuf[0], EEPROM_DTU_Address);
            AT25_WriteByte(mybuf[1], EEPROM_DTU_Address + 1);
        }

        printf("GPRS==TCP==change parameter 5001!!\r\n"); //DTU��ַ
    }

    else if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X5002)
    {
        mybuf[0] = USART1_my_frame[14];
        mybuf[1] = USART1_my_frame[15];
        mydata16 = mybuf[1];
        mydata16 = (mydata16 << 8) + mybuf[0];

        if(mydata16 != 0x0000)
        {
            MY_M_speed_heart = mydata16;
            AT25_WriteByte(mybuf[0], EEPROM_Hearttime_Address);
            AT25_WriteByte(mybuf[1], EEPROM_Hearttime_Address + 1);
        }


        printf("GPRS==TCP==change parameter 5002!!\r\n"); //������ʱ�䣨���ӣ�00Ĭ��5����
    }
    else if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X5003)
    {
        mybuf[0] = USART1_my_frame[14];
        mybuf[1] = USART1_my_frame[15];
        mydata16 = mybuf[1];
        mydata16 = (mydata16 << 8) + mybuf[0];

        if(mydata16 != 0x0000)
        {
            MY_M_speed_cyc = mydata16;
            AT25_WriteByte(mybuf[0], EEPROM_Cycetime_Address);
            AT25_WriteByte(mybuf[1], EEPROM_Cycetime_Address + 1);
        }
        printf("GPRS==TCP==change parameter 5003!!\r\n"); //��ʱ��������ʱ�䣨Ĭ�ϸ���ʱ�䣩
    }
    else if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X5004)
    {
        MY_IP[0] = mybuf[0] = USART1_my_frame[14];
        MY_IP[1] = mybuf[1] = USART1_my_frame[15];
        MY_IP[2] = mybuf[2] = USART1_my_frame[16];
        MY_IP[3] = mybuf[3] = USART1_my_frame[17];

        mybuf[4] = USART1_my_frame[18];
        mybuf[5] = USART1_my_frame[19];


        //PORT
        mydata16 = mybuf[5];
        mydata16 = (mydata16 << 8) + mybuf[4];

        if(mydata16 != 0x0000 && mydata16 < 65535)
        {
            MY_PORT = mydata16;
            AT25_WriteByte(mybuf[0], EEPROM_IPPort_Address);
            AT25_WriteByte(mybuf[1], EEPROM_IPPort_Address + 1);
            SPI_EE_BufferWrite2(MY_IP, EEPROM_IP_Address, 4); //��Ĭ��IP��ַ д�뵽EEPROM�� 222.222.118.3������ʮ���Ƶ�
        }

        printf("GPRS==TCP==change parameter 5004!!\r\n"); //Server��ַ��5���ֽ�,IP��ַ�Ӷ˿ںţ���222.222.118.3��2404������˳�������
    }

    else if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X5010)
    {
        mybuf[0] = USART1_my_frame[14];
        mybuf[1] = USART1_my_frame[15];
        mydata16 = mybuf[1];
        mydata16 = (mydata16 << 8) + mybuf[0];

        if(mydata16 != 0x0000)
        {
            MY_Speed_H_Gate = mydata16 / 10.0;
            AT25_WriteByte(mybuf[0], EEPROM_SPEED_Gate_H_Address);
            AT25_WriteByte(mybuf[1], EEPROM_SPEED_Gate_H_Address + 1);
        }

        printf("GPRS==TCP==change parameter 5010!!\r\n"); //�����ٶȵ�������
    }
    else if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X5011)
    {

        mybuf[0] = USART1_my_frame[14];
        mybuf[1] = USART1_my_frame[15];
        mydata16 = mybuf[1];
        mydata16 = (mydata16 << 8) + mybuf[0];

        if(mydata16 != 0x0000)
        {
            MY_H_speed_cyc = mydata16 / 10.0;
            AT25_WriteByte(mybuf[0], EEPROM_SPEED_H_Cyc_Address);
            AT25_WriteByte(mybuf[1], EEPROM_SPEED_H_Cyc_Address + 1);
        }
        printf("GPRS==TCP==change parameter 5011!!\r\n"); //����
    }
    else if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X5012)
    {
        mybuf[0] = USART1_my_frame[14];
        mybuf[1] = USART1_my_frame[15];
        mydata16 = mybuf[1];
        mydata16 = (mydata16 << 8) + mybuf[0];

        if(mydata16 != 0x0000)
        {
            MY_L_speed_cyc = mydata16 / 10.0;
            AT25_WriteByte(mybuf[0], EEPROM_SPEED_L_Cyc_Address);
            AT25_WriteByte(mybuf[1], EEPROM_SPEED_L_Cyc_Address + 1);
        }
        printf("GPRS==TCP==change parameter 5012!!\r\n"); //����
    }

    //ָʾ����������

    else if(my_GPRS_all_step == 0X0081 && (my_temp16 >= 0X5031 && my_temp16 <= 0X5050))
    {

        my_zsq_value.my_inf_add = my_temp16; //��Ϣ���ַ
        my_zsq_value.zsq_add = USART1_my_frame[15];
        my_zsq_value.zsq_add = (my_zsq_value.zsq_add << 8) + USART1_my_frame[14];
        my_zsq_value.data_buf[0] = USART1_my_frame[16]; //����
        my_zsq_value.data_buf[1] = USART1_my_frame[17];
        my_zsq_value.status = 1; //����״̬��1Ϊû�з���

        printf("GPRS==TCP==change parameter [%X]H!!\n", my_temp16); //����У��ϵ��

    }
    else if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X5051)
    {

        my_zsq_value.my_inf_add = my_temp16; //��Ϣ���ַ
        my_zsq_value.zsq_add = USART1_my_frame[15];
        my_zsq_value.zsq_add = (my_zsq_value.zsq_add << 8) + USART1_my_frame[14];
        my_zsq_value.data_buf[0] = USART1_my_frame[16]; //����
        my_zsq_value.data_buf[1] = USART1_my_frame[17];
        my_zsq_value.data_buf[2] = USART1_my_frame[18];
        my_zsq_value.data_buf[3] = USART1_my_frame[19];
        my_zsq_value.data_buf[4] = USART1_my_frame[20];
        my_zsq_value.data_buf[5] = USART1_my_frame[21];
        my_zsq_value.status = 1; //����״̬��1Ϊû�з���

        printf("GPRS==TCP==change parameter [%X]H!!\n", my_temp16);
        // ָʾ���µ�ַ

    }

    else
        printf("GPRS RX my_GPRS_ALL_STEP=[%XH]\r\n", my_GPRS_all_step);

    //=============




    return 1;

}

//�ն˷��ƹ���
uint8_t my_fun_GPRS_RX_turn_led(void) //
{
    uint16_t my_temp16 = 0, my_temp162 = 0;

    uint8_t mydata8 = 0;

    my_temp16 = USART1_my_frame[13];
    my_temp16 = (my_temp16 << 8) + USART1_my_frame[12]; //������ݵ���Ϣ���ַ1
    mydata8 = USART1_my_frame[14];
    my_temp162 = USART1_my_frame[16];
    my_temp162 = (my_temp162 << 8) + USART1_my_frame[15]; //������ݵ���Ϣ���ַ2

    //��·����
    if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X6001 && my_temp162 == 0x6002)
    {
        my_indicator_data[mydata8 - 1].duanlu_data = 0X01 ;
        printf("GPRS==TCP==turn LED 6002!!\r\n"); //
    }
    else if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X6001 && my_temp162 == 0x6005)
    {
        my_indicator_data[mydata8 - 1].jiedi_data = 0X01 ;
        printf("GPRS==TCP==turn LED 6005!!\r\n"); //
    }
    else if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X6001 && my_temp162 == 0x6003)
    {
        my_indicator_data[mydata8 - 1].jiedi_data = 0X00 ;
        my_indicator_data[mydata8 - 1].duanlu_data = 0X00 ;
        printf("GPRS==TCP==turn LED 6003!!\r\n"); //
    }
    else if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X6001 && my_temp162 == 0x6004)
    {
        my_indicator_data[mydata8 - 1].jiedi_data = 0X00;
        my_indicator_data[mydata8 - 1].duanlu_data = 0X00;
        printf("GPRS==TCP==turn LED 6004!!\r\n"); //
    }
    else
        printf("GPRS RX my_GPRS_ALL_STEP=[%XH]\r\n", my_GPRS_all_step);

    //=============

    return 1;

}


//��ѯ����
uint8_t my_fun_GPRS_RX_query_data(void) //
{
    uint16_t my_temp16 = 0;

    if(USART1_my_frame[0] == 0X68 && my_GPRS_all_step == 0x0071)
    {
        my_temp16 = USART1_my_frame[13];
        my_temp16 = (my_temp16 << 8) + USART1_my_frame[12]; //������ݵ���Ϣ���ַ1
        query_data = 0X1;
    }
    else
        query_data++;


    printf("GPRS RX my_GPRS_ALL_STEP=[%XH]\r\n", my_GPRS_all_step);


    //=============
    return 1;

}

uint8_t my_fun_GPRS_RX_query_data2(void) //
{
    uint16_t my_temp16 = 0;

    if(USART1_my_frame[0] == 0X68 && my_GPRS_all_step == 0x0077)
    {
        my_temp16 = USART1_my_frame[13];
        my_temp16 = (my_temp16 << 8) + USART1_my_frame[12]; //������ݵ���Ϣ���ַ1
        query_data2 = 0X1;
    }
    else
        query_data2++;


    printf("GPRS RX my_GPRS_ALL_STEP=[%XH]\r\n", my_GPRS_all_step);


    //=============
    return 1;

}
/*
���ܣ�����¼�����մ�����
*/
uint8_t my_fun_GPRS_RX_Rec_data(void)
{
    if(my_GPRS_all_step == 0X0051)
    {
        printf("\n\nGPRS==record data===start!!\r\n");
#if Use_indicatour_cyc_test_satus==1
        my_indicator_record_data[1].my_wave_type = 1;
        my_indicator_record_data[1].my_wave_alam = 2;
        int ii = 0;
        uint8_t xx = 0, yy = 0;
        for(ii = 0; ii < my_record_count; ii++)
        {
            my_indicator_record_data[1].my_wave_record_I_buf[ii] = xx++;
            my_indicator_record_data[1].my_wave_record_E_buf[ii] = yy--;
        }


#endif

    }
    else if(my_GPRS_all_step == 0X0052 || my_GPRS_all_step == 0X0053)
    {
        uint8_t my_temp81 = 0;
        my_temp81 = USART1_my_frame[14]; //���յ��Ķκ�
        printf("GPRS==record data= process=%d !!\r\n", my_temp81);

    }
    else if(my_GPRS_all_step == 0X0054)
    {
        uint8_t my_temp81 = 0;
        my_temp81 = USART1_my_frame[14]; //���յ��Ķκ�
        printf("GPRS==record data===finish=%d!!\r\n", my_temp81);

        //�������ͳɹ��������������״̬
        if(my_indicator_tx_index < my_indicator_count)
        {
            if(my_indicator_alarm_data[my_indicator_tx_index].TX_status_duanlu != 0)
                my_indicator_alarm_data[my_indicator_tx_index].TX_status_duanlu = 0X00;
            else if(my_indicator_alarm_data[my_indicator_tx_index].TX_status_jiedi != 0)
                my_indicator_alarm_data[my_indicator_tx_index].TX_status_jiedi = 0X00;
            else
            {
                my_indicator_alarm_data[my_indicator_tx_index].TX_status_duanlu = 0X00;
                my_indicator_alarm_data[my_indicator_tx_index].TX_status_jiedi = 0X00;
            }
            my_indicator_record_data[my_indicator_tx_index].my_wave_tx_status_I = 0;
            my_indicator_record_data[my_indicator_tx_index].my_wave_tx_status_E = 0;

            my_indicator_tx_index = 99;
        }


    }

    return 1;
}



//=====================
/*
���ܣ�M35��������

*/

void my_fun_M35_resume_init(void)
{

    if( GPRS_Heartdata_error_count >= 3) //M35�������ݲ��ɹ�3������
    {
        printf("\n==my GPRS_Heartdata_error_count=%d====\n", GPRS_Heartdata_error_count);
        BaseType_t xResult;
        BaseType_t xHigherPriorityTaskWoken = pdFAIL;
        xResult =	xEventGroupSetBitsFromISR(xCreatedEventGroup, 0X10, &xHigherPriorityTaskWoken);
        if(xResult != pdFAIL)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }


}

uint8_t my_query_record_index = 99;
void my_fun_GPRS_101_genert_record_data(uint8_t *txbu)
{
    uint8_t my_length = 0;
    uint16_t my_inf_add_rx = 0;
    uint16_t my_inf_add_tx = 0;
    uint8_t *pt_buf = NULL;
    uint16_t *pt_count = NULL;
    uint8_t ii = 0;


    //���ݵĳ���
    wdz_GPRS_string_to_array(TX_GPRS_101_Record_data, my_usart1_tx_buf1);
    if(my_tx_rec_count_finish < my_tx_rec_count_all)
        my_length = 254;
    else
        my_length = 25;

    //���յ�����Ϣ���ַ
    my_inf_add_rx = USART1_my_frame[13];
    my_inf_add_rx = (my_inf_add_rx << 8) + USART1_my_frame[12]; //������ݵ���Ϣ���ַ1
    my_inf_add_tx	=	my_inf_add_rx;
    //ָʾ�����


    //�޸ķ���¼������ָʾ����
    if(my_indicator_tx_index < my_indicator_count)
        my_query_record_index = my_indicator_tx_index;
    printf("==3=my_query_record_index=%d\n", my_query_record_index);
    //printf("alarm wave_type %d--%d--%d\n",my_indicator_record_data[0].my_wave_type,my_indicator_record_data[1].my_wave_type,my_indicator_record_data[2].my_wave_type);




    //====������ָ��
    if(my_inf_add_rx == 0X4501 || my_inf_add_rx == 0X4601)
    {
        pt_buf = &my_indicator_record_data[my_query_record_index].my_wave_record_I_buf[0];
        pt_count = &my_indicator_record_data[my_query_record_index].my_count_read;
    }
    else
    {
        pt_buf = &my_indicator_record_data[my_query_record_index].my_wave_record_E_buf[0];
        pt_count = &my_indicator_record_data[my_query_record_index].my_count_read;
    }

    //=============
    my_usart1_tx_buf1[12] = my_inf_add_tx;
    my_usart1_tx_buf1[13] = (my_inf_add_tx >> 8);
    //============
    if(my_tx_rec_count_finish == 1) //��һ������
    {
        //101֡����
        my_usart1_tx_buf1[1] = my_length;
        my_usart1_tx_buf1[2] = my_length;
        //��ͷ
        my_usart1_tx_buf1[14] = 9;
        my_usart1_tx_buf1[15] = my_tx_rec_count_finish;
        my_usart1_tx_buf1[16] = 240;
        my_usart1_tx_buf1[17] = 1;

        //��������
        *pt_count = 0;
        for(ii = 0; ii < 240; ii++)
        {
            my_usart1_tx_buf1[18 + ii] = pt_buf[ii];
            //printf("-pt_buf[%d]=%d\n",ii,pt_buf[ii]);
        }
        //��ָ���ƶ�

        *pt_count = *pt_count + ii;
    }

    else if(my_tx_rec_count_finish > 1 && my_tx_rec_count_finish < my_tx_rec_count_all) //��������
    {
        //101֡����
        my_usart1_tx_buf1[1] = my_length;
        my_usart1_tx_buf1[2] = my_length;
        //��ͷ
        my_usart1_tx_buf1[14] = 9;
        my_usart1_tx_buf1[15] = my_tx_rec_count_finish;
        my_usart1_tx_buf1[16] = 240;
        my_usart1_tx_buf1[17] = 1;

        //��������

        for(ii = 0; ii < 240; ii++)
        {
            my_usart1_tx_buf1[18 + ii] = pt_buf[ii + *pt_count];
        }
        //��ָ���ƶ�

        *pt_count = *pt_count + ii;
    }

    else if(my_tx_rec_count_finish == my_tx_rec_count_all) //���һ������
    {
        //101֡����
        my_usart1_tx_buf1[1] = my_length;
        my_usart1_tx_buf1[2] = my_length;
        //��ͷ
        my_usart1_tx_buf1[14] = 9;
        my_usart1_tx_buf1[15] = my_tx_rec_count_finish;
        my_usart1_tx_buf1[16] = 11;
        my_usart1_tx_buf1[17] = 0;

        //��������

        for(ii = 0; ii < 11; ii++)
        {
            my_usart1_tx_buf1[18 + ii] = pt_buf[ii + *pt_count];
        }
        //��ָ���ƶ�

        *pt_count = *pt_count + ii;
    }



}










//========CC1101����
uint8_t my_fun_dialog_CC1101_RX_heart(void)
{
    uint8_t my_address = 0;
#if Debug_uart_out_cc1101_rx_data_status==1
    my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //����ʹ�ã���ʾ���յ�������8���ֽ�
#endif


    //������������
    if(my_CC1101_all_step == 0xE000)
    {
        //��õ�ַ--����Դ
        my_address = my_CC1101_COM_Fram_buf[2]; //֡�еķ���Դ��ַ
        my_cc1101_dest_address = my_address; //�޸�CC1101��Ŀ�ĵ�ַ��Ϊ������׼��ʹ��
        printf("get cc1101 heart data id=%d", my_address);


    }

    return 1;

}

void my_fun_CC1101_TX_OK(void)
{
    uint8_t *pt;
    uint16_t my_temp16 = my_tim6_count; //ͬ������ֵ
    pt = my_cc1101_tx_buf;
    pt[0] = 0x10;
    pt[1] = 0x20; //IDΪ0X20����ʾOK֡
    pt[2] = my_CC1101_chip_address; //����DTU��CC1101��ַ��Ϊ0XFE,��0XFDΪ�������ĵ�ַ��01,02,03---Ϊָʾ����ַ
    pt[3] = my_cc1101_dest_address; //Ŀ���ַ

    pt[4] = my_temp16;                   //OK֡�а����ˣ�ͬ������ֵ
    pt[5] = (my_temp16 >> 8);


    pt[6] = my_fun_101check_generate(pt, 1);
    pt[7] = 0x16;

    CC1101SendPacket_add( pt, 8,  ADDRESS_CHECK, my_cc1101_dest_address);
    printf("after CC TX my_CC1101_all_step=[%XH]\n", my_CC1101_all_step);

#if Debug_uart_out_cc1101_tx_data_status==1
    my_fun_display_buf_16(pt, 8, 1); //����ʹ��
#endif

}

void my_fun_display_ZSQ_data(void)
{


    uint16_t  xx = 0;
    uint16_t ii = 0;
    double yy[12] = {0};
    double xx2 = 0, xx3 = 0, xx4 = 0;
    //double xx1 = 0;
    for(xx = 0; xx < 3; xx++) //��ʾ����ָʾ����������Ϣ
    {
        //��·�ͽӵ�״̬
        printf("\n========START DC1============\n");
        printf("---ZSQ=[%d]--timer=[%d]-[%d]-[%d]--DTU-timer=[%d]\n", xx + 1, my_ZSQ_time_count[0], my_ZSQ_time_count[1], my_ZSQ_time_count[2], my_tim6_count);
        printf("ALARM:duanlu=[%XH],jiedi=[%XH]\n", my_indicator_data[xx].duanlu_data, my_indicator_data[xx].jiedi_data);

        //ֱ������7��
        for(ii = 0; ii < 7; ii++)
        {
            yy[ii] = (my_indicator_data[xx].DC_data_buf[2 * ii] +
                      (my_indicator_data[xx].DC_data_buf[2 * ii + 1] << 8)) / 10.0;
        }
        printf(" DC:Temp=%.2f,vbat=%.2f,vref=%.2f\n", yy[0], yy[1], yy[2]);
        printf(" DC:GANbat=%.2f,Zaixian=%.2f,sunbat=%.2f,Libat=%.2f\n", yy[3], yy[4], yy[5], yy[6]);
        //���Ͻ�������3��������ȫ�����糡�������벨
        for(ii = 0; ii < 3; ii++)
        {
            yy[ii] = (my_indicator_data[xx].AC_data_buf[2 * ii] +
                      (my_indicator_data[xx].AC_data_buf[2 * ii + 1] << 8)) / 10.0;
        }
        printf("AC:A=%.1f,E=%.1f,HA=%.1f\n", yy[0], yy[1], yy[2]);
        printf("========END DC============\n");


        //================ң��AC12T
        printf("***AC12T data start*****\n");
        for(ii = 0; ii < 12; ii++)
        {
            xx2 = (my_indicator_data[xx].AC12T_ALL_Current_data_buf[2 * ii] +
                   (my_indicator_data[xx].AC12T_ALL_Current_data_buf[2 * ii + 1] << 8)) / 10.0;
            xx3 = (my_indicator_data[xx].AC12T_ALL_dianchang_data_buf[2 * ii] +
                   (my_indicator_data[xx].AC12T_ALL_dianchang_data_buf[2 * ii + 1] << 8)) / 10.0;
            xx4 = (my_indicator_data[xx].AC12T_HALF_Current_data_buf[2 * ii] +
                   (my_indicator_data[xx].AC12T_HALF_Current_data_buf[2 * ii + 1] << 8)) / 10.0;

            printf(" A=%.2f,E=%.2f,HA=%.2f\n", xx2, xx3, xx4);
        }
        printf("***AC12T data END*****\n");


        //¼������

//				if(my_indicator_record_data[xx].my_wave_type==1)
//				{
//						 printf("\n***A_960 data start*****\n");
//					for(ii = 0; ii < 960; ii++)
//					{
//							xx1 = (my_indicator_record_data[xx].my_wave_record_I_buf[2 * ii]
//										 + ((my_indicator_record_data[xx].my_wave_record_I_buf[2 * ii + 1]) << 8)) / 10.0;
//							printf("\n %.1f", xx1);

//					}
//					   printf("\n***A_960 data end*****\n");
//				}
//				else if(my_indicator_record_data[xx].my_wave_type==2)
//				{
//						 printf("\n***E_960 data start*****\n");
//					for(ii = 0; ii < 960; ii++)
//					{
//							xx1 = (my_indicator_record_data[xx].my_wave_record_E_buf[2 * ii]
//										 + ((my_indicator_record_data[xx].my_wave_record_E_buf[2 * ii + 1]) << 8)) / 10.0;
//							printf("\n %.1f", xx1);

//					}
//					   printf("\n***E_960 data end*****\n");
//				}




    }


}




//CC1101���մ����������պ�û�к�������
uint8_t my_fun_dialog_CC1101_RX_0(void)
{
    uint8_t temp_status = 0;
    uint16_t my_length = 0;
    uint8_t my_address = 0; //
    uint8_t my_indicator_index = 0;
    uint8_t my_re = 1;

    if(my_CC1101_COM_Fram_buf[0] == 0x10) //10֡��
    {
        //���ָ��ID
        temp_status = my_CC1101_COM_Fram_buf[1];
        my_CC1101_receive_cmd_ID = temp_status;
        //��õ�ַ--����Դ
        my_address = my_CC1101_COM_Fram_buf[2]; //֡�еķ���Դ��ַ
        my_cc1101_dest_address = my_address; //�޸�CC1101��Ŀ�ĵ�ַ��Ϊ������׼��ʹ��

        //printf("@@@@@ ID=%d, RSSI=%d \n",my_address,my_RSSI_dbm_all) ;
        my_indicator_data[my_address - 1].xinhao_db = my_RSSI_dbm_all; //�洢�ź�ǿ��

//        my_address_dest = my_CC1101_COM_Fram_buf[3]; //����Ŀ�ĵ�ַ
#if Debug_uart_out_cc1101_rx_data_status==1
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //����ʹ�ã���ʾ���յ�������8���ֽ�
#endif
    }
    else if (my_CC1101_COM_Fram_buf[0] == 0x68) //68��֡
    {
        //���ָ��ID
        temp_status = my_CC1101_COM_Fram_buf[6];
        printf("===============control word==[%X]\n", temp_status);
        my_CC1101_receive_cmd_ID = temp_status;

        my_length = my_CC1101_COM_Fram_buf[2];
        my_length = (my_length << 8) + my_CC1101_COM_Fram_buf[1] - 3; //��ó���

        //��õ�ַ
        my_address = my_CC1101_COM_Fram_buf[7];
        my_cc1101_dest_address = my_address; //�޸�CC1101��Ŀ�ĵ�ַ��Ϊ������׼��ʹ��

#if Debug_uart_out_cc1101_rx_data_status==1
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //����ʹ�ã���ʾ���յ�������8���ֽ�
#endif
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //@@@@����ʹ�ã���ʾ���յ�������8���ֽ�
    }

    //ָʾ����ַ�ж�
    if(my_address == 0X01)
    {
        my_indicator_index = 0;
    }
    else if(my_address == 0X02)
    {
        my_indicator_index = 1;
    }
    else if(my_address == 0X03)
    {
        my_indicator_index = 2;
    }
    else
        my_indicator_index = 0X10;
    //֡���������ж�
    if(temp_status == 0x01)
    {
        my_indicator_data[my_indicator_index].data_type = 0x01; //����
    }
    else if(temp_status == 0x02)
    {
        my_indicator_data[my_indicator_index].data_type = 0x02; //ң��

    }
    my_cc1101_tx_wait_time = 2000;
//================

    if(my_CC1101_receive_cmd_ID == 0X2F)
    {

        printf("config prarameter is start==ZSQ[%d]!!!\n", my_address);
    }
    else if(my_CC1101_receive_cmd_ID == 0X4F)
    {

        printf("config prarameter is finish!==ZSQ[%d]!!\n", my_address);
    }

    //===============
    return my_re;


}


/*
����CC1101��������ָ��
*/

void my_fun_CC1101_TX_config_parmeter(void)
{
    uint8_t *pt;
    //uint16_t my_temp16 = my_tim6_count; //ͬ������ֵ
    pt = my_cc1101_tx_buf;
    uint16_t st_len = 5;
    pt[0] = 0x68;
    pt[1] = st_len;
    pt[2] = (st_len >> 8);
    pt[3] = pt[1];
    pt[4] = pt[2];
    pt[5] = 0x68;

    pt[6] = 0x3F;
    pt[7] = my_CC1101_chip_address; //����DTU��CC1101��ַ��Ϊ0XFE,��0XFDΪ�������ĵ�ַ��01,02,03---Ϊָʾ����ַ
    pt[8] = my_cc1101_dest_address; //Ŀ���ַ
    //��Ϣ����,5���ȵ�ʱ��û����Ҫ���õ�����
    if(st_len == 5)
    {
        pt[9] = 0X01;
        pt[10] = 0X02;
    }
    //===
    //RTC���ò���
    {
        HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN);
        HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);

        st_len = st_len + 9;

        pt[1] = st_len;
        pt[2] = (st_len >> 8);
        pt[3] = pt[1];
        pt[4] = pt[2];

        pt[9] = 0X01;
        pt[10] = 0X40;


        pt[11] = my_tim6_count;
        pt[12] = (my_tim6_count >> 8);

        pt[13] = my_RTC_time.Seconds;
        pt[14] = 0;
        pt[15] = my_RTC_time.Minutes;
        pt[16] = my_RTC_time.Hours;
        pt[17] = my_RTC_date.Date;
        pt[18] = my_RTC_date.Month;
        pt[19] = my_RTC_date.Year;

    }


    pt[st_len + 6] = my_fun_101check_generate(pt, 1);
    pt[st_len + 7] = 0x16;

    CC1101SendPacket_add( pt, st_len + 8,  ADDRESS_CHECK, my_cc1101_dest_address);
    printf("after CC TX my_CC1101_all_step=[%XH]\n", my_CC1101_all_step);

#if Debug_uart_out_cc1101_tx_data_status==1
    my_fun_display_buf_16(pt, 8, 1); //����ʹ��
#endif

}

//����ȷ��
void  my_fun_GPRS_TX_Call_0(void)
{

    my_fun_GPRS_TX_OK_80();


    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;

    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = 0x0C; //
    my_usart1_tx_buf1[2] = 0x0C;
    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73
    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 0X46;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x07;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI; //���ͱ�ʶ����ʱ���ң��������Ϣ��
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //��Ϣ�����
    my_usart1_tx_buf1[9] = my_101_COT_low; //����ԭ��
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x00; //ң����Ϣ���׵�ַ
    my_usart1_tx_buf1[14] = 0x00;

    my_usart1_tx_buf1[15] = 20;

    my_usart1_tx_buf1[16] = my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_usart1_tx_buf1[17] = 0x16;

    my_at_senddata(my_usart1_tx_buf1); //



}


void  my_fun_GPRS_TX_test_data(void)
{
    //���ֽ�
    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;
    my_101_DIR = 0X80;
    my_101_PRM = 0X00;
    my_101_FC = 0X00;

    my_usart1_tx_buf1[1] = (my_101_DIR | my_101_PRM | my_101_FC);
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //

    //==========================

    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;



    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = 0x0D; //1����Ϣ�壬��Ϣֵ2���ֽ�
    my_usart1_tx_buf1[2] = 0x0D;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 104;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x07;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI;
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //��Ϣ�����
    my_usart1_tx_buf1[9] = my_101_COT_low; //����ԭ��
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

    my_usart1_tx_buf1[13] = 0x00; //ң����Ϣ���׵�ַ
    my_usart1_tx_buf1[14] = 0x00;

    my_usart1_tx_buf1[15] = my_get_data_buf[0];
    my_usart1_tx_buf1[16] = my_get_data_buf[1];

    my_usart1_tx_buf1[17] = my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_usart1_tx_buf1[18] = 0x16;

    my_at_senddata(my_usart1_tx_buf1); //

    //=================

}

void  my_fun_GPRS_TX_heart_toserver_data(void)
{
    //���ֽ�
    my_usart1_tx_buf1[0] = 0x10;
    my_usart1_tx_buf1[5] = 0x16;
    my_101_DIR = 0X80;
    my_101_PRM = 0X00;
    my_101_FC = 0X00;

    my_usart1_tx_buf1[1] = (my_101_DIR | my_101_PRM | my_101_FC);
    my_usart1_tx_buf1[2] = DTU_ADDRESS;
    my_usart1_tx_buf1[3] = DTU_ADDRESS >> 8;
    my_usart1_tx_buf1[4] = my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1); //

    //==========================


}


void  my_fun_GPRS_TX_CYC2_B(void)  //����ң����,���ڣ���ʱ��
{
    uint8_t ii = 0, kk = 0;
    uint8_t lenth = 13;

    //�������ݰ�
    {

        my_usart1_tx_buf1[0] = 0x68;
        my_usart1_tx_buf1[3] = 0x68;
        my_usart1_tx_buf1[1] = lenth * 10 + 9 ; //����ʱ��7���ֽ�
        my_usart1_tx_buf1[2] = lenth * 10 + 9;

        //�������봦��
        my_101_DIR = 0X80;
        my_101_PRM = 0X40;
        if(my_GPRS_all_count == 1)
            my_101_FCB = (~my_101_FCB) & 0X20;
        my_101_FCV = 0X10;
        my_101_FC = 0X03;

        my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73



        my_usart1_tx_buf1[5] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);


        my_usart1_tx_buf1[7] = 31; //TI  ң�ţ�˫����Ϣ����ʱ��

        my_usart1_tx_buf1[8] = lenth + 0x00; //��Ϣ�����
        my_usart1_tx_buf1[9] = 0X03; //����ԭ��
        my_usart1_tx_buf1[10] = 0; //����ԭ��

        my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
        my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

        my_usart1_tx_buf1[4 + 9 + lenth * 10  ] = 0XFF;
        my_usart1_tx_buf1[4 + 9 + lenth * 10 + 1 ] = 0x16;

    }

    //my_gprs_generate_101single_data(1, my_usart1_tx_buf1, my_shibiao_time, 0x03); //�ܹ���56����Ϣ���ַ
    for(ii = 1; ii <= lenth * 10; ii++)
    {
        my_usart1_tx_buf1[12 + ii] = 0X00; //ң��˫����Ϣ 01Ϊ������0X02Ϊ����

    }



    HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN); //��ȡRTCʱ��
    HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);

    my_usart1_tx_buf1[12 + 1] = 0X01;
    my_usart1_tx_buf1[12 + 2] = 0X00;
    my_usart1_tx_buf1[12 + 3] = MY_yaoxin_status_OK;
    my_usart1_tx_buf1[12 + 3 + 1] = (my_RTC_time.Seconds * 1000);
    my_usart1_tx_buf1[12 + 3 + 2] = ((my_RTC_time.Seconds * 1000) >> 8);
    my_usart1_tx_buf1[12 + 3 + 3] = my_RTC_time.Minutes;
    my_usart1_tx_buf1[12 + 3 + 4] = my_RTC_time.Hours;
    my_usart1_tx_buf1[12 + 3 + 5] = my_RTC_date.Date || (my_RTC_date.WeekDay << 5);
    my_usart1_tx_buf1[12 + 3 + 6] = my_RTC_date.Month;
    my_usart1_tx_buf1[12 + 3 + 7] = my_RTC_date.Year;


    for(kk = 0; kk < 3; kk++)
    {
        //��ַ
        my_usart1_tx_buf1[12 + 10 + kk * 40 + 1] = 0x10 * (kk + 1) + 0X02;
        my_usart1_tx_buf1[12 + 10 + kk * 40 + 2] = 0x00;

        my_usart1_tx_buf1[12 + 10 + kk * 40 + 11] = 0x10 * (kk + 1) + 0X03;
        my_usart1_tx_buf1[12 + 10 + kk * 40 + 12] = 0x00;

        my_usart1_tx_buf1[12 + 10 + kk * 40 + 21] = 0x10 * (kk + 1) + 0X04;
        my_usart1_tx_buf1[12 + 10 + kk * 40 + 22] = 0x00;

        my_usart1_tx_buf1[12 + 10 + kk * 40 + 31] = 0x10 * (kk + 1) + 0X08;
        my_usart1_tx_buf1[12 + 10 + kk * 40 + 32] = 0x00;


//				//ͨ�Ż���
//        if(my_indicator_data[kk].xinhao_db > 75)
//            my_usart1_tx_buf1[14 +10+ kk*10 + 3] = MY_yaoxin_status_ERROR; //�ɼ���Ԫͨ��״̬
//        else
//            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 1] = MY_yaoxin_status_OK; //�ɼ���Ԫͨ��״̬

        //��·״̬
        if(my_indicator_data[kk].duanlu_data == 0X01)
        {
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 3] = MY_yaoxin_status_OK; //˲ʱ�Զ�·
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 13] = MY_yaoxin_status_ERROR; 	//�����Զ�·

        }
        else if(my_indicator_data[kk].duanlu_data == 0X41 || my_indicator_data[kk].duanlu_data == 0X81)
        {
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 3] = MY_yaoxin_status_ERROR; //˲ʱ�Զ�·
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 13] = MY_yaoxin_status_OK; 	//�����Զ�·

        }
        else
        {
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 3] = MY_yaoxin_status_OK; //˲ʱ�Զ�·
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 13] = MY_yaoxin_status_OK; 	//�����Զ�·

        }
        //�ӵ�״̬
        if(my_indicator_data[kk].jiedi_data == 0X01)
        {
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 23] = MY_yaoxin_status_ERROR; //�ӵ�
        }
        else
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 23] = MY_yaoxin_status_OK;

        //my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 5] = MY_yaoxin_status_OK; //�¶ȳ���
        //my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 6] = MY_yaoxin_status_OK; //��������

        //���״̬
//        float yy = 0;
//        yy = (my_indicator_data[kk].DC_data_buf[12] + (my_indicator_data[kk].DC_data_buf[13] << 8)) / 10.0;
//        if(yy < 3.8)
//            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 7] = MY_yaoxin_status_ERROR;
//        else
//            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 7] = MY_yaoxin_status_OK; //���Ƿѹ����


        //��·ͣ��״̬
        if(my_indicator_data[kk].Line_STOP == 2)
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 33] = MY_yaoxin_status_ERROR; //��·�е�,ͣ����
        else
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 33] = MY_yaoxin_status_OK; //�������е�


        //ʱ��
        my_usart1_tx_buf1[12 + 10 + kk * 40  + 4] = my_indicator_alarm_data[kk].RTC_time_buf[0];
        my_usart1_tx_buf1[12 + 10 + kk * 40  + 5] = my_indicator_alarm_data[kk].RTC_time_buf[1];
        my_usart1_tx_buf1[12 + 10 + kk * 40  + 6] = my_indicator_alarm_data[kk].RTC_time_buf[2];
        my_usart1_tx_buf1[12 + 10 + kk * 40  + 7] = my_indicator_alarm_data[kk].RTC_time_buf[3];
        my_usart1_tx_buf1[12 + 10 + kk * 40  + 8] = my_indicator_alarm_data[kk].RTC_time_buf[4];
        my_usart1_tx_buf1[12 + 10 + kk * 40  + 9] = my_indicator_alarm_data[kk].RTC_time_buf[5];
        my_usart1_tx_buf1[12 + 10 + kk * 40  + 10] = my_indicator_alarm_data[kk].RTC_time_buf[6];
    }




    wdz_GPRS_101check_generate(my_usart1_tx_buf1);
    my_at_senddata(my_usart1_tx_buf1);

    printf("my_GPRS send CYC data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);
}


void  my_fun_GPRS_TX_CYC3_B(void)  //ң�⣬���ڣ���ʱ��
{
    uint8_t jj = 0, shibiao = 0; //ʱ�곤��
    union MY_float my_AC_data, my_vol_data, my_temperature_data;
    my_gprs_generate_101analog_data(1, my_usart1_tx_buf1, shibiao, 0x01);

    //
    for(jj = 0; jj < MY_GPRS_Call_Analog_data_number * 5; jj++)
    {
        my_usart1_tx_buf1[15 + jj] = 0;
    }

    for(jj = 0; jj < 3; jj++)
    {
        my_AC_data.value = (my_indicator_data[jj].AC_data_buf[0] + (my_indicator_data[jj].AC_data_buf[1] << 8)) / 10.0;
        my_vol_data.value = (my_indicator_data[jj].DC_data_buf[12] + (my_indicator_data[jj].DC_data_buf[13] << 8)) / 10.0;
        my_temperature_data.value = (my_indicator_data[jj].DC_data_buf[0] + (my_indicator_data[jj].DC_data_buf[1] << 8)) / 10.0;

        my_usart1_tx_buf1[15 + jj * 5 + 0] = my_AC_data.byte[0];
        my_usart1_tx_buf1[15 + jj * 5 + 1] = my_AC_data.byte[1];
        my_usart1_tx_buf1[15 + jj * 5 + 2] = my_AC_data.byte[2];
        my_usart1_tx_buf1[15 + jj * 5 + 3] = my_AC_data.byte[3];
        my_usart1_tx_buf1[15 + jj * 5 + 4] = 0;

        my_usart1_tx_buf1[15 + 20 + jj * 5 + 0] = my_temperature_data.byte[0];
        my_usart1_tx_buf1[15 + 20 + jj * 5 + 1] = my_temperature_data.byte[1];
        my_usart1_tx_buf1[15 + 20 + jj * 5 + 2] = my_temperature_data.byte[2];
        my_usart1_tx_buf1[15 + 20 + jj * 5 + 3] = my_temperature_data.byte[3];
        my_usart1_tx_buf1[15 + 20 + jj * 5 + 4] = 0;

        my_usart1_tx_buf1[15 + 40 + jj * 5 + 0] = my_vol_data.byte[0];
        my_usart1_tx_buf1[15 + 40 + jj * 5 + 1] = my_vol_data.byte[1];
        my_usart1_tx_buf1[15 + 40 + jj * 5 + 2] = my_vol_data.byte[2];
        my_usart1_tx_buf1[15 + 40 + jj * 5 + 3] = my_vol_data.byte[3];
        my_usart1_tx_buf1[15 + 40 + jj * 5 + 4] = 0;


    }
    if(shibiao == 7)
    {
        my_usart1_tx_buf1[15 + 54 + 1] = (my_RTC_time.Seconds * 1000);
        my_usart1_tx_buf1[15 + 54 + 2] = ((my_RTC_time.Seconds * 1000) >> 8);
        my_usart1_tx_buf1[15 + 54 + 3] = my_RTC_time.Minutes;
        my_usart1_tx_buf1[15 + 54 + 4] = my_RTC_time.Hours;
        my_usart1_tx_buf1[15 + 54 + 5] = my_RTC_date.Date || (my_RTC_date.WeekDay << 5);
        my_usart1_tx_buf1[15 + 54 + 6] = my_RTC_date.Month;
        my_usart1_tx_buf1[15 + 54 + 7] = my_RTC_date.Year;
    }


//        printf("ZSQ[%d]-A=%.1f,E=%.1f\n", jj + 1,
//               (my_indicator_data[jj].AC_data_buf[1] * 256 + my_indicator_data[jj].AC_data_buf[0]) / 10.0,
//               (my_indicator_data[jj].AC_data_buf[3] * 256 + my_indicator_data[jj].AC_data_buf[2]) / 10.0
//              );

    //ϵͳ�����������������ݴ���  0XFB��ָʾ������Ϊ0X11
    if(my_system_restart_status == 1)
    {
    }
    my_system_restart_status = 0;


    wdz_GPRS_101check_generate(my_usart1_tx_buf1);
    my_at_senddata(my_usart1_tx_buf1);

    printf("my_GPRS send CYC data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);


}

void  my_fun_GPRS_TX_ALarm_data_yaoxin(void)
{
    uint8_t  jj = 0;;
    uint8_t x1 = 0, x2 = 0, x3 = 0, x_count = 0;

    if(my_GPRS_all_step == 0X9100 ) //ң��-��ʱ��
    {
        if(my_indicator_tx_index == 0)  //
        {
            return;
        }
        //===========

        jj = my_indicator_tx_index;
        if(jj & 0X01 == 0x01) x1 = 1;
        if(jj & 0X02 == 0x02) x2 = 1;
        if(jj & 0x04 == 0x04) x3 = 1;


        //====A��===
        if(x1 == 1)
        {
            if(my_indicator_alarm_data[00].TX_status_duanlu >= 0X01)
            {

                //��·״̬
                if(my_indicator_alarm_data[00].duanlu_data == 0X01)
                {   x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x13; //�����Զ�·
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;
                }
                else if(my_indicator_alarm_data[00].duanlu_data == 0X41 || my_indicator_alarm_data[00].duanlu_data == 0X81)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x12; //˲ʱ���Զ�·
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;

                }
                else if(my_indicator_alarm_data[00].duanlu_data == 0)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x13; //�����Զ�·�ָ���
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x18; //ͣ��
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;



                }
                else if(my_indicator_alarm_data[00].duanlu_data == 0XFE)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x18; //ͣ��
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;

                }

            }
            if(my_indicator_alarm_data[00].TX_status_jiedi >= 0X01)
            {
                if(my_indicator_alarm_data[00].TX_status_jiedi == 0X01)
                {   x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x14; //�ӵ�
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;
                }
                else if(my_indicator_alarm_data[00].TX_status_jiedi == 0X00)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x14; //�ӵ�
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;

                }
            }

        }

        //===B��

        if(x2 == 1)
        {
            if(my_indicator_alarm_data[01].TX_status_duanlu >= 0X01)
            {

                //��·״̬
                if(my_indicator_alarm_data[01].duanlu_data == 0X01)
                {   x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x23; //�����Զ�·
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;
                }
                else if(my_indicator_alarm_data[01].duanlu_data == 0X41 || my_indicator_alarm_data[01].duanlu_data == 0X81)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x22; //˲ʱ���Զ�·
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;

                }
                else if(my_indicator_alarm_data[01].duanlu_data == 0)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x23; //�����Զ�·�ָ���
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x28; //ͣ��
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;



                }
                else if(my_indicator_alarm_data[01].duanlu_data == 0XFE)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x28; //ͣ��
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;

                }

            }
            if(my_indicator_alarm_data[1].TX_status_jiedi >= 0X01)
            {
                if(my_indicator_alarm_data[1].jiedi_data == 0X01)
                {   x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x24; //�ӵ�
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;
                }
                else if(my_indicator_alarm_data[1].jiedi_data == 0X00)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x24; //�ӵ�
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;

                }
            }

        }



        //---C��
        if(x3 == 1)
        {
            if(my_indicator_alarm_data[02].TX_status_duanlu >= 0X01)
            {

                //��·״̬
                if(my_indicator_alarm_data[02].duanlu_data == 0X01)
                {   x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x33; //�����Զ�·
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;
                }
                else if(my_indicator_alarm_data[02].duanlu_data == 0X41 || my_indicator_alarm_data[02].duanlu_data == 0X81)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x32; //˲ʱ���Զ�·
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;

                }
                else if(my_indicator_alarm_data[02].duanlu_data == 0)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x33; //�����Զ�·�ָ���
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x38; //ͣ��
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;



                }
                else if(my_indicator_alarm_data[02].duanlu_data == 0XFE)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x38; //ͣ��
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;

                }

            }
            if(my_indicator_alarm_data[2].TX_status_jiedi >= 0X01)
            {
                if(my_indicator_alarm_data[2].jiedi_data == 0X01)
                {   x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x34; //�ӵ�
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;
                }
                else if(my_indicator_alarm_data[2].jiedi_data == 0X00)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x34; //�ӵ�
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;

                }
            }

        }






        //======

        my_usart1_tx_buf1[0] = 0x68;
        my_usart1_tx_buf1[3] = 0x68;
        my_usart1_tx_buf1[1] = 9 + x_count * 3 + 7; //����
        my_usart1_tx_buf1[2] = 9 + x_count * 3 + 7;


        //�������봦��
        my_101_DIR = 0X80;
        my_101_PRM = 0X40;
        if(my_GPRS_all_count == 1)
            my_101_FCB = (~my_101_FCB) & 0X20;
        my_101_FCV = 0X10;
        my_101_FC = 0X03;

        my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73


        my_usart1_tx_buf1[5] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

        my_101_TI = 31;
        my_101_VSQ_1_7 = x_count;
        my_101_VSQ_8_SQ = 0x00;
        my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
        my_101_COT_low = 0x03;
        my_101_COT_high = 0;

        my_usart1_tx_buf1[7] = my_101_TI;
        my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //��Ϣ�����
        my_usart1_tx_buf1[9] = my_101_COT_low; //����ԭ��
        my_usart1_tx_buf1[10] = my_101_COT_high;

        my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
        my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

        x_count++;
        HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN); //��ȡRTCʱ��
        HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);

        my_usart1_tx_buf1[13 + x_count * 3 + 0] =  (my_RTC_time.Seconds * 1000); //���ݲ���
        my_usart1_tx_buf1[13 + x_count * 3 + 1] = ((my_RTC_time.Seconds * 1000) >> 8);
        my_usart1_tx_buf1[13 + x_count * 3 + 2] = my_RTC_time.Minutes;
        my_usart1_tx_buf1[13 + x_count * 3 + 3] = my_RTC_time.Hours;
        my_usart1_tx_buf1[13 + x_count * 3 + 4] = (my_RTC_date.Date + (my_RTC_date.WeekDay << 5));
        my_usart1_tx_buf1[13 + x_count * 3 + 5] = my_RTC_date.Month;
        my_usart1_tx_buf1[13 + x_count * 3 + 6] = my_RTC_date.Year;

        my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

        return;


    }






}



void  my_fun_GPRS_TX_ALarm_data_yaoce(void)
{

    uint8_t jj = 0; //ʱ�곤��
    uint8_t length = 0;
    union MY_float my_AC_data ;
    //my_gprs_generate_101analog_data(1, my_usart1_tx_buf1, shibiao, 0x01);

    //=============================
    length = 3;

    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = length * 5 + 11; //12*5+11=61
    my_usart1_tx_buf1[2] = length * 5 + 11;

    //�������봦��
    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73

    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_usart1_tx_buf1[7] = 13; //���������̸�����
    my_usart1_tx_buf1[8] = length + 0x80; //��Ϣ�����
    my_usart1_tx_buf1[9] = 3; //����ԭ��
    my_usart1_tx_buf1[10] = 00; //����ԭ��

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x11; //ң����Ϣ���׵�ַ
    my_usart1_tx_buf1[14] = 0x40;


    my_usart1_tx_buf1[4 + 11 + length * 5 ] = 0XFF;
    my_usart1_tx_buf1[4 + 11  + length * 5 + 1 ] = 0x16;

    //==================================
    for(jj = 0; jj < length * 5; jj++)
    {
        my_usart1_tx_buf1[15 + jj] = 0;
    }

    for(jj = 0; jj < 3; jj++)
    {
        my_AC_data.value = (my_indicator_data[jj].AC_data_buf[0] + (my_indicator_data[jj].AC_data_buf[1] << 8)) / 10.0;
        //my_vol_data.value = (my_indicator_data[jj].DC_data_buf[12] + (my_indicator_data[jj].DC_data_buf[13] << 8)) / 10.0;
        //my_temperature_data.value = (my_indicator_data[jj].DC_data_buf[0] + (my_indicator_data[jj].DC_data_buf[1] << 8)) / 10.0;

        my_usart1_tx_buf1[15 + jj * 5 + 0] = my_AC_data.byte[0];
        my_usart1_tx_buf1[15 + jj * 5 + 1] = my_AC_data.byte[1];
        my_usart1_tx_buf1[15 + jj * 5 + 2] = my_AC_data.byte[2];
        my_usart1_tx_buf1[15 + jj * 5 + 3] = my_AC_data.byte[3];
        my_usart1_tx_buf1[15 + jj * 5 + 4] = 0;



    }



    wdz_GPRS_101check_generate(my_usart1_tx_buf1);
    my_at_senddata(my_usart1_tx_buf1);

    printf("my_GPRS send CYC data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);



}


//���ϱ�����ң�ź�ң��ϲ���һ��
void  my_fun_GPRS_TX_ALarm_data_yaoxin_yaoce(void)
{

    uint8_t jj = 0;;
    uint8_t x1 = 0, x2 = 0, x3 = 0, x_count = 0;
    uint8_t my_add_star_yaoxin = 0;
    uint8_t length = 0;
    union MY_float my_AC_data ;

    //ң��-��ʱ��

    if(my_indicator_tx_index == 0)  //
    {
        return;
    }
    //===========

    jj = my_indicator_tx_index;
    if(jj & 0X01 == 0x01) x1 = 1;
    if(jj & 0X02 == 0x02) x2 = 1;
    if(jj & 0x04 == 0x04) x3 = 1;

    my_add_star_yaoxin = 15;
    //====A��===
    if(x1 == 1)
    {
        if(my_indicator_alarm_data[00].TX_status_duanlu >= 0X01)
        {

            //��·״̬
            if(my_indicator_alarm_data[00].duanlu_data == 0X01)
            {   x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x13; //�����Զ�·
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_ERROR;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[00].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[00].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[00].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[00].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[00].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[00].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[00].RTC_time_buf[6];

            }
            else if(my_indicator_alarm_data[00].duanlu_data == 0X41 || my_indicator_alarm_data[00].duanlu_data == 0X81)
            {

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x12; //˲ʱ���Զ�·
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_ERROR;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[00].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[00].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[00].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[00].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[00].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[00].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[00].RTC_time_buf[6];

            }
            else if(my_indicator_alarm_data[00].duanlu_data == 0)
            {

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x13; //�����Զ�·�ָ���
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_OK;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[00].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[00].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[00].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[00].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[00].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[00].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[00].RTC_time_buf[6];

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x18; //ͣ��
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_OK;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[00].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[00].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[00].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[00].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[00].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[00].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[00].RTC_time_buf[6];



            }
            else if(my_indicator_alarm_data[00].duanlu_data == 0XFE)
            {

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x18; //ͣ��
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_ERROR;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[00].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[00].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[00].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[00].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[00].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[00].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[00].RTC_time_buf[6];

            }

        }
        if(my_indicator_alarm_data[00].TX_status_jiedi >= 0X01)
        {
            if(my_indicator_alarm_data[00].TX_status_jiedi == 0X01)
            {   x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x14; //�ӵ�
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_ERROR;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[00].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[00].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[00].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[00].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[00].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[00].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[00].RTC_time_buf[6];

            }
            else if(my_indicator_alarm_data[00].TX_status_jiedi == 0X00)
            {

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x14; //�ӵ�
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_OK;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[00].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[00].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[00].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[00].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[00].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[00].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[00].RTC_time_buf[6];

            }
        }

    }

    //===B��

    if(x2 == 1)
    {
        if(my_indicator_alarm_data[1].TX_status_duanlu >= 0X01)
        {

            //��·״̬
            if(my_indicator_alarm_data[1].duanlu_data == 0X01)
            {   x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x23; //�����Զ�·
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_ERROR;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[1].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[1].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[1].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[1].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[1].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[1].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[1].RTC_time_buf[6];
            }
            else if(my_indicator_alarm_data[1].duanlu_data == 0X41 || my_indicator_alarm_data[1].duanlu_data == 0X81)
            {

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x22; //˲ʱ���Զ�·
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_ERROR;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[1].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[1].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[1].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[1].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[1].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[1].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[1].RTC_time_buf[6];

            }
            else if(my_indicator_alarm_data[01].duanlu_data == 0)
            {

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x23; //�����Զ�·�ָ���
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_OK;

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x28; //ͣ��
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_OK;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[1].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[1].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[1].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[1].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[1].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[1].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[1].RTC_time_buf[6];



            }
            else if(my_indicator_alarm_data[01].duanlu_data == 0XFE)
            {

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x28; //ͣ��
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_ERROR;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[1].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[1].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[1].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[1].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[1].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[1].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[1].RTC_time_buf[6];

            }

        }
        if(my_indicator_alarm_data[1].TX_status_jiedi >= 0X01)
        {
            if(my_indicator_alarm_data[1].jiedi_data == 0X01)
            {   x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x24; //�ӵ�
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_ERROR;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[1].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[1].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[1].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[1].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[1].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[1].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[1].RTC_time_buf[6];
            }
            else if(my_indicator_alarm_data[1].jiedi_data == 0X00)
            {

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x24; //�ӵ�
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_OK;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[1].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[1].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[1].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[1].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[1].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[1].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[1].RTC_time_buf[6];

            }
        }

    }



    //---C��
    if(x3 == 1)
    {
        if(my_indicator_alarm_data[2].TX_status_duanlu >= 0X01)
        {

            //��·״̬
            if(my_indicator_alarm_data[2].duanlu_data == 0X01)
            {   x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x33; //�����Զ�·
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_ERROR;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[2].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[2].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[2].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[2].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[2].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[2].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[2].RTC_time_buf[6];
            }
            else if(my_indicator_alarm_data[2].duanlu_data == 0X41 || my_indicator_alarm_data[02].duanlu_data == 0X81)
            {

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x32; //˲ʱ���Զ�·
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_ERROR;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[2].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[2].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[2].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[2].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[2].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[2].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[2].RTC_time_buf[6];

            }
            else if(my_indicator_alarm_data[2].duanlu_data == 0)
            {

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x33; //�����Զ�·�ָ���
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_OK;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[2].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[2].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[2].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[2].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[2].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[2].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[2].RTC_time_buf[6];

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x38; //ͣ��
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_OK;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[2].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[2].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[2].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[2].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[2].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[2].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[2].RTC_time_buf[6];



            }
            else if(my_indicator_alarm_data[02].duanlu_data == 0XFE)
            {

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x38; //ͣ��
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_ERROR;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[2].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[2].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[2].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[2].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[2].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[2].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[2].RTC_time_buf[6];

            }

        }
        if(my_indicator_alarm_data[2].TX_status_jiedi >= 0X01)
        {
            if(my_indicator_alarm_data[2].jiedi_data == 0X01)
            {   x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x34; //�ӵ�
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_ERROR;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[2].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[2].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[2].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[2].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[2].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[2].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[2].RTC_time_buf[6];
            }
            else if(my_indicator_alarm_data[2].jiedi_data == 0X00)
            {

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x34; //�ӵ�
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_OK;

                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 3] = my_indicator_alarm_data[2].RTC_time_buf[0];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 4] = my_indicator_alarm_data[2].RTC_time_buf[1];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 5] = my_indicator_alarm_data[2].RTC_time_buf[2];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 6] = my_indicator_alarm_data[2].RTC_time_buf[3];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 7] = my_indicator_alarm_data[2].RTC_time_buf[4];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 8] = my_indicator_alarm_data[2].RTC_time_buf[5];
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 9] = my_indicator_alarm_data[2].RTC_time_buf[6];

            }
        }

    }


    //ң�����ݣ�û��ʱ��
    length = 3;
    for(jj = 0; jj < length * 6; jj++)
    {
        my_usart1_tx_buf1[my_add_star_yaoxin + (x_count + 1) * 10 + 2 + jj] = 0;
    }

    for(jj = 0; jj < 3; jj++)
    {
        my_AC_data.value = (my_indicator_data[jj].AC_data_buf[0] + (my_indicator_data[jj].AC_data_buf[1] << 8)) / 10.0;
        //my_vol_data.value = (my_indicator_data[jj].DC_data_buf[12] + (my_indicator_data[jj].DC_data_buf[13] << 8)) / 10.0;
        //my_temperature_data.value = (my_indicator_data[jj].DC_data_buf[0] + (my_indicator_data[jj].DC_data_buf[1] << 8)) / 10.0;

        my_usart1_tx_buf1[my_add_star_yaoxin + (x_count + 1) * 10 + 2 + jj * 6 + 0 ] = 0x11 + jj;
        my_usart1_tx_buf1[my_add_star_yaoxin + (x_count + 1) * 10 + 2 + jj * 6 + 1 ] = 0X40;
        my_usart1_tx_buf1[my_add_star_yaoxin + (x_count + 1) * 10 + 2 + jj * 6 + 2 ] = my_AC_data.byte[0];
        my_usart1_tx_buf1[my_add_star_yaoxin + (x_count + 1) * 10 + 2 + jj * 6 + 3 ] = my_AC_data.byte[1];
        my_usart1_tx_buf1[my_add_star_yaoxin + (x_count + 1) * 10 + 2 + jj * 6 + 4 ] = my_AC_data.byte[2];
        my_usart1_tx_buf1[my_add_star_yaoxin + (x_count + 1) * 10 + 2 + jj * 6 + 5 ] = my_AC_data.byte[3];

    }




    //======

    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = 9 + 2 + x_count * 10 + 2 + length * 6; //����
    my_usart1_tx_buf1[2] = 9 + 2 + x_count * 10 + 2 + length * 6;


    //�������봦��
    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 42;
    my_101_VSQ_1_7 = x_count + 3;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x03;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI;
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //��Ϣ�����
    my_usart1_tx_buf1[9] = my_101_COT_low; //����ԭ��
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

    my_usart1_tx_buf1[13] = x_count;
    my_usart1_tx_buf1[14] = 3;

    my_usart1_tx_buf1[my_add_star_yaoxin + (x_count + 1) * 10] = length;
    my_usart1_tx_buf1[my_add_star_yaoxin + (x_count + 1) * 10 + 1] = 13;



    my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1);
    printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);






}


void  my_fun_GPRS_TX_CYC4_B(void)  //  ����
{
    union MY_float my_DTU_vol_data;
    uint8_t my_cot = 0;
    my_cot = 1;

    my_gprs_generate_101MCU_data(1, my_usart1_tx_buf1, my_cot);

    my_DTU_vol_data.value = MY_Bat_value / 10.0;
    my_usart1_tx_buf1[15 + 0] = my_DTU_vol_data.byte[0];
    my_usart1_tx_buf1[15 + 1] = my_DTU_vol_data.byte[1];
    my_usart1_tx_buf1[15 + 2] = my_DTU_vol_data.byte[2];
    my_usart1_tx_buf1[15 + 3] = my_DTU_vol_data.byte[3];
    my_usart1_tx_buf1[15 + 4] = 0;


    wdz_GPRS_101check_generate(my_usart1_tx_buf1);

    my_at_senddata(my_usart1_tx_buf1);

    printf("my_GPRS send CYC data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

}


//  �ٻ�Ŀ¼
void  my_fun_GPRS_TX_catalog(void)
{


    uint8_t mylength = 0; //�ļ��������ݰ�
    uint8_t my_file_name_lenth = 0; //�ļ����ֳ���	==7��17��8
    uint32_t my_file_data_count = 0; //�ļ������ֽ�����

    //�ļ�Ŀ¼�ж�
    if(strcmp(my_file_catalog_buf, "HISTORY/SOE") == 0)
    {
        my_file_name_lenth = 7;

    }
    else if(strcmp(my_file_catalog_buf, "HISTORY/FIXPT") == 0)
    {
        my_file_name_lenth = 17;

    }
    else if(strcmp(my_file_catalog_buf, "HISTORY/ULOG") == 0)
    {
        my_file_name_lenth = 8;

    }
    else
    {
        my_file_name_lenth = 0;
    }
    //�ļ������ж�


    if(my_file_name_lenth == 7 && my_file_catalog_status == 0) //�����ļ�
    {
        my_file_data_count = 256;
        mylength = 9 + my_file_name_lenth + 12;
    }
    else if(my_file_name_lenth == 17 && my_file_catalog_status == 0) //�����ļ�
    {
        my_file_data_count = 300;
        mylength = 9 + my_file_name_lenth + 12;
    }
    else if(my_file_name_lenth == 8 && my_file_catalog_status == 0) //�����ļ�
    {

        my_file_data_count = 400;
        mylength = 9 + my_file_name_lenth + 12;
    }
    else if(my_file_catalog_status == 1) //ʱ�����䷶Χ�ڵ��ļ�
    {
        //�Ȳ�����
        my_file_data_count = 0;
        mylength = 8;
        //���������ж��ļ����ͣ�Ȼ���ȡ��Ӧ���ļ������ļ����Ƿ������ʱ��Σ�����������ݴ���û���򷵻�0


    }

    else //�������ļ�
    {
        my_file_data_count = 0;
        mylength = 8;
    }


    my_fun_GPRS_TX_OK_80(); //����ݻظ�OK֡




    //======

    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = 9 + 3 + mylength; //����
    my_usart1_tx_buf1[2] = 9 + 3 + mylength;


    //�������봦��
    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 126;
    my_101_VSQ_1_7 = 1;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 5;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI;
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //��Ϣ�����
    my_usart1_tx_buf1[9] = my_101_COT_low; //����ԭ��
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

    my_usart1_tx_buf1[13] = 00;
    my_usart1_tx_buf1[14] = 00;

    my_usart1_tx_buf1[15] = 2; //��Ŀ¼

    //===========�ļ��������ݰ�������======
    if(my_file_data_count != 0) //�����ļ�
    {
        my_usart1_tx_buf1[15 + 1] = 2;
        my_usart1_tx_buf1[15 + 2] = 0; //0��ʾ�ɹ���1��ʾʧ�ܣ�û���ļ�

        my_usart1_tx_buf1[15 + 3] = 0;
        my_usart1_tx_buf1[15 + 4] = 0;
        my_usart1_tx_buf1[15 + 5] = 0;
        my_usart1_tx_buf1[15 + 6] = 0;

        my_usart1_tx_buf1[15 + 7] = 0;
        my_usart1_tx_buf1[15 + 8] = 1; //��֡�ļ�����



        my_usart1_tx_buf1[15 + 9] = my_file_name_lenth; //�ļ�������  7,17,8
        if(my_file_name_lenth == 7)
        {
            strcpy((char *)(my_usart1_tx_buf1[24 + 1]), "soe.msg");


        }
        else if(my_file_name_lenth == 17)
        {
            strcpy((char *)(my_usart1_tx_buf1[24 + 1]), "fixpt20180303.msg");
        }
        else if(my_file_name_lenth == 8)
        {
            strcpy((char *)(my_usart1_tx_buf1[24 + 1]), "ulog.msg");
        }
        else
        {
            strcpy((char *)(my_usart1_tx_buf1[24 + 1]), "none.msg");

        }


        my_usart1_tx_buf1[25 + my_file_name_lenth + 1] = 0;

        my_usart1_tx_buf1[25 + my_file_name_lenth + 2] = my_file_data_count;
        my_usart1_tx_buf1[25 + my_file_name_lenth + 3] = my_file_data_count >> 8;
        my_usart1_tx_buf1[25 + my_file_name_lenth + 4] = my_file_data_count >> 16;
        my_usart1_tx_buf1[25 + my_file_name_lenth + 5] = my_file_data_count >> 24;


        HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN);
        HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);
        my_usart1_tx_buf1[25 + my_file_name_lenth + 6] = my_RTC_time.Seconds;
        my_usart1_tx_buf1[25 + my_file_name_lenth + 7] = 0X00;
        my_usart1_tx_buf1[25 + my_file_name_lenth + 9] = my_RTC_time.Minutes;
        my_usart1_tx_buf1[25 + my_file_name_lenth + 10] = my_RTC_time.Hours;
        my_usart1_tx_buf1[25 + my_file_name_lenth + 11] = my_RTC_date.Date;
        my_usart1_tx_buf1[25 + my_file_name_lenth + 12] = my_RTC_date.Month;
        my_usart1_tx_buf1[25 + my_file_name_lenth + 13] = my_RTC_date.Year;

    }

    else //�������ļ�
    {
        my_usart1_tx_buf1[15 + 1] = 2;
        my_usart1_tx_buf1[15 + 2] = 1; //0��ʾ�ɹ���1��ʾʧ�ܣ�û���ļ�

        my_usart1_tx_buf1[15 + 3] = 0;
        my_usart1_tx_buf1[15 + 4] = 0;
        my_usart1_tx_buf1[15 + 5] = 0;
        my_usart1_tx_buf1[15 + 6] = 0;

        my_usart1_tx_buf1[15 + 7] = 0;
        my_usart1_tx_buf1[15 + 8] = 0; //��֡�ļ�����

    }
    //=================
    my_usart1_tx_buf1[15 + mylength + 1] = 0XFF;
    my_usart1_tx_buf1[15 + mylength + 2] = 0X16;

    my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1);
    printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

}



void  my_fun_GPRS_TX_file_data_1(void) //��ȡ�ļ�����ȷ��
{
    //����ȷ��OK�ļ�����֡
		my_fun_GPRS_TX_OK_80(); //�ظ�OK֡

    //�����ļ�����ȷ�ϣ���֡
	my_fun_GPRS_generate_68_fram(my_usart1_tx_buf1,(4+9+3+11+my_file_name_count),0x03,210,5,0X00,0X01);
	
	
	my_usart1_tx_buf1[4+9+1]=0X00;
	my_usart1_tx_buf1[4+9+2]=0X00;
	my_usart1_tx_buf1[4+9+3]=0X02;
	
	my_usart1_tx_buf1[4+9+3+1]=4;
	my_usart1_tx_buf1[4+9+3+2]=0;
	my_usart1_tx_buf1[4+9+3+3]=my_file_name_count;

	
	//�ļ�����--��ʼ
	uint8_t kk=0;
	for(kk=0;kk<my_file_name_count;kk++)
	{
		my_usart1_tx_buf1[4+9+3+4+kk]=my_file_name_buf[kk];
		
	}
	
	//�ļ�����--����
	
	my_usart1_tx_buf1[4+9+3+3+my_file_name_count+1]=0;
	my_usart1_tx_buf1[4+9+3+3+my_file_name_count+2]=0;
	my_usart1_tx_buf1[4+9+3+3+my_file_name_count+3]=0;
	my_usart1_tx_buf1[4+9+3+3+my_file_name_count+4]=0;
	
	my_usart1_tx_buf1[4+9+3+3+my_file_name_count+4+1]=my_file_data_count;
	my_usart1_tx_buf1[4+9+3+3+my_file_name_count+4+2]=my_file_data_count>>8;
	my_usart1_tx_buf1[4+9+3+3+my_file_name_count+4+3]=my_file_data_count>>16;
	my_usart1_tx_buf1[4+9+3+3+my_file_name_count+4+4]=my_file_data_count>>24;
	
	
	my_fun_101check_generate(my_usart1_tx_buf1, 0);
  my_at_senddata(my_usart1_tx_buf1);

}

void  my_fun_GPRS_TX_file_data_2(void) //��ȡ�ļ�����������
{
		 //�����ļ�����ȷ�ϣ���֡
	my_fun_GPRS_generate_68_fram(my_usart1_tx_buf1,(4+9+3+11+my_file_name_count),0x03,210,5,0X00,0X01);
	
	my_usart1_tx_buf1[4+9+1]=0X00;
	my_usart1_tx_buf1[4+9+2]=0X00;
	my_usart1_tx_buf1[4+9+3]=0X02;
	
	my_usart1_tx_buf1[4+9+3+1]=5; //������ʶ
	
	//�ļ�ID
	my_usart1_tx_buf1[4+9+3+2]=0;
	my_usart1_tx_buf1[4+9+3+3]=0;
	my_usart1_tx_buf1[4+9+3+4]=0;
	my_usart1_tx_buf1[4+9+3+5]=0;
	
	//���ݶκ�
	my_usart1_tx_buf1[4+9+3+6]=0X01;
	my_usart1_tx_buf1[4+9+3+7]=0X02;
	my_usart1_tx_buf1[4+9+3+8]=0X03;
	my_usart1_tx_buf1[4+9+3+9]=0X04;
	
	//������ʶ
	my_usart1_tx_buf1[4+9+3+10]=1; //0��ʶ����û�����ݣ�1��ʶ������ʱ��
	
	//�ļ�����
	uint8_t my_temp_part_data_count=0;
	if(my_file_part_count!=1)
	{
		my_temp_part_data_count=my_file_part_data_count_aver; //���㱾�����ݳ���
	}
	else if(my_file_part_count==1)
	{
		my_temp_part_data_count=my_file_part_data_count_end ; //���㱾�����ݳ���
	}
	//�ļ��������
	uint8_t kk=0;
	for(kk=0;kk<my_temp_part_data_count;kk++)
	{
		my_usart1_tx_buf1[4+9+3+10+1+kk]=kk+my_file_part_count;
	}
	
	
	
		//У���룬�ļ����ݵ�У����
	uint8_t my_file_data_CRC=0;	
	for(kk=0;kk<my_temp_part_data_count;kk++)
	{
		my_file_data_CRC=my_file_data_CRC+my_usart1_tx_buf1[4+9+3+10+1+kk];
	}
	
	my_usart1_tx_buf1[4+9+3+10+my_temp_part_data_count+1]=my_file_data_CRC;
	
	
	//�ļ��������������
	my_fun_101check_generate(my_usart1_tx_buf1, 0);
  my_at_senddata(my_usart1_tx_buf1);

}

void  my_fun_GPRS_TX_file_data_3(void) //��ȡ�ļ�����������
{

    my_fun_GPRS_TX_OK_80(); //�ļ����գ��ظ�OK֡

    //����������������һ���ļ�����
		my_file_part_count--;
    if(my_file_part_count!= 0)
    {
        my_GPRS_all_step = 0;
        uint16_t my_step = 0X5500;
        xQueueSend(myQueue01Handle, &my_step, 100);	//��ʶ��һ��״̬,�����ļ�����
    }

}
/*
������
1���������׵�ַ
2.����
3.FC
4.TI
5.COT
6.VSQ_SQ
7.VSQ_1_7

*/

void my_fun_GPRS_generate_68_fram(uint8_t *my_buf,uint8_t my_length,uint8_t my_FC,uint8_t my_TI,uint8_t my_COT,uint8_t my_VSQ_SQ,uint8_t my_VSQ_1_7)
{


    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = my_length; //12*5+11=61
    my_usart1_tx_buf1[2] = my_length;

    //�������봦��
    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = my_FC;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //��������Ϊ53/73

    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_usart1_tx_buf1[7] = my_TI; //������ʶ�룬���ݷ�ʽ����һ����������ʱ��
    my_usart1_tx_buf1[8] = (my_VSQ_SQ|my_VSQ_1_7); //���з�ʽ����Ϣ�����
    my_usart1_tx_buf1[9] = my_COT; //����ԭ��
    my_usart1_tx_buf1[10] = 00; //����ԭ��

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //�������ַ
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x00; //ң����Ϣ���׵�ַ
    my_usart1_tx_buf1[14] = 0x00;


    my_usart1_tx_buf1[4+ my_length] = 0XFF;
    my_usart1_tx_buf1[4 + my_length + 1 ] = 0x16;

    //==================================

}

