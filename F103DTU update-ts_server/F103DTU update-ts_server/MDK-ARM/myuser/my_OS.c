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
//GPRS链路状态标识，0为没有建立链路，1为建立链路了
uint8_t my_101_DIR = 0X80; //101规约中,控制域的DIR位 最高位D8
uint8_t my_101_PRM = 0X40; //101规约中，控制域的PRM  D7
uint8_t my_101_FCB = 0; //控制域D6
uint8_t my_101_FCV = 0; //控制月D5

uint8_t my_101_FC = 0; //控制域，功能码
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
uint16_t query_data = 0X00; //查询发送的数据
uint16_t query_data2 = 0X00;
#define my_indicator_count 3 //系统中指示器数据
uint8_t my_tx_rec_count_all = 9; //发送总的段数
uint8_t my_tx_rec_count_finish = 0; //已经发送的段数

//周期数据使用
struct indicator_class my_indicator_data[my_indicator_count];
struct indicator_class_parameter my_indicator_parameter_data[3];
//报警数据使用
struct indicator_alarm_class my_indicator_alarm_data[my_indicator_count];
struct indicator_record_class my_indicator_record_data[3];

uint8_t my_indicator_tx_index = 99; //准备发送的报警数据指示器的编号0，1,2,3,4，，5,6，注意，编号从0开始


uint8_t my_CC1101_record_wave_last_index = 0; //记录获得的最新的录波的指示器编号，01，02,03
uint8_t my_CC1101_receive_cmd_ID = 0;

uint16_t my_gprs_count_time = 0; //GPRS通信，周期数据，传递给SERVER的DTU收到的zsq的计数值
uint8_t  my_gprs_RTC_buf[7] = {0};
uint8_t  my_get_data_buf[2] = {0};
char my_file_catalog_buf[15] = {0};
uint8_t my_file_catalog_status = 0; //0目录下所有文件，1满足时间段文件
uint8_t my_file_RTC_start_time[7] = {0}; //查询开始时间
uint8_t my_file_RTC_end_time[7] = {0}; //查询结束时间
uint16_t my_file_part = 0; //文件准备发送的 段 数量，0表示没有发送的段，最大65535


union MY_float
{
    float value;
    unsigned char byte[4];
};


//=====================函数部分
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
    EventBits_t	uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* 事件标志组句柄 */
                         0X01,            /* 等待bit0和bit1被设置 */
                         pdTRUE,             /* 退出前bit0和bit1被清除，这里是bit0和bit1都被设置才表示“退出”*/
                         pdTRUE,             /* 设置为pdTRUE表示等待bit1和bit0都被设置*/
                         200); 	 /* 等待延迟时间 */

    if((uxBits & 0x01) == 0x01)
    {
        /* 接收到bit1和bit0都被设置的消息 */
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
        /* 接收到bit1和bit0都被设置的消息 */
        printf("BinarySem01 take ok！\r\n");
    }

}

void my_fun_give_Queue1(void)
{
    temp8 = 0;
    temp8++;

    my_GPRS_all_step = temp8;
    BaseType_t pt = NULL;
    BaseType_t xResult;
    xResult = xQueueSendFromISR(myQueue01Handle, &temp8, &pt); //队列1对应，M35的发送状态队列

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
    xResult = xQueueSendFromISR(myQueue03Handle, &temp_step, &pt); //队列1对应，M35的发送状态队列

    if(xResult != pdFAIL)
    {
        portYIELD_FROM_ISR(pt);
    }
    //printf("queue3 is %d\r\n",temp8);
}
/*

功能：向队列发送数据
参数1，队列指针，参数2，发送的数据16位
*/

void my_fun_give_Queue(osMessageQId *my_QHL, uint16_t temp_step)
{
    BaseType_t pt = NULL;
    BaseType_t xResult;
    xResult = xQueueSendFromISR(*my_QHL, &temp_step, &pt); //队列1对应，M35的发送状态队列

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
        /* 接收到bit1和bit0都被设置的消息 */
        printf("queue is =%d\r\n", xx);
    }

}


//=======================

//=====GPRS对话发送 接收使用==
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
    //无条件处理接收到的数据
    if(my_get_step == my_now_step && my_before_step == 0x00)
    {
        my_status = 1;
    }
    //条件绑定，有前提条件
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
        my_GPRS_all_step = my_now_step;	//当前状态
        my_temp = ptfun(); //过程处理用的，接收处理函数
        GPRS_Heartdata_error_count = 0;
        if(my_temp == 1)
        {
            //my_GPRS_all_step = my_now_step;	//当前状态
            xQueueSend(*QHL_send, &my_next_step, 100);	//标识下一个状态
        }
        else
        {   my_GPRS_all_step = my_old_step;
            printf("GPRS接收数据错误，不进行状态转换\r\n");
        }
    }
    else if(my_status == 1 && end_status == 1)
    {
        // printf("CC1101 RX-step = [%XH]\r\n",my_now_step);
        my_GPRS_all_step = my_now_step;
        ptfun();	 //结束用的，接收处理函数
        my_GPRS_all_step = 0X00; //结束状态
        GPRS_Heartdata_error_count = 0;
        my_gprs_TX_status = 0;

    }

    //

}

//=====GPRS对话发送 发送使用==
void my_fun_gprs_time_dialog_tx(
    uint16_t my_get_step,  //获得状态
    uint16_t my_before_step,  //前一个状态
    uint16_t my_now_step,   //当前准备进行状态
    uint8_t end_status,   //结束状态
    void (*ptfun)(void)
)
{

    //===判断部分
    if(my_get_step == my_now_step && my_before_step == 0XFF00) //设置前一个状态为0XFF，则可以中断之前的对话，进入新对话，否则只能等待前次对话结束
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

    //===重复发送部分
    if( my_now_step == my_GPRS_all_step && my_GPRS_all_count < 3)
    {
        my_GPRS_all_count++;
        printf("GPRS TX-Fun= [%XH]--%d\r\n", my_now_step, my_GPRS_all_count);
        //osDelay(1);
        //HAL_Delay(1);//GPRS发送数据，要延时一下，因为操作系统任务切换需要1ms
        ptfun();		//调用对应的函数
    }
    else if(my_GPRS_all_count >= 3)
    {
        my_GPRS_all_count = 0;
        my_GPRS_all_step = 0x00;
        GPRS_Heartdata_error_count++;
        my_gprs_TX_status = 0;
    }

    //====只发送一次就结束
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
    if( my_now_step == my_CC1101_all_step && my_CC1101_all_count < 3 &&  my_now_step != 0)
    {
        my_CC1101_all_count++;
        printf("CC1101 TX-Fun[%d]= [%XH]--%d\r\n", my_cc1101_dest_address, my_now_step, my_CC1101_all_count);
        //osDelay(1);
        //HAL_Delay(1);//CC1101发送数据，要延时一下，因为操作系统任务切换需要1ms
        ptfun();		//调用对应的函数
    }
    else if(my_CC1101_all_count >= 3)
    {
        my_CC1101_all_count = 0;
        my_CC1101_all_step = 0x00;
    }

    //====只发送一次就结束
    if(end_status == 1 && my_CC1101_all_count > 0 && my_CC1101_all_step == my_now_step)
    {
        my_CC1101_all_count = 0;
        my_CC1101_all_step = 0x00;
    }

}

//=====CC1101对话 接收使用
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
    if(my_get_step == my_now_step && my_before_step == 0x00) //无条件处理接收到的数据
    {
        my_status = 1;
    }
    else if(my_get_step == my_now_step && my_before_step == my_CC1101_all_step) //条件绑定，有前提条件
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
            my_CC1101_all_step = my_now_step;	//当前状态
            xQueueSend(*QHL_send, &my_next_step, 100);	//标识下一个状态
        }
        else
        {   my_CC1101_all_step = my_old_setp;
            printf("接收数据错误，不进行状态转换\r\n");
        }
    }
    else if(my_status == 1 && end_status == 1)
    {

        ptfun();
        my_CC1101_all_step = 0X00; //结束状态

    }

    //

}


//=========

//=====GPRS 发送处理函数======
/*
功能:GPRS发送心跳包
*/
uint8_t my_GPRS_heart_count = 0;
extern uint16_t DTU_ADDRESS;
uint8_t my_cc1101_error_count = 0;



/*
功能：CC1101重新初始化
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
    if(my_status != 0x01 && my_status != 0x0D &&  my_status != 0x13  ) //0x11,接收溢出,//0X01空闲，0X0D接收，0X13发送
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
功能：串口重新初始化函数
*/

void my_fun_usart_init_resume(void)
{
    if(my_UART3_Status == 0X01)
    {
        MX_USART3_UART_Init();
        my_UART3_Status = 0;
        HAL_UART_Receive_IT(&huart3, &rsbuf3[rsbuf3pt_write], 1); //开启接收USART3函数
    }
    if(my_UART1_Status == 0X01)
    {
        MX_USART1_UART_Init();
        my_UART1_Status = 0;
        HAL_UART_Receive_IT(&huart1, &rsbuf1[rsbuf1pt_write], 1); //开启接收USART1函数
    }


}

/*
功能：显示操作系统每个任务剩余的堆栈区
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
功能：CC1101发送，处理函数，返回OK帧
*/
extern uint16_t my_tim6_count;
void my_fun_CC1101_test1(void)
{
    uint8_t *pt;
    uint16_t my_temp16 = my_tim6_count;
    pt = my_cc1101_tx_buf;
    pt[0] = 0x10;
    pt[1] = 0x20; //ID为0X20，表示OK帧
    pt[2] = my_CC1101_chip_address; //代表DTU的CC1101地址，为0XFE,，0XFD为调试器的地址，01,02,03---为指示器地址
    pt[3] = my_cc1101_dest_address; //目标地址

    pt[4] = my_temp16;                   //my_CC1101_receive_cmd_ID; //收到的指令的ID值
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
    my_fun_display_buf_16(pt, 8, 1); //测试使用
#endif

}

/*
功能：CC1101的接收，处理函数，周期数据，报警数据处理
*/
uint16_t my_ZSQ_time_count[3] = {0}; //指示器遥信数据，对应的全局计数值

uint8_t my_fun_dialog_CC1101_RX_1(void)
{
    uint8_t temp_status = 0;
    uint16_t my_length = 0;
    uint8_t my_address = 0;
//    uint8_t my_address_dest = 0;
    uint8_t my_indicator_index = 0;
    uint16_t ii = 0;
    uint8_t my_re = 0;

    if(my_CC1101_COM_Fram_buf[0] == 0x10) //10帧，
    {
        //获得指令ID
        temp_status = my_CC1101_COM_Fram_buf[1];
        my_CC1101_receive_cmd_ID = temp_status;
        //获得地址--发送源
        my_address = my_CC1101_COM_Fram_buf[2]; //帧中的发送源地址
        my_cc1101_dest_address = my_address; //修改CC1101的目的地址，为发送做准备使用

//        my_address_dest = my_CC1101_COM_Fram_buf[3]; //发送目的地址
#if Debug_uart_out_cc1101_rx_data_status==1
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //调试使用，显示接收到的数据8个字节
#endif
    }
    else if (my_CC1101_COM_Fram_buf[0] == 0x68) //68长帧
    {
        //获得指令ID
        temp_status = my_CC1101_COM_Fram_buf[6];
        printf("===============control word==[%X]\n", temp_status);
        my_CC1101_receive_cmd_ID = temp_status;

        my_length = my_CC1101_COM_Fram_buf[2];
        my_length = (my_length << 8) + my_CC1101_COM_Fram_buf[1] - 3; //获得长度

        //获得地址
        my_address = my_CC1101_COM_Fram_buf[7];
        my_cc1101_dest_address = my_address; //修改CC1101的目的地址，为发送做准备使用
//        my_address_dest = my_CC1101_COM_Fram_buf[8];

#if Debug_uart_out_cc1101_rx_data_status==1
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //调试使用，显示接收到的数据8个字节
#endif
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //@@@@调试使用，显示接收到的数据8个字节
    }

    //指示器地址判断
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
    //帧数据类型判断
    if(temp_status == 0x01)
    {
        my_indicator_data[my_indicator_index].data_type = 0x01; //周期
    }
    else if(temp_status == 0x02)
    {
        my_indicator_data[my_indicator_index].data_type = 0x02; //遥信

    }
    my_cc1101_tx_wait_time = 2000;
//==========周期、报警处理部分===========
    if(temp_status == 0x01 || temp_status == 0x02) //遥信--01为周期，02为报警
    {
        my_indicator_data[my_indicator_index].duanlu_data = my_CC1101_COM_Fram_buf[9]; //短路
        my_indicator_data[my_indicator_index].jiedi_data =	my_CC1101_COM_Fram_buf[11]; //接地
        //timer计数值
        my_ZSQ_time_count[my_indicator_index] = my_CC1101_COM_Fram_buf[12]; //计数值高字节
        my_ZSQ_time_count[my_indicator_index] = (my_ZSQ_time_count[my_indicator_index] << 8) + my_CC1101_COM_Fram_buf[10];//计数值低字节
        my_indicator_data[my_indicator_index].count_time[0] = my_CC1101_COM_Fram_buf[10]; //计数值低字节
        my_indicator_data[my_indicator_index].count_time[1] = my_CC1101_COM_Fram_buf[12]; //计数值高字节
        //RTC时间
        HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN);
        HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);
        my_indicator_data[my_indicator_index].RTC_time_buf[0] = my_RTC_time.Seconds; //RTC时间
        my_indicator_data[my_indicator_index].RTC_time_buf[1] = 0; //RTC时间
        my_indicator_data[my_indicator_index].RTC_time_buf[2] = my_RTC_time.Minutes; //RTC时间
        my_indicator_data[my_indicator_index].RTC_time_buf[3] = my_RTC_time.Hours; //RTC时间
        my_indicator_data[my_indicator_index].RTC_time_buf[4] = my_RTC_date.Date; //RTC时间
        my_indicator_data[my_indicator_index].RTC_time_buf[5] = my_RTC_date.Month; //RTC时间
        my_indicator_data[my_indicator_index].RTC_time_buf[6] = my_RTC_date.Year; //RTC时间

        //信号强度
        my_indicator_data[my_indicator_index].xinhao_db = my_RSSI_dbm_all;

        //

        my_gprs_count_time = my_indicator_data[my_indicator_index].count_time[1];
        my_gprs_count_time = (my_gprs_count_time << 8) + my_indicator_data[my_indicator_index].count_time[0];
        for(ii = 0; ii < 7; ii++)
        {
            my_gprs_RTC_buf[ii] = my_indicator_data[my_indicator_index].RTC_time_buf[ii];

        }


        //报警
        if(temp_status == 0x02)
        {


            //timer计数值

            my_indicator_alarm_data[my_indicator_index].count_time[0] = my_CC1101_COM_Fram_buf[10]; //计数值低字节
            my_indicator_alarm_data[my_indicator_index].count_time[1] = my_CC1101_COM_Fram_buf[12]; //计数值高字节
            //RTC时间

            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[0] = my_RTC_time.Seconds; //RTC时间
            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[1] = 0; //RTC时间
            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[2] = my_RTC_time.Minutes; //RTC时间
            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[3] = my_RTC_time.Hours; //RTC时间
            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[4] = my_RTC_date.Date; //RTC时间
            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[5] = my_RTC_date.Month; //RTC时间
            my_indicator_alarm_data[my_indicator_index].RTC_time_buf[6] = my_RTC_date.Year; //RTC时间




            //报警数据发送状态记录
            if(my_indicator_alarm_data[my_indicator_index].duanlu_data != 0 && my_CC1101_COM_Fram_buf[9] == 0)
                my_indicator_alarm_data[my_indicator_index].TX_status_duanlu = 0x01; //表示有报警数据产生，还没有发送。如果发送完了，把这个字节清零
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
            //更新报警状态
            my_indicator_alarm_data[my_indicator_index].duanlu_data = my_CC1101_COM_Fram_buf[9]; //短路
            my_indicator_alarm_data[my_indicator_index].jiedi_data =	my_CC1101_COM_Fram_buf[11]; //接地


        }



        my_re = 1;
    }
    else if(temp_status == 0x40 || temp_status == 0x50)	//周期DC，1温度，2电源，3参考电压，4干电池，5线上电压，6太阳能，7锂电池
    {
        for(ii = 0; ii < my_length; ii++)
        {
            my_indicator_data[my_indicator_index].DC_data_buf[ii] = my_CC1101_COM_Fram_buf[9 + ii];

        }
        //判断指示器是否重启，指示器0X11重启状态
        if(					my_indicator_data[my_indicator_index].DC_data_buf[0] == 0X11
                            && 	my_indicator_data[my_indicator_index].DC_data_buf[1] == 0X11
                            &&	my_indicator_data[my_indicator_index].DC_data_buf[2] == 0X11
                            &&	my_indicator_data[my_indicator_index].DC_data_buf[3] == 0X11
          )
        {
            //利用CC1101周期方式发送数据
            //GPRS主动发送数据，周期数据，发送到01号队列，对应04号任务
            if(my_GPRS_all_step == 0 && my_gprs_TX_status == 0)
            {
                printf("====GPRS CYC time ZSQ[%d] reset========\n", my_indicator_index);
                my_gprs_TX_status = 1;
                my_fun_give_Queue(&myQueue01Handle, 0XB100);

            }

        }

        //报警部分
        if(temp_status == 0x50)
        {

            for(ii = 0; ii < my_length; ii++)
            {
                my_indicator_alarm_data[my_indicator_index].DC_data_buf[ii] = my_CC1101_COM_Fram_buf[9 + ii];
            }


        }


        my_re = 1;
    }
    else if(temp_status == 0x41 || temp_status == 0x51) //AC有效值(全波电流，电场、半波电流)
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
        //报警部分
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
    else if(temp_status == 0x42 || temp_status == 0x52) //AC12T（电流、电场，半波电流）
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

        //报警部分
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
        //my_indicator_record_data[my_indicator_index].my_wave_type = 0x01; //电流全波
        my_indicator_record_data[my_indicator_index].my_wave_alam = 0x02; //报警
        my_indicator_record_data[my_indicator_index].my_wave_tx_status_I = 1;
        my_CC1101_record_wave_last_index = my_indicator_index; //记录获得最新的录波数据指示器编号

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
        //my_indicator_record_data[my_indicator_index].my_wave_type = 0x02; //电场全波
        my_indicator_record_data[my_indicator_index].my_wave_alam = 0x02; //报警
        my_indicator_record_data[my_indicator_index].my_wave_tx_status_E = 1;
        my_CC1101_record_wave_last_index = my_indicator_index; //记录获得最新的录波数据指示器编号

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
//====周期 报警 处理结束===========

//利用串口显示数据
#if Debug_Uart_out_DC_AC_DATA_Status==1

    double yy[7] = {0};
    //if(temp_status == 0x41 || temp_status == 0x51)
    if(temp_status == 0x42 || temp_status == 0x52)
    {
        //报警状态，2个，短路和接地
        printf("========START DC2============\n");
        printf("---ZSQ=[%d]--TIME=[%d]-[%d]-[%d]--local time=[%d]\n", my_address, my_ZSQ_time_count[0], my_ZSQ_time_count[1], my_ZSQ_time_count[2], my_tim6_count);
        printf("ALARM:duanlu=[%XH],jiedi=[%XH]\n", my_indicator_data[my_indicator_index].duanlu_data, my_indicator_data[my_indicator_index].jiedi_data);
        printf("RSSI=%d\n", my_indicator_data[my_indicator_index].xinhao_db);
        //直流量，7个
        for(ii = 0; ii < 7; ii++)
        {
            yy[ii] = (my_indicator_data[my_indicator_index].DC_data_buf[2 * ii] +
                      (my_indicator_data[my_indicator_index].DC_data_buf[2 * ii + 1] << 8)) / 10.0;
        }
        printf(" DC:Temp=%.2f,vbat=%.2f,vref=%.2f\n", yy[0], yy[1], yy[2]);
        printf(" DC:GANbat=%.2f,Zaixian=%.2f,sunbat=%.2f,Libat=%.2f\n", yy[3], yy[4], yy[5], yy[6]);
        //线上交流量，3个，电流全波，电场，电流半波
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
    if(temp_status == 0x53) //周期AC_record
    {
        printf("\n***960 data start—I*****\n");
        printf("my_indicator_index=%d\n", my_indicator_index + 1);
        for(ii = 0; ii < 960; ii++)
        {
            xx1 = (my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[2 * ii + 11]
                   + ((my_indicator_record_data[my_indicator_index].my_wave_record_I_buf[2 * ii + 1 + 11]) << 8)) / 10.0;
            printf("\n %.1f", xx1);

        }
        printf("\n***960 data end---I*****\n");
    }
    if(temp_status == 0x54 ) //周期AC_record
    {
        printf("\n***960 data start—E*****\n");
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


//===显示部分结束======
    return my_re;


}


//CC1101发送心跳帧
void my_fun_CC1101_send_heart_data(void)
{
    uint8_t *pt;
    uint16_t my_temp16_Broadaddress = 0XFF; //广播地址
    pt = my_cc1101_tx_buf;
    pt[0] = 0x10;
    pt[1] = 0x20; //ID为0X1F，表示心跳帧
    pt[2] = my_CC1101_chip_address; //代表DTU的CC1101地址，为0XFE,，0XFD为调试器的地址，01,02,03---为指示器地址
    pt[3] = my_temp16_Broadaddress;

    pt[4] = my_tim6_count; //收到的指令的ID值
    pt[5] = (my_tim6_count >> 8);
    pt[6] = my_fun_101check_generate(pt, 1);
    pt[7] = 0x16;



    CC1101SendPacket_add( pt, 8,  BROADCAST, my_temp16_Broadaddress);
#if Debug_uart_out_cc1101_tx_data_status==1
    my_fun_display_buf_16(pt, 8, 1); //测试使用
#endif

}


//========================
//==================GPRS 对话过程=============

void  my_fun_GPRS_TX_start1_server(void)
{

    //单字节
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

    //单字节
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

    //单字节
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

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 0X46;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x04;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI; //类型标识，带时标的遥测数据信息，
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //信息体个数
    my_usart1_tx_buf1[9] = my_101_COT_low; //传输原因
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x00; //遥信信息体首地址
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

    //单字节
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

    //单字节
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
    my_usart1_tx_buf1[1] = 0x0C; //0X24 共36个信息体，每个信息体3个字节
    my_usart1_tx_buf1[2] = 0x0C;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 0X46;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0X00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x04;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI; //类型标识，带时标的遥测数据信息，
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //信息体个数
    my_usart1_tx_buf1[9] = my_101_COT_low; //传输原因
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x00; //遥信信息体首地址
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

    //单字节
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

    //单字节
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
    //单字节
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
    my_usart1_tx_buf1[1] = 0x12; //0X24 共36个信息体，每个信息体3个字节
    my_usart1_tx_buf1[2] = 0x12;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 103;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x07;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI;
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //信息体个数
    my_usart1_tx_buf1[9] = my_101_COT_low; //传输原因
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x00; //遥信信息体首地址
    my_usart1_tx_buf1[14] = 0x00;

    HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN); //读取RTC时间
    HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);

    my_usart1_tx_buf1[15] =  (my_RTC_time.Seconds * 1000); //数据部分
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
    //单字节
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

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 103;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x05;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI;
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //信息体个数
    my_usart1_tx_buf1[9] = my_101_COT_low; //传输原因
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

    my_usart1_tx_buf1[13] = 0x00; //遥信信息体首地址
    my_usart1_tx_buf1[14] = 0x00;


    HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN); //读取RTC时间
    HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);

    my_usart1_tx_buf1[15] =  (my_RTC_time.Seconds * 1000); //数据部分
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
功能：GPRS发送周期数据
*/


void  my_fun_GPRS_TX_CYC1(void)  //发送握手信号
{
#if Use_indicatour_cyc_test_satus==1
    //产生指示器模拟信号
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
    wdz_GPRS_string_to_array(TX_GPRS_101_testdata, my_usart1_tx_buf1); //密码指令

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



void  my_fun_GPRS_TX_CYC2(void)  //发送遥信量,总召，无时标
{
    uint8_t ii = 0, kk = 0, my_shibiao_time = 0;
    my_gprs_generate_101single_data(1, my_usart1_tx_buf1, my_shibiao_time, 20); //总共有56个信息体地址
    for(ii = 1; ii <= MY_GPRS_Call_Single_data_number; ii++)
    {
        my_usart1_tx_buf1[14 + ii] = 0X00; //遥信双点信息 01为正常，0X02为故障

    }

    my_usart1_tx_buf1[14 + 1] = MY_yaoxin_status_OK;
    my_usart1_tx_buf1[14 + 2] = MY_yaoxin_status_OK;
    //A相状态

    for(kk = 0; kk < 3; kk++)
    {
        //通信环节
        if(my_indicator_data[kk].xinhao_db > 75)
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 1] = MY_yaoxin_status_ERROR; //采集单元通信状态
        else
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 1] = MY_yaoxin_status_OK; //采集单元通信状态

        //短路状态
        if(my_indicator_data[kk].duanlu_data == 0X01)
        {
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 2] = MY_yaoxin_status_OK; //瞬时性短路
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 3] = MY_yaoxin_status_ERROR; 	//永久性短路

        }
        else if(my_indicator_data[kk].duanlu_data == 0X41 || my_indicator_data[kk].duanlu_data == 0X81)
        {
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 2] = MY_yaoxin_status_ERROR; //瞬时性短路
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 3] = MY_yaoxin_status_OK; 	//永久性短路

        }
        else
        {
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 2] = MY_yaoxin_status_OK; //瞬时性短路
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 3] = MY_yaoxin_status_OK; 	//永久性短路

        }
        //接地状态
        if(my_indicator_data[kk].jiedi_data == 0X01)
        {
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 4] = MY_yaoxin_status_ERROR; //接地
        }
        else
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 4] = MY_yaoxin_status_OK;

        my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 5] = MY_yaoxin_status_OK; //温度超限
        my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 6] = MY_yaoxin_status_OK; //电流超限

        //电池状态
        float yy = 0;
        yy = (my_indicator_data[kk].DC_data_buf[12] + (my_indicator_data[kk].DC_data_buf[13] << 8)) / 10.0;
        if(yy < 3.8)
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 7] = MY_yaoxin_status_ERROR;
        else
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 7] = MY_yaoxin_status_OK; //电池欠压报警


        //线路停电状态
        if(my_indicator_data[kk].Line_STOP == 2)
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 8] = MY_yaoxin_status_ERROR; //线路有电,停电了
        else
            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 8] = MY_yaoxin_status_OK; //正常，有电

    }


    //时标
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

void  my_fun_GPRS_TX_CYC3(void)  //遥测，总召,无时标
{
    uint8_t jj = 0, shibiao = 0; //时标长度
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

    //系统重启，发送特殊数据处理  0XFB，指示器重启为0X11
    if(my_system_restart_status == 1)
    {
    }
    my_system_restart_status = 0;


    wdz_GPRS_101check_generate(my_usart1_tx_buf1);
    my_at_senddata(my_usart1_tx_buf1);

    printf("my_GPRS send CYC data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);


}


void  my_fun_GPRS_TX_CYC4(void)  //  环境
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

void  my_fun_GPRS_TX_CYC5(void)  //  总召结束
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
    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73
    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 0X46;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 10;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI; //类型标识，带时标的遥测数据信息，
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //信息体个数
    my_usart1_tx_buf1[9] = my_101_COT_low; //传输原因
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x00; //遥信信息体首地址
    my_usart1_tx_buf1[14] = 0x00;

    my_usart1_tx_buf1[15] = 20;

    my_usart1_tx_buf1[16] = my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_usart1_tx_buf1[17] = 0x16;

    my_at_senddata(my_usart1_tx_buf1); //
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

}








/*

*/
void  my_fun_GPRS_TX_RESET(void)  //  DTU重启
{
    //单字节
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
    my_usart1_tx_buf1[1] = 0x0C; //1个信息体，信息值1个字节
    my_usart1_tx_buf1[2] = 0x0C;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 104;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x07;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI;
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //信息体个数
    my_usart1_tx_buf1[9] = my_101_COT_low; //传输原因
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

    my_usart1_tx_buf1[13] = 0x00; //遥信信息体首地址
    my_usart1_tx_buf1[14] = 0x00;

    my_usart1_tx_buf1[15] = 1;

    my_usart1_tx_buf1[16] = my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_usart1_tx_buf1[17] = 0x16;

    my_at_senddata(my_usart1_tx_buf1); //


}

void  my_fun_GPRS_TX_changeparameter(void)  //  DTU重启
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

        //修改帧 DTU地址，信息体的数据
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

        //修改帧 DTU地址，信息体的数据
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

void  my_fun_GPRS_TX_TurnLED(void)  //  翻牌
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



        //修改帧 DTU地址，信息体的数据
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
//查询参数
void  my_fun_GPRS_TX_query_data(void)  //  查询参数
{
    //68 0B 0B 68 53 01 00 69 01 07 01 00 00 00 01 C7 16
    uint16_t my_temp16 = 0, my_temp162 = 0;



    my_temp16 = USART1_my_frame[13];
    my_temp16 = (my_temp16 << 8) + USART1_my_frame[12];


    if(my_GPRS_all_step == 0X7100 &&  query_data == 1)
    {
        my_fun_GPRS_TX_OK();
        wdz_GPRS_string_to_array(TX_GPRS_101_Chaxun_data, my_usart1_tx_buf1); //
        my_temp162 = DTU_ADDRESS;      //DTU地址
        my_usart1_tx_buf1[12] = 0x01;
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = my_temp162;
        my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);;
        //修改帧 DTU地址，信息体的数据
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
        my_temp162 = MY_M_speed_heart; //心跳帧间隔
        my_usart1_tx_buf1[12] = 0x02;
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = my_temp162;
        my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);;
        //修改帧 DTU地址，信息体的数据
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
        my_temp162 = MY_M_speed_cyc; //周期间隔
        my_usart1_tx_buf1[12] = 0x03;
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = my_temp162;
        my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);
        //修改帧 DTU地址，信息体的数据
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
        //修改帧 DTU地址，信息体的数据
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
        //DTU RTC时间
        my_usart1_tx_buf1[12] = 0x19;
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = my_RTC_time.Seconds;
        my_usart1_tx_buf1[15] = 00;
        my_usart1_tx_buf1[16] = my_RTC_time.Minutes;
        my_usart1_tx_buf1[17] = my_RTC_time.Hours;
        my_usart1_tx_buf1[18] = my_RTC_date.Date;
        my_usart1_tx_buf1[19] = my_RTC_date.Month;
        my_usart1_tx_buf1[20] = my_RTC_date.Year;

        //修改帧 DTU地址，信息体的数据
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }
    else if(my_GPRS_all_step == 0X7300 && query_data > 5) //结束
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_Chaxun_data, my_usart1_tx_buf1); //

        my_usart1_tx_buf1[12] = 0x20;  //结束
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = 00;
        my_usart1_tx_buf1[15] = 00;
        //修改帧 DTU地址，信息体的数据
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
void  my_fun_GPRS_TX_query_data2(void)  //  查询参数
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
        //修改帧 DTU地址，信息体的数据
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
            my_temp162 = my_indicator_parameter_data[my_query_index].P2_Add_value; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x32;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);
        }
        else if(query_data2 == 3)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P3_E_mul; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x33;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 4)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P4_E_mul2; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x34;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 5)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P5_I_deta; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x35;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 6)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P6_I_max; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x36;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 7)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P7_I_0min; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x37;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 8)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P8_E_down_baifenbi; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x38;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 9)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P9_E_0min; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x39;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 10)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P10_E_down_min; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x3A;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 11)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P11_V_Libat; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x3B;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 12)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P12_CYC_time_MIN; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x3C;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 13)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P13_CYC_time_MAX; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x3D;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 14)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P14_sleep_time; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x3E;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 15)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P15_awake_time; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x3F;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 16)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P16; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x40;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 17)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P17_reset_LED_time; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x41;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }
        else if(query_data2 == 18)
        {
            my_temp162 = my_indicator_parameter_data[my_query_index].P18_reset_sys_time; //心跳帧间隔
            my_usart1_tx_buf1[12] = 0x42;
            my_usart1_tx_buf1[13] = 0x50;
            my_usart1_tx_buf1[14] = my_temp162;
            my_usart1_tx_buf1[15] = (uint8_t)(my_temp162 >> 8);

        }


        //修改帧 DTU地址，信息体的数据
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


    else if(my_GPRS_all_step == 0X7300 ) //结束
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_Chaxun_data, my_usart1_tx_buf1); //

        my_usart1_tx_buf1[12] = 0x21;  //结束
        my_usart1_tx_buf1[13] = 0x50;
        my_usart1_tx_buf1[14] = 00;
        my_usart1_tx_buf1[15] = 00;
        //修改帧 DTU地址，信息体的数据
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

    }

    //===============================================


}


//发送信号强度
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
    my_usart1_tx_buf1[17] = my_indicator_data[0].xinhao_db; //CC1101信号强度
    my_usart1_tx_buf1[18] = my_indicator_data[1].xinhao_db;;
    my_usart1_tx_buf1[19] = my_indicator_data[2].xinhao_db;;

    //修改帧 DTU地址，信息体的数据
    my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
    my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_at_senddata(my_usart1_tx_buf1);
    printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);
}


//发送计数值和RTC
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

        //修改帧 DTU地址，信息体的数据
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

功能：报警数据发送
*/
void  my_fun_GPRS_TX_ALarm_data(void)
{
    uint8_t ii = 0;
    uint8_t inf_add = 0;
    uint8_t alarm_data = 00;
    //产生报警模拟数据
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

    //模拟数据OK


    if(my_GPRS_all_step == 0X9100 ) //遥信-无时标
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_ALarm_single_notime_data, my_usart1_tx_buf1); //

        if(my_indicator_tx_index >= my_indicator_count)  //my_indicator_tx_index从0开始
        {
            //my_GPRS_all_step=0;
            return;
        }
        ii = my_indicator_tx_index;
        if(my_indicator_alarm_data[ii].TX_status_duanlu >= 0X01)
        {
            inf_add = ii * 2; //短路遥信地址号
            alarm_data = my_indicator_alarm_data[ii].duanlu_data;
        }
        else if(my_indicator_alarm_data[ii].TX_status_jiedi >= 0x01)
        {
            inf_add = ii * 2 + 1; //接地遥信地址号
            alarm_data = my_indicator_alarm_data[ii].jiedi_data;

        }
        my_usart1_tx_buf1[12] = inf_add;
        my_usart1_tx_buf1[13] = 0x00;//
        my_usart1_tx_buf1[14] = alarm_data;




        //修改帧 DTU地址，信息体的数据
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);


    }
    else if(my_GPRS_all_step == 0X9200) //遥信--有时标
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

            //修改帧 DTU地址，信息体的数据
            my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
            my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
            my_fun_101check_generate(my_usart1_tx_buf1, 0);

            my_at_senddata(my_usart1_tx_buf1);
            printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
            my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

            return;

        }

    }

    else if(my_GPRS_all_step == 0X9300)  // 遥测-线上交流值
    {
        wdz_GPRS_string_to_array(TX_GPRS_101_ALarm_single_AC_data, my_usart1_tx_buf1); //
        if((int)my_indicator_tx_index >= 0 && my_indicator_tx_index < 10)
        {
            my_usart1_tx_buf1[12] = my_indicator_tx_index + 1;
            my_usart1_tx_buf1[13] = 0X44;

        }
        for(ii = 0; ii < 3; ii++)
        {
            if(ii == my_indicator_tx_index)  //报警数据填充
            {

                my_usart1_tx_buf1[14 + ii * 6] = my_indicator_alarm_data[ii].AC_data_buf[0];
                my_usart1_tx_buf1[15 + ii * 6] = my_indicator_alarm_data[ii].AC_data_buf[1];
                my_usart1_tx_buf1[16 + ii * 6] = my_indicator_alarm_data[ii].AC_data_buf[2];
                my_usart1_tx_buf1[17 + ii * 6] = my_indicator_alarm_data[ii].AC_data_buf[3];
                my_usart1_tx_buf1[18 + ii * 6] = my_indicator_alarm_data[ii].AC_data_buf[4];
                my_usart1_tx_buf1[19 + ii * 6] = my_indicator_alarm_data[ii].AC_data_buf[5];
            }
            else  //其它相利用周期数据填充
            {
                my_usart1_tx_buf1[14 + ii * 6] = my_indicator_data[ii].AC_data_buf[0];
                my_usart1_tx_buf1[15 + ii * 6] = my_indicator_data[ii].AC_data_buf[1];
                my_usart1_tx_buf1[16 + ii * 6] = my_indicator_data[ii].AC_data_buf[2];
                my_usart1_tx_buf1[17 + ii * 6] = my_indicator_data[ii].AC_data_buf[3];
                my_usart1_tx_buf1[18 + ii * 6] = my_indicator_data[ii].AC_data_buf[4];
                my_usart1_tx_buf1[19 + ii * 6] = my_indicator_data[ii].AC_data_buf[5];


            }


        }
        //修改帧 DTU地址，信息体的数据
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

        return;



    }

    else if(my_GPRS_all_step == 0X9400) //遥测--交流AC12T
    {
        my_fun_gprs_generate_12T_data(my_usart1_tx_buf1);//产生12T数据
        if((int)my_indicator_tx_index >= 0 && my_indicator_tx_index < 10)
        {
            my_usart1_tx_buf1[12] = my_indicator_tx_index + 1;
            my_usart1_tx_buf1[12] = 1;
            my_usart1_tx_buf1[13] = 0X43;

        }

        //修改帧 DTU地址，信息体的数据
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

        //修改帧 DTU地址，信息体的数据
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);

        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);


    }





}

/*
功能：总召录波数据
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

        //修改帧 DTU地址，信息体的数据
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

        //修改帧 DTU地址，信息体的数据
        my_usart1_tx_buf1[5] = my_usart1_tx_buf1[10] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = my_usart1_tx_buf1[11] = DTU_ADDRESS >> 8;
        my_fun_101check_generate(my_usart1_tx_buf1, 0);
        my_at_senddata(my_usart1_tx_buf1);
        printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
        my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);
        if(my_tx_rec_count_finish < my_tx_rec_count_all - 1) //**总共N个段，前N-1个段，例如，总共9段，第8段就应该退出
            my_GPRS_all_step = 0X5100;

    }

    else if(my_GPRS_all_step == 0X5300)
    {

        if(my_GPRS_all_count == 1)
            my_tx_rec_count_finish++;

        my_fun_GPRS_101_genert_record_data(my_usart1_tx_buf1);

        //修改帧 DTU地址，信息体的数据
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
//GPRS  接收处理函数==========
uint8_t my_fun_GPRS_RX_test1(void) //此函数为结束函数，收到OK帧后，结束对话过程
{

    //A版程序
    //握手结束

    if(my_GPRS_all_step == 0X00E3 || my_GPRS_all_step == 0X0048)
    {
        printf("GPRS==TCP==start time is over-1!!\n\n");
        NET_Server_status = 1;

    }

    //RTC同步
    else if(my_GPRS_all_step == 0X00D1)
    {
        //RTC时间处理
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
    //dtu进程复位
    else if(my_GPRS_all_step == 0X00A2)
    {

        printf("GPRS==DTU Reset==  is over-7!!\n\n");

        HAL_NVIC_SystemReset();
    }
    else if(my_GPRS_all_step == 0X0051)
    {
        uint8_t ii = 0;
        uint8_t  length = 0;

        //目录名称
        length = USART1_my_frame[22];
        if(length == 0) //默认目录
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

        //查询目录状态
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

        //查询目录起止时间


    }

    //读取文件激活确认
    else if(my_GPRS_all_step == 0X0054)
    {
        //文件激活接收

        //发送文件
        my_GPRS_all_step = 0;
        uint16_t my_step = 0X5500;
        xQueueSend(myQueue01Handle, &my_step, 100);	//标识下一个状态,开启文件发送



    }





    //C版程序
    //计数值同步指令
    else if(my_GPRS_all_step == 0X00E5)
    {
        uint16_t my_temp16 = 0;
        my_temp16 = USART1_my_frame[15];
        my_temp16 = (my_temp16 << 8) + USART1_my_frame[14];
        //全局计数时间同步，tim6count
        //my_tim6_count=my_temp16;
        printf("==TCP_start Count_syn=%d===\n", my_temp16);
        printf("GPRS==TCP count_syn==time is over-2!!\n\n");
    }
    //心跳结束
    else if(my_GPRS_all_step == 0X001F)
    {
        printf("GPRS RX==heart==time is over-3!!\n\n");
    }

    else if(my_GPRS_all_step == 0X00D2)
    {
        printf("GPRS RX==RTC time FINISH===!!\n\n");
    }

    //周期结束
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

//GPRS 遥信接收到OK帧
uint8_t my_fun_GPRS_RX_test2(void)  //接收到过程帧，不做处理
{

    printf("GPRS dialog get OK frame---[%XH]\r\n", my_GPRS_all_step);
    return 1;
}



/*
功能：DTU参数设置
*/



struct my_ZSQ_change_vale my_zsq_value; //指示器设置参数使用
uint8_t my_fun_GPRS_RX_change_parameter(void) //
{
    uint16_t my_temp16 = 0;
    uint16_t mydata16 = 0;
    uint8_t mybuf[8] = {0};

    my_temp16 = USART1_my_frame[13];
    my_temp16 = (my_temp16 << 8) + USART1_my_frame[12]; //获得数据的信息体地址


    //DTU参数部分
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

        printf("GPRS==TCP==change parameter 5001!!\r\n"); //DTU地址
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


        printf("GPRS==TCP==change parameter 5002!!\r\n"); //心跳包时间（分钟）00默认5分钟
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
        printf("GPRS==TCP==change parameter 5003!!\r\n"); //定时发送数据时间（默认高速时间）
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
            SPI_EE_BufferWrite2(MY_IP, EEPROM_IP_Address, 4); //把默认IP地址 写入到EEPROM中 222.222.118.3，都是十进制的
        }

        printf("GPRS==TCP==change parameter 5004!!\r\n"); //Server地址（5个字节,IP地址加端口号），222.222.118.3：2404，发送顺序从左到右
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

        printf("GPRS==TCP==change parameter 5010!!\r\n"); //周期速度调整门限
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
        printf("GPRS==TCP==change parameter 5011!!\r\n"); //高速
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
        printf("GPRS==TCP==change parameter 5012!!\r\n"); //低速
    }

    //指示器参数部分

    else if(my_GPRS_all_step == 0X0081 && (my_temp16 >= 0X5031 && my_temp16 <= 0X5050))
    {

        my_zsq_value.my_inf_add = my_temp16; //信息体地址
        my_zsq_value.zsq_add = USART1_my_frame[15];
        my_zsq_value.zsq_add = (my_zsq_value.zsq_add << 8) + USART1_my_frame[14];
        my_zsq_value.data_buf[0] = USART1_my_frame[16]; //数据
        my_zsq_value.data_buf[1] = USART1_my_frame[17];
        my_zsq_value.status = 1; //发送状态，1为没有发送

        printf("GPRS==TCP==change parameter [%X]H!!\n", my_temp16); //电流校正系数

    }
    else if(my_GPRS_all_step == 0X0081 && my_temp16 == 0X5051)
    {

        my_zsq_value.my_inf_add = my_temp16; //信息体地址
        my_zsq_value.zsq_add = USART1_my_frame[15];
        my_zsq_value.zsq_add = (my_zsq_value.zsq_add << 8) + USART1_my_frame[14];
        my_zsq_value.data_buf[0] = USART1_my_frame[16]; //数据
        my_zsq_value.data_buf[1] = USART1_my_frame[17];
        my_zsq_value.data_buf[2] = USART1_my_frame[18];
        my_zsq_value.data_buf[3] = USART1_my_frame[19];
        my_zsq_value.data_buf[4] = USART1_my_frame[20];
        my_zsq_value.data_buf[5] = USART1_my_frame[21];
        my_zsq_value.status = 1; //发送状态，1为没有发送

        printf("GPRS==TCP==change parameter [%X]H!!\n", my_temp16);
        // 指示器新地址

    }

    else
        printf("GPRS RX my_GPRS_ALL_STEP=[%XH]\r\n", my_GPRS_all_step);

    //=============




    return 1;

}

//终端翻牌功能
uint8_t my_fun_GPRS_RX_turn_led(void) //
{
    uint16_t my_temp16 = 0, my_temp162 = 0;

    uint8_t mydata8 = 0;

    my_temp16 = USART1_my_frame[13];
    my_temp16 = (my_temp16 << 8) + USART1_my_frame[12]; //获得数据的信息体地址1
    mydata8 = USART1_my_frame[14];
    my_temp162 = USART1_my_frame[16];
    my_temp162 = (my_temp162 << 8) + USART1_my_frame[15]; //获得数据的信息体地址2

    //短路翻牌
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


//查询参数
uint8_t my_fun_GPRS_RX_query_data(void) //
{
    uint16_t my_temp16 = 0;

    if(USART1_my_frame[0] == 0X68 && my_GPRS_all_step == 0x0071)
    {
        my_temp16 = USART1_my_frame[13];
        my_temp16 = (my_temp16 << 8) + USART1_my_frame[12]; //获得数据的信息体地址1
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
        my_temp16 = (my_temp16 << 8) + USART1_my_frame[12]; //获得数据的信息体地址1
        query_data2 = 0X1;
    }
    else
        query_data2++;


    printf("GPRS RX my_GPRS_ALL_STEP=[%XH]\r\n", my_GPRS_all_step);


    //=============
    return 1;

}
/*
功能：总召录波接收处理函数
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
        my_temp81 = USART1_my_frame[14]; //接收到的段号
        printf("GPRS==record data= process=%d !!\r\n", my_temp81);

    }
    else if(my_GPRS_all_step == 0X0054)
    {
        uint8_t my_temp81 = 0;
        my_temp81 = USART1_my_frame[14]; //接收到的段号
        printf("GPRS==record data===finish=%d!!\r\n", my_temp81);

        //报警发送成功，清除报警发送状态
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
功能：M35重新启动

*/

void my_fun_M35_resume_init(void)
{

    if( GPRS_Heartdata_error_count >= 3) //M35发送数据不成功3次以上
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


    //数据的长度
    wdz_GPRS_string_to_array(TX_GPRS_101_Record_data, my_usart1_tx_buf1);
    if(my_tx_rec_count_finish < my_tx_rec_count_all)
        my_length = 254;
    else
        my_length = 25;

    //接收到的信息体地址
    my_inf_add_rx = USART1_my_frame[13];
    my_inf_add_rx = (my_inf_add_rx << 8) + USART1_my_frame[12]; //获得数据的信息体地址1
    my_inf_add_tx	=	my_inf_add_rx;
    //指示器序号


    //修改发送录波数据指示器号
    if(my_indicator_tx_index < my_indicator_count)
        my_query_record_index = my_indicator_tx_index;
    printf("==3=my_query_record_index=%d\n", my_query_record_index);
    //printf("alarm wave_type %d--%d--%d\n",my_indicator_record_data[0].my_wave_type,my_indicator_record_data[1].my_wave_type,my_indicator_record_data[2].my_wave_type);




    //====数据区指针
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
    if(my_tx_rec_count_finish == 1) //第一段数据
    {
        //101帧长度
        my_usart1_tx_buf1[1] = my_length;
        my_usart1_tx_buf1[2] = my_length;
        //段头
        my_usart1_tx_buf1[14] = 9;
        my_usart1_tx_buf1[15] = my_tx_rec_count_finish;
        my_usart1_tx_buf1[16] = 240;
        my_usart1_tx_buf1[17] = 1;

        //段中数据
        *pt_count = 0;
        for(ii = 0; ii < 240; ii++)
        {
            my_usart1_tx_buf1[18 + ii] = pt_buf[ii];
            //printf("-pt_buf[%d]=%d\n",ii,pt_buf[ii]);
        }
        //读指针移动

        *pt_count = *pt_count + ii;
    }

    else if(my_tx_rec_count_finish > 1 && my_tx_rec_count_finish < my_tx_rec_count_all) //段中数据
    {
        //101帧长度
        my_usart1_tx_buf1[1] = my_length;
        my_usart1_tx_buf1[2] = my_length;
        //段头
        my_usart1_tx_buf1[14] = 9;
        my_usart1_tx_buf1[15] = my_tx_rec_count_finish;
        my_usart1_tx_buf1[16] = 240;
        my_usart1_tx_buf1[17] = 1;

        //段中数据

        for(ii = 0; ii < 240; ii++)
        {
            my_usart1_tx_buf1[18 + ii] = pt_buf[ii + *pt_count];
        }
        //读指针移动

        *pt_count = *pt_count + ii;
    }

    else if(my_tx_rec_count_finish == my_tx_rec_count_all) //最后一段数据
    {
        //101帧长度
        my_usart1_tx_buf1[1] = my_length;
        my_usart1_tx_buf1[2] = my_length;
        //段头
        my_usart1_tx_buf1[14] = 9;
        my_usart1_tx_buf1[15] = my_tx_rec_count_finish;
        my_usart1_tx_buf1[16] = 11;
        my_usart1_tx_buf1[17] = 0;

        //段中数据

        for(ii = 0; ii < 11; ii++)
        {
            my_usart1_tx_buf1[18 + ii] = pt_buf[ii + *pt_count];
        }
        //读指针移动

        *pt_count = *pt_count + ii;
    }



}










//========CC1101部分
uint8_t my_fun_dialog_CC1101_RX_heart(void)
{
    uint8_t my_address = 0;
#if Debug_uart_out_cc1101_rx_data_status==1
    my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //调试使用，显示接收到的数据8个字节
#endif


    //心跳包处理部分
    if(my_CC1101_all_step == 0xE000)
    {
        //获得地址--发送源
        my_address = my_CC1101_COM_Fram_buf[2]; //帧中的发送源地址
        my_cc1101_dest_address = my_address; //修改CC1101的目的地址，为发送做准备使用
        printf("get cc1101 heart data id=%d", my_address);


    }

    return 1;

}

void my_fun_CC1101_TX_OK(void)
{
    uint8_t *pt;
    uint16_t my_temp16 = my_tim6_count; //同步计数值
    pt = my_cc1101_tx_buf;
    pt[0] = 0x10;
    pt[1] = 0x20; //ID为0X20，表示OK帧
    pt[2] = my_CC1101_chip_address; //代表DTU的CC1101地址，为0XFE,，0XFD为调试器的地址，01,02,03---为指示器地址
    pt[3] = my_cc1101_dest_address; //目标地址

    pt[4] = my_temp16;                   //OK帧中包含了，同步计数值
    pt[5] = (my_temp16 >> 8);


    pt[6] = my_fun_101check_generate(pt, 1);
    pt[7] = 0x16;

    CC1101SendPacket_add( pt, 8,  ADDRESS_CHECK, my_cc1101_dest_address);
    printf("after CC TX my_CC1101_all_step=[%XH]\n", my_CC1101_all_step);

#if Debug_uart_out_cc1101_tx_data_status==1
    my_fun_display_buf_16(pt, 8, 1); //测试使用
#endif

}

void my_fun_display_ZSQ_data(void)
{


    uint16_t  xx = 0;
    uint16_t ii = 0;
    double yy[12] = {0};
    double xx2 = 0, xx3 = 0, xx4 = 0;
    //double xx1 = 0;
    for(xx = 0; xx < 3; xx++) //显示三个指示器的数据信息
    {
        //短路和接地状态
        printf("\n========START DC1============\n");
        printf("---ZSQ=[%d]--timer=[%d]-[%d]-[%d]--DTU-timer=[%d]\n", xx + 1, my_ZSQ_time_count[0], my_ZSQ_time_count[1], my_ZSQ_time_count[2], my_tim6_count);
        printf("ALARM:duanlu=[%XH],jiedi=[%XH]\n", my_indicator_data[xx].duanlu_data, my_indicator_data[xx].jiedi_data);

        //直流量，7个
        for(ii = 0; ii < 7; ii++)
        {
            yy[ii] = (my_indicator_data[xx].DC_data_buf[2 * ii] +
                      (my_indicator_data[xx].DC_data_buf[2 * ii + 1] << 8)) / 10.0;
        }
        printf(" DC:Temp=%.2f,vbat=%.2f,vref=%.2f\n", yy[0], yy[1], yy[2]);
        printf(" DC:GANbat=%.2f,Zaixian=%.2f,sunbat=%.2f,Libat=%.2f\n", yy[3], yy[4], yy[5], yy[6]);
        //线上交流量，3个，电流全波，电场，电流半波
        for(ii = 0; ii < 3; ii++)
        {
            yy[ii] = (my_indicator_data[xx].AC_data_buf[2 * ii] +
                      (my_indicator_data[xx].AC_data_buf[2 * ii + 1] << 8)) / 10.0;
        }
        printf("AC:A=%.1f,E=%.1f,HA=%.1f\n", yy[0], yy[1], yy[2]);
        printf("========END DC============\n");


        //================遥测AC12T
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


        //录波数据

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




//CC1101接收处理函数，接收后没有后续动作
uint8_t my_fun_dialog_CC1101_RX_0(void)
{
    uint8_t temp_status = 0;
    uint16_t my_length = 0;
    uint8_t my_address = 0; //
    uint8_t my_indicator_index = 0;
    uint8_t my_re = 1;

    if(my_CC1101_COM_Fram_buf[0] == 0x10) //10帧，
    {
        //获得指令ID
        temp_status = my_CC1101_COM_Fram_buf[1];
        my_CC1101_receive_cmd_ID = temp_status;
        //获得地址--发送源
        my_address = my_CC1101_COM_Fram_buf[2]; //帧中的发送源地址
        my_cc1101_dest_address = my_address; //修改CC1101的目的地址，为发送做准备使用

        //printf("@@@@@ ID=%d, RSSI=%d \n",my_address,my_RSSI_dbm_all) ;
        my_indicator_data[my_address - 1].xinhao_db = my_RSSI_dbm_all; //存储信号强度

//        my_address_dest = my_CC1101_COM_Fram_buf[3]; //发送目的地址
#if Debug_uart_out_cc1101_rx_data_status==1
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //调试使用，显示接收到的数据8个字节
#endif
    }
    else if (my_CC1101_COM_Fram_buf[0] == 0x68) //68长帧
    {
        //获得指令ID
        temp_status = my_CC1101_COM_Fram_buf[6];
        printf("===============control word==[%X]\n", temp_status);
        my_CC1101_receive_cmd_ID = temp_status;

        my_length = my_CC1101_COM_Fram_buf[2];
        my_length = (my_length << 8) + my_CC1101_COM_Fram_buf[1] - 3; //获得长度

        //获得地址
        my_address = my_CC1101_COM_Fram_buf[7];
        my_cc1101_dest_address = my_address; //修改CC1101的目的地址，为发送做准备使用

#if Debug_uart_out_cc1101_rx_data_status==1
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //调试使用，显示接收到的数据8个字节
#endif
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf, 8, 0); //@@@@调试使用，显示接收到的数据8个字节
    }

    //指示器地址判断
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
    //帧数据类型判断
    if(temp_status == 0x01)
    {
        my_indicator_data[my_indicator_index].data_type = 0x01; //周期
    }
    else if(temp_status == 0x02)
    {
        my_indicator_data[my_indicator_index].data_type = 0x02; //遥信

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
发送CC1101参数设置指令
*/

void my_fun_CC1101_TX_config_parmeter(void)
{
    uint8_t *pt;
    //uint16_t my_temp16 = my_tim6_count; //同步计数值
    pt = my_cc1101_tx_buf;
    uint16_t st_len = 5;
    pt[0] = 0x68;
    pt[1] = st_len;
    pt[2] = (st_len >> 8);
    pt[3] = pt[1];
    pt[4] = pt[2];
    pt[5] = 0x68;

    pt[6] = 0x3F;
    pt[7] = my_CC1101_chip_address; //代表DTU的CC1101地址，为0XFE,，0XFD为调试器的地址，01,02,03---为指示器地址
    pt[8] = my_cc1101_dest_address; //目标地址
    //信息内容,5长度的时候，没有需要设置的数据
    if(st_len == 5)
    {
        pt[9] = 0X01;
        pt[10] = 0X02;
    }
    //===
    //RTC设置部分
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
    my_fun_display_buf_16(pt, 8, 1); //测试使用
#endif

}

//总召确认
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
    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73
    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 0X46;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x07;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI; //类型标识，带时标的遥测数据信息，
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //信息体个数
    my_usart1_tx_buf1[9] = my_101_COT_low; //传输原因
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x00; //遥信信息体首地址
    my_usart1_tx_buf1[14] = 0x00;

    my_usart1_tx_buf1[15] = 20;

    my_usart1_tx_buf1[16] = my_fun_101check_generate(my_usart1_tx_buf1, 0);
    my_usart1_tx_buf1[17] = 0x16;

    my_at_senddata(my_usart1_tx_buf1); //



}


void  my_fun_GPRS_TX_test_data(void)
{
    //单字节
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
    my_usart1_tx_buf1[1] = 0x0D; //1个信息体，信息值2个字节
    my_usart1_tx_buf1[2] = 0x0D;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 104;
    my_101_VSQ_1_7 = 0X01;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x07;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI;
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //信息体个数
    my_usart1_tx_buf1[9] = my_101_COT_low; //传输原因
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

    my_usart1_tx_buf1[13] = 0x00; //遥信信息体首地址
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
    //单字节
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


void  my_fun_GPRS_TX_CYC2_B(void)  //发送遥信量,周期，有时标
{
    uint8_t ii = 0, kk = 0;
    uint8_t lenth = 13;

    //生成数据包
    {

        my_usart1_tx_buf1[0] = 0x68;
        my_usart1_tx_buf1[3] = 0x68;
        my_usart1_tx_buf1[1] = lenth * 10 + 9 ; //带有时标7个字节
        my_usart1_tx_buf1[2] = lenth * 10 + 9;

        //控制域码处理
        my_101_DIR = 0X80;
        my_101_PRM = 0X40;
        if(my_GPRS_all_count == 1)
            my_101_FCB = (~my_101_FCB) & 0X20;
        my_101_FCV = 0X10;
        my_101_FC = 0X03;

        my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73



        my_usart1_tx_buf1[5] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);


        my_usart1_tx_buf1[7] = 31; //TI  遥信，双点信息，带时标

        my_usart1_tx_buf1[8] = lenth + 0x00; //信息体个数
        my_usart1_tx_buf1[9] = 0X03; //传输原因
        my_usart1_tx_buf1[10] = 0; //传输原因

        my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
        my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

        my_usart1_tx_buf1[4 + 9 + lenth * 10  ] = 0XFF;
        my_usart1_tx_buf1[4 + 9 + lenth * 10 + 1 ] = 0x16;

    }

    //my_gprs_generate_101single_data(1, my_usart1_tx_buf1, my_shibiao_time, 0x03); //总共有56个信息体地址
    for(ii = 1; ii <= lenth * 10; ii++)
    {
        my_usart1_tx_buf1[12 + ii] = 0X00; //遥信双点信息 01为正常，0X02为故障

    }



    HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN); //读取RTC时间
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
        //地址
        my_usart1_tx_buf1[12 + 10 + kk * 40 + 1] = 0x10 * (kk + 1) + 0X02;
        my_usart1_tx_buf1[12 + 10 + kk * 40 + 2] = 0x00;

        my_usart1_tx_buf1[12 + 10 + kk * 40 + 11] = 0x10 * (kk + 1) + 0X03;
        my_usart1_tx_buf1[12 + 10 + kk * 40 + 12] = 0x00;

        my_usart1_tx_buf1[12 + 10 + kk * 40 + 21] = 0x10 * (kk + 1) + 0X04;
        my_usart1_tx_buf1[12 + 10 + kk * 40 + 22] = 0x00;

        my_usart1_tx_buf1[12 + 10 + kk * 40 + 31] = 0x10 * (kk + 1) + 0X08;
        my_usart1_tx_buf1[12 + 10 + kk * 40 + 32] = 0x00;


//				//通信环节
//        if(my_indicator_data[kk].xinhao_db > 75)
//            my_usart1_tx_buf1[14 +10+ kk*10 + 3] = MY_yaoxin_status_ERROR; //采集单元通信状态
//        else
//            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 1] = MY_yaoxin_status_OK; //采集单元通信状态

        //短路状态
        if(my_indicator_data[kk].duanlu_data == 0X01)
        {
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 3] = MY_yaoxin_status_OK; //瞬时性短路
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 13] = MY_yaoxin_status_ERROR; 	//永久性短路

        }
        else if(my_indicator_data[kk].duanlu_data == 0X41 || my_indicator_data[kk].duanlu_data == 0X81)
        {
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 3] = MY_yaoxin_status_ERROR; //瞬时性短路
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 13] = MY_yaoxin_status_OK; 	//永久性短路

        }
        else
        {
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 3] = MY_yaoxin_status_OK; //瞬时性短路
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 13] = MY_yaoxin_status_OK; 	//永久性短路

        }
        //接地状态
        if(my_indicator_data[kk].jiedi_data == 0X01)
        {
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 23] = MY_yaoxin_status_ERROR; //接地
        }
        else
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 23] = MY_yaoxin_status_OK;

        //my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 5] = MY_yaoxin_status_OK; //温度超限
        //my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 6] = MY_yaoxin_status_OK; //电流超限

        //电池状态
//        float yy = 0;
//        yy = (my_indicator_data[kk].DC_data_buf[12] + (my_indicator_data[kk].DC_data_buf[13] << 8)) / 10.0;
//        if(yy < 3.8)
//            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 7] = MY_yaoxin_status_ERROR;
//        else
//            my_usart1_tx_buf1[14 + 0X10 * (kk + 1) + 7] = MY_yaoxin_status_OK; //电池欠压报警


        //线路停电状态
        if(my_indicator_data[kk].Line_STOP == 2)
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 33] = MY_yaoxin_status_ERROR; //线路有电,停电了
        else
            my_usart1_tx_buf1[12 + 10 + kk * 40  + 33] = MY_yaoxin_status_OK; //正常，有电


        //时标
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


void  my_fun_GPRS_TX_CYC3_B(void)  //遥测，周期，有时标
{
    uint8_t jj = 0, shibiao = 0; //时标长度
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

    //系统重启，发送特殊数据处理  0XFB，指示器重启为0X11
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

    if(my_GPRS_all_step == 0X9100 ) //遥信-带时标
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


        //====A相===
        if(x1 == 1)
        {
            if(my_indicator_alarm_data[00].TX_status_duanlu >= 0X01)
            {

                //短路状态
                if(my_indicator_alarm_data[00].duanlu_data == 0X01)
                {   x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x13; //永久性短路
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;
                }
                else if(my_indicator_alarm_data[00].duanlu_data == 0X41 || my_indicator_alarm_data[00].duanlu_data == 0X81)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x12; //瞬时性性短路
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;

                }
                else if(my_indicator_alarm_data[00].duanlu_data == 0)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x13; //永久性短路恢复了
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x18; //停电
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;



                }
                else if(my_indicator_alarm_data[00].duanlu_data == 0XFE)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x18; //停电
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;

                }

            }
            if(my_indicator_alarm_data[00].TX_status_jiedi >= 0X01)
            {
                if(my_indicator_alarm_data[00].TX_status_jiedi == 0X01)
                {   x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x14; //接地
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;
                }
                else if(my_indicator_alarm_data[00].TX_status_jiedi == 0X00)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x14; //接地
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;

                }
            }

        }

        //===B相

        if(x2 == 1)
        {
            if(my_indicator_alarm_data[01].TX_status_duanlu >= 0X01)
            {

                //短路状态
                if(my_indicator_alarm_data[01].duanlu_data == 0X01)
                {   x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x23; //永久性短路
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;
                }
                else if(my_indicator_alarm_data[01].duanlu_data == 0X41 || my_indicator_alarm_data[01].duanlu_data == 0X81)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x22; //瞬时性性短路
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;

                }
                else if(my_indicator_alarm_data[01].duanlu_data == 0)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x23; //永久性短路恢复了
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x28; //停电
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;



                }
                else if(my_indicator_alarm_data[01].duanlu_data == 0XFE)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x28; //停电
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;

                }

            }
            if(my_indicator_alarm_data[1].TX_status_jiedi >= 0X01)
            {
                if(my_indicator_alarm_data[1].jiedi_data == 0X01)
                {   x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x24; //接地
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;
                }
                else if(my_indicator_alarm_data[1].jiedi_data == 0X00)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x24; //接地
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;

                }
            }

        }



        //---C相
        if(x3 == 1)
        {
            if(my_indicator_alarm_data[02].TX_status_duanlu >= 0X01)
            {

                //短路状态
                if(my_indicator_alarm_data[02].duanlu_data == 0X01)
                {   x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x33; //永久性短路
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;
                }
                else if(my_indicator_alarm_data[02].duanlu_data == 0X41 || my_indicator_alarm_data[02].duanlu_data == 0X81)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x32; //瞬时性性短路
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;

                }
                else if(my_indicator_alarm_data[02].duanlu_data == 0)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x33; //永久性短路恢复了
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x38; //停电
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;



                }
                else if(my_indicator_alarm_data[02].duanlu_data == 0XFE)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x38; //停电
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;

                }

            }
            if(my_indicator_alarm_data[2].TX_status_jiedi >= 0X01)
            {
                if(my_indicator_alarm_data[2].jiedi_data == 0X01)
                {   x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x34; //接地
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_ERROR;
                }
                else if(my_indicator_alarm_data[2].jiedi_data == 0X00)
                {

                    x_count++;
                    my_usart1_tx_buf1[13 + x_count * 3] = 0x34; //接地
                    my_usart1_tx_buf1[13 + x_count * 3 + 1] = 00;
                    my_usart1_tx_buf1[13 + x_count * 3 + 2] = MY_yaoxin_status_OK;

                }
            }

        }






        //======

        my_usart1_tx_buf1[0] = 0x68;
        my_usart1_tx_buf1[3] = 0x68;
        my_usart1_tx_buf1[1] = 9 + x_count * 3 + 7; //长度
        my_usart1_tx_buf1[2] = 9 + x_count * 3 + 7;


        //控制域码处理
        my_101_DIR = 0X80;
        my_101_PRM = 0X40;
        if(my_GPRS_all_count == 1)
            my_101_FCB = (~my_101_FCB) & 0X20;
        my_101_FCV = 0X10;
        my_101_FC = 0X03;

        my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73


        my_usart1_tx_buf1[5] = DTU_ADDRESS;
        my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

        my_101_TI = 31;
        my_101_VSQ_1_7 = x_count;
        my_101_VSQ_8_SQ = 0x00;
        my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
        my_101_COT_low = 0x03;
        my_101_COT_high = 0;

        my_usart1_tx_buf1[7] = my_101_TI;
        my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //信息体个数
        my_usart1_tx_buf1[9] = my_101_COT_low; //传输原因
        my_usart1_tx_buf1[10] = my_101_COT_high;

        my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
        my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

        x_count++;
        HAL_RTC_GetDate(&hrtc, &my_RTC_date, RTC_FORMAT_BIN); //读取RTC时间
        HAL_RTC_GetTime(&hrtc, &my_RTC_time, RTC_FORMAT_BIN);

        my_usart1_tx_buf1[13 + x_count * 3 + 0] =  (my_RTC_time.Seconds * 1000); //数据部分
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

    uint8_t jj = 0; //时标长度
    uint8_t length = 0;
    union MY_float my_AC_data ;
    //my_gprs_generate_101analog_data(1, my_usart1_tx_buf1, shibiao, 0x01);

    //=============================
    length = 3;

    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = length * 5 + 11; //12*5+11=61
    my_usart1_tx_buf1[2] = length * 5 + 11;

    //控制域码处理
    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73

    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_usart1_tx_buf1[7] = 13; //测量量，短浮点数
    my_usart1_tx_buf1[8] = length + 0x80; //信息体个数
    my_usart1_tx_buf1[9] = 3; //传输原因
    my_usart1_tx_buf1[10] = 00; //传输原因

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);;

    my_usart1_tx_buf1[13] = 0x11; //遥信信息体首地址
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


//故障报警，遥信和遥测合并在一起。
void  my_fun_GPRS_TX_ALarm_data_yaoxin_yaoce(void)
{

    uint8_t jj = 0;;
    uint8_t x1 = 0, x2 = 0, x3 = 0, x_count = 0;
    uint8_t my_add_star_yaoxin = 0;
    uint8_t length = 0;
    union MY_float my_AC_data ;

    //遥信-带时标

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
    //====A相===
    if(x1 == 1)
    {
        if(my_indicator_alarm_data[00].TX_status_duanlu >= 0X01)
        {

            //短路状态
            if(my_indicator_alarm_data[00].duanlu_data == 0X01)
            {   x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x13; //永久性短路
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x12; //瞬时性性短路
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x13; //永久性短路恢复了
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x18; //停电
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x18; //停电
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x14; //接地
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x14; //接地
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

    //===B相

    if(x2 == 1)
    {
        if(my_indicator_alarm_data[1].TX_status_duanlu >= 0X01)
        {

            //短路状态
            if(my_indicator_alarm_data[1].duanlu_data == 0X01)
            {   x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x23; //永久性短路
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x22; //瞬时性性短路
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x23; //永久性短路恢复了
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 1] = 00;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10 + 2] = MY_yaoxin_status_OK;

                x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x28; //停电
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x28; //停电
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x24; //接地
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x24; //接地
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



    //---C相
    if(x3 == 1)
    {
        if(my_indicator_alarm_data[2].TX_status_duanlu >= 0X01)
        {

            //短路状态
            if(my_indicator_alarm_data[2].duanlu_data == 0X01)
            {   x_count++;
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x33; //永久性短路
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x32; //瞬时性性短路
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x33; //永久性短路恢复了
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x38; //停电
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x38; //停电
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x34; //接地
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
                my_usart1_tx_buf1[my_add_star_yaoxin + x_count * 10] = 0x34; //接地
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


    //遥测数据，没有时标
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
    my_usart1_tx_buf1[1] = 9 + 2 + x_count * 10 + 2 + length * 6; //长度
    my_usart1_tx_buf1[2] = 9 + 2 + x_count * 10 + 2 + length * 6;


    //控制域码处理
    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 42;
    my_101_VSQ_1_7 = x_count + 3;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 0x03;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI;
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //信息体个数
    my_usart1_tx_buf1[9] = my_101_COT_low; //传输原因
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
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


void  my_fun_GPRS_TX_CYC4_B(void)  //  环境
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


//  召唤目录
void  my_fun_GPRS_TX_catalog(void)
{


    uint8_t mylength = 0; //文件附加数据包
    uint8_t my_file_name_lenth = 0; //文件名字长度	==7，17，8
    uint32_t my_file_data_count = 0; //文件内容字节数量

    //文件目录判断
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
    //文件长度判断


    if(my_file_name_lenth == 7 && my_file_catalog_status == 0) //所有文件
    {
        my_file_data_count = 256;
        mylength = 9 + my_file_name_lenth + 12;
    }
    else if(my_file_name_lenth == 17 && my_file_catalog_status == 0) //所有文件
    {
        my_file_data_count = 300;
        mylength = 9 + my_file_name_lenth + 12;
    }
    else if(my_file_name_lenth == 8 && my_file_catalog_status == 0) //所有文件
    {

        my_file_data_count = 400;
        mylength = 9 + my_file_name_lenth + 12;
    }
    else if(my_file_catalog_status == 1) //时间区间范围内的文件
    {
        //先不处理
        my_file_data_count = 0;
        mylength = 8;
        //处理方法：判断文件类型，然后读取对应的文件，看文件中是否有这个时间段，有则进行数据处理，没有则返回0


    }

    else //不存在文件
    {
        my_file_data_count = 0;
        mylength = 8;
    }


    my_fun_GPRS_TX_OK_80(); //从身份回复OK帧




    //======

    my_usart1_tx_buf1[0] = 0x68;
    my_usart1_tx_buf1[3] = 0x68;
    my_usart1_tx_buf1[1] = 9 + 3 + mylength; //长度
    my_usart1_tx_buf1[2] = 9 + 3 + mylength;


    //控制域码处理
    my_101_DIR = 0X80;
    my_101_PRM = 0X40;
    if(my_GPRS_all_count == 1)
        my_101_FCB = (~my_101_FCB) & 0X20;
    my_101_FCV = 0X10;
    my_101_FC = 0X03;

    my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73


    my_usart1_tx_buf1[5] = DTU_ADDRESS;
    my_usart1_tx_buf1[6] = (DTU_ADDRESS >> 8);

    my_101_TI = 126;
    my_101_VSQ_1_7 = 1;
    my_101_VSQ_8_SQ = 0x00;
    my_101_VSQ_1_7 = (my_101_VSQ_1_7 | my_101_VSQ_8_SQ);
    my_101_COT_low = 5;
    my_101_COT_high = 0;

    my_usart1_tx_buf1[7] = my_101_TI;
    my_usart1_tx_buf1[8] = my_101_VSQ_1_7; //信息体个数
    my_usart1_tx_buf1[9] = my_101_COT_low; //传输原因
    my_usart1_tx_buf1[10] = my_101_COT_high;

    my_usart1_tx_buf1[11] = DTU_ADDRESS; //公共域地址
    my_usart1_tx_buf1[12] = (DTU_ADDRESS >> 8);

    my_usart1_tx_buf1[13] = 00;
    my_usart1_tx_buf1[14] = 00;

    my_usart1_tx_buf1[15] = 2; //读目录

    //===========文件附加数据包处理部分======
    if(my_file_data_count != 0) //存在文件
    {
        my_usart1_tx_buf1[15 + 1] = 2;
        my_usart1_tx_buf1[15 + 2] = 0; //0表示成功，1表示失败，没有文件

        my_usart1_tx_buf1[15 + 3] = 0;
        my_usart1_tx_buf1[15 + 4] = 0;
        my_usart1_tx_buf1[15 + 5] = 0;
        my_usart1_tx_buf1[15 + 6] = 0;

        my_usart1_tx_buf1[15 + 7] = 0;
        my_usart1_tx_buf1[15 + 8] = 1; //本帧文件数量



        my_usart1_tx_buf1[15 + 9] = my_file_name_lenth; //文件名长度  7,17,8
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

    else //不存在文件
    {
        my_usart1_tx_buf1[15 + 1] = 2;
        my_usart1_tx_buf1[15 + 2] = 1; //0表示成功，1表示失败，没有文件

        my_usart1_tx_buf1[15 + 3] = 0;
        my_usart1_tx_buf1[15 + 4] = 0;
        my_usart1_tx_buf1[15 + 5] = 0;
        my_usart1_tx_buf1[15 + 6] = 0;

        my_usart1_tx_buf1[15 + 7] = 0;
        my_usart1_tx_buf1[15 + 8] = 0; //本帧文件数量

    }
    //=================
    my_usart1_tx_buf1[15 + mylength + 1] = 0XFF;
    my_usart1_tx_buf1[15 + mylength + 2] = 0X16;

    my_fun_101check_generate(my_usart1_tx_buf1, 0);

    my_at_senddata(my_usart1_tx_buf1);
    printf("my_GPRS send turnled_ack data-[%XH]:", my_GPRS_all_step);
    my_fun_display_buf_16(my_usart1_tx_buf1, 10, 1);

}



void  my_fun_GPRS_TX_file_data_1(void) //读取文件激活确认
{
    //发送确认OK文件，短帧

    //发送文件激活确认，长帧

}

void  my_fun_GPRS_TX_file_data_2(void) //读取文件，发送数据
{


}

void  my_fun_GPRS_TX_file_data_3(void) //读取文件，发送数据
{

    my_fun_GPRS_TX_OK_80(); //文件接收，回复OK帧

    //根据条件，启动下一段文件传输
    if(my_file_part != 0)
    {
        my_GPRS_all_step = 0;
        uint16_t my_step = 0X5500;
        xQueueSend(myQueue01Handle, &my_step, 100);	//标识下一个状态,开启文件发送
    }

}

