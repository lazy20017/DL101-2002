#include "my_gprs.h"
#include "my_cc1101.h"
#include "my_globle_extern.h"

/*
GPRS发送数据利用  uint8_t my_at_senddata(uint8_t *string);
GPRS分析接收数据利用
uint8_t my_usart_101frame(uint8_t usart_port);
void my_process_resive_101usart1(void); //M35使用
*/

uint8_t  WDZ_GPRS_101FCB = 0X00;


extern  uint32_t TimingDelay;
extern uint16_t DTU_ADDRESS;

extern	uint16_t USART1_address_first;
extern	uint16_t USART1_address_second;
extern	uint8_t USART1_length;
extern	uint8_t USART1_FRAME_status;
extern	uint8_t USART1_my_frame[256];
extern	uint8_t USART1_TRANSMINT_STATUS;
extern	uint8_t USART1_RESIVER_STATUS;


extern uint8_t rsbuf1[];	  //USART1缓冲器
extern uint8_t txbuf1[];
extern uint16_t rsbuf1pt_write;
extern uint16_t rsbuf1pt_read;
extern uint16_t rsbuf1pt_TEMP_read;
extern uint16_t rsbuf1pt_COMM_read;

//=====
extern	uint16_t USART3_address_first;
extern	uint16_t USART3_address_second;
extern	uint8_t USART3_length;
extern	uint8_t USART3_FRAME_status;
extern	uint8_t USART3_my_frame[256];
extern	uint8_t USART3_TRANSMINT_STATUS;
extern	uint8_t USART3_RESIVER_STATUS;


extern uint8_t rsbuf3[];	  //USART1缓冲器
extern uint8_t txbuf3[];
extern uint16_t rsbuf3pt_write;
extern uint16_t rsbuf3pt_read;
extern uint16_t rsbuf3pt_TEMP_read;
extern uint16_t rsbuf3pt_COMM_read;


//=====



extern struct rtc_time systmtime;  //RTC实时时钟使用
extern u8 rtcbuffer[];

//-------------------
extern uint8_t MY_GPRS_MCU_RsBuf[8];  //存储，周期性电池电压、太阳能电压、温度、湿度共4类8个字节的数据

extern uint8_t MY_GPRS_Call_Single_data_buf[40];
extern uint8_t MY_GPRS_Call_Analog_data_buf[110];
extern uint8_t MY_GPRS_Call_Time_data_buf[7];
extern uint8_t MY_GPRS_Call_Single_data_number;
extern uint8_t MY_GPRS_Call_Analog_data_number;
extern uint8_t MY_GPRS_Call_Status;    //存储获得总召数据状态，为1表示有总召数据，为0表示没有总召数据

extern uint8_t MY_EEPROM_Buf[256];  //用来存储EEPROM中，读出的数据
extern uint8_t MY_GPRS_Cycle_Transmintdata_status;  //用来存储读取到总召数据发送状态，1为已发送，2为未发送


extern uint8_t GPRS_Status;  //标识最终手机模块，GPRS网络状态，1为正常，可以连接服务器，0为有问题，需要处理
extern uint8_t MESS_Status;  //短信网络状态
extern uint8_t NET_Status;  //NET联网状态
extern uint8_t NET_Server_status; //远端服务器server状态

//-----------------
extern	uint8_t MY_433_Alarmdata_NOtimeBuf[256]; //存储 无时标 报警数据
extern	uint8_t MY_433_Alarmdata_TimeBuf[256];  //存储，有时标，报警数据
extern	uint8_t MY_433_ALarmdata_number;  // 存储，报警信息体个数


extern	uint8_t MY_433_ALarmdata_NOtime_status; //为1，表示收到无时标报警数据
extern	uint8_t MY_433_ALarmdata_Time_status;   //为1，表示收到有时标报警数据

//extern uint8_t AT_MESS_telphonenumber[];

extern  uint8_t MY_MCU_RsBuf[];
extern uint8_t MY_433_Call_Single_data_buf[];
extern uint8_t MY_433_Call_Analog_data_buf[];
extern uint8_t MY_433_Call_Time_data_buf[];
extern uint8_t MY_433_Call_Single_data_number;
extern uint8_t MY_433_Call_Analog_data_number;

//------------
extern uint32_t MY_Table1_Alarmdata_StartAddress;
extern uint32_t MY_Table1_Alarmdata_EndAddress;
extern uint32_t MY_Table1_Alarmdata_WriteAddress;
extern uint32_t MY_Table1_Alarmdata_ReadAddress;


extern  uint8_t RE_ALarmData_Status;

extern uint8_t MY_MCU_getdata_status;
extern uint8_t MY_433_Call_Status;

extern	uint8_t my_433_anlag_buf[110];  //存储433模拟量，不进行清0处理，不存储，只要有变化，利用标志位进行转发。
extern	uint8_t my_433_anlag_flag;  //标志位，为0表示已经发送了新数据，为1表示有新数据但是还没有发送



extern uint16_t MY_H_speed_cyc;  //周期10分钟
extern uint16_t MY_H_speed_heart;  //心跳5分钟

extern uint16_t MY_M_speed_cyc;  //周期15分钟
extern uint16_t MY_M_speed_heart;  //心跳9分钟

extern uint16_t MY_L_speed_cyc;  //周期30分钟
extern uint16_t MY_L_speed_heart; //心跳7分钟

extern float MY_Speed_H_Gate;
extern float MY_Speed_L_Gate;

extern uint16_t my_tim6_count;

extern struct indicator_class my_indicator_data[];
extern uint8_t my_usart1_tx_buf1[];



extern uint8_t my_101_DIR ; //101规约中,控制域的DIR位 最高位D8
extern uint8_t my_101_PRM ; //101规约中，控制域的PRM  D7
extern uint8_t my_101_FCB ; //控制域D6
extern uint8_t my_101_FCV ; //控制月D5

extern uint8_t my_101_FC ; //控制域，功能码
extern uint8_t my_101_add_low ;
extern uint8_t my_101_add_high ;
extern uint8_t my_101_TI ;
extern uint8_t my_101_VSQ_1_7 ;
extern uint8_t my_101_VSQ_8_SQ ;
extern uint8_t my_101_COT_low ;
extern uint8_t my_101_COT_high ;


extern uint8_t my_GPRS_all_count;










/*
校验字检查
*/
uint8_t wdz_GPRS_101char_check(uint8_t *buffer)
{
    uint16_t k = 0;
    uint8_t status68 = 0;
    uint8_t temp = 0;
    uint8_t check_char = 0x00;

    if(buffer[0] == 0x10)
    {
        status68 = 1;
    }
    else if(buffer[0] == 0x68)
    {
        status68 = 2;
    }

    if(status68 == 1) //固定长度校验位检查
    {
        check_char = buffer[1] + buffer[2] + buffer[3];
        if(check_char == buffer[4])
            temp = 1;
        else temp = 0;


    }
    else if(status68 == 2) //非固定长度校验位检查
    {
        for(k = 0; k < buffer[1]; k++)
        {
            check_char = check_char + buffer[k + 4];
        }

        if(check_char == buffer[buffer[1] + 4])
            temp = 1;
        else temp = 0;
    }
    return temp;


}







/*
生成校验字
*/
void wdz_GPRS_101check_generate(uint8_t *buffer)
{
    uint16_t k = 0;
    uint8_t status68 = 0;

    uint8_t check_char = 0x00;

    if(buffer[0] == 0x10)
    {
        status68 = 1;
    }
    else if(buffer[0] == 0x68)
    {
        status68 = 2;
    }


    if(status68 == 1) //固定长度校验位检查
    {
        check_char = buffer[1] + buffer[2] + buffer[3];
        buffer[4] = check_char;
    }
    else if(status68 == 2) //非固定长度校验位检查
    {
        for(k = 0; k < buffer[1]; k++)
        {
            check_char = check_char + buffer[k + 4];
        }
        buffer[buffer[1] + 4] = check_char;
    }


}









/*
功能：数据接收分析，此部分为分析101协议数据，有帧头，有帧尾，有帧长，这三个参数很重要，取完整的一帧数据。
输入参数：串口号
输出参数： 取帧数据成功返回1，取帧数据失败返回0
*/
uint8_t my_usart_GPRS_101frame(uint8_t usart_port)
{
    uint8_t *rsbuf = 0;
    uint16_t *rsbufpt_read = 0;
    uint16_t *rsbufpt_COMM_read = 0;
    uint16_t *rsbufpt_write = 0;
    uint8_t *my_frame = 0;
    uint8_t *USART_FRAME = 0;


    uint16_t *address_first = 0;
    uint16_t *address_second = 0;
    uint8_t *length = 0;

    uint8_t ch1 = 0;
    uint16_t pt = 0;
    uint8_t tmp_status = 0;
    uint8_t status = 0;
    uint16_t ii = 0;

    uint8_t my_temp_status = 0;



//******串口的选择****************
    if(usart_port == 1)
    {
        rsbuf = rsbuf1;
        rsbufpt_read = &rsbuf1pt_read;
        rsbufpt_COMM_read = &rsbuf1pt_COMM_read;
        rsbufpt_write = &rsbuf1pt_write;
        address_first = &USART1_address_first;
        address_second = &USART1_address_second;
        length = &USART1_length;
        my_frame = USART1_my_frame;
        USART_FRAME = &USART1_FRAME_status;

        *rsbufpt_COMM_read = *rsbufpt_read;

    }

    if(usart_port == 3)
    {
        rsbuf = rsbuf3;
        rsbufpt_read = &rsbuf3pt_read;
        rsbufpt_COMM_read = &rsbuf3pt_COMM_read;
        rsbufpt_write = &rsbuf3pt_write;
        address_first = &USART3_address_first;
        address_second = &USART3_address_second;
        length = &USART3_length;
        my_frame = USART3_my_frame;
        USART_FRAME = &USART3_FRAME_status;

        *rsbufpt_COMM_read = *rsbufpt_read;

    }

////////****串口选择结束***************



    while(*USART_FRAME == 0 && *rsbufpt_COMM_read != *rsbufpt_write) //前一个指令处理完成，还有未处理的字符，则进行处理。结束条件，有完整的一条指令或者所有字符处理完成
    {

        //取一个未处理的字符
        if(*rsbufpt_COMM_read == rsbuf_max - 1)
        {
            ch1 = rsbuf[*rsbufpt_COMM_read];
            *rsbufpt_COMM_read = 0;
            pt = *rsbufpt_COMM_read;
        }
        else
        {
            ch1 = rsbuf[*rsbufpt_COMM_read];
            *rsbufpt_COMM_read = *rsbufpt_COMM_read + 1;
            pt = *rsbufpt_COMM_read;
        }

        //进行0X68帧头和帧尾标记
        if(ch1 == 0x68)
        {
            tmp_status = 1;
        }
        else if(ch1 == 0x16)
        {
            tmp_status = 2;
        }
        else
        {
            *USART_FRAME = 0;
            my_temp_status = 0;
            tmp_status = 0;
        }


        //进行0X68帧头分析
        if(tmp_status == 1)
        {

            //
            if((pt > 0 && pt < 4) && ch1 == rsbuf[rsbuf_max + pt - 4] && ch1 == 0x68)
            {

                *address_first = rsbuf_max + pt - 4;
                if(pt - 3 == 0)*length = rsbuf[pt - 3];
                else *length = rsbuf[rsbuf_max + pt - 3];

            }
            else if(pt >= 4 && ch1 == rsbuf[pt - 4] && ch1 == 0x68)
            {

                *address_first = pt - 4;
                *length = rsbuf[pt - 3];
            }
            else if(pt == 0 && ch1 == rsbuf[rsbuf_max - 4] && ch1 == 0x68)
            {

                *address_first = rsbuf_max - 4;
                *length = rsbuf[rsbuf_max - 3];
            }

            else
            {
                *USART_FRAME = 0;
                my_temp_status = 0;
                tmp_status = 0;
            }

        }
        //进行由帧尾到帧头的分析
        if(tmp_status == 2)
        {

            //固定长度帧
            if(pt > 0 && pt < 6 && rsbuf[rsbuf_max + pt - 6] == 0x10)
            {

                *address_first = rsbuf_max + pt - 6;
                *length = 6;
                *address_second = pt - 1;
                *USART_FRAME = 1;
                my_temp_status = 1;

            }
            else if(pt >= 6 && rsbuf[pt - 6] == 0x10)
            {

                *address_first = pt - 6;
                *length = 6;
                *address_second = pt - 1;
                *USART_FRAME = 1;
                my_temp_status = 1;
            }
            else if(pt == 0 && rsbuf[rsbuf_max - 6] == 0x10)
            {

                *address_first = rsbuf_max - 6;
                *length = 6;
                *address_second = pt - 1;
                *USART_FRAME = 1;
                my_temp_status = 1;
            }
            //非固定长度帧
            if(pt - 6 - *address_first == (*length) && *address_first < pt)
            {
                *address_second = pt - 1;
                *USART_FRAME = 2;
                my_temp_status = 2;
            }
            else if(*address_first > pt && pt != 0)
            {
                if((pt + rsbuf_max - *address_first - 6) == (*length))
                {   *address_second = pt - 1;
                    *USART_FRAME = 2;
                    my_temp_status = 2;
                }
            }
            else if(pt == 0)
            {
                if((rsbuf_max - *address_first - 6) == (*length))
                {   *address_second = rsbuf_max - 1;
                    *USART_FRAME = 2;
                    my_temp_status = 2;
                }
            }
            //


        }
    }
//取一帧数据存入到指令数组中



    if(my_temp_status > 0)	 //如果有完整一帧数据，就开始处理，否则返回，不处理
    {
        //清空命令数组区
        for(ii = 0; ii < 256; ii++)
        {
            my_frame[ii] = 0;
        }

        //取固定长度指令
        if(rsbuf[*address_first] == 0x10)
        {
            for(ii = 0; ii < 6; ii++)
            {
                my_frame[ii] = rsbuf[*address_first];
                *address_first = *address_first + 1;
                if(*address_first >= rsbuf_max) *address_first = 0;
            }

            *rsbufpt_read = *address_second + 1;
            if(*rsbufpt_read >= rsbuf_max)*rsbufpt_read = 0;

            status = 1;
        }
        //取非固定长度指令
        else if(rsbuf[*address_first] == 0x68)
        {
            for(ii = 0; ii < 6 + *length; ii++)
            {
                my_frame[ii] = rsbuf[*address_first];
                *address_first = *address_first + 1;
                if(*address_first >= rsbuf_max) *address_first = 0;
            }

            *rsbufpt_read = *address_second + 1;
            if(*rsbufpt_read >= rsbuf_max)*rsbufpt_read = 0;
            status = 1;
        }
        else
        {
            status = 0;
        }
    }

    //进行返回处理
    if(status == 1)
    {
        //*USART_FRAME=0;  //取完一帧指令进行标记
        //printf("\r\nOK=%s",my_frame);
        //USART_printf(USARTx,"\r\nOK=%s",my_frame);
        //USART_printf(USARTx,"%s",my_frame);
        return(1);
    }
    else
    {
        //USART_FRAME=0;  //取完一帧指令进行标记
        //printf("\r\nERROR");
        return(0);
    }

}









/*
命令字符串复制到数组中
*/
void wdz_GPRS_string_to_array(uint8_t *my_string, uint8_t *txbuf)
{
    uint32_t k = 0;
    uint32_t length = 0;
    if(my_string[0] == 0x10)
    {
        length = 6;
    }
    else if(my_string[0] == 0x68)
    {
        length = my_string[1] + 6;
    }
    //=================

    for(k = 0; k < length; k++)
    {
        txbuf[k] = my_string[k];
    }
    //===========
    if(my_string[0] == 0x10)
    {
        txbuf[2] = DTU_ADDRESS;
        txbuf[3] = (DTU_ADDRESS >> 8);
    }
    else if(my_string[0] == 0x68)
    {
        txbuf[5] = DTU_ADDRESS;
        txbuf[6] = (DTU_ADDRESS >> 8);

        txbuf[10] = DTU_ADDRESS; //
        txbuf[11] = (DTU_ADDRESS >> 8);


    }
    txbuf[length - 2] = my_fun_101check_generate(txbuf, 0);
    txbuf[k] = 0;
}








//*************主动发送命令部分*******
/*
功能：发送心跳包，等待应答程序

发送心跳包，10 D2 01 00 D3 16 或者 10 F2 01 00 F3 16
接收确认包  10 80 01 00 81 16 或者 10 82 01 00 83 16
*/
extern uint8_t link_status_GPRS;
extern uint8_t GPRS_Heartdata_error_count;  //判断心跳包失败的次数，如果到5次了，就标识GPRS网络故障，然后利用这个计数值进行M35的重启判断




//*************发送测试周期数据命令*************

/*
68 0C 0C 68 73 01 00 68 01 06 00 01 00 00 AA 55 E3 16测试命令(激活)
        （肯定确认）10 80 01 00 81 16
 68 23 23 68 73 01 00 01 98 14 00 01 00 01 00 00 00 00 00 00 00 00 23 16(遥信数据包)
 （遥测数据包）
  （环境数据包）
				（肯定确认）10 80 01 00 81 16
*/

extern uint32_t MY_Table2_Cycledata_StartAddress;
extern uint32_t MY_Table2_Cycledata_EndAddress;
extern uint32_t MY_Table2_Cycledata_WriteAddress;
extern uint32_t MY_Table2_Cycledata_ReadAddress;
extern uint8_t MY_GPRS_Call_Single_data_number;
extern uint8_t MY_GPRS_Call_Analog_data_number;




/*
功能：生成遥信数据包  68 23 23 68 73 01 00 02 ?? 14 01 00 01 00 ?? ??  ??  ** 16
*/

#define my_single_inf_num 2   //遥信使用的单点信息为1，双点信息为2
void my_gprs_generate_101single_data(uint8_t temp, uint8_t *my_rsbuf,uint8_t my_shibiao_time,uint8_t my_cot)
{
    uint8_t length = 0;


    if(temp == 1) //生成数据包
    {
        length = MY_GPRS_Call_Single_data_number; //56个信息体，每个是1个字节，双点信息，
        my_rsbuf[0] = 0x68;
        my_rsbuf[3] = 0x68;
        my_rsbuf[1] = length + 11 + my_shibiao_time; //带有时标7个字节
        my_rsbuf[2] = length + 11 + my_shibiao_time;

        //控制域码处理
        my_101_DIR = 0X80;
        my_101_PRM = 0X40;
        if(my_GPRS_all_count == 1)
            my_101_FCB = (~my_101_FCB) & 0X20;
        my_101_FCV = 0X10;
        my_101_FC = 0X03;

        my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73



        my_rsbuf[5] = DTU_ADDRESS;
        my_rsbuf[6] = (DTU_ADDRESS >> 8);
				
				if(my_shibiao_time==7)
        my_rsbuf[7] = 31; //TI  遥信，双点信息，带时标
				else
				my_rsbuf[7] = 3;
				
        my_rsbuf[8] = length + 0x80; //信息体个数
        my_rsbuf[9] = my_cot; //传输原因
				my_rsbuf[10] = 0; //传输原因

        my_rsbuf[11] = DTU_ADDRESS; //公共域地址
        my_rsbuf[12] = (DTU_ADDRESS >> 8);

        my_rsbuf[13] = 0x01; //遥信信息体首地址
        my_rsbuf[14] = 0x00;



        my_rsbuf[4+11 + length + my_shibiao_time ] = 0XFF;
        my_rsbuf[4+11 + length + my_shibiao_time + 1 ] = 0x16;

    }
    //测试使用
    else if(temp == 0) //生成0数据体数据包
    {
      
    }


}








/*
功能;生成遥测数据包

//68 53 53 68 53 01 00 09 98 14 00 01 00 01 40 00 00 00 00 00 00 00 00 00 00 4B 16
*/
void my_gprs_generate_101analog_data(uint8_t temp, uint8_t *my_rsbuf,uint8_t shibiao,uint8_t my_cot)
{
    uint8_t length = 0;


    if(temp == 1) //生成数据包
    {
        length = MY_GPRS_Call_Analog_data_number;

        my_rsbuf[0] = 0x68;
        my_rsbuf[3] = 0x68;
        my_rsbuf[1] = length * 5 + 11+shibiao; //12*5+11=61
        my_rsbuf[2] = length * 5 + 11+shibiao;

        //控制域码处理
        my_101_DIR = 0X80;
        my_101_PRM = 0X40;
        if(my_GPRS_all_count == 1)
            my_101_FCB = (~my_101_FCB) & 0X20;
        my_101_FCV = 0X10;
        my_101_FC = 0X03;

        my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73

        my_rsbuf[5] = DTU_ADDRESS;
        my_rsbuf[6] = (DTU_ADDRESS >> 8);

        my_rsbuf[7] = 13; //测量量，短浮点数
        my_rsbuf[8] = length + 0x80; //信息体个数
        my_rsbuf[9] = my_cot; //传输原因
				my_rsbuf[10] = 00; //传输原因

        my_rsbuf[11] = DTU_ADDRESS; //公共域地址
        my_rsbuf[12] = (DTU_ADDRESS >> 8);;

        my_rsbuf[13] = 0x11; //遥信信息体首地址
        my_rsbuf[14] = 0x40;

      
        my_rsbuf[4+11 + length * 5 +shibiao] = 0XFF;
        my_rsbuf[4+11  + length * 5 +shibiao+ 1 ] = 0x16;


    }
    else if(temp == 0) //生成0数据体数据包
    {
        
    }



}







/*
功能;生成环境数据包,temp为1生成有数据的数据包，0生成0数据的数据包
*/
void my_gprs_generate_101MCU_data(uint8_t temp, uint8_t *my_rsbuf,uint8_t my_cot)
{
    uint8_t length = 0;

    if(temp == 1) //生成数据包
    {
        length = 1;

        my_rsbuf[0] = 0x68;
        my_rsbuf[3] = 0x68;
        my_rsbuf[1] = length * 5 + 11;
        my_rsbuf[2] = length * 5 + 11;

         //控制域码处理
        my_101_DIR = 0X80;
        my_101_PRM = 0X40;
        if(my_GPRS_all_count == 1)
            my_101_FCB = (~my_101_FCB) & 0X20;
        my_101_FCV = 0X10;
        my_101_FC = 0X03;

        my_usart1_tx_buf1[4] = (my_101_DIR | my_101_PRM | my_101_FCB | my_101_FCV | my_101_FC); //控制域码为53/73

        my_rsbuf[5] = DTU_ADDRESS;
        my_rsbuf[6] = (DTU_ADDRESS >> 8);

        my_rsbuf[7] = 13; //测量量，短浮点数
        my_rsbuf[8] = length + 0x80; //信息体个数
        my_rsbuf[9] = my_cot; //传输原因
				my_rsbuf[10] = 00; //传输原因

        my_rsbuf[12] = 0x01; //遥信信息体首地址
        my_rsbuf[13] = 0x40;

        

        my_rsbuf[4+11 + length * 5 ] = 0XFF;
        my_rsbuf[4+11 + length * 5 + 1 ] = 0x16;

        


    }
    else if(temp == 0) //生成0数据体数据包
    {
       
    }


}







/*
产生控制域码
*/
void my_GPRS_101_geneate_FCBword(uint8_t *my_rsbuf)
{   uint8_t FCB = 0X20;

    if(my_rsbuf[0] == 0x68)
    {
        //控制域码处理
        if(WDZ_GPRS_101FCB == 0x00)
            my_rsbuf[4] = my_rsbuf[4] & (~FCB);
        else if(WDZ_GPRS_101FCB == 0x20)
            my_rsbuf[4] = my_rsbuf[4] | (FCB);
    }
    else if(my_rsbuf[0] == 0x10)
    {
        //控制域码处理
        if(WDZ_GPRS_101FCB == 0x00)
            my_rsbuf[1] = my_rsbuf[1] & (~FCB);
        else if(WDZ_GPRS_101FCB == 0x20)
            my_rsbuf[1] = my_rsbuf[1] | (FCB);
    }

    //记录发送变化帧
    WDZ_GPRS_101FCB = WDZ_GPRS_101FCB ^ 0x20;

}






//-------------被动接收程序，校时、总召----






/*
功能：对收到的参数设置命令进行响应，设置相关的参数，主要包括DTU地址5001，心跳包时间5002，周期数据发送时间5003，Server地址6字节5004，
      增加手机号8字节5005，删除手机号码5006
*/
extern uint16_t Transmint_to_GPRS_hearttime;
extern uint16_t Transmint_to_GPRS_Cycledata;

extern uint8_t MY_IP[4]; //222.222.118.3
extern uint16_t MY_PORT;  //8080  16位数据










/*
	功能：存储mypt地址值的低3个字节到EEPROM中tabladdress开始的地址
	*/
void my_save_PTTO_EEROM(uint32_t mypt, uint32_t tableaddress)
{   uint8_t tempbuf[3] = {0};
    uint32_t first_address = mypt;
    tempbuf[0] = first_address & 0x0000ff;
    tempbuf[1] = (first_address >> 8) & 0x0000ff;
    tempbuf[2] = (first_address >> 16) & 0x0000ff;
    SPI_EE_BufferWrite2(tempbuf, tableaddress, 3);
}




void my_reset_mcu()  //重启MCU通过软命令
{
    __disable_fault_irq();
    NVIC_SystemReset();
}




/*
把收到的字符转换成ASCII码进行显示
*/

void my_display_ASCIIdata(uint8_t *rsbuf)
{
//	int tt=0;
    int length = 0;
//	uint8_t *mypt=rsbuf;
//	uint8_t my_temp1,my_temp2;
    uint8_t desbuf[512] = {0};

    if(*rsbuf == 0x10) length = 6;
    else if(*rsbuf == 0x68) length = *(rsbuf + 1) + 6;


    HexToStr2(desbuf, rsbuf, length);
    USART_printf(&huart3, desbuf);

    /*
    for(tt=0;tt<length;tt++)
    {
    	my_temp1=*mypt/16;
    	my_temp2=*mypt%16;
    	USART_printf(&huart3,"%d%d-",my_temp1,my_temp2);
    mypt++;
    }
    */
    USART_printf(&huart3, "\r\n");

}



/*
功能：发送AT+CSQ，发送信号质量

*/

extern uint8_t MY_AT_CSQ_Value;

/*
功能：产生GPRS信号质量
*/
void my_gprs_generate_101CSQ_data(uint8_t temp, uint8_t *my_rsbuf)
{
    uint8_t length = 0;

    if(temp == 1) //生成数据包
    {
        length = 1;

        my_rsbuf[0] = 0x68;
        my_rsbuf[3] = 0x68;
        my_rsbuf[1] = length * 2 + 10;
        my_rsbuf[2] = length * 2 + 10;

        my_rsbuf[4] = 0x73; //控制域码为53/73
        my_GPRS_101_geneate_FCBword(my_rsbuf);


        my_rsbuf[5] = DTU_ADDRESS & 0X00FF;
        my_rsbuf[6] = (DTU_ADDRESS >> 8) & 0X00FF;

        my_rsbuf[7] = 0X09; //类型标识，带时标的单点信息，
        my_rsbuf[8] = length + 0x80; //信息体个数
        my_rsbuf[9] = 0x14; //传输原因

        my_rsbuf[10] = DTU_ADDRESS & 0X00FF; //公共域地址
        my_rsbuf[11] = (DTU_ADDRESS >> 8) & 0X00FF;;

        my_rsbuf[12] = 0x00; //遥信信息体首地址
        my_rsbuf[13] = 0x42;

        //my_buf1_to_buf2(MY_GPRS_MCU_RsBuf,0,my_rsbuf,14,length*2);

        my_rsbuf[14] = MY_AT_CSQ_Value;
        my_rsbuf[15] = 0X00;

        //
        my_rsbuf[13 + length * 2 + 1] = 0XFF;
        my_rsbuf[13 + length * 2 + 1 + 1] = 0x16;

        wdz_GPRS_101check_generate(my_rsbuf); //生成校验字节



    }
    else if(temp == 0) //生成0数据体数据包
    {
        length = 2;
        my_rsbuf[0] = 0x68;
        my_rsbuf[3] = 0x68;
        my_rsbuf[1] = length + 10;
        my_rsbuf[2] = length + 10;

        my_rsbuf[4] = 0x73; //控制域码为53/73
        my_GPRS_101_geneate_FCBword(my_rsbuf);

        my_rsbuf[5] = DTU_ADDRESS & 0X00FF;
        my_rsbuf[6] = (DTU_ADDRESS >> 8) & 0X00FF;

        my_rsbuf[7] = 0X09; //类型标识，带时标的单点信息，
        my_rsbuf[8] = 0x84; //信息体个数
        my_rsbuf[9] = 0x14; //传输原因

        my_rsbuf[10] = DTU_ADDRESS & 0X00FF; //公共域地址
        my_rsbuf[11] = (DTU_ADDRESS >> 8) & 0X00FF;;

        my_rsbuf[12] = 0x00; //遥信信息体首地址
        my_rsbuf[13] = 0x42;

        my_rsbuf[14] = 0x00;
        my_rsbuf[15] = 0x00;

        my_rsbuf[16] = 0XFF;
        my_rsbuf[17] = 0x16;

        wdz_GPRS_101check_generate(my_rsbuf); //生成校验字节
    }


}

/*
功能：遥测补充
*/
void my_gprs_generate_101yaoce2_data(uint8_t *my_rsbuf)
{
    uint8_t length = 12; //信息体个数
    uint8_t jj = 0;
    //帧头
    my_rsbuf[0] = 0x68;
    my_rsbuf[3] = 0x68;
    my_rsbuf[1] = length * 2 + 10;
    my_rsbuf[2] = length * 2 + 10;
    //控制域部分
    my_rsbuf[4] = 0x73; //控制域码为53/73
    //my_GPRS_101_geneate_FCBword(my_rsbuf);
    my_rsbuf[5] = DTU_ADDRESS;
    my_rsbuf[6] = (DTU_ADDRESS >> 8);
    my_rsbuf[7] = 0X09; //类型标识，带时标的单点信息，
    my_rsbuf[8] = length + 0x80; //信息体个数
    my_rsbuf[9] = 0x67; //传输原因
    my_rsbuf[10] = DTU_ADDRESS; //公共域地址
    my_rsbuf[11] = (DTU_ADDRESS >> 8);;
    my_rsbuf[12] = 0x01; //遥信信息体首地址
    my_rsbuf[13] = 0x42;
    //帧尾
    my_rsbuf[13 + length * 2 + 1] = 0XFF;
    my_rsbuf[13 + length * 2 + 1 + 1] = 0x16;
    //数据部分
    for(jj = 0; jj < 3; jj++)
    {   //1温度，2电源，3参考电压，4干电池，5线上电压，6太阳能，7锂电池
        my_rsbuf[14 + 8 * jj] = my_indicator_data[jj].DC_data_buf[5 * 2]; //6太阳能
        my_rsbuf[15 + 8 * jj] = my_indicator_data[jj].DC_data_buf[5 * 2 + 1];

        my_rsbuf[16 + 8 * jj] = my_indicator_data[jj].DC_data_buf[4 * 2]; //5线上电压
        my_rsbuf[17 + 8 * jj] = my_indicator_data[jj].DC_data_buf[4 * 2 + 1];

        //干电池数据转换成，指示器对应的TIMER
        //my_rsbuf[18 + 8 * jj] = my_indicator_data[jj].DC_data_buf[3 * 2]; //4干电池
        //my_rsbuf[19 + 8 * jj] = my_indicator_data[jj].DC_data_buf[3 * 2 + 1];

        my_rsbuf[18 + 8 * jj] = my_indicator_data[jj].count_time[0]; //4干电池
        my_rsbuf[19 + 8 * jj] = my_indicator_data[jj].count_time[1];


        my_rsbuf[20 + 8 * jj] = my_indicator_data[jj].AC_data_buf[4]; //半波电流值
        my_rsbuf[21 + 8 * jj] = my_indicator_data[jj].AC_data_buf[5];

    }

    wdz_GPRS_101check_generate(my_rsbuf); //生成校验字节

}

/*
功能：遥测12T
*/
void my_gprs_generate_101yaoce12T_data(uint8_t *my_rsbuf)
{
    uint8_t length = 3; //信息体个数
    uint8_t jj = 0, ii = 0;
    //帧头
    my_rsbuf[0] = 0x68;
    my_rsbuf[3] = 0x68;
    my_rsbuf[1] = length * 48 + 10;
    my_rsbuf[2] = length * 48 + 10;
    //控制域部分
    my_rsbuf[4] = 0x73; //控制域码为53/73
    //my_GPRS_101_geneate_FCBword(my_rsbuf);
    my_rsbuf[5] = DTU_ADDRESS;
    my_rsbuf[6] = (DTU_ADDRESS >> 8);
    my_rsbuf[7] = 0X09; //类型标识，带时标的单点信息，
    my_rsbuf[8] = length + 0x80; //信息体个数
    my_rsbuf[9] = 0x68; //传输原因
    my_rsbuf[10] = DTU_ADDRESS; //公共域地址
    my_rsbuf[11] = (DTU_ADDRESS >> 8);;
    my_rsbuf[12] = 0x01; //遥信信息体首地址
    my_rsbuf[13] = 0x43;
    //帧尾
    my_rsbuf[13 + length * 2 + 1] = 0XFF;
    my_rsbuf[13 + length * 2 + 1 + 1] = 0x16;
    //数据部分
    for(jj = 0; jj < 3; jj++)
    {   //12T 电流+电场
        for(ii = 0; ii < 24; ii++)
        {
            my_rsbuf[14 + 48 * jj + ii] = my_indicator_data[jj].AC12T_ALL_Current_data_buf[ii ]; //电流

        }
        for(ii = 0; ii < 24; ii++)
        {
            my_rsbuf[14 + 48 * jj + ii + 24] = my_indicator_data[jj].AC12T_ALL_dianchang_data_buf[ii]; //电场

        }

    }

    wdz_GPRS_101check_generate(my_rsbuf); //生成校验字节

}
/*
功能：计数同步值
*/
extern uint16_t my_gprs_count_time; //GPRS通信，周期数据，传递给SERVER的DTU收到的zsq的计数值
extern uint8_t  my_gprs_RTC_buf[];

void my_gprs_generate_101yaoce1_COUNTSYN_data(uint8_t *my_rsbuf)
{
    uint8_t length = 1; //信息体个数

    //帧头
    my_rsbuf[0] = 0x68;
    my_rsbuf[3] = 0x68;
    my_rsbuf[1] = length * 9 + 10;
    my_rsbuf[2] = length * 9 + 10;
    //控制域部分
    my_rsbuf[4] = 0x73; //控制域码为53/73
    //my_GPRS_101_geneate_FCBword(my_rsbuf);
    my_rsbuf[5] = DTU_ADDRESS;
    my_rsbuf[6] = (DTU_ADDRESS >> 8);
    my_rsbuf[7] = 0XDC; //类型标识，带时标的单点信息，
    my_rsbuf[8] = length + 0x80; //信息体个数
    my_rsbuf[9] = 0x69; //传输原因
    my_rsbuf[10] = DTU_ADDRESS; //公共域地址
    my_rsbuf[11] = (DTU_ADDRESS >> 8);;
    my_rsbuf[12] = 0x01; //遥信信息体首地址
    my_rsbuf[13] = 0x4F;
    //帧尾
    my_rsbuf[13 + length * 9 + 1] = 0XFF;
    my_rsbuf[13 + length * 9 + 1 + 1] = 0x16;
    //数据部分


    my_rsbuf[14] = my_gprs_count_time;
    my_rsbuf[15] = (my_gprs_count_time >> 8);

    my_rsbuf[16] = my_gprs_RTC_buf[0];
    my_rsbuf[17] = my_gprs_RTC_buf[1];
    my_rsbuf[18] = my_gprs_RTC_buf[2];
    my_rsbuf[19] = my_gprs_RTC_buf[3];
    my_rsbuf[20] = my_gprs_RTC_buf[4];
    my_rsbuf[21] = my_gprs_RTC_buf[5];
    my_rsbuf[22] = my_gprs_RTC_buf[6];




    wdz_GPRS_101check_generate(my_rsbuf); //生成校验字节

}


/*
功能：产生报警发送使用的12个字符
*/
extern uint8_t my_indicator_tx_index;

extern struct indicator_alarm_class my_indicator_alarm_data[];
void my_fun_gprs_generate_12T_data(uint8_t *txbuf)
{
    uint8_t ii = 0, jj = 0;

    wdz_GPRS_string_to_array(TX_GPRS_101_ALarm_single_12T_data, my_usart1_tx_buf1);
    for(ii = 0; ii < 3; ii++) //报警相的12T数据
    {
        if(ii == my_indicator_tx_index)
        {
            for(jj = 0; jj < 24; jj++)
            {
                my_usart1_tx_buf1[14 + ii * 48 + jj] = my_indicator_alarm_data[ii].AC12T_ALL_Current_data_buf[jj];		//电流
            }
            for(jj = 0; jj < 24; jj++)
            {
                my_usart1_tx_buf1[14 + ii * 48 + jj + 24] = my_indicator_alarm_data[ii].AC12T_ALL_dianchang_data_buf[jj];	//电场
            }
        }
        else //其它相利用周期数据填充
        {
            for(jj = 0; jj < 24; jj++)
            {
                my_usart1_tx_buf1[14 + ii * 48 + jj] = my_indicator_data[ii].AC12T_ALL_Current_data_buf[jj];		//电流
            }
            for(jj = 0; jj < 24; jj++)
            {
                my_usart1_tx_buf1[14 + ii * 48 + jj + 24] = my_indicator_data[ii].AC12T_ALL_dianchang_data_buf[jj];			//电场
            }



        }


    }







}
