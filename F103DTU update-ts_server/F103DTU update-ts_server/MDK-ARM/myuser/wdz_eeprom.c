#include "wdz_eeprom.h"
#include "my_usart.h"
#include "my_def_value.h"
#include "my_extrn_value.h"








/*
功能：对周期性总召数据进行处理，在总召成功后把数据存储到EEPROM中，标识为未发送--2
			数据来源，PW的4个周期数据，指示器总召得来的遥信、遥测数据 共3个数组，时标在最后的遥测数组中，
			数据存储完后，把数组清零，同时撤销有数据有周期数据标识，产生新的GRPS有数据标识
格式：状态（1），遥信个数N1（1），遥测个数N2（1），电池电压（2），太阳能电压（2），温度（2），湿度（2），
			遥信信息体1值（1），遥信信息体2值（1），....，遥测信息体1值（3），遥测信息2值（3）。。。。时标7
*/

uint16_t my_cyc_data_write_cn=0;
void my_cycle_data_to_eeprom(void)
{
    uint32_t startaddressPT=MY_Table2_Cycledata_StartAddress;
    uint32_t endaddressPT=MY_Table2_Cycledata_EndAddress;
    uint32_t writeaddressPT=MY_Table2_Cycledata_WriteAddress;
    uint32_t readaddressPT=MY_Table2_Cycledata_ReadAddress;
    uint16_t k=0;
    uint16_t number_single=0;
    uint16_t number_anlog=0;
    uint8_t status_cycle=0;
    uint8_t tempbuf[12]= {0};
    uint8_t *PT_buf;
    uint32_t PT_writeaddress=0;
    uint16_t PT_number=0;
    uint8_t temp=0;
    uint8_t Zero_buf[256]= {0};

    uint8_t rs_temp_buf[512]= {0}; //测试使用


//***处理获得最新的表指针地址
    SPI_EE_BufferRead2(tempbuf,EEPROM_Table2_Address,12);
    if(tempbuf[0]==tempbuf[1]&&tempbuf[1]==tempbuf[2]&&tempbuf[2]==tempbuf[3]&&tempbuf[3]==tempbuf[4]&&tempbuf[4]==tempbuf[5])
    {

        my_val_writeto_eeprom(tempbuf, startaddressPT, endaddressPT, writeaddressPT, readaddressPT,EEPROM_Table2_Address);  //把初始表地址写入到EEPROM中

    }
    else
    {

        my_buf_writeto_val(tempbuf, &startaddressPT, &endaddressPT, &writeaddressPT, &readaddressPT);  //从EEPROM中读出最新表地址到数组中

        MY_Table2_Cycledata_StartAddress=startaddressPT;
        MY_Table2_Cycledata_EndAddress=endaddressPT;
        MY_Table2_Cycledata_WriteAddress=writeaddressPT;
        MY_Table2_Cycledata_ReadAddress=readaddressPT;

    }


//***********over****
    USART_printf(&huart3,"MCU_status=%d  433_CALL_Status=%d\r\n",MY_MCU_getdata_status,MY_433_Call_Status);  //测试使用@@@@
    if(MY_MCU_getdata_status==1 && MY_433_Call_Status==1 )
    {


        //存储遥信、遥测的信息体个数
        number_single=MY_433_Call_Single_data_number;
        number_anlog=MY_433_Call_Analog_data_number;

        //进行写地址指针边界检测，如果超过存储区的末地址，则从首地址开始重新存储，剩下的字节就不用了
        if((endaddressPT-writeaddressPT+1)<144)
        {
            SPI_EE_BufferWrite2(Zero_buf,writeaddressPT,endaddressPT-writeaddressPT+1);//把所有的不用区域清零
            writeaddressPT=startaddressPT;
            MY_Table2_Cycledata_WriteAddress=writeaddressPT;
            PT_writeaddress=writeaddressPT;
            USART_printf(&huart3,"CLear ear %d\r\n",endaddressPT-writeaddressPT+1);
        }


        //PT_writeaddress=writeaddressPT+3+8+number_single+number_anlog*3+7; //进行存入数据后末地址计算
        //if(PT_writeaddress>endaddressPT)
        //{
        //	SPI_EE_BufferWrite2(Zero_buf,writeaddressPT,endaddressPT-writeaddressPT+1);//把所有的不用区域清零

        //	writeaddressPT=startaddressPT;
        //	MY_Table2_Cycledata_WriteAddress=writeaddressPT;
        //}


        //开始进行数据存储
        for(k=0; k<256; k++)
            Zero_buf[k]=k+1;
        SPI_EE_BufferWrite2(Zero_buf,writeaddressPT,144); //清空所有数据单元 @@@@@@@@@

        //存储GPRS发送状态，1为发送完，2为没有发送
        AT25_WriteByte(0X02,writeaddressPT);   //地址0字节，Gprs状态，1为发送完，2为没有发送

        //存储遥信、遥测的信息体个数
        number_single=MY_433_Call_Single_data_number;
        number_anlog=MY_433_Call_Analog_data_number;
        AT25_WriteByte(number_single,writeaddressPT+1);  //地址1字节，总召获得的遥信信息体个数
        AT25_WriteByte(number_anlog,writeaddressPT+2);   //地址2字节，总召获得的遥测信息体个数

        //测试使用，进行读取，刚才写入的数据
        //----------------


        //-------------
        //存储PW环境数据
        PT_buf=MY_MCU_RsBuf;
        PT_writeaddress=writeaddressPT+3;
        PT_number=8;
        SPI_EE_BufferWrite2(PT_buf,PT_writeaddress,PT_number); //地址3-10字节，电池电压、太阳能电压、温度、湿度


        //存储遥信数据
        PT_buf=MY_433_Call_Single_data_buf;
        PT_writeaddress=writeaddressPT+11;
        PT_number=number_single;
        SPI_EE_BufferWrite2(PT_buf,PT_writeaddress,PT_number);
        //SPI_EE_BufferWrite2(MY_433_Call_Single_data_buf,writeaddressPT+11,number_single);


        //存储遥测数据
        PT_buf=MY_433_Call_Analog_data_buf;
        PT_writeaddress=writeaddressPT+11+number_single;
        PT_number=number_anlog*3;
        SPI_EE_BufferWrite2(PT_buf,PT_writeaddress,PT_number);
        //SPI_EE_BufferWrite2(MY_433_Call_Analog_data_buf,writeaddressPT+11+number_single,number_anlog*3);

        //存储时标数据
        PT_buf=MY_433_Call_Time_data_buf;
        PT_writeaddress=writeaddressPT+11+number_single+number_anlog*3;
        PT_number=7;
        SPI_EE_BufferWrite2(PT_buf,PT_writeaddress,PT_number);
        //SPI_EE_BufferWrite2(MY_433_Alarmdata_TimeBuf,);

        //------------------测试使用
        //测试使用，显示写入到内存中的过程数据@@@@@
        status_cycle=AT25_ReadByte(writeaddressPT);
        MY_GPRS_Call_Single_data_number=AT25_ReadByte(writeaddressPT+1);
        MY_GPRS_Call_Analog_data_number=AT25_ReadByte(writeaddressPT+2);
        SPI_EE_BufferRead2(MY_GPRS_MCU_RsBuf,writeaddressPT+3,8);
        SPI_EE_BufferRead2(MY_GPRS_Call_Single_data_buf,writeaddressPT+11,18);
        SPI_EE_BufferRead2(MY_GPRS_Call_Analog_data_buf,writeaddressPT+29,108);
        SPI_EE_BufferRead2(MY_GPRS_Call_Time_data_buf,writeaddressPT+29+108,7);


        USART_printf(&huart3,"real data to eeprom start_writeaddress=%d%d%d%d%d%d\r\n",
                     (writeaddressPT-EEPROM_Chip_size)/100000,(writeaddressPT-EEPROM_Chip_size)%100000/10000,
                     (writeaddressPT-EEPROM_Chip_size)%10000/1000,(writeaddressPT-EEPROM_Chip_size)%1000/100,
                     (writeaddressPT-EEPROM_Chip_size)%100/10,(writeaddressPT-EEPROM_Chip_size)%10
                    );
        USART_printf(&huart3,"status cycle=%d  sing data=%d    analog data=%d\r\n",status_cycle,MY_GPRS_Call_Single_data_number,MY_GPRS_Call_Analog_data_number);
        HexToStr2(rs_temp_buf,MY_GPRS_MCU_RsBuf,8);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");
        HexToStr2(rs_temp_buf,MY_GPRS_Call_Single_data_buf,18);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");
        HexToStr2(rs_temp_buf,MY_GPRS_Call_Analog_data_buf,108);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");
        HexToStr2(rs_temp_buf,MY_GPRS_Call_Time_data_buf,7);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");


        //--------------------



        //--------???考虑是否使用 把读指针直接移动到新指针的首地址-
        /*
        readaddressPT=MY_Table2_Cycledata_WriteAddress;
        temp=readaddressPT&0x000000ff;
        AT25_WriteByte(temp,222);

        temp=(writeaddressPT>>8)&0x000000ff;
        AT25_WriteByte(temp,223);

        temp=(writeaddressPT>>16)&0x000000ff;
        AT25_WriteByte(temp,224);
        */

        //-----------

        //获得最后的表写地址指针
        //writeaddressPT=PT_writeaddress+7;
        //
        writeaddressPT=writeaddressPT+144; //按一帧数据最大空间144个字节进行存储设计
        if(writeaddressPT>=endaddressPT)
            writeaddressPT=startaddressPT; //地址校正

        MY_Table2_Cycledata_WriteAddress=writeaddressPT;

        //把新写地址指针写入到EEPROM中
        //my_val_writeto_eeprom(tempbuf, startaddressPT, endaddressPT, writeaddressPT, readaddressPT,213);
        temp=writeaddressPT&0x000000ff;
        AT25_WriteByte(temp,219);

        temp=(writeaddressPT>>8)&0x000000ff;
        AT25_WriteByte(temp,220);

        temp=(writeaddressPT>>16)&0x000000ff;
        AT25_WriteByte(temp,221);


        //结束，进行返回状态处理
        //MY_MCU_getdata_status=0;  //PW环境总召数据处理完成
        //MY_433_Call_Status=0;    //433模块总召数据处理完成
        MY_Table2_Newdata_status=1;


        //测试使用，过程数据@@@@@@@
        USART_printf(&huart3,"real data to eeprom\r\n");


        HexToStr2(rs_temp_buf,MY_MCU_RsBuf,8);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");
        HexToStr2(rs_temp_buf,MY_433_Call_Single_data_buf,18);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");
        HexToStr2(rs_temp_buf,MY_433_Call_Analog_data_buf,108);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");
        HexToStr2(rs_temp_buf,MY_433_Call_Time_data_buf,7);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");


        //------------------------------------------
        //2016-05-28 测试
        my_cyc_data_write_cn++;
        USART_printf(&huart3,"*****cyc_data_write=%d\r\n",my_cyc_data_write_cn);
        //2016-05-28 结束

    }



}
//---------------------------

/*
功能：指针地址存入到数组中，并写入到EEPROM中
*/

void my_val_writeto_eeprom(uint8_t *tempbuf,uint32_t startaddressPT,uint32_t endaddressPT,uint32_t writeaddressPT,uint32_t readaddressPT,uint16_t table_startaddress)
{
    tempbuf[0]=startaddressPT&0x000000ff;
    tempbuf[1]=(startaddressPT>>8)&0x000000ff;
    tempbuf[2]=(startaddressPT>>16)&0x000000ff;

    tempbuf[3]=endaddressPT&0x000000ff;
    tempbuf[4]=(endaddressPT>>8)&0x000000ff;
    tempbuf[5]=(endaddressPT>>16)&0x000000ff;

    tempbuf[6]=writeaddressPT&0x000000ff;
    tempbuf[7]=(writeaddressPT>>8)&0x000000ff;
    tempbuf[8]=(writeaddressPT>>16)&0x000000ff;

    tempbuf[9]=readaddressPT&0x000000ff;
    tempbuf[10]=(readaddressPT>>8)&0x000000ff;
    tempbuf[11]=(readaddressPT>>16)&0x000000ff;

    SPI_EE_BufferWrite2(tempbuf,table_startaddress,12);

}
//------------------------------------------------
/*
功能：把数组数据写入到变量中

*/

void my_buf_writeto_val(uint8_t *tempbuf,uint32_t *startaddressPT,uint32_t *endaddressPT,uint32_t *writeaddressPT,uint32_t *readaddressPT)

{
    *startaddressPT=0;
    *startaddressPT=tempbuf[2];
    *startaddressPT=(*startaddressPT<<8)+tempbuf[1];
    *startaddressPT=(*startaddressPT<<8)+tempbuf[0];

    *endaddressPT=0;
    *endaddressPT=tempbuf[5];
    *endaddressPT=(*endaddressPT<<8)+tempbuf[4];
    *endaddressPT=(*endaddressPT<<8)+tempbuf[3];

    *writeaddressPT=0;
    *writeaddressPT=tempbuf[8];
    *writeaddressPT=(*writeaddressPT<<8)+tempbuf[7];
    *writeaddressPT=(*writeaddressPT<<8)+tempbuf[6];

    *readaddressPT=0;
    *readaddressPT=tempbuf[11];
    *readaddressPT=(*readaddressPT<<8)+tempbuf[10];
    *readaddressPT=(*readaddressPT<<8)+tempbuf[9];

}

//----------------------------------------------------*******************-

/*
功能：把收到的报警数据存入到EEPROM中
格式：状态字节（1），信息体个数（1），信息体1地址低字节（1），信息体1地址高字节（1），信息体1值（1），时标（7）
		信息体2地址低字节（1），信息体2地址高字节（1），信息体2值（1），时标（7）

*/
void my_alarm_data_to_eeprom(void)
{

    uint32_t startaddressPT=MY_Table1_Alarmdata_StartAddress;
    uint32_t endaddressPT=MY_Table1_Alarmdata_EndAddress;
    uint32_t writeaddressPT=MY_Table1_Alarmdata_WriteAddress;
    uint32_t readaddressPT=MY_Table1_Alarmdata_ReadAddress;
    //uint16_t k=0;
    uint16_t number_single=0;
    uint8_t tempbuf[12]= {0};
    uint8_t *PT_buf;
    uint32_t PT_writeaddress=0;
    uint16_t PT_number=0;
    uint8_t temp=0;
    uint8_t Zero_buf[256]= {0};

//***处理获得最新的表指针地址
    SPI_EE_BufferRead2(tempbuf,201,12);
    if(tempbuf[0]==tempbuf[1]&&tempbuf[1]==tempbuf[2]&&tempbuf[2]==tempbuf[3]&&tempbuf[3]==tempbuf[4]&&tempbuf[4]==tempbuf[5])
    {

        my_val_writeto_eeprom(tempbuf, startaddressPT, endaddressPT, writeaddressPT, readaddressPT,201);

    }
    else
    {

        my_buf_writeto_val(tempbuf, &startaddressPT, &endaddressPT, &writeaddressPT, &readaddressPT);

        MY_Table1_Alarmdata_StartAddress=startaddressPT;
        MY_Table1_Alarmdata_EndAddress=endaddressPT;
        MY_Table1_Alarmdata_WriteAddress=writeaddressPT;
        MY_Table1_Alarmdata_ReadAddress=readaddressPT;

    }


//***********over****

    if(MY_433_ALarmdata_Time_status==1 )
    {


        //存储 报警 的遥信 信息体个数
        number_single=MY_433_ALarmdata_number;
        number_single=1; //强制信息体个数为1，？？？？？
        //进行写地址指针边界检测，如果超过存储区的末地址，则从首地址开始重新存储，剩下的字节就不用了

        PT_writeaddress=writeaddressPT+2+number_single*10; //进行存入数据后末地址计算
        if(PT_writeaddress>endaddressPT)
        {
            SPI_EE_BufferWrite2(Zero_buf,writeaddressPT,endaddressPT-writeaddressPT+1);//把所有的不用区域清零
            writeaddressPT=startaddressPT;
            MY_Table1_Alarmdata_WriteAddress=writeaddressPT;


        }


        //存储GPRS发送状态，1为发送完，2为没有发送
        AT25_WriteByte(0X02,writeaddressPT);   //地址0字节，Gprs状态，1为发送完，2为没有发送

        //存储 报警 的遥信 信息体个数
        number_single=MY_433_ALarmdata_number;
        number_single=1; //强制信息体个数为1，？？？？？报警数据时分离的，一次一个信息体，存储用12字节
        AT25_WriteByte(number_single,writeaddressPT+1);  //地址1字节，报警获得的遥信信息体个数

        //存储遥信数据
        PT_buf=MY_433_Alarmdata_TimeBuf;
        PT_writeaddress=writeaddressPT+2;
        PT_number=number_single*10;
        SPI_EE_BufferWrite2(PT_buf,PT_writeaddress,PT_number);
        //SPI_EE_BufferWrite2(MY_433_Call_Single_data_buf,writeaddressPT+2,number_single);

        //--------???考虑是否使用 把读指针直接移动到新指针的首地址-
        //readaddressPT=MY_Table1_Alarmdata_WriteAddress;
        //temp=readaddressPT&0x000000ff;
        //AT25_WriteByte(temp,210);

        //temp=(writeaddressPT>>8)&0x000000ff;
        //AT25_WriteByte(temp,211);

        //temp=(writeaddressPT>>16)&0x000000ff;
        //AT25_WriteByte(temp,212);


        //获得最后的表写地址指针
        writeaddressPT=PT_writeaddress+number_single*10;
        MY_Table1_Alarmdata_WriteAddress=writeaddressPT;


        //把新写地址指针写入到EEPROM中
        //my_val_writeto_eeprom(tempbuf, startaddressPT, endaddressPT, writeaddressPT, readaddressPT,213);
        temp=writeaddressPT&0x000000ff;
        AT25_WriteByte(temp,207);

        temp=(writeaddressPT>>8)&0x000000ff;
        AT25_WriteByte(temp,208);

        temp=(writeaddressPT>>16)&0x000000ff;
        AT25_WriteByte(temp,209);


        //结束，进行返回状态处理
        //MY_433_ALarmdata_Time_status=0;  //报警数据 存入 EEPROM 处理完成
        //MY_433_ALarmdata_NOtime_status=0;
        MY_Table1_Newdata_status=1;


    }

}


/*
功能：把数组1，起始地址startaddress1开始的N个数据，传输到数组2中
*/
void my_buf1_to_buf2(uint8_t *source_buf,uint8_t startaddress1,uint8_t *direc_buf,uint8_t startaddress2,uint8_t number)
{
    uint8_t k=0;
    for(k=0; k<number; k++)
    {
        direc_buf[k+startaddress2]=source_buf[k+startaddress1];
    }



}

/*
功能：EEPROM中周期数据到 特定数组中
格式：

*/



uint8_t MY_GPRS_MCU_RsBuf[8];  //存储，周期性电池电压、太阳能电压、温度、湿度共4类8个字节的数据

uint8_t MY_GPRS_Call_Single_data_buf[40]= {0};
uint8_t MY_GPRS_Call_Analog_data_buf[110]= {0};
uint8_t MY_GPRS_Call_Time_data_buf[7]= {0};
uint8_t MY_GPRS_Call_Single_data_number=0X12;  //18个信息体，6*3，可以带3个支线，每个信息体1个字节
uint8_t MY_GPRS_Call_Analog_data_number=0X24;  // 36  36/3=12  12/3=4  每个指示器4个数据，电流、电场、温度、内部电压，共36个信息体，每个信息体3个字节
uint8_t MY_GPRS_Call_Status=0;    //存储获得总召数据状态，为1表示有总召数据，为0表示没有总召数据

uint8_t MY_EEPROM_Buf[256]= {0}; //用来存储EEPROM中，读出的数据
uint8_t MY_GPRS_Cycle_Transmintdata_status=0;  //用来存储读取到总召数据发送状态，1为已发送，2为未发送

uint8_t my_eeprom_data_to_cycle_array(void) //EEPROM中周期数据到 特定数组
{

    uint32_t startaddressPT=MY_Table2_Cycledata_StartAddress;
    uint32_t endaddressPT=MY_Table2_Cycledata_EndAddress;
    uint32_t writeaddressPT=MY_Table2_Cycledata_WriteAddress;
    uint32_t readaddressPT=MY_Table2_Cycledata_ReadAddress;
    uint16_t number_single=0;
    uint16_t number_anlog=0;
    uint8_t tempbuf[12]= {0};
    uint32_t my_int1=0;
    uint32_t my_int2=0;
    uint8_t rs_temp_buf[512]= {0}; //测试使用，@@@
    //uint8_t *PT_buf;
//	uint32_t PT_writeaddress=0;
//	uint16_t PT_number=0;
//	uint8_t temp=0;

//***处理获得最新的表指针地址
    SPI_EE_BufferRead2(tempbuf,213,12);

    my_buf_writeto_val(tempbuf, &startaddressPT, &endaddressPT, &writeaddressPT, &readaddressPT);
    MY_Table2_Cycledata_StartAddress=startaddressPT;
    MY_Table2_Cycledata_EndAddress=endaddressPT;
    MY_Table2_Cycledata_WriteAddress=writeaddressPT;
    MY_Table2_Cycledata_ReadAddress=readaddressPT;
//USART_printf(&huart3,"write add=%d  read add=%d\r\n",writeaddressPT-EEPROM_Chip_size,readaddressPT-EEPROM_Chip_size); //测试使用@@@@@
    my_int1=(writeaddressPT-EEPROM_Chip_size);
    my_int2=readaddressPT-EEPROM_Chip_size;
    USART_printf(&huart3,"write add=%d%d%d%d%d%d  read add=%d%d%d%d%d%d   block=%d \r\n",my_int1/100000,my_int1%100000/10000,my_int1%10000/1000,my_int1%1000/100,my_int1%100/10,my_int1%10
                 ,my_int2/100000,my_int2%100000/10000,my_int2%10000/1000,my_int2%1000/100,my_int2%100/10,my_int2%10,(my_int1-my_int2)/144);

    if(readaddressPT!=writeaddressPT)
    {
        SPI_EE_BufferRead2(MY_EEPROM_Buf,readaddressPT,160); //利用读指针，读出最新的数据

        MY_GPRS_Cycle_Transmintdata_status=MY_EEPROM_Buf[0];
        number_single=MY_EEPROM_Buf[1];
        number_anlog=MY_EEPROM_Buf[2];   //
        MY_GPRS_Call_Single_data_number=number_single;
        MY_GPRS_Call_Analog_data_number=number_anlog;

//读指针强制校验环节，如果读取EEPROM的数据是错误的，不符合规则则返回0.  2015-11.16
        if((MY_GPRS_Cycle_Transmintdata_status!=1 && MY_GPRS_Cycle_Transmintdata_status!=2)
                || (MY_GPRS_Call_Single_data_number!=18)
                || (MY_GPRS_Call_Analog_data_number!=36))
        {
            MY_GPRS_Call_Status=0;
            USART_printf(&huart3,"Read data from eeprom first 3 bytes is error!!\r\n");
            return MY_GPRS_Call_Status;
        }


        number_single=0x12; 	//强制数量，18个信息体
        number_anlog=0x24;   //强制数量，36个信息体


        MY_GPRS_Call_Single_data_number=number_single;
        MY_GPRS_Call_Analog_data_number=number_anlog;

        my_buf1_to_buf2(MY_EEPROM_Buf,3,MY_GPRS_MCU_RsBuf,0,8); //环境参数
        my_buf1_to_buf2(MY_EEPROM_Buf,11,MY_GPRS_Call_Single_data_buf,0,number_single);  //遥信数据
        my_buf1_to_buf2(MY_EEPROM_Buf,11+number_single,MY_GPRS_Call_Analog_data_buf,0,number_anlog*3); //遥测数据
        my_buf1_to_buf2(MY_EEPROM_Buf,11+number_single+number_anlog*3,MY_GPRS_Call_Time_data_buf,0,7);  //时标数据

        //测试使用，显示过程数据@@@@@
        USART_printf(&huart3,"Read data address=%d%d%d%d%d%d\r\n",(readaddressPT-EEPROM_Chip_size)/100000,
                     (readaddressPT-EEPROM_Chip_size)%100000/10000,(readaddressPT-EEPROM_Chip_size)%10000/1000,
                     (readaddressPT-EEPROM_Chip_size)%1000/100,(readaddressPT-EEPROM_Chip_size)%100/10,
                     (readaddressPT-EEPROM_Chip_size)%10);
        HexToStr2(rs_temp_buf,MY_EEPROM_Buf,160);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");
        /*
        USART_printf(&huart3,"sing number=%d  analog number=%d\r\n",number_single,number_anlog);
        HexToStr2(rs_temp_buf,MY_GPRS_MCU_RsBuf,8);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");
        HexToStr2(rs_temp_buf,MY_GPRS_Call_Single_data_buf,18);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");
        HexToStr2(rs_temp_buf,MY_GPRS_Call_Analog_data_buf,108);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");
        HexToStr2(rs_temp_buf,MY_GPRS_Call_Time_data_buf,7);
        USART_printf(&huart3,rs_temp_buf);
        USART_printf(&huart3,"\r\n");
        */

        //------------------------------------------

        //-------------------
        MY_GPRS_Call_Status=1;
    }
    else MY_GPRS_Call_Status=0;


    return MY_GPRS_Call_Status;

}



/*
功能：EEPROM中报警数据存入到特定数组中

*/

uint8_t MY_GPRS_Alarmdata_TimeBuf[256]= {0}; //存储，有时标，报警数据
uint8_t MY_GPRS_ALarmdata_NOtime_status=0; //为1，表示收到无时标报警数据

uint8_t MY_GPRS_ALarmdata_Time_status=0;   //为1，表示收到有时标报警数据
uint8_t MY_GPRS_ALarmdata_number=0;  // 存储，报警信息体个数
uint8_t	MY_GPRS_ALarm_Transmintdata_status=0;

uint8_t my_eeprom_data_to_alarm_array(void)//EEPROM中报警数据到 特定数组中
{
    uint32_t startaddressPT=MY_Table1_Alarmdata_StartAddress;
    uint32_t endaddressPT=MY_Table1_Alarmdata_EndAddress;
    uint32_t writeaddressPT=MY_Table1_Alarmdata_WriteAddress;
    uint32_t readaddressPT=MY_Table1_Alarmdata_ReadAddress;
    uint16_t number_alarm=0;
    uint8_t tempbuf[12]= {0};
//	uint8_t *PT_buf;
//	uint32_t PT_writeaddress=0;
//	uint16_t PT_number=0;
//	uint8_t temp=0;

    //***处理获得最新的表指针地址
    SPI_EE_BufferRead2(tempbuf,201,12);

    my_buf_writeto_val(tempbuf, &startaddressPT, &endaddressPT, &writeaddressPT, &readaddressPT);
    MY_Table1_Alarmdata_StartAddress=startaddressPT;
    MY_Table1_Alarmdata_EndAddress=endaddressPT;
    MY_Table1_Alarmdata_WriteAddress=writeaddressPT;
    MY_Table1_Alarmdata_ReadAddress=readaddressPT;

    if(writeaddressPT!=readaddressPT)
    {
        SPI_EE_BufferRead2(MY_EEPROM_Buf,readaddressPT,256); //利用读指针，读出最新的数据
        MY_GPRS_ALarm_Transmintdata_status=MY_EEPROM_Buf[0];  //存GPRS发送状态
        number_alarm=MY_EEPROM_Buf[1];
        MY_GPRS_ALarmdata_number=number_alarm;
        my_buf1_to_buf2(MY_EEPROM_Buf,2,MY_GPRS_Alarmdata_TimeBuf,0,number_alarm*10); //带时标的报警数据，每组数据10个字节

        //--------------------
        MY_GPRS_ALarmdata_Time_status=1;
    }
    else MY_GPRS_ALarmdata_Time_status=0;




    return MY_GPRS_ALarmdata_Time_status;

}

/*

功能：移动表的读指针，同时修改已发送帧的GPRS发送状态字节为1，
			表示发送完了,transmin_OK为1，表示发送成功，为0表示发送失败
*/
void my_GPRS_chang_tablereadpt(uint8_t table_number,uint8_t transmint_OK)  //table_number为1表示报警表，为2表示周期数据表
{
    uint32_t startaddressPT=0;      		//MY_Table2_Cycledata_StartAddress;
    uint32_t endaddressPT=0;        	 //MY_Table2_Cycledata_EndAddress;
    uint32_t temp_writeaddressPT=0;      	 //MY_Table2_Cycledata_WriteAddress;
    uint32_t temp_readaddressPT=0;  			//MY_Table2_Cycledata_ReadAddress;
    //uint16_t number_single=0;
    //uint16_t number_anlog=0;
    uint8_t tempbuf[12]= {0};
    uint32_t tableaddress=0;
    int length=0;

    if(table_number==1)
    {
        SPI_EE_BufferRead2(tempbuf,201,12);
        my_buf_writeto_val(tempbuf, &startaddressPT, &endaddressPT, &temp_writeaddressPT, &temp_readaddressPT);
        tableaddress=210;

    }
    else if(table_number==2)
    {
        SPI_EE_BufferRead2(tempbuf,213,12);
        my_buf_writeto_val(tempbuf, &startaddressPT, &endaddressPT, &temp_writeaddressPT, &temp_readaddressPT);
        tableaddress=222;

    }
    if(transmint_OK==1)
        AT25_WriteByte(0X01,temp_readaddressPT); //修改GPRS发送状态字节为1，表示发送完了,2表示没有发送出去
    else if(transmint_OK==0 && table_number==1)
    {
        AT25_WriteByte(0X02,temp_readaddressPT);
        AT25_WriteByte(0X01,200);  //ROM中200地址的单元标识，报警数据发送的状态，为1表示有报警数据因为GPRS问题没有发送出去，为0表示所有报警数据都发送出去了
        RE_ALarmData_Status=01;
    }
    else if(transmint_OK==0 && table_number==2)
    {
        AT25_WriteByte(0X02,temp_readaddressPT);
        AT25_WriteByte(0X01,199);  //ROM中200地址的单元标识，报警数据发送的状态，为1表示有报警数据因为GPRS问题没有发送出去，为0表示所有报警数据都发送出去了
        RE_CycleData_Status=01;
    }

    //当发送数据成功了就进行读指针移动，如果没成功不移动数据

    if(transmint_OK==1 && table_number==1)
    {

        length=endaddressPT-temp_readaddressPT+1;  //判断是否到了存储区的末尾
        if(length>12)    //如果大于12表示还没到末尾  //每个信息体占10个字节，地址2个，值一个，时标7个，加一个状态，一个数量
        {
            SPI_EE_BufferRead2(tempbuf,temp_readaddressPT,12);  //读取一帧的数据
            if(tempbuf[0]==0x00&&tempbuf[1]==00)
                temp_readaddressPT=startaddressPT;  //如果都为00，说明末尾，没有数据了，这个应该没用，有问题，实际运行在考虑一下？？？
            else   //有效数据，进行读指针移动
            {
                if(table_number==1)
                {
                    temp_readaddressPT=temp_readaddressPT+2+tempbuf[1]*10;  //多个数据
                }
                else if(table_number==2)
                {
                    temp_readaddressPT=temp_readaddressPT+11+tempbuf[1]+tempbuf[2]*3+7;
                }
            }
        }
        else  //如果小于12表示到了末尾
        {
            temp_readaddressPT=startaddressPT;
        }
        MY_Table1_Alarmdata_ReadAddress=temp_readaddressPT;
        tempbuf[0]=temp_readaddressPT&0x0000ff;
        tempbuf[1]=(temp_readaddressPT>>8)&0x0000ff;
        tempbuf[2]=(temp_readaddressPT>>16)&0x0000ff;
        SPI_EE_BufferWrite2(tempbuf,tableaddress,3);


    }
    else if(transmint_OK==1 && table_number==2)
    {

        //SPI_EE_BufferRead2(tempbuf,temp_readaddressPT,20);  //读取一帧的数据
        //temp_readaddressPT=temp_readaddressPT+11+tempbuf[1]+tempbuf[2]*3+7;
        temp_readaddressPT=temp_readaddressPT+144;

        length=endaddressPT-temp_readaddressPT+1;  //判断是否到了存储区的末尾

        if(temp_readaddressPT>temp_writeaddressPT)
        {
            if(length<144)//如果大于144表示还没到末尾3+8+18+3*9*4+7=144,小于144到结尾了		3是,8是DTU（4个参数） ,18是报警(2个参数，9个指示器)
                temp_readaddressPT=startaddressPT;  //没有数据
        }

        MY_Table2_Cycledata_ReadAddress=temp_readaddressPT;
        tempbuf[0]=temp_readaddressPT&0x0000ff;
        tempbuf[1]=(temp_readaddressPT>>8)&0x0000ff;
        tempbuf[2]=(temp_readaddressPT>>16)&0x0000ff;
        SPI_EE_BufferWrite2(tempbuf,tableaddress,3);

    }


}


/*
功能：修改帧发送单元状态，发送完了，由2变为1，有问题
*/
void my_GPRS_chang_Transmitword_status(uint8_t table_number)  //table_number为1表示报警表，为2表示周期数据表
{
    uint32_t startaddressPT=0;      		//MY_Table2_Cycledata_StartAddress;
    uint32_t endaddressPT=0;        	 //MY_Table2_Cycledata_EndAddress;
    uint32_t writeaddressPT=0;      	 //MY_Table2_Cycledata_WriteAddress;
    uint32_t readaddressPT=0;  				//MY_Table2_Cycledata_ReadAddress;
    //uint16_t number_single=0;
    //uint16_t number_anlog=0;
    uint8_t tempbuf[12]= {0};
//	uint32_t tableaddress=0;
//	uint8_t length=0;

    if(table_number==1)
    {
        SPI_EE_BufferRead2(tempbuf,201,12);
        my_buf_writeto_val(tempbuf, &startaddressPT, &endaddressPT, &writeaddressPT, &readaddressPT);
        //tableaddress=210;

    }
    else if(table_number==2)
    {
        SPI_EE_BufferRead2(tempbuf,213,12);

        my_buf_writeto_val(tempbuf, &startaddressPT, &endaddressPT, &writeaddressPT, &readaddressPT);
// tableaddress=222;

    }
    AT25_WriteByte(0X01,readaddressPT); //修改GPRS发送状态字节为1，表示发送完了,这个有问题，如果没有发送出去，但是需要移动读指针
}






/*
功能：MCU重启，初始化重要固定参数，DTU地址，Server的IP地址和端口号， 电话号、Server心跳时间、Server周期数据时间
*/

extern uint16_t Transmint_to_GPRS_hearttime;
extern uint16_t Transmint_to_GPRS_Cycledata;

extern uint8_t MY_IP[4]; //222.222.118.3
extern uint16_t MY_PORT;  //8080  16位数据
extern uint16_t DTU_ADDRESS;
//extern uint8_t AT_MESS_telphonenumber[100];
//extern uint8_t AT_MESS_telphonenumber2[50];
uint8_t FLASH_DTU=1;  //FLASH_DTU清除状态，为1表示清除DTU里面EEPROM地址，为0表示不清除EEPROM地址
extern uint8_t my_sys_init_flash;
void MY_Value_init(void)
{

    //uint8_t k=0;
    uint16_t my_temp_val16=0;
    uint8_t my_buf[8]= {0};
    uint8_t my_status=0;


    uint32_t startaddressPT=MY_Table2_Cycledata_StartAddress;
    uint32_t endaddressPT=MY_Table2_Cycledata_EndAddress;
    uint32_t writeaddressPT=MY_Table2_Cycledata_WriteAddress;
    uint32_t readaddressPT=MY_Table2_Cycledata_ReadAddress;
    uint8_t tempbuf[12]= {0};
    float temp_t1=0;//temp_t2=0;

    //=============

//    my_buf[0]=AT25_ReadByte(EEPROM_sys_start_status);
//    my_buf[1]=AT25_ReadByte(EEPROM_sys_start_status+1);
//    my_temp_val16=my_buf[1];
//    my_temp_val16=(my_temp_val16<<8)+ my_buf[0];
//    if(my_temp_val16==0xCDAB) //非第一次上电
//    {
//        //FLASH_DTU=0;  //非第一次上电
//        //my_sys_init_flash=0;
//			  //my_sys_init_flash=1; //第一次上电
//        FLASH_DTU=1;
//    }
//    else
//    {
//        //my_sys_init_flash=1; //第一次上电
//        FLASH_DTU=1;
//    }
    //=================
//清除内存数据部分，设置DTU为0X0000
    

    if(my_sys_init_flash==1)
    {
        my_status=2;  //第一次上电，写入默认值
    }
    else my_status=1;  //非第一次上电，读取EEPROM值到变量中

    if(my_status==1)
    {

        //(1)把读到的新DTU地址写入到变量中
        my_buf[0]=AT25_ReadByte(EEPROM_DTU_Address);
        my_buf[1]=AT25_ReadByte(EEPROM_DTU_Address+1);
        my_temp_val16=my_buf[1];
        my_temp_val16=((my_temp_val16<<8)&0xff00)+my_buf[0];
        DTU_ADDRESS=my_temp_val16;

        //(2)Server心跳包时间
        my_buf[0]=AT25_ReadByte(EEPROM_Hearttime_Address);
        my_buf[1]=AT25_ReadByte(EEPROM_Hearttime_Address+1);
        my_temp_val16=my_buf[1];
        my_temp_val16=((my_temp_val16<<8)&0xff00)+my_buf[0];

        //Transmint_to_GPRS_hearttime=my_temp_val16;
        if(my_temp_val16>60 && my_temp_val16<60*10)
            MY_M_speed_heart=my_temp_val16;


        //（3）Server周期数据时间

        my_buf[0]=AT25_ReadByte(EEPROM_Cycetime_Address);
        my_buf[1]=AT25_ReadByte(EEPROM_Cycetime_Address+1);
        my_temp_val16=my_buf[1];
        my_temp_val16=((my_temp_val16<<8)&0xff00)+my_buf[0];
        //Transmint_to_GPRS_Cycledata=my_temp_val16;
        if(my_temp_val16>60*3 && my_temp_val16<60*60)
            MY_M_speed_cyc=my_temp_val16;

        //读取阀值，上线

        my_buf[0]=AT25_ReadByte(EEPROM_SPEED_Gate_H_Address);
        my_buf[1]=AT25_ReadByte(EEPROM_SPEED_Gate_H_Address+1);
        my_temp_val16=my_buf[1];
        my_temp_val16=((my_temp_val16<<8)&0xff00)+my_buf[0];
        //Transmint_to_GPRS_hearttime=my_temp_val16;
        temp_t1=my_temp_val16/10.0;
        MY_Speed_H_Gate=temp_t1;

        //读取高速 周期
        my_buf[0]=AT25_ReadByte(EEPROM_SPEED_H_Cyc_Address);
        my_buf[1]=AT25_ReadByte(EEPROM_SPEED_H_Cyc_Address+1);
        my_temp_val16=my_buf[1];
        my_temp_val16=((my_temp_val16<<8)&0xff00)+my_buf[0];
        if(my_temp_val16>60*1 && my_temp_val16<60*60)
            MY_H_speed_cyc=my_temp_val16;


        //读取低速 周期
        my_buf[0]=AT25_ReadByte(EEPROM_SPEED_L_Cyc_Address);
        my_buf[1]=AT25_ReadByte(EEPROM_SPEED_L_Cyc_Address+1);
        my_temp_val16=my_buf[1];
        my_temp_val16=((my_temp_val16<<8)&0xff00)+my_buf[0];
        if(my_temp_val16>60*1 && my_temp_val16<60*60)
            MY_L_speed_cyc=my_temp_val16;




        //（4）Server的IP地址及端口号
        SPI_EE_BufferRead2(MY_IP,EEPROM_IP_Address,4);  //读取EEPROM中的地址到变量中
        my_buf[0]=AT25_ReadByte(EEPROM_IPPort_Address);
        my_buf[1]=AT25_ReadByte(EEPROM_IPPort_Address+1);
        my_temp_val16=my_buf[1];
        my_temp_val16=((my_temp_val16<<8)&0xff00)+my_buf[0];
        MY_PORT=my_temp_val16;   //端口号

        //(7)读取报警数据补发状态
        RE_ALarmData_Status=AT25_ReadByte(EEPROM_RE_Alarmdata_t_status);


    }
    else if(my_status==2)  //第一次运行
    {
        //把默认值写入到EEPROM中
        //(1) DTU
        my_temp_val16=DTU_ADDRESS;      //默认值为0001
        my_buf[0]=my_temp_val16&0x00ff;
        my_buf[1]=(my_temp_val16>>8)&0x00ff;
        if(my_temp_val16!=0x0000)
        {
            AT25_WriteByte(my_buf[0],EEPROM_DTU_Address);
            AT25_WriteByte(my_buf[1],EEPROM_DTU_Address+1);
        }
        //(2)心跳包时间
        //my_temp_val16=Transmint_to_GPRS_hearttime;  //默认值为10，时间为10*5S=50秒
        my_temp_val16=MY_M_speed_heart;
        my_buf[0]=my_temp_val16&0x00ff;
        my_buf[1]=(my_temp_val16>>8)&0x00ff;
        AT25_WriteByte(my_buf[0],EEPROM_Hearttime_Address);
        AT25_WriteByte(my_buf[1],EEPROM_Hearttime_Address+1);


        //（3）Server周期数据时间

        //my_temp_val16=Transmint_to_GPRS_Cycledata;  //默认值为24，时间为10*5S=120秒
        my_temp_val16=MY_M_speed_cyc;
        my_buf[0]=my_temp_val16&0x00ff;
        my_buf[1]=(my_temp_val16>>8)&0x00ff;
        AT25_WriteByte(my_buf[0],EEPROM_Cycetime_Address);
        AT25_WriteByte(my_buf[1],EEPROM_Cycetime_Address+1);

        //门限阀值，上线
        my_temp_val16=(uint16_t)(MY_Speed_H_Gate*10);  //上线
        my_buf[0]=my_temp_val16&0x00ff;
        my_buf[1]=(my_temp_val16>>8)&0x00ff;
        AT25_WriteByte(my_buf[0],EEPROM_SPEED_Gate_H_Address);
        AT25_WriteByte(my_buf[1],EEPROM_SPEED_Gate_H_Address+1);

        //高速周期
        my_temp_val16=MY_H_speed_cyc;
        my_buf[0]=my_temp_val16&0x00ff;
        my_buf[1]=(my_temp_val16>>8)&0x00ff;
        AT25_WriteByte(my_buf[0],EEPROM_SPEED_H_Cyc_Address);
        AT25_WriteByte(my_buf[1],EEPROM_SPEED_H_Cyc_Address+1);



        //低速周期
        my_temp_val16=MY_L_speed_cyc;
        my_buf[0]=my_temp_val16&0x00ff;
        my_buf[1]=(my_temp_val16>>8)&0x00ff;
        AT25_WriteByte(my_buf[0],EEPROM_SPEED_L_Cyc_Address);
        AT25_WriteByte(my_buf[1],EEPROM_SPEED_L_Cyc_Address+1);

        //（4）Server的IP地址及端口号

        SPI_EE_BufferWrite2(MY_IP,EEPROM_IP_Address,4); //把默认IP地址 写入到EEPROM中 222.222.118.3，都是十进制的

        my_temp_val16=MY_PORT;    //默认端口号，8080,都是十进制的
        my_buf[0]=my_temp_val16&0x00ff;
        my_buf[1]=(my_temp_val16>>8)&0x00ff;
        AT25_WriteByte(my_buf[0],EEPROM_IPPort_Address);
        AT25_WriteByte(my_buf[1],EEPROM_IPPort_Address+1);

        //(7) 补发报警数据状态
        RE_ALarmData_Status=0;
        AT25_WriteByte(RE_ALarmData_Status,200);
        //(8)报警数据表
        startaddressPT=MY_Table1_Alarmdata_StartAddress;
        endaddressPT=MY_Table1_Alarmdata_EndAddress;
        writeaddressPT=MY_Table1_Alarmdata_WriteAddress;
        readaddressPT=MY_Table1_Alarmdata_ReadAddress;
        my_val_writeto_eeprom(tempbuf, startaddressPT, endaddressPT, writeaddressPT, readaddressPT,EEPROM_Table1_Address);

        //(9)周期数据表
        startaddressPT=MY_Table2_Cycledata_StartAddress;
        endaddressPT=MY_Table2_Cycledata_EndAddress;
        writeaddressPT=MY_Table2_Cycledata_WriteAddress;
        readaddressPT=MY_Table2_Cycledata_ReadAddress;
        my_val_writeto_eeprom(tempbuf, startaddressPT, endaddressPT, writeaddressPT, readaddressPT,EEPROM_Table2_Address);  //把初始表地址写入到EEPROM中



    }


}



//--------------测试使用程序-----------
/*
功能：对EEPRom中的项量表清零
*/
void my_test_Eepromtable_zero(void)
{
    uint8_t tempbuf[24]= {0};

    SPI_EE_BufferWrite2(tempbuf,201,24);
}




/*
功能：测试程序，对遥信、遥测、环境存储数组进行赋值
*/
uint8_t temp=0x00; //测试用生成总召模拟数据
void my_test_analoydata_array_to_eeprom(void)
{
    uint8_t k=0;



    //遥信
    MY_433_Call_Single_data_number=2;
    for(k=0; k<MY_433_Call_Single_data_number; k++)
    {
        temp=temp+1;
        MY_433_Call_Single_data_buf[k]=temp;
    }

    //遥测
    MY_433_Call_Analog_data_number=4;
    for(k=0; k<MY_433_Call_Analog_data_number*3; k++)
    {
        temp=temp+1;
        MY_433_Call_Analog_data_buf[k]=temp;
    }


    //环境
    for(k=0; k<4*2; k++)
    {

        temp=temp+1;
        MY_MCU_RsBuf[k]=temp;
    }

    //时标
    for(k=0; k<7; k++)
    {
        temp=temp+1;
        MY_433_Call_Time_data_buf[k]=temp;
    }

    MY_MCU_getdata_status=1;
    MY_433_Call_Status=1;

}

//功能：测试使用，对table2周期数据，的读写指针进行初始化为0x2000
void my_test_TABLE2_init()
{
    uint8_t tempbuf[12]= {0};
    uint32_t startaddressPT=MY_Table2_Cycledata_StartAddress;
    uint32_t endaddressPT=MY_Table2_Cycledata_EndAddress;
    uint32_t writeaddressPT=MY_Table2_Cycledata_WriteAddress;
    uint32_t readaddressPT=MY_Table2_Cycledata_ReadAddress;
    //uint32_t tableaddress=0;


    my_val_writeto_eeprom(tempbuf, startaddressPT, endaddressPT, writeaddressPT, readaddressPT,EEPROM_Table2_Address);  //把初始表地址写入到EEPROM中

}

//-----------调试使用程序------------
//参数：第一个 输出目标字符串，第2个16进制字符串，第3个16进制字节数据
void HexToStr2(uint8_t *pbDest, uint8_t *pbSrc, int nLen)
{
    uint8_t	ddl,ddh;
    int i;

//if(pbSrc[0]==0x10) nLen=6;
//else if(pbSrc[0]==0x68)nLen=6+pbSrc[1];

    for (i=0; i<nLen; i++)
    {
        ddh = 48 + pbSrc[i] / 16;
        ddl = 48 + pbSrc[i] % 16;
        if (ddh > 57) ddh = ddh + 7;
        if (ddl > 57) ddl = ddl + 7;
        pbDest[i*3] = ddh;
        pbDest[i*3+1] = ddl;
        pbDest[i*3+2] ='-';

    }

    pbDest[nLen*3] = '\0';
}

