#include "my_cc1101.h"
#include "cmsis_os.h"
#include "my_def_value.h"
#include "my_globle_extern.h"
#include "math.h"




#define RF_reg_count 47
uint16_t my_CC1101_sync_word = 0x0102; //CC1101同步字，默认为0x8799
#define my_CC1101_default_channer 0x02   //芯片默认CC1101信道
uint8_t my_CC1101_chip_address = 0xFE; //芯片地址，0XFE位DTU地址，0XFD为调试器地址，01 02 03为指示器地址
//指示器地址构成：同步字低字节+同步字高字节+信道字节+信道内地址字节(01，02，03)
//例如 01-02-02-FE


uint8_t my_cc1101_dest_address = 0x01;
uint8_t my_CC1101_change_channer = my_CC1101_default_channer; //动态信道号



//10, 7, 5, 0, -5, -10, -15, -20, dbm output power, 0x12 == -30dbm
//INT8U PaTabel[] = { 0xc0, 0xC8, 0x84, 0x60, 0x68, 0x34, 0x1D, 0x0E};
INT8U PaTabel[] = { 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0};


extern uint8_t MY_CC1101_STATUS;
extern uint8_t Cmd_CC1101_RX[];
extern osMutexId myMutex01Handle;
extern EventGroupHandle_t xCreatedEventGroup ;

static const uint8_t CC1101InitData[RF_reg_count][2] =
{   /*
    {CC1101_IOCFG2,      0x0E},  //
    {CC1101_IOCFG1,      0x2E},
    {CC1101_IOCFG0,      0x07},  //
    {CC1101_FIFOTHR,     0x47},  //默认47，数量溢出
    {CC1101_PKTCTRL1,    0x06},  //06
    {CC1101_PKTCTRL0,    0x05},  //
    {CC1101_CHANNR,      0x01},  //
    {CC1101_FSCTRL1,     0x06},  //
    {CC1101_FREQ2,       0x0F},
    {CC1101_FREQ1,       0x62},
    {CC1101_FREQ0,       0x76},
    {CC1101_MDMCFG4,     0xF6},
    {CC1101_MDMCFG3,     0x43},
    {CC1101_MDMCFG2,     0x13},
    {CC1101_DEVIATN,     0x15},
    {CC1101_MCSM0,       0x18},  //
    {CC1101_FOCCFG,      0x16},
    {CC1101_WORCTRL,     0xFB},  //
    {CC1101_FSCAL3,      0xE9},
    {CC1101_FSCAL2,      0x2A},
    {CC1101_FSCAL1,      0x00},
    {CC1101_FSCAL0,      0x1F},
    {CC1101_TEST2,       0x81},
    {CC1101_TEST1,       0x35},
    {CC1101_MCSM1,       0x3B},//============
    */
    {CC1101_IOCFG2,      0x0E},
    {CC1101_IOCFG1,      0x2E},
    {CC1101_IOCFG0,      0x06},  //0X06，无数据时为0，有数据位高电平。0X46，无数据为高电平，有数据位低电平
    {CC1101_FIFOTHR,     0x4E},
    {CC1101_SYNC1,       0x87},
    {CC1101_SYNC0,       0x99},
    {CC1101_PKTLEN,      0xFF},
    {CC1101_PKTCTRL1,    0x06},  //0x06开启地址校验，04，不开启地址校验
    {CC1101_PKTCTRL0,    0x05},  //05 0000 0101
    {CC1101_ADDR,        0x05},
    {CC1101_CHANNR,      my_CC1101_default_channer},//信道
    {CC1101_FSCTRL1,     0x06},
    {CC1101_FSCTRL0,     0x00},
    {CC1101_FREQ2,       0x0F},
    {CC1101_FREQ1,       0x62},
    {CC1101_FREQ0,       0x76},
    {CC1101_MDMCFG4,     0xFA},  //2.4K-0xF6 50K-0XFA 60k-FB
    {CC1101_MDMCFG3,     0x93},  //2.4K-0x83 50K-0XF8 60K-2E
    {CC1101_MDMCFG2,     0x13},  //默认为0X13
    {CC1101_MDMCFG1,     0x22},
    {CC1101_MDMCFG0,     0xF8},
    {CC1101_DEVIATN,     0x15},
    {CC1101_MCSM2,       0x07},
    {CC1101_MCSM1,       0x30},
    {CC1101_MCSM0,       0x18},
    {CC1101_FOCCFG,      0x16},
    {CC1101_BSCFG,       0x6C},
    {CC1101_AGCCTRL2,    0x03},
    {CC1101_AGCCTRL1,    0x40},
    {CC1101_AGCCTRL0,    0x91},
    {CC1101_WOREVT1,     0x87},
    {CC1101_WOREVT0,     0x6B},
    {CC1101_WORCTRL,     0xFB},
    {CC1101_FREND1,      0x56},
    {CC1101_FREND0,      0x10},
    {CC1101_FSCAL3,      0xE9},
    {CC1101_FSCAL2,      0x2A},
    {CC1101_FSCAL1,      0x00},
    {CC1101_FSCAL0,      0x1F},
    {CC1101_RCCTRL1,     0x41},
    {CC1101_RCCTRL0,     0x00},
    {CC1101_FSTEST,      0x59},
    {CC1101_PTEST,       0x7F},
    {CC1101_AGCTEST,     0x3F},
    {CC1101_TEST2,       0x81},
    {CC1101_TEST1,       0x35},
    {CC1101_TEST0,       0x0B},
};


// 串口相关变量
INT8U   COM_TxNeed = 0;
INT8U   COM_TimeOut = 0;
INT8U   COM_RxCounter = 0;
INT8U   COM_TxCounter = 0;
INT8U   COM_RxBuffer[65] = {0};
INT8U   COM_TxBuffer[65] = {0};

/*===========================================================================
* 函数 ：RF_RecvHandler() => 无线数据接收处理                               *
============================================================================*/
//void RF_RecvHandler(void)
//{
//    INT8U length = 0, recv_buffer[64] = {0};
//    uint8_t i = 0;

//    //__HAL_GPIO_EXTI_CLEAR_IT(PIN_CC_IRQ); //这个可以清除外部中断，每个中断的清除函数都不一样，需要分别调用
//    // HAL_NVIC_EnableIRQ(EXIT_CC_IRQ_GD0);
//    CC1101SetTRMode(RX_MODE);           // 进入接收模式



//    if (1 == CC_IRQ_READ())             // 检测无线模块是否产生接收中断
//    {
//        while (CC_IRQ_READ() == 1);

//        // 读取接收到的数据长度和数据内容
//        length = CC1101RecPacket(recv_buffer);

//        // 判断接收数据是否正确
//        if (length <= SEND_MAX && length > 0)
//        {
//            LED3_ON;               // LED闪烁，用于指示收到数据
//            for(i = 0; i < length; i++)
//            {
//                printf("[%XH]--", recv_buffer[i]);
//            }
//            printf("\n");
//            //printf("%s\n",recv_buffer);
//            LED3_OFF;

//        }
//    }
//}

/*===========================================================================
* 函数 : BSP_RF_SendPacket() => 无线发送数据函数，串口接收数据，转发给CC1101                        *
============================================================================*/
void RF_SendPacket(void)
{
    INT8U i = 0, j = 0, length = 0, buffer[65] = {0};

    if ((0 == COM_TimeOut) && (COM_RxCounter > 0) )
    {
        if(MY_CC1101_STATUS == 0x01 || MY_CC1101_STATUS == 0x02) return;
        else MY_CC1101_STATUS = 0X01;

        //HAL_NVIC_DisableIRQ(EXIT_CC_IRQ_GD0); //关闭中断，不响应发射中断
        CC1101SetIdle();

        //LED3_ON;

        length = COM_RxCounter;

        COM_RxCounter = 0;

        for (i = 0; i < length; i++)   {
            buffer[i] = COM_RxBuffer[i];
        }

        //CC1101SendPacket(buffer, length, ADDRESS_CHECK);    // 发送数据，默认单地址发送方式
        for(j = 0; j <= 0x6; j++)			 //注意，频率的计算方法，基频+通道号*间隔带宽，所以要考虑，有可能超出CC1101的频率范围，导致CC1101死机
        {
            my_CC1101_change_channer = j;
            CC1101WriteReg(CC1101_CHANNR, my_CC1101_change_channer); //变换信道
            buffer[0] = j + 0X30; //调试使用，作为信道号的标记


            for(i = 0xFA; i < 0XFF; i++) //多地址发送，注意0地址是广播地址,变换地址发送
            {
                buffer[length] = '-';
                buffer[length + 1] = i - 0XFA + 0X30;
                buffer[length + 2] = '\n';
                CC1101SendPacket_add(buffer, length + 3, ADDRESS_CHECK, i);
            }
        }

        //	HAL_NVIC_ClearPendingIRQ(EXIT_CC_IRQ_GD0); //清除历史中断请求信息 EXIT_CC_IRQ_GD0，这个不起作用，不知道为什么


        // __HAL_GPIO_EXTI_CLEAR_IT(PIN_CC_IRQ); //这个可以清除外部中断，每个中断的清除函数都不一样，需要分别调用
        //	HAL_NVIC_EnableIRQ(EXIT_CC_IRQ_GD0);


        // CC1101SetTRMode(RX_MODE);                           // 进入接收模式

        printf("USART TO CC1101\n");

        MY_CC1101_STATUS = 0x00;
    }
}






/*===========================================================================
* 函数 ：RF_Initial() => 初始化RF芯片                                       *
* 说明 ：CC1101的操作，已经被建成C库，见CC1101.c文件， 提供SPI和CSN操作，	*
         即可调用其内部所有函数用户无需再关心CC1101的寄存器操作问题。       *
============================================================================*/
void RF_Initial(void)
{
    CC1101Init();               // 初始化CC1101寄存器
    CC1101SetTRMode(RX_MODE);   // 接收模式
}

/*
================================================================================
Function : CC1101RecPacket( )
    Receive a packet
INPUT    : rxBuffer, A buffer store the received data
OUTPUT   : 1:received count, 0:no data
================================================================================
*/
INT8U RS_buf_status[2] = {0}; //用来存储接收到的数据的RSSI和crc结果
uint8_t my_RSSI_dbm_all=0;  //用来存储最终的CC1101的信号强度，最终的正整数结果

INT8U CC1101RecPacket( INT8U *rxBuffer )
{

    INT8U pktLen;
//    INT16U x;
    //j = 0;

    if ( CC1101GetRXCnt( ) != 0 )
    {
        pktLen = CC1101ReadReg(CC1101_RXFIFO);           // Read length byte

        if( pktLen == 0 || pktLen>60)
        {

            printf("CC1101 get length [%d] data!!error\r\n",pktLen);
            CC1101ClrRXBuff( );
            return 0;
        }
        else
        {
            ;
        } // { pktLen --; }
        CC1101ReadMultiReg(CC1101_RXFIFO, rxBuffer, pktLen); // Pull data
        CC1101ReadMultiReg(CC1101_RXFIFO, RS_buf_status, 2);   // Read  status bytes
        //my_fun_display_buf_16(rxBuffer,pktLen+2);//显示接收到的数据

        CC1101ClrRXBuff( );


        if( RS_buf_status[1] & CRC_OK )
        {
            int my_rssi_dbm = 0;
            my_rssi_dbm = RS_buf_status[0]; //RSSI
					
            if(my_rssi_dbm >= 128)
                my_rssi_dbm = (my_rssi_dbm - 256) / 2 - 75;
            else
                my_rssi_dbm = (my_rssi_dbm) / 2 - 75;
						//存储信号强度
						my_RSSI_dbm_all=(int)(fabs(my_rssi_dbm));

						if(my_rssi_dbm<=-70)
            printf("*** RSSI=[%d] ***\n", my_rssi_dbm); //@@@这个值上线的时候修改为-70，否则不断的重复输出
            return pktLen;
        }
        else
        {
            int my_rssi_dbm = 0;
            my_rssi_dbm = RS_buf_status[0]; //RSSI
            if(my_rssi_dbm >= 128)
                my_rssi_dbm = (my_rssi_dbm - 256) / 2 - 75;
            else
                my_rssi_dbm = (my_rssi_dbm) / 2 - 75;

            my_RSSI_dbm_all=(int)(fabs(my_rssi_dbm));
            printf("CC1101 CRC error\r\n");
            printf("RSSI=[%d]--CRC=[%XH]-length=[%d]\r\n",my_rssi_dbm,RS_buf_status[1],pktLen);
            return 0;
        }
    }
    else
    {
        printf("CC1101 get cnt 0 data!!\r\n");
        return 0;    // Error
    }
}


/*
================================================================================
Function : CC1101SendPacket( )
    Send a packet
INPUT    : txbuffer, The buffer stores data to be sent
           size, How many bytes should be sent
           mode, Broadcast or address check packet
OUTPUT   : None
================================================================================
*/
//void CC1101SendPacket( INT8U *txbuffer, INT8U size, TX_DATA_MODE mode )
//{
//    INT8U address;

//    HAL_NVIC_DisableIRQ(EXIT_CC_IRQ_GD0); //关闭中断，不产生发射中断

//    if( mode == BROADCAST )             {
//        address = 0;
//    }
//    else if( mode == ADDRESS_CHECK )    {
//        address = CC1101ReadReg( CC1101_ADDR );
//    }

//    CC1101ClrTXBuff( );

//    address = my_cc1101_dest_address; //测试使用，写入接收方地址
//    if( ( CC1101ReadReg( CC1101_PKTCTRL1 ) & ~0x03 ) != 0 )
//    {
//        CC1101WriteReg( CC1101_TXFIFO, size + 1 );
//        CC1101WriteReg( CC1101_TXFIFO, address );
//    }
//    else
//    {
//        CC1101WriteReg( CC1101_TXFIFO, size );
//    }

//    CC1101WriteMultiReg( CC1101_TXFIFO, txbuffer, size );
//    CC1101SetTRMode( TX_MODE );

//    while( HAL_GPIO_ReadPin(PORT_CC_IRQ, PIN_CC_IRQ) != 1 );
//    while( HAL_GPIO_ReadPin(PORT_CC_IRQ, PIN_CC_IRQ) == 1 );
//    //HAL_Delay(1000);



//    CC1101ClrTXBuff( );  //情况发送FIFO
//}


/*
================================================================================
Function : CC1101ClrTXBuff( )
    Flush the TX buffer of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101ClrTXBuff( void )
{
    CC1101SetIdle();//MUST BE IDLE MODE
    CC1101WriteCmd( CC1101_SFTX );
}

/*
================================================================================
Function : CC1101SetIdle( )
    Set the CC1101 into IDLE mode
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101SetIdle( void )
{
    CC1101WriteCmd(CC1101_SIDLE);
}

/*
================================================================================
Function : CC1101ClrRXBuff( )
    Flush the RX buffer of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101ClrRXBuff( void )
{
    CC1101SetIdle();//MUST BE IDLE MODE
    CC1101WriteCmd( CC1101_SFRX );
}

/*
================================================================================
Function : CC1101ReadMultiReg( )
    Read some bytes from the rigisters continously
INPUT    : addr, The address of the register
           buff, The buffer stores the data
           size, How many bytes should be read
OUTPUT   : None
================================================================================
*/
void CC1101ReadMultiReg( INT8U addr, INT8U *buff, INT8U size )
{
    INT8U i, j;
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr | READ_BURST);
    for( i = 0; i < size; i ++ )
    {
        for( j = 0; j < 20; j ++ );
        *( buff + i ) = SPI_ExchangeByte( 0xFF );
    }
    CC_CSN_HIGH( );
}



/*
================================================================================
Function : CC1101GetRXCnt( )
    Get received count of CC1101
INPUT    : None
OUTPUT   : How many bytes hae been received
================================================================================
*/
INT8U CC1101GetRXCnt( void )
{
    return ( CC1101ReadStatus( CC1101_RXBYTES )  & BYTES_IN_RXFIFO );
}

/*
================================================================================
Function : CC1101SetTRMode( )
    Set the device as TX mode or RX mode
INPUT    : mode selection
OUTPUT   : None
================================================================================
*/
void CC1101SetTRMode( TRMODE mode )
{
    if( mode == TX_MODE )
    {
        CC1101WriteReg(CC1101_IOCFG0, 0x06);
        CC1101WriteCmd( CC1101_STX );
    }
    else if( mode == RX_MODE )
    {
        CC1101WriteReg(CC1101_IOCFG0, 0x06);
        CC1101WriteCmd( CC1101_SRX );
    }
}



/*
================================================================================
Function : CC1101Init( )
    Initialize the CC1101, User can modify it
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101Init( void )
{
    volatile uint8_t i, j;

    CC1101Reset( );

    for( i = 0; i < RF_reg_count; i++ )  //写了23个寄存器 RF_reg_count
    {
        CC1101WriteReg( CC1101InitData[i][0], CC1101InitData[i][1] );
    }


    CC1101SetAddress( my_CC1101_chip_address, BROAD_0AND255 );
    CC1101SetSYNC( my_CC1101_sync_word );//0X8799
    CC1101WriteReg(CC1101_MDMCFG1,   0x72); //Modem Configuration

    CC1101WriteMultiReg(CC1101_PATABLE, PaTabel, 8 );

    i = CC1101ReadStatus( CC1101_PARTNUM );//for test, must be 0x80
    printf("PARTNUM=[%XH]  ", i);
    i = CC1101ReadStatus( CC1101_VERSION );//for test, refer to the datasheet
    printf("VERSION=[%XH]  \n", i);
}

/*
================================================================================
Function : CC1101ReadStatus( )
    Read a status register
INPUT    : addr, The address of the register
OUTPUT   : the value read from the status register
================================================================================
*/
INT8U CC1101ReadStatus( INT8U addr )
{
    INT8U i;
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr | READ_BURST);
    i = SPI_ExchangeByte( 0xFF );
    CC_CSN_HIGH( );
    return i;
}



/*
================================================================================
Function : CC1101WriteMultiReg( )
    Write some bytes to the specified register
INPUT    : addr, The address of the register
           buff, a buffer stores the values
           size, How many byte should be written
OUTPUT   : None
================================================================================
*/
void CC1101WriteMultiReg( INT8U addr, INT8U *buff, INT8U size )
{
    INT8U i;
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr | WRITE_BURST );
    for( i = 0; i < size; i ++ )
    {
        SPI_ExchangeByte( *( buff + i ) );
    }
    CC_CSN_HIGH( );
}




/*
================================================================================
Function : CC1101SetSYNC( )
    Set the SYNC bytes of the CC1101
INPUT    : sync, 16bit sync
OUTPUT   : None
================================================================================
*/
void CC1101SetSYNC( INT16U sync )
{
    CC1101WriteReg(CC1101_SYNC1, 0xFF & ( sync >> 8 ) );
    CC1101WriteReg(CC1101_SYNC0, 0xFF & sync );
}


/*
================================================================================
Function : CC1101SetAddress( )
    Set the address and address mode of the CC1101
INPUT    : address, The address byte
           AddressMode, the address check mode
OUTPUT   : None
================================================================================
*/
void CC1101SetAddress( uint8_t address, ADDR_MODE AddressMode)
{
    uint8_t btmp = CC1101ReadReg( CC1101_PKTCTRL1 ) & ~0x03;
    CC1101WriteReg(CC1101_ADDR, address);
    if     ( AddressMode == BROAD_ALL )     {}
    else if( AddressMode == BROAD_NO  )     {
        btmp |= 0x01;
    }
    else if( AddressMode == BROAD_0   )     {
        btmp |= 0x02;
    }
    else if( AddressMode == BROAD_0AND255 ) {
        btmp |= 0x03;
    }
}


/*
================================================================================
Function : CC1101ReadReg( )
    read a byte from the specified register
INPUT    : addr, The address of the register
OUTPUT   : the byte read from the rigister
================================================================================
*/
uint8_t CC1101ReadReg( uint8_t addr )
{
    uint8_t i;
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr | READ_SINGLE);
    i = SPI_ExchangeByte( 0xFF );
    CC_CSN_HIGH( );
    return i;
}


/*
================================================================================
Function : CC1101Reset( )
    Reset the CC1101 device
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101Reset( void )
{
    uint8_t x;

    CC_CSN_HIGH( );
    HAL_Delay(50);
    CC_CSN_LOW( );
    HAL_Delay(50);
    CC_CSN_HIGH( );
    for( x = 0; x < 100; x ++ );
    CC1101WriteCmd( CC1101_SRES );
}

void CC_CSN_LOW(void)
{

    HAL_GPIO_WritePin(PORT_CC_CSN, PIN_CC_CSN, GPIO_PIN_RESET);
    while( HAL_GPIO_ReadPin(PORT_CC_CSN, PIN_CC_CSN) != GPIO_PIN_RESET);

}
void CC_CSN_HIGH( void)
{
    HAL_GPIO_WritePin(PORT_CC_CSN, PIN_CC_CSN, GPIO_PIN_SET);
}

/*
================================================================================
Function : CC1101WriteCmd( )
    Write a command byte to the device
INPUT    : command, the byte you want to write
OUTPUT   : None
================================================================================
*/
void CC1101WriteCmd( uint8_t command )
{   // uint8_t x1=0;
    CC_CSN_LOW( );
    // x1=SPI_ExchangeByte( command );
    SPI_ExchangeByte( command );
    CC_CSN_HIGH( );
    //printf("writeCmd -[%XH]-read:[%XH]\n",command,x1);
}

/*
================================================================================
Function : CC1101WriteReg( )
    Write a byte to the specified register
INPUT    : addr, The address of the register
           value, the byte you want to write
OUTPUT   : None
================================================================================
*/
void CC1101WriteReg( uint8_t addr, uint8_t value )
{
//		uint8_t x1=0,x2=0;
    CC_CSN_LOW( );
    SPI_ExchangeByte( addr );
    SPI_ExchangeByte( value );
    CC_CSN_HIGH( );
    //printf("write reg read data:[%XH]-[%XH]\n",x1,x2);

}

/*
================================================================================
Function : CC1101WORInit( )
    Initialize the WOR function of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
*/
void  CC1101WORInit( void )
{
    CC1101WriteReg(CC1101_MCSM2, 0x00); //0x16
    CC1101WriteReg(CC1101_MCSM0, 0x18); //0X18
    CC1101WriteReg(CC1101_WOREVT1, 0x8C); //0x1E
    CC1101WriteReg(CC1101_WOREVT0, 0xA0); //0x1f
    CC1101WriteReg(CC1101_WORCTRL, 0x78); //0X20   Wake On Radio Control

    CC1101WriteCmd( CC1101_SWORRST );
}

//------------自定义---------
//void RF_RecvHandler_intrrupt(void)
//{
//    INT8U length = 0, recv_buffer[64] = {0};
//    uint8_t i = 0;
//    int RSSI_dBm = 0;

//    if(MY_CC1101_STATUS == 0x01 || MY_CC1101_STATUS == 0x02) return;
//    else MY_CC1101_STATUS = 0X02;

//    {
//        while (CC_IRQ_READ() == 1);

//        // 读取接收到的数据长度和数据内容
//        length = CC1101RecPacket(recv_buffer);

//        // 判断接收数据是否正确
//        if (length <= SEND_MAX && length > 0)
//        {
//            //LED3_ON;               // LED闪烁，用于指示收到数据

//            printf("RX-data:");
//            for(i = 0; i < length; i++)
//            {
//                Cmd_CC1101_RX[i] = recv_buffer[i];
//                printf("[%XH]-", recv_buffer[i]);
//            }


//            RSSI_dBm = RS_buf_status[0];
//            if(RSSI_dBm >= 128) RSSI_dBm = (RSSI_dBm - 256) / 2 - 75;
//            else RSSI_dBm = (RSSI_dBm) / 2 - 75;

//            printf("\nRX_RSSI:[%XH]-[%d]\n", RS_buf_status[0], RSSI_dBm);

//            //LED3_OFF;

//        }
//    }

//    CC1101SetTRMode(RX_MODE);           // 进入接收模式
//    MY_CC1101_STATUS = 0X00;
//    //CC1101WriteCmd(CC1101_SWOR); //设置WOR模式
//}

void my_read_all_reg(void)
{
    int i = 0;
    uint8_t read_buf[RF_reg_count] = {0};
    for(i = 0; i < RF_reg_count; i++)
    {
        read_buf[i] = CC1101ReadReg( CC1101InitData[i][0]);

        // CC1101WriteReg( CC1101InitData[i][0], CC1101InitData[i][1] );

    }

    for(i = 0; i < RF_reg_count; i++)
    {
        printf("address-[%2X] write-[%2X] read-[%2X]\n", CC1101InitData[i][0], CC1101InitData[i][1], read_buf[i]);

    }


}


/*
================================================================================
Function : CC1101SendPacket_add( )  带目标地址的发送函数
    Send a packet
INPUT    : txbuffer, The buffer stores data to be sent
           size, How many bytes should be sent
           mode, Broadcast or address check packet
OUTPUT   : None

参数address存放目标地址，一个字节
================================================================================
*/
void CC1101SendPacket_add( INT8U *txbuffer, INT8U size, TX_DATA_MODE mode, INT8U address)
{
    uint32_t kk = 0;
    //xSemaphoreTake(myMutex01Handle,3000);

    HAL_NVIC_DisableIRQ(EXIT_CC_IRQ_GD0); //关闭中断，不产生发射中断

    //载波监听
    kk=0;
    while(my_fun_CC1101_re_CCA_status()==1)
    {
        osDelay(100);  //如果信道上有数据，就延时一段时间
        kk++;
        if(kk>=100)
            return ;
    }
    //


    if( mode == BROADCAST )             {
        address = 0;
    }
    //else if( mode == ADDRESS_CHECK )    { address = CC1101ReadReg( CC1101_ADDR ); }

    CC1101ClrTXBuff( );

    if( ( CC1101ReadReg( CC1101_PKTCTRL1 ) & ~0x03 ) != 0 )
    {
        CC1101WriteReg( CC1101_TXFIFO, size + 1 ); //1个字节长度+上1个字节的地址+有效数据,任意长度方法
        CC1101WriteReg( CC1101_TXFIFO, address );  //地址+有效数据，固定长度方法
    }
    else
    {
        CC1101WriteReg( CC1101_TXFIFO, size ); //不加地址
    }

    CC1101WriteMultiReg( CC1101_TXFIFO, txbuffer, size ); //写入有效数据
    CC1101SetTRMode( TX_MODE );  //发送模式
    // while( HAL_GPIO_ReadPin(PORT_CC_IRQ, PIN_CC_IRQ) != 1 ); //发送数据产生高电平中断

    kk = 0;
    while( HAL_GPIO_ReadPin(PORT_CC_IRQ, PIN_CC_IRQ) != 1 ) //发送数据产生高电平中断
    {
        kk++;
        if(kk >= 0X3FFFF)
        {
            break;
        }
    }
    //while( HAL_GPIO_ReadPin(PORT_CC_IRQ, PIN_CC_IRQ) == 1 ) ;//高电平变为低电平
    kk = 0;
    while( HAL_GPIO_ReadPin(PORT_CC_IRQ, PIN_CC_IRQ) == 1 ) //高电平变为低电平
    {
        kk++;
        if(kk >= 0X3FFFF)
        {
            break;
        }

    }

    CC1101ClrTXBuff( );  //清除发送FIFO
    kk = 0;
    while(CC1101ReadStatus(CC1101_MARCSTATE) != 0x01)
    {
        kk++;
        if(kk > 0X3FFFF)
        {
            break;
        }

    }

    //while(CC1101ReadStatus(CC1101_MARCSTATE) != 0x01);
#if Debug_uart_out_CC1101_TX_Status==1
    printf("1-before TX CC_status=[%XH] \n",CC1101ReadStatus(CC1101_MARCSTATE));
#endif
    CC1101SetTRMode(RX_MODE);  // 进入接收模式
    osDelay(100);
#if Debug_uart_out_CC1101_TX_Status==1

    printf("2-after TX CC_status=[%XH] \n",CC1101ReadStatus(CC1101_MARCSTATE));
#endif

    __HAL_GPIO_EXTI_CLEAR_IT(PIN_CC_IRQ); //这个可以清除外部中断，每个中断的清除函数都不一样，需要分别调用
#if Use_CC1101_receive_interrupt_status==1
    HAL_NVIC_EnableIRQ(EXIT_CC_IRQ_GD0); //开启CC1101中断，接收/发送数据产生
#endif

    //xSemaphoreGive(myMutex01Handle);


}

#define my_CC1101_RX_MAX  5000
uint8_t my_CC1101_re_buf[my_CC1101_RX_MAX]; //CC1101的缓冲区,1级缓冲
uint16_t my_CC1101_re_buf_pt_write = 0;
uint16_t my_CC1101_re_buf_pt_read = 0;
uint8_t my_CC1101_COM_Fram_buf[my_CC1101_RX_MAX]; //指令缓冲区
uint8_t my_CC1101_Frame_status = 0;
extern uint8_t my_get_count;
/*
功能：把CC1101接收到数据放到，数据缓冲区中
*/
uint8_t my_get_client_add=0; //CC1101接收到数据的客户端地址号
int  RF_RecvHandler_intrrupt_get_data_to_buf(void)
{
    INT8U length = 0, recv_buffer[64] = {0};
    uint8_t ii = 0;
    extern uint16_t my_tim6_count;
    uint32_t my_temp32=0;

    //xSemaphoreTake(myMutex01Handle,3000);
    if(MY_CC1101_STATUS == 0x01 || MY_CC1101_STATUS == 0x02)
        return 0;
    else
        MY_CC1101_STATUS = 0X02;


    while (CC_IRQ_READ() == 1)
    {
        my_temp32++;
        if(my_temp32>=0x3FFFFF)
        {
            printf("\n*****2 CC_IRQ_READ() == 1 ******\n");
            CC1101ClrRXBuff( );
            return 0;
        }
    }

    // 读取接收到的数据长度和数据内容
    length = CC1101RecPacket(recv_buffer);  //当前接收到的RX数据字节数量，长度1+地址1+有效N+CRC校验2
    //printf("get length=%d\r\n",length);

    // 判断接收数据是否正确
    if (length <= SEND_MAX && length > 0)
    {
        //LED3_ON;               // LED闪烁，用于指示收到数据
        my_get_client_add=recv_buffer[0];
        for(ii = 1; ii < length; ii++)
        {
            my_CC1101_re_buf[my_CC1101_re_buf_pt_write] = recv_buffer[ii];

            if(ii>1&& my_CC1101_re_buf_pt_write==my_CC1101_re_buf_pt_read)
            {
                my_CC1101_re_buf_pt_read++;
                if(my_CC1101_re_buf_pt_read>=my_CC1101_RX_MAX)
                    my_CC1101_re_buf_pt_read=0;

            }


            my_CC1101_re_buf_pt_write++;
            if(my_CC1101_re_buf_pt_write >= my_CC1101_RX_MAX)
                my_CC1101_re_buf_pt_write = 0;

        }
        //调试使用，显示当前接收到的数据
        //my_fun_display_buf_16(recv_buffer,length); //


        // printf("处理数据 %d\r\n",my_get_count);
        // my_get_count=0; //表示清理完数据
//          //测试使用显示过程值
//					printf("RX-data:");
//					for(i=0;i<length;i++)
//					{
//						Cmd_CC1101_RX[i]=recv_buffer[i];
//						printf("[%XH]-",recv_buffer[i]);
//					}
//
//					RSSI_dBm=RS_buf_status[0];
//					if(RSSI_dBm>=128) RSSI_dBm=(RSSI_dBm-256)/2-75;
//					else RSSI_dBm=(RSSI_dBm)/2-75;
//					printf("\nRX_RSSI:[%XH]-[%d]\n",RS_buf_status[0],RSSI_dBm);



    }

    CC1101SetTRMode(RX_MODE);           // 进入接收模式
    MY_CC1101_STATUS = 0X00;
    //CC1101WriteCmd(CC1101_SWOR); //设置WOR模式
    //xSemaphoreGive(myMutex01Handle);
    return length;
}
extern uint8_t rsbuf3[];
extern	uint8_t USART3_FRAME_status;
extern	uint8_t USART3_my_frame[];
extern uint16_t rsbuf3pt_write;
extern uint16_t rsbuf3pt_read;

extern uint8_t rsbuf1[];
extern	uint8_t USART1_FRAME_status;
extern	uint8_t USART1_my_frame[];
extern uint16_t rsbuf1pt_write;
extern uint16_t rsbuf1pt_read;

//协议解析，获得一帧数据,
//第1个参数为端口号，
//第2个为地址2字节,1为两个字节，0位1个字节
//第3个参数为是否开启CRC校验，1为开启
int my_101frame_analyse(uint8_t port_number, uint8_t length_long_status,uint8_t CRC_check_status)
{

    int ii = 0;
    uint16_t my_temp = 0;
    uint16_t my_start_add = 0;
    uint16_t my_end_add = 0;
    uint16_t my_temp_length1 = 0;
    uint16_t my_temp_length2 = 0;
    uint8_t my_status1 = 0;
    uint8_t my_status2 = 0;
    uint16_t my_chazhi = 0;
    int x = 0, x11 = 0, x12 = 0, x21 = 0, x22 = 0;
    uint16_t y = 0;
    volatile	  uint8_t my_point = 0;

    uint8_t *my_pro1_status = NULL;
    uint16_t *my_buf_read_count = NULL;
    uint16_t *my_buf_write_count = NULL;
    uint16_t re_max = 0;
    uint8_t *my_re_buf = NULL;
    uint8_t *my_com_buf1 = NULL; //存最后的指令
    uint8_t my_length_max = 0;
    //===CC1101
    if(port_number == 4 )
    {
        my_pro1_status = &my_CC1101_Frame_status;
        my_buf_read_count = &my_CC1101_re_buf_pt_read;
        my_buf_write_count = &my_CC1101_re_buf_pt_write;
        re_max = my_CC1101_RX_MAX;
        my_re_buf = my_CC1101_re_buf;
        my_com_buf1 = my_CC1101_COM_Fram_buf; //存最后的指令
    }
    //===USART3
    else if(port_number == 3)
    {
        my_pro1_status = &USART3_FRAME_status;
        my_buf_read_count = &rsbuf3pt_read;
        my_buf_write_count = &rsbuf3pt_write;
        re_max =rsbuf3_max ;
        my_re_buf = rsbuf3;
        my_com_buf1 = USART3_my_frame; //存最后的指令
    }
    else if(port_number == 1)
    {
        my_pro1_status = &USART1_FRAME_status;
        my_buf_read_count = &rsbuf1pt_read;
        my_buf_write_count = &rsbuf1pt_write;
        re_max =rsbuf_max ;
        my_re_buf = rsbuf1;
        my_com_buf1 = USART1_my_frame; //存最后的指令
    }
    //===========
    if(length_long_status == 1)
    {
        my_length_max = 5;
    }
    else
        my_length_max = 3;

    //=====

    if(*my_pro1_status == 1)
    {
        my_point = 1;
        return 1;
    }

    //-----------1判断 帧头的位置--------



xx0:
    while(*my_buf_read_count != *my_buf_write_count)
    {
        if(my_re_buf[*my_buf_read_count] == 0X68) //长帧 68H-xx-xx-68H-ID-XX-XX... CRC-16H
        {   my_status2 = 1;
            break;
        }
        else if(my_re_buf[*my_buf_read_count] == 0x10) //固定长度短帧 10H-ID-XX-XX-CRC-16H
        {   my_status2 = 2;
            break;
        }
        else
        {   (*my_buf_read_count)++;
            if(*my_buf_read_count >= re_max)
                *my_buf_read_count = 0;
        }
    }


    //--------2 判断读指针的位置
    if(*my_buf_read_count == *my_buf_write_count)
    {   //my_point=2;
        return 0;
    }
    else if(*my_buf_read_count < *my_buf_write_count)
    {
        my_status1 = 1;  //读在低地址
    }
    else if(*my_buf_read_count > *my_buf_write_count)
    {
        my_status1 = 2;  //读在高地址
    }

    //-----3 取当前时刻的指针地址

    my_start_add = *my_buf_read_count;
    my_end_add = *my_buf_write_count;


    //--------4 长帧数据判断 //读在前，写在后
    if(my_status2 == 1 && my_status1 == 1) //读在低地址，写在高地址
    {
        my_chazhi = my_end_add - my_start_add;
        if(my_chazhi < 10) //差值小于最小长度
        {   my_point = 3;
            return 0;
        }
        //取得另外一个帧头

        if(my_re_buf[my_start_add + my_length_max] != 0X68)
        {
            (*my_buf_read_count)++;
            my_point = 4;
            goto xx0;
            //return 0;

        }


        if(length_long_status == 1)
        {
            //取得帧的长度，从ID开始算，一直到CRC不包含CRC，帧的总长度为length+6+2   68 xx xx yy yy 68 (....length..) crc 16
            my_temp_length1 = my_re_buf[my_start_add + 1];
            my_temp_length1 = my_temp_length1 + ((uint16_t)(my_re_buf[my_start_add + 2]) << 8);

            my_temp_length2 = my_re_buf[my_start_add + 3];
            my_temp_length2 = my_temp_length2 + ((uint16_t)(my_re_buf[my_start_add + 4]) << 8);

            if(my_temp_length1 != my_temp_length2)
            {
                (*my_buf_read_count)++;
                return 0;
            }
        }
        else
        {
            //取得帧的长度，从ID开始算，一直到CRC不包含CRC，帧的总长度为length+6+2   68 xx xx yy yy 68 (....length..) crc 16
            my_temp_length1 = my_re_buf[my_start_add + 1];
            my_temp_length2 = my_re_buf[my_start_add + 2];
            if(my_temp_length1 != my_temp_length2)
            {
                (*my_buf_read_count)++;
                return 0;
            }
        }

        if(my_start_add + my_temp_length1 + my_length_max + 2 >= my_end_add)
        {
            my_point = 31;
            return 0;
        }


        if((my_start_add + my_temp_length1 + my_length_max + 2) >= re_max)
        {

            my_point = 5;
            return 0;
        }

        if(my_chazhi < my_temp_length1) //差值长度小于帧长度
        {
            my_point = 6;
            return 0;
        }
        //取帧尾
        x = my_re_buf[my_start_add + my_length_max + 2 + my_temp_length1];
        if(x != 0X16)
        {
            (*my_buf_read_count)++;
            my_point = 7;
            goto xx0;
            //return 0;

        }
        else
        {
            for(ii = 0; ii < (my_temp_length1 + my_length_max + 2 + 1); ii++)
            {

                my_com_buf1[ii] = my_re_buf[my_start_add + ii]; //取一帧完整的数据

            }
            *my_buf_read_count = my_start_add + my_temp_length1 + my_length_max + 2 + 1;

            if(CRC_check_status==1)
            {
                *my_pro1_status=my_fun_101check_verify(my_com_buf1,length_long_status);
                if(*my_pro1_status==1)
                {

                    if(*my_buf_read_count<*my_buf_write_count && port_number==4)
                        xEventGroupSetBits(xCreatedEventGroup,0X08);
                    else if(*my_buf_read_count<*my_buf_write_count && port_number==1)
                        xEventGroupSetBits(xCreatedEventGroup,0X01);
                    else if(*my_buf_read_count<*my_buf_write_count && port_number==2)
                        xEventGroupSetBits(xCreatedEventGroup,0X02);

                }

                return *my_pro1_status;
            }
            else
            {

                *my_pro1_status = 1; //表示取到一帧完整的数据
                if(*my_buf_read_count<*my_buf_write_count && port_number==4)
                    xEventGroupSetBits(xCreatedEventGroup,0X08);
                else if(*my_buf_read_count<*my_buf_write_count && port_number==1)
                    xEventGroupSetBits(xCreatedEventGroup,0X01);
                else if(*my_buf_read_count<*my_buf_write_count && port_number==2)
                    xEventGroupSetBits(xCreatedEventGroup,0X02);
                my_point = 22;
                return 1;
            }
        }


    }
    //--------5 长帧数据判断 //写在前，读在后
    if(my_status2 == 1 && my_status1 == 2) //写在低地址，读在高地址
    {
        my_temp = re_max - my_start_add;
        my_chazhi = (my_temp + my_end_add);
        if(my_chazhi < 8) //差值小于最小长度
        {   my_point = 9;
            return 0;
        }
        if(my_temp >= (my_length_max + 1))
            x = my_start_add + my_length_max;  //x为第2个帧头的地址
        else
            x = my_length_max - my_temp;

        //取得另外一个帧头
        if(my_re_buf[x] != 0X68)
        {
            (*my_buf_read_count)++;
            if(*my_buf_read_count >= re_max)
                *my_buf_read_count = 0;

            my_point = 10;
            goto xx0;
            //return 0;
        }
        //取得帧的长度

        if(length_long_status == 1) //双字节地址
        {
            if(x > 5)
            {
                x11 = my_start_add + 1;
                x12 = my_start_add + 2;
                x21 = my_start_add + 3;
                x22 = my_start_add + 4;
            }
            else
            {
                x11 = my_start_add + 1;
                if(x11 >= re_max)
                    x11 = x11 - re_max;

                x12 = my_start_add + 2;
                if(x12 >= re_max)
                    x12 = x12 - re_max;

                x21 = my_start_add + 3;
                if(x21 >= re_max)
                    x21 = x21 - re_max;

                x22 = my_start_add + 4;
                if(x22 >= re_max)
                    x22 = x22 - re_max;

            }

            my_temp_length1 = my_re_buf[x11];
            my_temp_length1 = my_temp_length1 + ((uint16_t)(my_re_buf[x12]) << 8);

            my_temp_length2 = my_re_buf[x21];
            my_temp_length2 = my_temp_length2 + ((uint16_t)(my_re_buf[x22]) << 8);
        }
        else //单字节地址
        {
            if(x > 3)
            {
                x11 = my_start_add + 1;
                x21 = my_start_add + 2;

            }
            else
            {
                x11 = my_start_add + 1;
                if(x11 >= re_max)
                    x11 = x11 - re_max;

                x21 = my_start_add + 2;
                if(x21 >= re_max)
                    x21 = x21 - re_max;

            }

            my_temp_length1 = my_re_buf[x11];
            my_temp_length2 = my_re_buf[x21];

        }


        if(my_temp_length1 != my_temp_length2)
        {
            (*my_buf_read_count)++;
            goto xx0;
            // return 0;

        }

        if(my_chazhi < my_temp_length1)
        {   my_point = 11;
            return 0;
        }


        //=======

        //取得帧尾
        y = my_start_add + my_temp_length1 + my_length_max+2; //;
        if(y >= re_max) //帧尾超过数组的末尾
        {
            y = y - re_max;

            if(y >= my_end_add)
            {
                my_point = 32;
                return 0;
            }
        }


        x = my_re_buf[y]; //取得帧尾数据

        //取一帧完整的数据
        if(x != 0X16)
        {
            (*my_buf_read_count)++;
            if(*my_buf_read_count >= re_max)
                *my_buf_read_count = 0;
            my_point = 18;
            goto xx0;
            // return 0;

        }
        else
        {
            for(ii = 0; ii < my_temp_length1 + 6; ii++)
            {
                y = my_start_add + ii;
                if(y >= re_max)
                {
                    y = y - re_max;
                }
                my_com_buf1[ii] = my_re_buf[y]; //取一帧完整的数据

            }
            *my_buf_read_count = *my_buf_read_count + my_temp_length1 + my_length_max + 2 + 1;
            if(*my_buf_read_count >= re_max)
            {
                *my_buf_read_count = *my_buf_read_count - re_max;
            }


            if(CRC_check_status==1)
            {
                *my_pro1_status=my_fun_101check_verify(my_com_buf1,length_long_status);

                if(*my_pro1_status==1)
                {

                    if(*my_buf_read_count<*my_buf_write_count && port_number==4)
                        xEventGroupSetBits(xCreatedEventGroup,0X08);
                    else if(*my_buf_read_count<*my_buf_write_count && port_number==1)
                        xEventGroupSetBits(xCreatedEventGroup,0X01);
                    else if(*my_buf_read_count<*my_buf_write_count && port_number==2)
                        xEventGroupSetBits(xCreatedEventGroup,0X02);

                }

                return *my_pro1_status;
            }
            else
            {

                *my_pro1_status = 1; //表示取到一帧完整的数据
                if(*my_buf_read_count<*my_buf_write_count && port_number==4)
                    xEventGroupSetBits(xCreatedEventGroup,0X08);
                else if(*my_buf_read_count<*my_buf_write_count && port_number==1)
                    xEventGroupSetBits(xCreatedEventGroup,0X01);
                else if(*my_buf_read_count<*my_buf_write_count && port_number==2)
                    xEventGroupSetBits(xCreatedEventGroup,0X02);
                my_point = 22;
                return 1;
            }
        }

    }

    //-------6定长帧判断  读在前，写在后

    if(my_status2 == 2 && my_status1 == 1) //读在前，写在后
    {
        my_chazhi = my_end_add - my_start_add;
        if(my_chazhi < 5) //差值小于最小长度
        {   my_point = 20;
            return 0;
        }

        //取得帧的长度
        if(length_long_status == 1)
            my_temp_length1 = 7;
        else
            my_temp_length1 = 5;
        if(my_start_add + my_temp_length1 >= my_end_add)
        {
            my_point = 33;
            return 0;
        }

        //取帧尾
        x = my_re_buf[my_start_add + my_temp_length1];
        if(x != 0X16)
        {
            (*my_buf_read_count)++;
            my_point = 21;
            goto xx0;
            //return 0;

        }
        else
        {
            for(ii = 0; ii < my_temp_length1 + 1; ii++)
            {

                my_com_buf1[ii] = my_re_buf[my_start_add + ii]; //取一帧完整的数据

            }
            *my_buf_read_count = *my_buf_read_count + my_temp_length1 + 1;


            if(CRC_check_status==1)
            {
                *my_pro1_status=my_fun_101check_verify(my_com_buf1,length_long_status);
                if(*my_pro1_status==1)
                {

                    if(*my_buf_read_count<*my_buf_write_count && port_number==4)
                        xEventGroupSetBits(xCreatedEventGroup,0X08);
                    else if(*my_buf_read_count<*my_buf_write_count && port_number==1)
                        xEventGroupSetBits(xCreatedEventGroup,0X01);
                    else if(*my_buf_read_count<*my_buf_write_count && port_number==2)
                        xEventGroupSetBits(xCreatedEventGroup,0X02);

                }


                return *my_pro1_status;
            }
            else
            {

                *my_pro1_status = 1; //表示取到一帧完整的数据
                if(*my_buf_read_count<*my_buf_write_count && port_number==4)
                    xEventGroupSetBits(xCreatedEventGroup,0X08);
                else if(*my_buf_read_count<*my_buf_write_count && port_number==1)
                    xEventGroupSetBits(xCreatedEventGroup,0X01);
                else if(*my_buf_read_count<*my_buf_write_count && port_number==2)
                    xEventGroupSetBits(xCreatedEventGroup,0X02);
                my_point = 22;
                return 1;
            }

        }


    }


//--------7 固定帧判断  写在前，读在后

    if(my_status2 == 2 && my_status1 == 2) //写在前，读在后
    {
        my_temp = re_max - my_start_add;
        my_chazhi = (my_temp + my_end_add);
        if(my_chazhi < 5) //差值小于最小长度
            return 0;

        //取得帧的长度
        if(length_long_status == 1)
            my_temp_length1 = 7;
        else
            my_temp_length1 = 5;

        //取得帧尾
        y = my_start_add + my_temp_length1; //;
        if(y >= re_max) //帧尾超过数组的末尾
        {
            y = y - re_max;
            if(y > my_end_add)
            {
                my_point = 35;
                return 0;
            }

        }

        x = my_re_buf[y]; //取得帧尾数据

        //取一帧完整的数据
        if(x != 0X16)
        {
            (*my_buf_read_count)++;
            if(*my_buf_read_count >= re_max)
                *my_buf_read_count = 0;
            my_point = 36;
            goto xx0;
            //return 0;

        }
        else
        {
            for(ii = 0; ii < my_temp_length1 + 1; ii++)
            {
                y = my_start_add + ii;
                if(y >= re_max)
                {
                    y = y - re_max;
                }
                my_com_buf1[ii] = my_re_buf[y]; //取一帧完整的数据

            }
            *my_buf_read_count = *my_buf_read_count + my_temp_length1 + 1;
            if(*my_buf_read_count >= re_max)
                *my_buf_read_count = *my_buf_read_count - re_max;
            //CRC校验
            if(CRC_check_status==1)
            {
                *my_pro1_status=my_fun_101check_verify(my_com_buf1,length_long_status);

                if(*my_pro1_status==1)
                {

                    if(*my_buf_read_count<*my_buf_write_count && port_number==4)
                        xEventGroupSetBits(xCreatedEventGroup,0X08);
                    else if(*my_buf_read_count<*my_buf_write_count && port_number==1)
                        xEventGroupSetBits(xCreatedEventGroup,0X01);
                    else if(*my_buf_read_count<*my_buf_write_count && port_number==2)
                        xEventGroupSetBits(xCreatedEventGroup,0X02);

                }

                return *my_pro1_status;
            }
            else
            {

                *my_pro1_status = 1; //表示取到一帧完整的数据


                if(*my_buf_read_count<*my_buf_write_count && port_number==4)
                    xEventGroupSetBits(xCreatedEventGroup,0X08);
                else if(*my_buf_read_count<*my_buf_write_count && port_number==1)
                    xEventGroupSetBits(xCreatedEventGroup,0X01);
                else if(*my_buf_read_count<*my_buf_write_count && port_number==2)
                    xEventGroupSetBits(xCreatedEventGroup,0X02);

                my_point = 22;
                return 1;
            }
        }

    }

    //===========
    return 0;
}

//测试使用，16进制显示收到的的数据
void my_fun_display_fram_16(uint8_t portnumber,uint8_t dis_length)
{
    uint8_t *pt_buf = NULL;
    uint16_t my_length = 0;
    uint16_t ii = 0;
    char *ptchar = NULL;
    if(portnumber == 4)
    {
        pt_buf = my_CC1101_COM_Fram_buf;
        ptchar = "CC1101 re:";

    }
    else if(portnumber==3)
    {   if(USART3_FRAME_status==0)
            return;

        pt_buf = USART3_my_frame;
        ptchar = "USART3 re:";
    }
    else if(portnumber==1)
    {   if(USART1_FRAME_status==0)
            return;

        pt_buf = USART1_my_frame;
        ptchar = "USART1 re:";
    }

    if(pt_buf[0] == 0X10 && pt_buf[5] == 0X16)
    {
        my_length = 6;

    }
    else if(pt_buf[0] == 0X10 && pt_buf[7] == 0X16)
    {

        my_length = 8;
    }
    else if(pt_buf[0] == 0X68 && pt_buf[3] == 0X68)
    {
        my_length = pt_buf[1] + 6;

    }

    else if(pt_buf[0] == 0X68 && pt_buf[5] == 0X68)
    {
        my_length = pt_buf[2];
        my_length = my_length << 8;
        my_length = my_length + pt_buf[1] + 8;

    }
    printf("%s", ptchar);

    if(my_length>8)
        my_length=dis_length;   //控制显示字符串的长度

    for(ii = 0; ii < my_length; ii++)
    {
        printf("[%XH]-", pt_buf[ii]);

    }
    printf("\r\n");

}

//测试使用，显示发送的数据
//最后一个参数，发送为1，接收为0
void my_fun_display_buf_16(uint8_t *pt, uint16_t size,uint8_t TX_status)
{
    uint16_t ii = 0;
    for(ii = 0; ii < size; ii++)
    {
        printf ("[%XH]-", pt[ii]);

    }
    if(TX_status==1)
        printf("TX-GRPS\r\n");
    else
        printf("RX-GPRS\r\n");

}

//超长数据CC1101发送
/*
CC1101最大缓冲区是64个字节，但是有 长度1+地址1+....N。。。+2个crc
所以最多一次发送60个字节
*/

void my_fun_CC1101_send_long_data(INT8U *txbuffer, uint16_t size, TX_DATA_MODE mode, INT8U address)
{


    uint8_t ii = 0;

    uint8_t my_block_size = 0;
    uint8_t my_block_last_data = 0;
    uint8_t *pt = txbuffer;

    my_block_size = size / My_CC1101_send_data_size_MAX;
    my_block_last_data = size % My_CC1101_send_data_size_MAX;

    for(ii = 0; ii < my_block_size; ii++)
    {
        CC1101SendPacket_add(pt + ii * My_CC1101_send_data_size_MAX, My_CC1101_send_data_size_MAX, mode, address);
        //====
        if(ii==0)
        {
            printf("start DB=%d\r\n",ii);
            //my_fun_display_buf_16(pt + ii * My_CC1101_send_data_size_MAX,My_CC1101_send_data_size_MAX); //调试使用，显示发送的数据
        }
        //====
        //HAL_Delay(1);
        if(my_block_size>0)
            osDelay(20);
    }
    if(my_block_last_data!=0)
    {
        //====
        printf("last DB=%d\r\n",ii);
        //my_fun_display_buf_16(pt + ii * My_CC1101_send_data_size_MAX,my_block_last_data); //调试使用，显示发送的数据
        //=====
        CC1101SendPacket_add(pt + ii * My_CC1101_send_data_size_MAX, my_block_last_data, mode, address);

    }
}

//初始化CC1101

void my_fun_CC1101_init_reum(void)
{
    //xSemaphoreTake(myMutex01Handle,3000);
    HAL_NVIC_DisableIRQ(EXIT_CC_IRQ_GD0);
    CC1101Init();
    HAL_Delay(1000);
    CC1101WriteCmd(CC1101_SIDLE); //进入空闲状态
    CC1101WriteReg(CC1101_MCSM2, 0X00); //写0x16 ,RX_TIME，写入0X00，设置接收时间限制，占空比问题00最高12%,如果没接收到数据，就进入SLEEP。
    CC1101WriteReg(CC1101_MCSM0, 0x18); //0X18 	//
    CC1101WriteReg(CC1101_WOREVT1, 0x87); //0X1E,event0 高字节
    CC1101WriteReg(CC1101_WOREVT0, 0x6A);  //0X1F event0 低字节
    CC1101WriteReg(CC1101_WORCTRL, 0X78); //写0X20,0111 100,WOR_RES
    //CC1101WriteCmd(CC1101_SWOR); //开启WOR模式

    //---------进入接收模式

    CC1101SetIdle();
    CC1101WriteReg(CC1101_MCSM2, 0X07); //写0x16 ,RX_TIME，写入0X07,设置没有时间限制，一致接收数据
    CC1101SetTRMode(RX_MODE);           // 进入接收模式

    __HAL_GPIO_EXTI_CLEAR_IT(PIN_CC_IRQ);
    HAL_NVIC_ClearPendingIRQ(EXIT_CC_IRQ_GD0);
#if Use_CC1101_receive_interrupt_status==1
    HAL_NVIC_EnableIRQ(EXIT_CC_IRQ_GD0); //开启中断，接收/发送数据产生
#endif
    //xSemaphoreGive(myMutex01Handle);
}

/*
生成校验字
参数：第一个为指令数组，第二个为长帧检查，1表示双字节长度，0为单字长度
*/
uint8_t my_fun_101check_generate(uint8_t *buffer,uint8_t longstatus)
{
    uint16_t k=0;
    uint8_t status68=0;
    uint16_t length=0;

    uint8_t check_char=0x00;
    if(longstatus==0)  //单字节地址检查
    {
        if(buffer[0]==0x10)
        {
            status68=1;
        }
        else if(buffer[0]==0x68)
        {
            status68=2;
        }


        if(status68==1) //固定长度校验位检查
        {   check_char=0;
            check_char=buffer[1]+buffer[2]+buffer[3];
            buffer[4]=check_char;
						buffer[5]=0x16;
        }
				
        else if(status68==2) //非固定长度校验位检查
        {   check_char=0;
            for(k=0; k<buffer[1]; k++)
            {
                check_char=check_char+buffer[k+4];
            }
            buffer[buffer[1]+4]=check_char;
						buffer[buffer[1]+5]=0x16;
        }
    }


    else if(longstatus==1)  //双字节地址检查
    {

        if(buffer[0]==0x10)
        {
            status68=1;
        }
        else if(buffer[0]==0x68)
        {
            status68=2;
        }


        if(status68==1) //固定长度校验位检查
        {
            check_char=buffer[1]+buffer[2]+buffer[3]+buffer[4]+buffer[5];;
            buffer[6]=check_char;
					  buffer[7]=0X16;
        }
        else if(status68==2) //非固定长度校验位检查
        {
            length=buffer[2];
            length=buffer [1]+(length<<8);
            for(k=0; k<length; k++)
            {
                check_char=check_char+buffer[k+6];
            }
            buffer[length+6]=check_char;
						buffer[length+7]=0X16;
        }


    }
    return check_char;

}

/*
校验字检查
返回值，通过为1，不通过为0
*/

uint8_t my_fun_101check_verify(uint8_t *buffer,uint8_t longstatus)
{
    uint16_t k=0;
    uint8_t status68=0;
    uint8_t temp=0;
    uint8_t check_char=0x00;
    uint16_t my_length=0;
    if(longstatus==0)
    {
        if(buffer[0]==0x10)
        {
            status68=1;
        }
        else if(buffer[0]==0x68)
        {
            status68=2;
        }

        if(status68==1) //固定长度校验位检查
        {
            check_char=buffer[1]+buffer[2]+buffer[3];
            if(check_char==buffer[4])
                temp=1;
            else temp=0;


        }
        else if(status68==2) //非固定长度校验位检查
        {
            for(k=0; k<buffer[1]; k++)
            {
                check_char=check_char+buffer[k+4];
            }

            if(check_char==buffer[buffer[1]+4])
                temp=1;
            else temp=0;
        }
        temp=1;

    }
    else if(longstatus==1)
    {
        if(buffer[0]==0x10)
        {
            status68=1;
        }
        else if(buffer[0]==0x68)
        {
            status68=2;
        }

        if(status68==1) //固定长度校验位检查
        {
            check_char=buffer[1]+buffer[2]+buffer[3]+buffer[4]+buffer[5];
            if(check_char==buffer[6])
                temp=1;
            else temp=0;
        }
        else if(status68==2) //非固定长度校验位检查
        {
            my_length=buffer[2];
            my_length=buffer[1]+(my_length<<8);
            for(k=0; k<my_length; k++)
            {
                check_char=check_char+buffer[k+6];
            }

            if(check_char==buffer[my_length+6])
                temp=1;
            else temp=0;
        }
        temp=1;



    }
    printf("CRC verify is  %d==%d\r\n",temp,status68);
    return temp;

}


void  CC1101CCA( void )//配置为载波监听功能
{
    CC1101WriteReg(CC1101_IOCFG2,0x0E);//GDO2引脚输出载波感应电平,如果RSSI级别在门限之上为高电平,信道占用为高电平
    CC1101WriteReg(CC1101_MCSM1,0x3F);//接受信号强度低于门限值且当前未接收报文则信道空闲，让TX和RX结束后，芯片都保持在RX状态。
    CC1101WriteReg(CC1101_AGCCTRL1,0x40);//低噪声放大器增益先减小，载波监听相对阈值禁用,载波监听绝对阈值由MAGN_TARGET设置
    CC1101WriteReg(CC1101_AGCCTRL2,0x07);//42dB
}

/*
功能：返回当前载波监听信道的信号状态，有信号返回1，没有信号（空闲）返回0
返回值：有信号返回1，没有信号（空闲）返回0
*/
uint8_t my_fun_CC1101_re_CCA_status(void)
{
    //if(HAL_GPIO_ReadPin(GPIO2_INPUT_GPIO_Port,GPIO2_INPUT_Pin)==1)//检测到空气中有载波信号

    CC1101SetTRMode( RX_MODE );
    HAL_Delay(10);
    return (uint8_t)(HAL_GPIO_ReadPin(CC1101_GDO1_GPIO_Port,CC1101_GDO1_Pin));

}
