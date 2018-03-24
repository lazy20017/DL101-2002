#include "bsp_spi_flash.h"
#include "stm32f1xx_hal.h"
#include "my_usart.h"
#include "my_def_value.h"
#include "my_extrn_value.h"



#define Dummy_Byte                0xFF






/*********************************************END OF FILE**********************/

void NSS_CS_ENABLE(uint8_t NSS)
{
    if(NSS==1)      {
        NSS1_LOW();
        NSS2_HIGH();
    }
    else if(NSS==2) {
        NSS2_LOW();
        NSS1_HIGH();
    }
}
void NSS_CS_DISABLE()
{
    NSS1_HIGH();
    NSS2_HIGH();
}

void SPI_WREN(uint8_t nss)
{
    NSS_CS_ENABLE(nss);

    SPI_WriteByte(WREN);
    NSS_CS_DISABLE();
    // Delay_ms(1);

}
void SPI_WRDI(uint8_t nss)
{
    NSS_CS_ENABLE(nss);
    SPI_WriteByte(WRDI);
    NSS_CS_DISABLE();
    //  Delay_ms(1);
}

void SPI_EEPROM_WRITE_Start(void)
{

    SPI_WREN(1);
    NSS_CS_ENABLE(1);
    SPI_WriteByte(WRSR);
    SPI_WriteByte(0X02);
    NSS_CS_DISABLE( );

    SPI_WREN(2);
    NSS_CS_ENABLE(2);
    SPI_WriteByte(WRSR);
    SPI_WriteByte(0X02);
    NSS_CS_DISABLE( );



}

void SPI_EEPROM_WRITE_END(void)
{
    SPI_WRDI(1);
    SPI_WRDI(2);

}


uint8_t SPI_WriteByte(uint8_t data)
{

    uint8_t re_data=0;


    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX); // 等待数据传输完成
    HAL_SPI_TransmitReceive(&hspi1,&data,&re_data,1,1000);


    return re_data;


}

uint8_t SPI_ReadByte(void)
{
    return (SPI_WriteByte(Dummy_Byte));

    //while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET);
// return SPI_I2S_ReceiveData(SPI1);


}

uint8_t AT25_GetStatus(uint8_t NSS)
{
    uint8_t tmp=0;
    SPI_WREN(NSS);
    NSS_CS_ENABLE(NSS);
    SPI_WriteByte(RDSR);
    tmp = SPI_ReadByte();
    NSS_CS_DISABLE();

    return tmp;


}


void AT25_WaitReady(uint8_t NSS)
{
    while((0x01 & AT25_GetStatus(NSS))!=0);
}


/*
单字节操作，读和写
*/



uint8_t AT25_ReadByte(uint32_t addr)
{
    uint8_t tmp;

    uint8_t nss=1;

    if(addr >=(uint32_t)(EEPROM_Chip_size*2))
        return 0;

    if(addr>=(uint32_t)(EEPROM_Chip_size))
    {
        addr=addr-(uint32_t)(EEPROM_Chip_size);
        nss=2;
    }
    else nss=1;

    AT25_WaitReady(nss);
    SPI_WREN(nss);
    NSS_CS_ENABLE(nss);
    SPI_WriteByte(READ);
    if(I2C_PageSize>=256)  //启用M1，M2
        SPI_WriteByte((uint8_t)((addr & EEPROM_Chip_high_pin)>>16));/* A16,A17*/
    SPI_WriteByte((uint8_t)((addr & 0x00FF00)>>8));/* A15-A8*/
    SPI_WriteByte((uint8_t)(addr & 0x0000FF));/* A7-A0*/
    tmp = SPI_ReadByte();
    NSS_CS_DISABLE( );

    return tmp;
}


void AT25_WriteByte(uint8_t data, uint32_t addr)
{

    uint8_t nss=1;
    uint32_t tt=EEPROM_Chip_size*2;

    if(addr >= tt)
        return ;

    if(addr>=(uint32_t)(EEPROM_Chip_size))
    {   addr=addr-(uint32_t)(EEPROM_Chip_size);
        nss=2;
    }

    else nss=1;


    AT25_WaitReady(nss);
    SPI_WREN(nss);
    NSS_CS_ENABLE(nss);
    SPI_WriteByte(WRITE);
    if(I2C_PageSize>=256)  //启用M1，M2
        SPI_WriteByte((uint8_t)((addr & EEPROM_Chip_high_pin)>>16));/* A16,a17*/
    SPI_WriteByte((uint8_t)((addr & 0x00FF00)>>8));/* A15-A8*/
    SPI_WriteByte((uint8_t)(addr & 0x0000FF));/* A7-A0*/
    SPI_WriteByte(data);
    NSS_CS_DISABLE( );


}






////////////多字节操作,修改源程序的错误
/**
  * @brief   将缓冲区中的数据写到I2C EEPROM中
  * @param
  *		@arg pBuffer:缓冲区指针
  *		@arg WriteAddr:写地址
  *     @arg NumByteToWrite:写的字节数
  * @retval  无   程序更新，修正BUG
  */
void SPI_EE_BufferWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite,uint8_t NSS)
{



    u32 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0;

    __disable_irq(); //关闭中断
    Addr = WriteAddr % I2C_PageSize;  //PAGE 256
    count = I2C_PageSize - Addr;
    NumOfPage =  NumByteToWrite / I2C_PageSize;
    NumOfSingle = NumByteToWrite % I2C_PageSize;

    /* If WriteAddr is I2C_PageSize aligned  */
    if(Addr == 0)
    {
        /* If NumByteToWrite < I2C_PageSize */
        if(NumOfPage == 0)
        {
            SPI_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle,NSS);

        }
        /* If NumByteToWrite > I2C_PageSize */
        else
        {
            while(NumOfPage--)
            {
                SPI_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize,NSS);

                WriteAddr +=  I2C_PageSize;
                pBuffer += I2C_PageSize;
            }

            if(NumOfSingle!=0)
            {
                SPI_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle,NSS);

            }
        }
    }
    /* If WriteAddr is not I2C_PageSize aligned  */
    else
    {
        /* If NumByteToWrite < I2C_PageSize */
        if(NumOfPage== 0)
        {
            //NumByteToWrite -= count;
            if(NumByteToWrite<=count)
            {
                count=0;
            }
            else if(NumByteToWrite>count)
            {
                NumByteToWrite -= count;
            }
            NumOfPage =  NumByteToWrite / I2C_PageSize;
            NumOfSingle = NumByteToWrite % I2C_PageSize;
            if(count > 0)
            {
                SPI_EE_PageWrite(pBuffer, WriteAddr, count,NSS);

                WriteAddr += count;
                pBuffer += count;
            }
            while(NumOfPage--)  // 此循环实际不执行
            {
                SPI_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize,NSS);

                WriteAddr +=  I2C_PageSize;
                pBuffer += I2C_PageSize;
            }
            if(NumOfSingle > 0)
            {
                SPI_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle,NSS);

            }
            //------------------
            //SPI_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle,NSS);

        }
        /* If NumByteToWrite > I2C_PageSize */
        else
        {
            NumByteToWrite -= count;
            NumOfPage =  NumByteToWrite / I2C_PageSize;
            NumOfSingle = NumByteToWrite % I2C_PageSize;

            if(count != 0)
            {
                SPI_EE_PageWrite(pBuffer, WriteAddr, count,NSS);

                WriteAddr += count;
                pBuffer += count;
            }

            while(NumOfPage--)
            {
                SPI_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize,NSS);

                WriteAddr +=  I2C_PageSize;
                pBuffer += I2C_PageSize;
            }
            if(NumOfSingle != 0)
            {
                SPI_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle,NSS);

            }
        }
    }

    __enable_irq(); //开中断
}



/**
  * @brief   在EEPROM的一个写循环中可以写多个字节，但一次写入的字节数
  *          不能超过EEPROM页的大小，AT24C02每页有8个字节
  * @param
  *		@arg pBuffer:缓冲区指针
  *		@arg WriteAddr:写地址
  *     @arg NumByteToWrite:写的字节数
  * @retval  无
*/
void SPI_EE_PageWrite(u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite,uint8_t NSS)
{

    AT25_WaitReady(NSS);
    SPI_WREN(NSS);
    NSS_CS_ENABLE(NSS);
    SPI_WriteByte(WRITE);
    if(I2C_PageSize>=256)  //启用M1，M2
        SPI_WriteByte((uint8_t)((WriteAddr & EEPROM_Chip_high_pin)>>16));/* A16,A17*/
    SPI_WriteByte((uint8_t)((WriteAddr & 0x00FF00)>>8));/* A15-A8*/
    SPI_WriteByte((uint8_t)(WriteAddr & 0x0000FF));/* A7-A0*/

    /* While there is data to be written */
    while(NumByteToWrite--)
    {
        //Delay_us(1);//测试使用@@@@@@@
        /* Send the current byte */
        SPI_WriteByte(*pBuffer);

        /* Point to the next byte to be written */
        pBuffer++;


    }

    /* Send STOP condition */
    NSS_CS_DISABLE();

}

/**
  * @brief   从EEPROM里面读取一块数据
  * @param
  *		@arg pBuffer:存放从EEPROM读取的数据的缓冲区指针
  *		@arg WriteAddr:接收数据的EEPROM的地址
  *     @arg NumByteToWrite:要从EEPROM读取的字节数
  * @retval  无
  */
void SPI_EE_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead,uint8_t NSS)
{

    uint8_t nss=NSS;

    __disable_irq(); //关闭中断


    AT25_WaitReady(nss);
    SPI_WREN(NSS);
    NSS_CS_ENABLE(nss);
    SPI_WriteByte(READ);
    if(I2C_PageSize>=256)  //启用M1，M2
        SPI_WriteByte((uint8_t)((ReadAddr & EEPROM_Chip_high_pin)>>16));/* A16,A17*/
    SPI_WriteByte((uint8_t)((ReadAddr & 0x00FF00)>>8));/* A15-A8*/
    SPI_WriteByte((uint8_t)(ReadAddr & 0x0000FF));/* A7-A0*/


    /* While there is data to be read */
    while(NumByteToRead)
    {   //Delay_us(1);//测试使用@@@@
        *pBuffer = SPI_ReadByte();
        pBuffer++;
        NumByteToRead--;
    }

    /* Enable Acknowledgement to be ready for another reception */
    NSS_CS_DISABLE( );


    __enable_irq(); //开中断
}

//*****wdz***使用这个函数进行内存的写取* //
//参数1：准备写入的数组首地址
//参数2：写入内存的首地址
//参数3:写自己个数
void SPI_EE_BufferWrite2(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    int32_t temp=WriteAddr+NumByteToWrite-1;
    int32_t number=WriteAddr+NumByteToWrite-EEPROM_Chip_size;

    if(temp<EEPROM_Chip_size)
    {
        SPI_EE_BufferWrite(pBuffer,WriteAddr,NumByteToWrite,1);
    }
    else if(WriteAddr<EEPROM_Chip_size && temp>=EEPROM_Chip_size  )
    {
        SPI_EE_BufferWrite(pBuffer,WriteAddr,NumByteToWrite-number,1);
        SPI_EE_BufferWrite(pBuffer+NumByteToWrite-number,0,number,2);

    }
    else if(WriteAddr>=EEPROM_Chip_size)
    {
        SPI_EE_BufferWrite(pBuffer,WriteAddr-EEPROM_Chip_size,NumByteToWrite,2);
    }

}

//**使用这个函数进行内存的读取****
//
void SPI_EE_BufferRead2(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead)
{
    int32_t temp=ReadAddr+NumByteToRead-1;
    int32_t number=ReadAddr+NumByteToRead-EEPROM_Chip_size;

    if(temp<EEPROM_Chip_size)
    {
        SPI_EE_BufferRead(pBuffer,ReadAddr,NumByteToRead,1);
    }
    else if(ReadAddr<EEPROM_Chip_size && temp>=EEPROM_Chip_size  )
    {
        SPI_EE_BufferRead(pBuffer,ReadAddr,NumByteToRead-number,1);
        SPI_EE_BufferRead(pBuffer+NumByteToRead-number,0,number,2);

    }
    else if(ReadAddr>=EEPROM_Chip_size)
    {
        SPI_EE_BufferRead(pBuffer,ReadAddr-EEPROM_Chip_size,NumByteToRead,2);
    }

}

void SPI_Read_status(void)
{
    uint8_t mygetbyte;


    SPI_WREN(1);
    NSS1_LOW();
    SPI_WriteByte(RDSR);
    mygetbyte=SPI_ReadByte();
    USART_printf(&huart3,"\n ST1=%d",mygetbyte);


    SPI_WREN(2);
    NSS2_LOW();
    SPI_WriteByte(RDSR);
    mygetbyte=SPI_ReadByte();
    USART_printf(&huart3,"\n ST2=%d",mygetbyte);

}
//=============
/*
功能：DTU参数初始化
*/
extern uint8_t my_sys_init_flash;
void my_fun_init_value_flash(void)
{
    uint8_t my_temp8[8]= {0};
    my_temp8[0]=AT25_ReadByte(25);
    my_temp8[1]=AT25_ReadByte(26);
    if(my_temp8[0]==0XAB||my_temp8[0]==0XCD)
    {
        //my_sys_init_flash=1;
    }
    else
    {

       // my_sys_init_flash=0;
    }
    //==========
    if(my_sys_init_flash==1)  //把默认参数写进缓冲区
    {
        //RTC
        AT25_WriteByte(0,0); //2017-10-16 21：09
        AT25_WriteByte(0,1);
        AT25_WriteByte(9,2);
        AT25_WriteByte(21,3);
        AT25_WriteByte(16,4);
        AT25_WriteByte(10,5);
        AT25_WriteByte(17,6);


    }



}

