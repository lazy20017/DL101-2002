/**
  ******************************************************************************
  * 文件名程: bsp_iap.c
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2015-10-04
  * 功    能: IAP底层驱动实现
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  *
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_iap.h"
#include "bsp_stm_flash.h"
#include "my_cc1101.h"
#include "my_usart.h"
#include "bsp_iap.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/



uint64_t ulBuf_Flash_App[256];



/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/


__asm void MSR_MSP ( uint32_t ulAddr )
{
    MSR MSP, r0 			                   //set Main Stack value
    BX r14
}
//跳转到应用程序段
//ulAddr_App:用户代码起始地址.
void IAP_ExecuteApp ( uint32_t ulAddr_App )
{
    pIapFun_TypeDef pJump2App;
    uint32_t my_add=( * ( __IO uint32_t * ) ulAddr_App ) ;

    if ( ( my_add& 0x2FFE0000 ) == 0x20000000 )	  //检查栈顶地址是否合法.
    {
        printf("栈顶正确  %XH\r\n",my_add);
        pJump2App = ( pIapFun_TypeDef ) * ( __IO uint32_t * ) ( ulAddr_App + 4 );	//用户代码区第二个字为程序开始地址(复位地址)
        MSR_MSP( * ( __IO uint32_t * ) ulAddr_App );					                            //初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
        pJump2App ();								                                    	//跳转到APP.
    }
    else
    {
        printf("栈顶错误  %XH\r\n",my_add);
    }
}




/*
函数功能：把指定长度的数据，写入到FLASH中
参数：第一个为，FLASH首地址，第2个为数据换成区，字节为单位，第3个写的字节数
*/

void IAP_Write_App_Bin ( uint32_t ulStartAddr, uint8_t * pBin_DataBuf, uint32_t ulBufLength )
{


    uint16_t us, usCtr=0;
    uint64_t usTemp;
    uint32_t ulAdd_Write = ulStartAddr;                                //当前写入的地址
    uint8_t * pData = pBin_DataBuf;

    for ( us = 0; us < ulBufLength; us += 4 )
    {
        usTemp =  ( uint64_t ) pData[3];
        usTemp=(usTemp<<8)+( uint64_t ) pData[2];
        usTemp=(usTemp<<8)+( uint64_t ) pData[1];
        usTemp=(usTemp<<8)+( uint64_t ) pData[0];

        pData += 4;                                                      //偏移4个字节
        ulBuf_Flash_App [ usCtr ++ ] = usTemp;
        if ( usCtr == 256 )
        {
            usCtr = 0;
            STMFLASH_Write ( ulAdd_Write, ulBuf_Flash_App, 256 );	//写的单位为2个字节，但是给第地址是以1个字节为单位，一次写1个扇区2048个字节
            ulAdd_Write += 2048;                                           //偏移2048  16=2*8.所以要乘以2.
        }
    }
    if ( usCtr )
        STMFLASH_Write ( ulAdd_Write, ulBuf_Flash_App, usCtr );//将最后的一些内容字节写进去.


}



/*

*/
extern uint8_t my_CC1101_COM_Fram_buf[];
uint32_t my_update_length=0;
uint8_t my_fun_write_update_data_to_FLASH(void)
{
    uint8_t temp_status=0;
    uint8_t my_block=0;
    uint16_t my_length=0;

    if(my_CC1101_COM_Fram_buf[0]==0x10)
    {
        temp_status=my_CC1101_COM_Fram_buf[1];
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf,8,0);  //调试使用，显示接收到的数据8个字节
    }
    else if (my_CC1101_COM_Fram_buf[0]==0x68)
    {
        temp_status=my_CC1101_COM_Fram_buf[6];
        my_block=my_CC1101_COM_Fram_buf[7];  //获得块号

        my_length=my_CC1101_COM_Fram_buf[2];
        my_length=(my_length<<8)+my_CC1101_COM_Fram_buf[1]-2; //获得长度

        my_fun_display_buf_16(my_CC1101_COM_Fram_buf,8,0);   //调试使用，显示接收到的数据8个字节
    }

    //------
    if(temp_status==0xF0)
    {
        printf("接收升级数据开始  F0\r\n");
        my_update_length=0;
        return 1;

    }
    else if(temp_status==0xF2)
    {
        if(my_length==STM_SECTOR_SIZE)
        {
            printf("接收到数据包，开入写入FLASH  block=%d   F2\r\n",my_block);
            IAP_Write_App_Bin(APP_START_ADDR2+my_block*STM_SECTOR_SIZE,(my_CC1101_COM_Fram_buf+8),my_length);

            printf("FLASH 写入成功-----block=%d   length=%d    F2\r\n",my_block,my_length);
            my_update_length=my_update_length+my_length;
            return 1;
        }
        else
        {
            printf("接收到的数据库长度错误  block=%d  length=%d\r\n   F4",my_block,my_length);
            return 0;
        }

    }
    else if(temp_status==0xF4)
    {
        printf("接收最后的数据包   F4\r\n");
        return 1;
    }
    else if(temp_status==0xF6)
    {

        if(my_length<=STM_SECTOR_SIZE)
        {
            printf("接收到--最后---数据包，开入写入FLASH  block=%d   F6\r\n",my_block);
            IAP_Write_App_Bin(APP_START_ADDR2+my_block*STM_SECTOR_SIZE,(my_CC1101_COM_Fram_buf+8),my_length);
            printf("FLASH --最后数据包---写入成功-----block=%d   length=%d     F6\r\n",my_block,my_length);
            my_update_length=my_update_length+my_length;
            if(my_update_length==(my_block*STM_SECTOR_SIZE+my_length))
            {
                printf("==写入FLASH数据   OK   data=[%XH]\r\n",my_update_length);
                return 1;
            }
            else
            {
                printf("==写入FLASH数据  ERROR!!   right_data=[%XH], error_data=[%XH\r\n",(my_block*STM_SECTOR_SIZE+my_length),my_update_length);
                return 0;
            }

        }
        else
        {
            printf("接收到的最后的数据长度错误  block=%d  length=%d\r\n   F6",my_block,my_length);

            return 0;
        }



    }
    else if(temp_status==0xF8)
    {
        uint64_t passsword=APP_update_password;
        uint64_t length_L=my_update_length;


        STMFLASH_Write(APP_update_status1_add,&passsword,1);
        STMFLASH_Write(APP_update_status2_add,&passsword,1);

        STMFLASH_Write(APP_update_length1_add,&length_L,1);

        STMFLASH_Write(APP_update_length2_add,&length_L,1);


        printf("升级数据传输结束   F8\r\n");
        return 1;
    }

    return 0;

}

