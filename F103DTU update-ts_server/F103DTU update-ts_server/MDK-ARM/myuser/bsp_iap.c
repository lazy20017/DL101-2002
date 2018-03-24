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
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/



static uint16_t ulBuf_Flash_App[1024];



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


    uint16_t us, usCtr=0, usTemp;
    uint32_t ulAdd_Write = ulStartAddr;                                //当前写入的地址
    uint8_t * pData = pBin_DataBuf;

    for ( us = 0; us < ulBufLength; us += 2 )
    {
        usTemp =  ( uint16_t ) pData[1]<<8;
        usTemp += ( uint16_t ) pData[0];
        pData += 2;                                                      //偏移2个字节
        ulBuf_Flash_App [ usCtr ++ ] = usTemp;
        if ( usCtr == 1024 )
        {
            usCtr = 0;
            STMFLASH_Write ( ulAdd_Write, ulBuf_Flash_App, 1024 );	//写的单位为2个字节，但是给第地址是以1个字节为单位，一次写1个扇区2048个字节
            ulAdd_Write += 2048;                                           //偏移2048  16=2*8.所以要乘以2.
        }
    }
    if ( usCtr )
        STMFLASH_Write ( ulAdd_Write, ulBuf_Flash_App, usCtr );//将最后的一些内容字节写进去.


}









/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

/*

//extern uint8_t USART3_FRAME_status;
//extern uint8_t USART3_my_frame[];
//extern uint8_t rsbuf3[];	 //串口接收缓冲区
//extern uint8_t txbuf3[];
//extern uint16_t rsbuf3pt_write;
//extern uint16_t rsbuf3pt_read;


//uint16_t my_update_get_data_count=0;
//uint8_t my_update_status=0;  //0为没有升级，1为升级成功，2为正在升级

//void my_fun_update_get_data_to_flash(void)
//{
//	uint16_t my_status1=0,my_status2=0;
//	uint32_t my_length1=0,my_length2=0,my_length0=0;
//  //1  接收开始指令
//	//10 F0 01 02 03 16
//	if(USART3_FRAME_status==1&&USART3_my_frame[1]==0XF0) //开始升级
//	{
//		my_update_get_data_count=0;
//		rsbuf3pt_write=0;
//		rsbuf3pt_read=0;
//		printf("update get data started  block=%d\r\n",my_update_get_data_count);
//    my_update_status=2; //正在升级
//		USART3_FRAME_status=0;
//		return;
//	}
//	//2  接收最后一个文件指令
//	//10 F1 01 02 03 16
//	if(USART3_FRAME_status==1&&USART3_my_frame[1]==0XF1) //升级发送最后一个块
//	{
//
//		rsbuf3pt_write=0;
//		rsbuf3pt_read=0;
//		printf("update get data  lasted block=%d  !!\r\n",my_update_get_data_count);


//    my_update_status=3; //接收最后一个文件块
//		USART3_FRAME_status=0;
//		return;
//	}
//
//	 //3  中间接收的过程数据块
//		if(my_update_status==2 && rsbuf3pt_write==APP_FLASH_LEN) //中间块
//	{
//		HAL_Delay(500);
//		if(rsbuf3pt_write>APP_FLASH_LEN)
//		{
//				printf("get data is error=%d!!  \r\n",my_update_get_data_count);
//			  return;
//		}

//		printf("block=%d   write_add=%d\r\n",my_update_get_data_count,rsbuf3pt_write);
//		IAP_Write_App_Bin(APP_START_ADDR2+(my_update_get_data_count*APP_FLASH_LEN),rsbuf3,APP_FLASH_LEN);
//		my_update_get_data_count++;
//		printf("write data to flash block=%d!!\r\n",my_update_get_data_count-1);
//
//		rsbuf3pt_write=0;
//		rsbuf3pt_read=0;
//
//    my_update_status=2; //升级中
//
//		return;
//	}
//	//4  接收的最后一个结尾块
//		if(my_update_status==3 && rsbuf3pt_write<=APP_FLASH_LEN && rsbuf3pt_write>0) //升级结束
//	{
//		uint16_t my_temp=rsbuf3pt_write;
//		HAL_Delay(1000);
//		if(my_temp!=rsbuf3pt_write)
//			return;
//
//		if(rsbuf3pt_write>APP_FLASH_LEN)
//		{
//				printf("get data is error=%d!!  \r\n",my_update_get_data_count);
//			  return;
//		}
//
//		printf("block=%d   write_add=%d\r\n",my_update_get_data_count,rsbuf3pt_write);
//		IAP_Write_App_Bin(APP_START_ADDR2+(my_update_get_data_count*APP_FLASH_LEN),rsbuf3,rsbuf3pt_write);
//		my_update_get_data_count++;
//		printf("write data to flash block=%d!!\r\n",my_update_get_data_count-1);
//		printf("===write data to flash finish!!===\r\n");
//
//		if((rsbuf3pt_write)%2==0)
//		my_length0=(my_update_get_data_count-1)*APP_FLASH_LEN+rsbuf3pt_write;  //获得总的升级代码的字节数量
//		else
//		my_length0=(my_update_get_data_count-1)*APP_FLASH_LEN+rsbuf3pt_write+1;  //获得总的升级代码的字节数量
//
//		rsbuf3pt_write=0;
//		rsbuf3pt_read=0;
//		my_update_get_data_count=0;
//    my_update_status=1; //升级结束
//
//		//5  写密码，和长度（字节为单位）
//		//密码
//		my_status1=STMFLASH_ReadHalfWord(APP_update_status1_add);
//		my_status2=STMFLASH_ReadHalfWord(APP_update_status2_add);
//		printf("\r\n my_status1=%XH,  my_status2=%XH\r\n",my_status1,my_status2);
//		my_status1=APP_update_password; //写的密码
//		STMFLASH_Write(APP_update_status1_add,&my_status1,1);
//		STMFLASH_Write(APP_update_status2_add,&my_status1,1);
//		my_status1=STMFLASH_ReadHalfWord(APP_update_status1_add);
//		my_status2=STMFLASH_ReadHalfWord(APP_update_status2_add);
//		printf("my_status1=%XH,   my_status2=%XH\r\n",my_status1,my_status2);
//		//长度
//		my_length1=STMFLASH_ReadHalfWord(APP_update_length1_add+2);
//		my_length1=(my_length1<<16)+STMFLASH_ReadHalfWord(APP_update_length1_add);
//		my_length2=STMFLASH_ReadHalfWord(APP_update_length2_add+2);
//		my_length2=(my_length2<<16)+STMFLASH_ReadHalfWord(APP_update_length2_add);
//		printf("my_length1=%XH,my_length2=%XH\r\n",my_length1,my_length2);
//
//		my_status1=my_length0;  //写的长度
//		my_status2=my_length0>>16;
//		STMFLASH_Write(APP_update_length1_add,&my_status1,1);
//		STMFLASH_Write(APP_update_length1_add+2,&my_status2,1);
//		STMFLASH_Write(APP_update_length2_add,&my_status1,1);
//		STMFLASH_Write(APP_update_length2_add+2,&my_status2,1);
//
//		my_length1=STMFLASH_ReadHalfWord(APP_update_length1_add+2);
//		my_length1=(my_length1<<16)+STMFLASH_ReadHalfWord(APP_update_length1_add);
//		my_length2=STMFLASH_ReadHalfWord(APP_update_length2_add+2);
//		my_length2=(my_length2<<16)+STMFLASH_ReadHalfWord(APP_update_length2_add);
//		printf("my_length1=%XH,my_length2=%XH\r\n",my_length1,my_length2);
//
//
//		return;
//	}
//
//	if((my_update_status==2||my_update_status==3)&&rsbuf3pt_write>APP_FLASH_LEN)
//	{
//		printf("blok error=%d      write_add=%d\r\n",my_update_get_data_count,rsbuf3pt_write);
//		rsbuf3pt_write=0;
//		rsbuf3pt_read=0;
//
//
//	}
//
//}



//功能：接收到命令后进行升级数据的发送，利用CC1101进行发送。
//发送的过程：已2K字节为一个帧，利用CC1101进行长帧的发送，CC1101接收到1帧后，把此帧数据写入到FLASH中，每收到1帧，确认1帧。
//采用断点续传的方法，每1帧都有1个帧号。


//uint32_t my_update_send_count=0;
//uint32_t my_update_send_length=0;
//	uint16_t my_send_buf[1024];
//	uint8_t my_send_buf2[2048+12];
//extern uint8_t my_cc1101_dest_address;

//extern uint8_t my_CC1101_Frame_status;
//extern uint8_t my_CC1101_COM_Fram_buf[];
//#define my_time_max 10         //秒为单位

//void my_fun_update_send_data(uint8_t my_port)
//{
//
//	uint16_t my_status1=0,my_status2=0;
//	uint32_t my_length1=0,my_length2=0;
//	uint16_t my_block=0;
//	uint16_t my_remain_data=0;
//	uint16_t ii=0,jj=0;
//	uint8_t my_count=0;
//	uint8_t my_CRC_check=0;
//	//===
//	uint8_t *my_fram_status=NULL;
//	uint8_t *pt_my_frame_buf=NULL;

//
//	if(my_port==3)
//	{
//		my_fram_status=&USART3_FRAME_status;
//		pt_my_frame_buf=USART3_my_frame;
//		my_CRC_check=my_UART3_CRC_check;
//	}
//	else if(my_port==4)
//	{
//		my_fram_status=&my_CC1101_Frame_status;
//		pt_my_frame_buf=my_CC1101_COM_Fram_buf;
//		my_CRC_check=CC1101_CRC_check;
//	}
//
//
//	//========0X00=====
//  //1  接收开始指令
//	//10 F2 01 02 03 16
//			if(USART3_FRAME_status==1&&USART3_my_frame[1]==0XF2) //利用CC1101发送升级数据
//			{
//					USART3_FRAME_status=0;
//					 //	if(my_update_status==1)
//				my_update_get_data_count=0;
//				my_update_status=0X10;   //进入发送升级状态
//       printf("接收到升级指令,进入0X10状态\r\n");
//			}
//
//	//======0X10==========
//		if(my_update_status==0X10)  //检查发送，检查状态
//		{
//			my_status1=STMFLASH_ReadHalfWord(APP_update_status1_add);
//			my_status2=STMFLASH_ReadHalfWord(APP_update_status2_add);
//			printf("\r\nmy_status1=%XH,  my_status2=%XH\r\n",my_status1,my_status2);
//
//			my_length1=STMFLASH_ReadHalfWord(APP_update_length1_add+2);
//			my_length1=(my_length1<<16)+STMFLASH_ReadHalfWord(APP_update_length1_add);
//			my_length2=STMFLASH_ReadHalfWord(APP_update_length2_add+2);
//			my_length2=(my_length2<<16)+STMFLASH_ReadHalfWord(APP_update_length2_add);
//			printf("my_length1=%XH,my_length2=%XH\r\n",my_length1,my_length2);
//
//			if(my_status1==my_status2&& my_status1==APP_update_password && my_length1==my_length2 &&my_length1!=0XFFFFFFFF )
//			{
//				my_update_status=0x20;  //检查成功，进入发送状态
//				my_update_send_length=my_length1;   //发送的总的数据长度，字节为单位
//				printf("进入0X20 状态,发送数据包\r\n");
//
//			}
//			else
//			{
//				my_update_status=0x00;
//				printf("FALSH中APP程序有问题，需要重新上传\r\n");
//			}
//		}
//		//========0X20==========
//			if(my_update_status==0X20)
//			{
//				//===
////				for(ii=1;ii<=58;ii++)
////				{
////					my_send_buf2[ii-1]=ii;
////				}
////					CC1101SendPacket_add(my_send_buf2,58,ADDRESS_CHECK,my_cc1101_dest_address);
//				//===
//				//发送开始命令
//				my_send_buf2[0]=0x10;
//				my_send_buf2[1]=0xF0;  //开始指令
//				my_send_buf2[2]=0xFD; //调试器地址
//				my_send_buf2[3]=0x00;
//				my_send_buf2[4]=0x01;//长度
//				my_send_buf2[5]=0x08;
//				my_send_buf2[6]=my_fun_101check_generate(my_send_buf2,1);
//				my_send_buf2[7]=0x16;
//				CC1101SendPacket_add(my_send_buf2,8,ADDRESS_CHECK,my_cc1101_dest_address);
//				printf("CC1101发送开始指令=1\r\n");
//
//
//				my_count=0;
//				while(1)
//				{
//										HAL_Delay(1000);
//										my_101frame_analyse(my_port,1,my_CRC_check);  //检测指令
//
//										if(*my_fram_status==1 && pt_my_frame_buf[1]==0X20) //接收到确认帧
//										{
//
//												*my_fram_status=0; //接收到ok帧
//												my_update_status=0X30;
//												break;
//
//										}
//										else if(*my_fram_status==1 && pt_my_frame_buf[1]!=0X20)
//										{
//											*my_fram_status=0;
//										}
//
//										my_count++;
//										if(my_count>=15)
//										{
//											my_update_status=0x00;
//											my_update_send_count=0;
//											*my_fram_status=0;
//											printf("超时，结束发送=0X20\r\n");
//											break;
//										}
//										if(my_count%5==0)
//										{
//											printf("CC1101发送开始指令--重复=%d\r\n",my_count/5+1);
//											CC1101SendPacket_add(my_send_buf2,8,ADDRESS_CHECK,my_cc1101_dest_address); //重复发送
//										}
//
//
//				}
//			}
//				//==========0X30=====
//				if(my_update_status==0x30)
//				{
//
//									//发送过程数据包
//									my_block=my_update_send_length/STM_SECTOR_SIZE;  //2K为单位进行发送
//									my_remain_data=my_update_send_length%STM_SECTOR_SIZE;
//									ii=my_update_send_count/STM_SECTOR_SIZE;
//
//									my_send_buf2[0]=0X68;
//									my_send_buf2[1]=0x02;
//									my_send_buf2[2]=0X08;
//									my_send_buf2[3]=0x02;
//									my_send_buf2[4]=0X08;
//									my_send_buf2[5]=0x68;
//
//
//									for(;ii<my_block;ii++)
//									{
//										STMFLASH_Read(APP_START_ADDR2+ii*STM_SECTOR_SIZE,my_send_buf,STM_SECTOR_SIZE/2);
//										my_send_buf2[6]=0XF2;
//										my_send_buf2[7]=ii;
//										for(jj=0;jj<STM_SECTOR_SIZE/2;jj++)
//										{
//											my_send_buf2[jj*2+8]=my_send_buf[jj];
//											my_send_buf2[jj*2+1+8]=(my_send_buf[jj]>>8);
//										}
//										my_send_buf2[jj*2+8]=my_fun_101check_generate(my_send_buf2,1);
//										my_send_buf2[jj*2+8+1]=0X16;
//										printf("发送数据包%d --开始--1次\r\n",ii);
//										my_fun_CC1101_send_long_data(my_send_buf2,STM_SECTOR_SIZE+10,ADDRESS_CHECK,my_cc1101_dest_address); //发送块数据
//
//										//重复发送2次
//										my_count=0;
//										while(1)
//										{
//															HAL_Delay(1000);
//															my_101frame_analyse(my_port,1,my_CRC_check);  //检测指令
//															if(*my_fram_status==1 && pt_my_frame_buf[1]==0X20) //接收到确认帧
//															{
//																//if(USART3_my_frame[2]==ii)  //对接收帧进行校验
//																{
//																	my_update_send_count=my_update_send_count+ii*STM_SECTOR_SIZE; //已结发送的字节数据量
//																	*my_fram_status=0;
//																	my_update_status=0X40;
//																	break;
//
//																}
//															}
//															else if(*my_fram_status==1 && pt_my_frame_buf[1]!=0X20)
//															{
//																*my_fram_status=0;
//															}
//
//															my_count++;
//															if(my_count>=my_time_max*3)
//															{
//															my_update_status=0x00;
//															my_update_send_count=0;
//															*my_fram_status=0;
//															printf("超时，结束发送=0X30\r\n");
//
//															break;
//
//															}
//															if(my_count%my_time_max==0)
//															{
//																 	printf("重复发送数据包%d--%d次 \r\n",ii,my_count/my_time_max+1);
//																my_fun_CC1101_send_long_data(my_send_buf2,STM_SECTOR_SIZE+10,ADDRESS_CHECK,my_cc1101_dest_address); //发送块数据

//															}
//
//										}
//										//====while end ====
//										if(my_update_status==0)
//											break;
//									}
//									//====for end=====
//				}
//				//=======0X40========
//				if(my_update_status==0X40)
//				{
//											//发送最后一个数据包指令
//										my_send_buf2[0]=0x10;
//										my_send_buf2[1]=0xF4; //最后一个数据包
//										my_send_buf2[2]=0xFD; //调试器地址
//										my_send_buf2[3]=0x00;
//										my_send_buf2[4]=0x01;//长度
//										my_send_buf2[5]=0x08;
//										my_send_buf2[6]=my_fun_101check_generate(my_send_buf2,1);
//										my_send_buf2[7]=0x16;
//										CC1101SendPacket_add(my_send_buf2,8,ADDRESS_CHECK,my_cc1101_dest_address);
//										printf("CC1101发送-----最后数据包指令----1次-\r\n");
//										my_count=0;
//										while(1)
//										{
//																HAL_Delay(1000);
//																my_101frame_analyse(4,1,my_CRC_check);  //检测指令
//
//																if(*my_fram_status==1 && pt_my_frame_buf[1]==0X20) //接收到确认帧
//																{
//
//																		*my_fram_status=0; //接收到ok帧
//																		my_update_status=0X50;
//																		break;
//
//																}
//																else if(*my_fram_status==1 && pt_my_frame_buf[1]!=0X20)
//																{
//																	*my_fram_status=0;
//																}
//
//																my_count++;
//																if(my_count>=15)
//																{
//																my_update_status=0x00;
//																my_update_send_count=0;
//																*my_fram_status=0;
//																printf("超时，结束发送=0X40\r\n");
//																	break;
//																}
//																if(my_count%5==0)
//																{
//																	printf("CC1101发送---最后数据包指令--重复%d次\r\n",my_count/5+1);
//																	CC1101SendPacket_add(my_send_buf2,8,ADDRESS_CHECK,my_cc1101_dest_address); //重复发送
//																}
//
//
//										}
//			}
//
//
//			//========0X50=============
//

//      	if(my_update_status==0X50)
//				{

//					if(my_remain_data==0)
//							{
//								my_update_status=0x00;
//								my_update_send_count=0;
//								printf ("发送结束，没有剩余包 \r\n");
//								return;
//
//							}
//
//							STMFLASH_Read(APP_START_ADDR2+ii*STM_SECTOR_SIZE,my_send_buf,my_remain_data/2);			//发送剩余的数据
//							my_send_buf2[0]=0X68;
//							my_send_buf2[1]=my_remain_data+2;
//							my_send_buf2[2]=((my_remain_data+2)>>8);
//							my_send_buf2[3]=my_send_buf2[1];
//							my_send_buf2[4]=my_send_buf2[2];
//							my_send_buf2[5]=0x68;
//							my_send_buf2[6]=0XF6;
//							my_send_buf2[7]=ii;
//
//							for(jj=0;jj<my_remain_data/2;jj++)
//							{
//								my_send_buf2[jj*2+8]=my_send_buf[jj];
//								my_send_buf2[jj*2+1+8]=(my_send_buf[jj]>>8);
//							}
//							my_send_buf2[jj*2+8]=my_fun_101check_generate(my_send_buf2,1);
//							my_send_buf2[jj*2+8+1]=0X16;
//							printf("发送最后的数据包--1次\r\n");
//							my_fun_CC1101_send_long_data(my_send_buf2,my_remain_data+10,ADDRESS_CHECK,my_cc1101_dest_address); //发送块数据

//             	//重复发送2次
//					my_count=0;
//					while(1)
//					{
//						HAL_Delay(1000);
//						my_101frame_analyse(my_port,1,my_CRC_check);
//						if(*my_fram_status==1 && pt_my_frame_buf[1]==0X20) //接收到确认帧
//						{
//							//if(USART3_my_frame[2]==ii) //对接收帧进行校验
//							{
//								my_update_send_count=my_update_send_count+my_remain_data; //已结发送的字节数据量
//
//								if(my_update_send_count==my_update_send_length)
//								{
//
//									my_update_status=0X60;
//									*my_fram_status=0;
//									printf("lasted data  is finish=[%XH]\r\n",my_update_send_count);
//									my_update_send_count=0;
//								}
//								else
//								{
//									*my_fram_status=0;
//									my_update_send_count=0;
//									my_update_status=0;
//									printf("lasted data  is error=[%XH]\r\n",my_update_send_count);
//								}
//								break;
//
//							}
//						}
//						else if(*my_fram_status==1 && pt_my_frame_buf[1]!=0X20)
//						{
//							*my_fram_status=0;
//
//						}
//					  my_count++;
//						if(my_count>=3*my_time_max)
//						{
//							my_update_status=0x00;
//							my_update_send_count=0;
//							*my_fram_status=0;
//							printf("超时，结束发送=0X50\r\n");
//							break;
//						}
//						if(my_count%my_time_max==0)
//						{
//								printf("发送最后的数据包--重复发送-%d次\r\n",my_count/my_time_max+1);
//								my_fun_CC1101_send_long_data(my_send_buf2,my_remain_data+10,ADDRESS_CHECK,my_cc1101_dest_address); //发送块数据
//						}
//
//					}
//

//
//				}
//    //========0X60==========
//					if(my_update_status==0X60)
//				{
//											//发送最后一个结束指令
//										my_send_buf2[0]=0x10;
//										my_send_buf2[1]=0xF8; //结束指令
//										my_send_buf2[2]=0xFD; //调试器地址
//										my_send_buf2[3]=0x00;
//										my_send_buf2[4]=0x01;//长度
//										my_send_buf2[5]=0x08;
//										my_send_buf2[6]=my_fun_101check_generate(my_send_buf2,1);
//										my_send_buf2[7]=0x16;
//										CC1101SendPacket_add(my_send_buf2,8,ADDRESS_CHECK,my_cc1101_dest_address);
//										printf("CC1101发送-----结束指令----1次-\r\n");
//										my_count=0;
//										while(1)
//										{
//																HAL_Delay(1000);
//																my_101frame_analyse(my_port,1,my_CRC_check);  //检测指令
//
//																if(*my_fram_status==1 && pt_my_frame_buf [1]==0X20) //接收到确认帧
//																{
//
//																		my_update_status=0x00;
//																		my_update_send_count=0;
//																		*my_fram_status=0;
//																	  printf("发送升级数据结束--正确\r\n");
//																		break;
//
//																}
//																else if(*my_fram_status==1 && pt_my_frame_buf [1]!=0X20)
//																{
//																	*my_fram_status=0;
//																}
//
//																my_count++;
//																if(my_count>=15)
//																{
//																my_update_status=0x00;
//																my_update_send_count=0;
//																*my_fram_status=0;
//																printf("超时，结束发送=0X40\r\n");
//																	break;
//																}
//																if(my_count%5==0)
//																{
//																	printf("CC1101发送---结束指令--重复%d次\r\n",my_count/5+1);
//																	CC1101SendPacket_add(my_send_buf2,8,ADDRESS_CHECK,my_cc1101_dest_address); //重复发送
//																}
//
//
//										}
//			}
//
//		//=====
//}

*/

//======================
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
#if Debug_uart_out_cc1101_rx_data_status==1
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf,8,0);  //调试使用，显示接收到的数据8个字节
#endif
    }
    else if (my_CC1101_COM_Fram_buf[0]==0x68)
    {
        temp_status=my_CC1101_COM_Fram_buf[6];
        my_block=my_CC1101_COM_Fram_buf[7];  //获得块号

        my_length=my_CC1101_COM_Fram_buf[2];
        my_length=(my_length<<8)+my_CC1101_COM_Fram_buf[1]-2; //获得长度
#if Debug_uart_out_cc1101_rx_data_status==1
        my_fun_display_buf_16(my_CC1101_COM_Fram_buf,8,0);   //调试使用，显示接收到的数据8个字节
#endif
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
        uint16_t passsword=APP_update_password;
        uint16_t length_L=my_update_length;
        uint16_t length_H=(my_update_length>>16);

        STMFLASH_Write(APP_update_status1_add,&passsword,1);
        STMFLASH_Write(APP_update_status2_add,&passsword,1);

        STMFLASH_Write(APP_update_length1_add,&length_L,1);
        STMFLASH_Write(APP_update_length1_add+2,&length_H,1);
        STMFLASH_Write(APP_update_length2_add,&length_L,1);
        STMFLASH_Write(APP_update_length2_add+2,&length_H,1);

        printf("升级数据传输结束   F8\r\n");
        return 1;
    }

    return 0;

}

