#ifndef __BSP_IAP_H__
#define	__BSP_IAP_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
//#include "my_gloabal_val.h"

/* 类型定义 ------------------------------------------------------------------*/
/************************** IAP 数据类型定义********************************/
typedef  void ( * pIapFun_TypeDef ) ( void ); //定义一个函数类型的参数.

/* 宏定义 --------------------------------------------------------------------*/
/************************** IAP 宏参数定义********************************/
//是否更新 APP 到 FLASH，否则更新到 RAM
#define User_Flash

#ifdef User_Flash
#define APP_START_ADDR2       	0x8030000  		//应用程序起始地址(存放在FLASH)，bank备份区
#define APP_START_ADDR         	0x8010000  		//应用程序起始地址(存放在FLASH)，执行区
#define APP_START_DATA_ADD 			0X807F000     //数据的起始区域
#endif

/************************** IAP 外部变量********************************/
//#define APP_FLASH_LEN  			 (rsbuf3_max-1024) // 51k-1k
#define APP_update_status1_add  APP_START_DATA_ADD+0X20  //0x807F020   0x807F800
#define APP_update_status2_add  APP_START_DATA_ADD+0X28  //0x807F028

#define APP_update_length1_add 	APP_START_DATA_ADD+0X10		//0x807F010
#define APP_update_length2_add 	APP_START_DATA_ADD+0X18		//0x807F018

#define APP_update_password 0xF1F1



/* 函数声明 ------------------------------------------------------------------*/
void IAP_Write_App_Bin( uint32_t appxaddr, uint8_t * appbuf, uint32_t applen);	//在指定地址开始,写入bin
void IAP_ExecuteApp( uint32_t appxaddr );			                              //执行flash里面的app程序

#endif /* __BSP_IAP_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
void my_fun_update_get_data_to_flash(void);
void my_fun_update_send_data(uint8_t my_port);
//__asm void MSR_MSP ( uint32_t ulAddr );

uint8_t my_fun_write_update_data_to_FLASH(void);
void MX_CC1101_CS_Init(uint8_t use_status_on); //防止CC1101端口漏电，1为片选信号启用，0位不用，设置为输入

