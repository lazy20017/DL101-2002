#include "stm32f1xx_hal.h"

//常量部分
//********串口部分**************************
#define USART_GPRS &huart1   //定义M35对应的串口号
#define USART_DEBUG &huart5   //定义调试端口
#define USART_GPRS_BUFFER rsbuf1  //缓冲区名字
#define USART_GPRS_BUFFERpt_READ rsbuf1pt_read
#define USART_GPRS_BUFFERpt_WRITE rsbuf1pt_write


#define rsbuf_max 256
#define rsbuf_min	256
#define rsbuf3_max    rsbuf_max//51K,  51*1024=52224  41K=41984

#define  first_status  0  //定义第一次上电状态，1为 第一次上电，刷新内容，0为正常使用

//====EEPROM
//软件 选通命令 用
#define NSS1_LOW()       GPIOA->BRR |= GPIO_PIN_4//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_BIT_RESET)//GPIO_ResetBits(GPIOA, GPIO_Pin_4) 
#define NSS1_HIGH()      GPIOA->BSRR |= GPIO_PIN_4//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_BIT_SET)//GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define NSS2_LOW()       GPIOC->BRR |= GPIO_PIN_4//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_BIT_RESET)//GPIO_ResetBits(GPIOC, GPIO_Pin_4)
#define NSS2_HIGH()      GPIOC->BSRR |= GPIO_PIN_4//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_BIT_SET)//GPIO_SetBits(GPIOC, GPIO_Pin_4)


#define WREN  0x06 /* Set Write Enable Latch */
#define WRDI  0x04 /* Reset Write Enable Latch */
#define RDSR  0x05 /* Read Status Register */
#define WRSR  0x01 /* Write Status Register */
#define READ  0x03 /* Read Data From Memory Array */
#define WRITE 0x02 /* Write Data to Memory Array */




#define eeprom 32

#if eeprom==32
//CAT25256 32K
#define I2C_PageSize           64   //每个页面的字节数量，可调
#define EEPROM_Chip_size 1024*32    //进行EEPROM芯片调整使用，如果为1024*128，也可以
#define EEPROM_Chip_high_pin 0x010000  //使用M2为1024*256 需要A16、A17两个最高地址线，M1为1024*128只需要A16一个最高地址线0x010000#

#elif eeprom==64
//CAT25512  64K
#define I2C_PageSize           128   //每个页面的字节数量，可调
#define EEPROM_Chip_size 1024*64    //进行EEPROM芯片调整使用，如果为1024*128，也可以
#define EEPROM_Chip_high_pin 0x010000  //使用M2为1024*256 需要A16、A17两个最高地址线，M1为1024*128只需要A16一个最高地址线0x010000

#endif


//===STM FLASH
/* 获取缓冲区的长度 */
#define  FLASH_WriteAddress     0x807FB00           // 写在靠后位置，防止破坏程序
#define  FLASH_ReadAddress      FLASH_WriteAddress
#define  FLASH_TESTSIZE         512                 //实际是512*2=1024字节


