
#include "my_spi.h"

SPI_HandleTypeDef SpiHandle;

#define SPI_FLASH_CS_HIGH()		GPIOB->BSRR |= GPIO_PIN_6
#define SPI_FLASH_CS_LOW()		GPIOB->BRR |= GPIO_PIN_6


#define W25_PORT_RCC_EN()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define W25_NSS_PORT		GPIOB
#define W25_NSS_PIN		  GPIO_PIN_6


#define SPIx                             SPI2
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI2_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI2_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI2_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_13
#define SPIx_SCK_GPIO_PORT               GPIOB
#define SPIx_SCK_AF                      GPIO_AF5_SPI1
#define SPIx_MISO_PIN                    GPIO_PIN_14
#define SPIx_MISO_GPIO_PORT              GPIOB
#define SPIx_MISO_AF                     GPIO_AF5_SPI1
#define SPIx_MOSI_PIN                    GPIO_PIN_15
#define SPIx_MOSI_GPIO_PORT              GPIOB
#define SPIx_MOSI_AF                     GPIO_AF5_SPI1


void w25qxx_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;


    // W25_PORT_RCC_EN();
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;


    GPIO_InitStructure.Pin = W25_NSS_PIN;
    HAL_GPIO_Init(W25_NSS_PORT,&GPIO_InitStructure);
    // SPI_FLASH_CS_LOW();
    spi_init();
    // SPI_FLASH_CS_HIGH();
}

//--------------------------------------------------------------------------
//spi_init function,it will enable the spix
//--------------------------------------------------------------------------
void spi_init(void)
{

    /*** Configure the SPI peripheral ***/

    spi2_gpio_config();//config the spi gpio
    HAL_SPI_DeInit(&SpiHandle);
    /* Enable SPI clock */
    SPIx_CLK_ENABLE();
    /*##-1- Configure the SPI peripheral #######################################*/
    /* Set the SPI parameters */
    SpiHandle.Instance               = SPIx;
    SpiHandle.Init.Mode              = SPI_MODE_MASTER;
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLED;


    if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }


    __HAL_SPI_ENABLE(&SpiHandle);
    //spi_write_read(0xFF);
}
void spi2_gpio_config(void)
{
    GPIO_InitTypeDef  gpioinitstruct = {0};

    /*** Configure the GPIOs ***/
    /* Enable GPIO clock */
    SPIx_SCK_GPIO_CLK_ENABLE();
    SPIx_MISO_GPIO_CLK_ENABLE();
    SPIx_MOSI_GPIO_CLK_ENABLE();

    /* Configure SPI SCK */

    gpioinitstruct.Mode       = GPIO_MODE_AF_PP;
    gpioinitstruct.Pull       = GPIO_NOPULL;
    gpioinitstruct.Speed      = GPIO_SPEED_HIGH;

    gpioinitstruct.Pin        = SPIx_SCK_PIN;
    gpioinitstruct.Alternate  = SPIx_SCK_AF;
    HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &gpioinitstruct);

    /* Configure SPI MISO and MOSI */
    gpioinitstruct.Pin        = SPIx_MISO_PIN;
    gpioinitstruct.Alternate  = SPIx_MISO_AF;
    HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &gpioinitstruct);

    gpioinitstruct.Alternate  = SPIx_MOSI_AF;
    gpioinitstruct.Pin        = SPIx_MOSI_PIN;
    HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &gpioinitstruct);


}


/*===========================================================================
* 函数 ：SPI_ExchangeByte() => 通过SPI进行数据交换                          *
* 输入 ：需要写入SPI的值                                                    *
* 输出 ：通过SPI读出的值                                                    *
============================================================================*/
uint8_t SPI_ExchangeByte(uint8_t input)
{
    uint8_t re_data=0;
    //while (RESET == SPI_GetFlagStatus(SPI_FLAG_TXE));   // 等待数据传输完成
    //SPI_SendData(input);
    //while (RESET == SPI_GetFlagStatus(SPI_FLAG_RXNE));  // 等待数据接收完成
    //return (SPI_ReceiveData());

    while(HAL_SPI_GetState(&hspi2)==HAL_SPI_STATE_BUSY_TX); // 等待数据传输完成
    HAL_SPI_TransmitReceive(&hspi2,&input,&re_data,1,1000);


    return re_data;

}
