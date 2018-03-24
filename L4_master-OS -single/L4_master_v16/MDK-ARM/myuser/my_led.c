#include "my_led.h"

void MX_CC1101_CS_Init(uint8_t use_status_on)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    HAL_GPIO_WritePin(SPI3_NSSI_CSN_GPIO_Port, SPI3_NSSI_CSN_Pin, GPIO_PIN_SET);
    if(use_status_on==1)
    {
        /*Configure GPIO pin : PtPin *///使用片选信号
        GPIO_InitStruct.Pin = SPI3_NSSI_CSN_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(SPI3_NSSI_CSN_GPIO_Port, &GPIO_InitStruct);
        /*Configure GPIO pin : PtPin *///中断输入端
        GPIO_InitStruct.Pin = GDO0_IRQ_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GDO0_IRQ_GPIO_Port, &GPIO_InitStruct);
        /* EXTI interrupt init*/
        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
    else
    {

        GPIO_InitStruct.Pin = SPI3_NSSI_CSN_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(SPI3_NSSI_CSN_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GDO0_IRQ_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GDO0_IRQ_GPIO_Port, &GPIO_InitStruct);
    }

}
//=============

void DisableGPIO(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

//Configure all GPIO port pins in Analog Input mode (floating input trigger OFF)
    GPIO_InitStruct.Pin =GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    //GPIOs Periph clock disable

//	__HAL_RCC_ADC_CLK_DISABLE();
//	__HAL_RCC_DAC1_CLK_DISABLE();
//	__HAL_RCC_TIM6_CLK_DISABLE();
//	__HAL_RCC_TIM7_CLK_DISABLE();
//	__HAL_RCC_DMA1_CLK_DISABLE();
//	__HAL_RCC_DMA2_CLK_DISABLE();
//	__HAL_RCC_SPI3_CLK_DISABLE();
//	__HAL_RCC_USART2_CLK_DISABLE();


}



