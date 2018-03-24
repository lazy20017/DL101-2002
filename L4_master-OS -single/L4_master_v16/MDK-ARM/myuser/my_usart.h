
#include "usart.h"


void USART_printf(UART_HandleTypeDef* USARTx, uint8_t *Data,...);

void MX_USART3_AS_UART_Init_self(void);
void MX_USART3_AS_GPIO_Init_self(void);
void USART_printf1(UART_HandleTypeDef* USARTx, uint8_t *Data,double my_data);
