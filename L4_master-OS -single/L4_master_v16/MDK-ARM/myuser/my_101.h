#include "my_usart.h"
#include "stm32l4xx_hal.h"
#include "my_gloabal_val.h"
#include "my_wave_rec.h"
#include "my_cc1101.h"








//typedef enum { TX_MODE, RX_MODE } TRMODE;
//typedef enum { BROAD_ALL, BROAD_NO, BROAD_0, BROAD_0AND255 } ADDR_MODE;  //µØÖ·Ä£Ê½
//typedef enum { BROADCAST, ADDRESS_CHECK} TX_DATA_MODE;



void my_fun_101_send_long_data(UART_HandleTypeDef* USARTx,uint8_t control_bye,uint8_t *send_buf,uint16_t send_number,TX_DATA_MODE mode, uint8_t desc_address);
void my_fun_101_send_short_data(UART_HandleTypeDef* USARTx,uint8_t control_bye,uint16_t data,TX_DATA_MODE mode, uint8_t desc_address);

void my_fun_101send_DC_data(UART_HandleTypeDef* USARTx,uint8_t my_status,uint8_t my_contorl_byte);
void my_fun_101send_AC_data(UART_HandleTypeDef* USARTx,uint8_t my_status,uint8_t my_contorl_byte);
void my_fun_101send_AC12T_Cyc_data(UART_HandleTypeDef* USARTx,uint8_t my_status,uint8_t my_contorl_byte);
void my_fun_101send_AC_Rec_data(UART_HandleTypeDef* USARTx,uint8_t my_status,uint8_t my_contorl_byte);

void my_fun_conversation_mcu_TX(UART_HandleTypeDef* USARTx);
void my_fun_conversation_mcu_RX(UART_HandleTypeDef* USARTx);

//void my_fun_protocol_analysis1(void);

uint8_t my_fun_101check_generate(uint8_t *buffer,uint8_t longstatus);
uint8_t my_fun_101check_verify(uint8_t *buffer,uint8_t longstatus);

void my_fun_101send_Alarm_status_data(UART_HandleTypeDef* USARTx,uint8_t my_status, uint8_t my_contorl_byte);
void my_fun_101send_PWR_heart_data(UART_HandleTypeDef* USARTx, uint8_t my_contorl_byte);
void my_fun_101send_PWR_OK_data(UART_HandleTypeDef* USARTx, uint8_t my_contorl_byte);
