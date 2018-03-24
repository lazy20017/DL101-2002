#include "stm32f1xx_hal.h"
#include "bsp_usart1.h"
//#include "bsp_SysTick.h"
#include <string.h>
#include "bsp_led.h"
#include "wdz_101.h"



uint8_t WDZ_MCUtransmint_commd_wait_commd(uint8_t type,uint8_t *txbuf,uint8_t address_type,uint8_t *rxbuf);
uint8_t WDZ_MCUreceive_testactive(uint8_t Link_control,uint8_t type_identification,uint8_t transmit_reason,uint16_t time);

uint8_t WDZ_MCUtransmit_heartdata(void);
uint8_t WDZ_MCUtransmit_Calldata(void);



