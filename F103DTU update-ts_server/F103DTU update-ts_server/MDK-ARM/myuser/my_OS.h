#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "my_gprs.h"
#include "my_cc_TX_RX.h"

void my_fun_set_group(void);
void my_fun_take_group(void);
void my_fun_give_BinarySem(void);
void my_fun_take_BinarySem(void);
void my_fun_give_Queue1(void);
//void my_fun_give_Queue31(void);
//void my_fun_give_Queue3(uint16_t temp_step);


void my_fun_give_Queue(osMessageQId *my_QHL,uint16_t temp_step);
void my_fun_take_Queue(void);
//=========================
void my_fun_CC1101_init_resume(void);
void my_fun_usart_init_resume(void);
void my_fun_task_heap_value(void);

void my_fun_CC1101_test1(void);



//模拟指示器使用




void  my_fun_GPRS_TX_test1(void);
void  my_fun_GPRS_TX_test2(void);
void  my_fun_GPRS_TX_test3(void);
void  my_fun_GPRS_TX_test4(void);
void  my_fun_GPRS_TX_test5(void);

uint8_t my_fun_GPRS_RX_test1(void);
uint8_t my_fun_GPRS_RX_test2(void);
uint8_t my_fun_GPRS_RX_test3(void);
void my_fun_M35_resume_init(void) ;



//===========
uint8_t my_fun_dialog_CC1101_RX_1(void);


//==========================
void my_fun_gprs_time_dialog_rx(
    osMessageQId *QHL_send,
    uint16_t my_get_step,   //
    uint16_t my_before_step,
    uint16_t my_now_step,
    uint16_t my_next_step,
    uint8_t end_status,
    uint8_t (*ptfun)(void)
);

void my_fun_gprs_time_dialog_tx(
    uint16_t my_get_step,
    uint16_t my_before_step,
    uint16_t my_now_step,
    uint8_t end_status,
    void (*ptfun)(void)
);
//=========
void my_fun_CC1101_time_dialog_rx2(
    osMessageQId *QHL_send,
    uint16_t my_get_step,   //
    uint16_t my_before_step,
    uint16_t my_now_step,
    uint16_t my_next_step,
    uint8_t end_status,
    uint8_t (*ptfun)(void)
);
void my_fun_CC1101_time_dialog_tx2(
    uint16_t my_get_step,
    uint16_t my_before_step,
    uint16_t my_now_step,
    uint8_t end_status,
    void (*ptfun)(void)
);
//==============
void my_fun_CC1101_time_dialog_tx3(
    uint16_t my_get_step,
    uint16_t my_before_step,
    uint16_t my_now_step,
    //uint16_t my_next_step,
    uint8_t end_status,
    void (*ptfun)(void)
);



//==================
void my_fun_CC1101_send_heart_data(void);
void  my_fun_GPRS_TX_start1(void);
void  my_fun_GPRS_TX_start2(void);
void  my_fun_GPRS_TX_start3(void);
void  my_fun_GPRS_TX_start4(void);

void  my_fun_GPRS_TX_OK(void);

void  my_fun_GPRS_TX_CYC1(void);  //周期发送遥信
void  my_fun_GPRS_TX_CYC2(void);
void  my_fun_GPRS_TX_CYC3(void);
void  my_fun_GPRS_TX_CYC4(void);

void  my_fun_GPRS_TX_RTC_data(void);
void  my_fun_GPRS_TX_CYC5(void);
void  my_fun_GPRS_TX_CYC6(void);
void  my_fun_GPRS_TX_CYC7(void);
void  my_fun_GPRS_TX_CYC8(void);
void  my_fun_GPRS_TX_RESET(void);

uint8_t my_fun_GPRS_RX_change_parameter(void);
void  my_fun_GPRS_TX_changeparameter(void);

uint8_t my_fun_GPRS_RX_turn_led(void); //终端翻牌功能
void  my_fun_GPRS_TX_TurnLED(void);
uint8_t my_fun_GPRS_RX_query_data(void);
void  my_fun_GPRS_TX_query_data(void);
uint8_t my_fun_GPRS_RX_query_data2(void);
void  my_fun_GPRS_TX_query_data2(void);
void  my_fun_GPRS_TX_xinhaoqiangdu(void);
void  my_fun_GPRS_TX_TIME_RTC(void);
void  my_fun_GPRS_TX_ALarm_data(void);
void  my_fun_GPRS_TX_rec_data(void);
uint8_t my_fun_GPRS_RX_Rec_data(void);

void my_fun_GPRS_101_genert_record_data(uint8_t *txbu);
uint8_t my_fun_dialog_CC1101_RX_heart(void);
void my_fun_CC1101_TX_OK(void);
void my_fun_display_ZSQ_data(void);
uint8_t my_fun_dialog_CC1101_RX_0(void);
void my_fun_CC1101_TX_config_parmeter(void);
