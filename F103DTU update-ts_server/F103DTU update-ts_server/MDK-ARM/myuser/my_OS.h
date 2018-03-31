#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "my_gprs.h"
#include "my_cc_TX_RX.h"
//#include "my_globle_extern.h"

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



//ģ��ָʾ��ʹ��




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
//void  my_fun_GPRS_TX_start4(void);

void  my_fun_GPRS_TX_OK(void);

void  my_fun_GPRS_TX_CYC1(void);  //���ڷ���ң��
void  my_fun_GPRS_TX_CYC2(void);
void  my_fun_GPRS_TX_CYC3(void);
void  my_fun_GPRS_TX_CYC4(void);

void  my_fun_GPRS_TX_RTC_data(void);
void  my_fun_GPRS_TX_CYC5(void);
//void  my_fun_GPRS_TX_CYC6(void);
//void  my_fun_GPRS_TX_CYC7(void);
//void  my_fun_GPRS_TX_CYC8(void);
void  my_fun_GPRS_TX_RESET(void);

uint8_t my_fun_GPRS_RX_change_parameter(void);
void  my_fun_GPRS_TX_changeparameter(void);

uint8_t my_fun_GPRS_RX_turn_led(void); //�ն˷��ƹ���
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


//================
void  my_fun_GPRS_TX_start1_server(void);
void  my_fun_GPRS_TX_start2_server(void);
void  my_fun_GPRS_TX_start3_server(void);
void  my_fun_GPRS_TX_start4_server(void);

void  my_fun_GPRS_TX_OK_80(void);
void  my_fun_GPRS_TX_Call_0(void);
void  my_fun_GPRS_TX_RTC_data_read(void);
void  my_fun_GPRS_TX_test_data(void);
void  my_fun_GPRS_TX_heart_toserver_data(void);
void  my_fun_GPRS_TX_CYC2_B(void);  //����ң����,���ڣ���ʱ��
void  my_fun_GPRS_TX_CYC3_B(void);  //ң�⣬���ڣ���ʱ��
void  my_fun_GPRS_TX_CYC4_B(void);
void  my_fun_GPRS_TX_ALarm_data_yaoxin(void);
void  my_fun_GPRS_TX_ALarm_data_yaoce(void);
void  my_fun_GPRS_TX_ALarm_data_yaoxin_yaoce(void);
void  my_fun_GPRS_TX_catalog(void);
void  my_fun_GPRS_TX_file_data_1(void);
void  my_fun_GPRS_TX_file_data_2(void);
void  my_fun_GPRS_TX_file_data_3(void);
void my_fun_GPRS_generate_68_fram(uint8_t *my_buf,uint8_t my_length,uint8_t my_FC,uint8_t my_TI,uint8_t my_COT,uint8_t my_VSQ_SQ,uint8_t my_VSQ_1_7);
#define MY_yaoxin_status_OK  0X01    //�������˫����Ϣ������0X01������0x02���ϡ�����ǵ�����Ϣ��00Ϊ������01Ϊ����
#define MY_yaoxin_status_ERROR 0x02


