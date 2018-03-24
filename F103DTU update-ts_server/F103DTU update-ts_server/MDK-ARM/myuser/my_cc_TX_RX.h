#include "my_cc1101.h"

void my_cmd_cc1101_tx_WakeCmd(void);
void my_cmd_cc1101_tx(uint8_t *pt_cmd,uint8_t cmd_size,uint8_t tx_channel,uint8_t des_address,uint16_t des_SYNC_word);
void RF_SendPacket_cyc(void);
void my_cmd_cc1101_tx_cyc_data(void);
void my_cmd_cc1101_tx_ALM_single_data(void);
void my_cmd_cc1101_tx_ALM_multi_data(void);
void my_cmd_cc1101_tx_ALM_end_data(void);
void my_cmd_cc1101_tx_OK_fram(void);
void my_cmd_cc1101_tx_config_data(void);
void my_cmd_cc1101_tx_record_data(void);
void my_cmd_cc1101_tx_record_end_data(void);

