#include "fdcan.h"

void fdcan_user_init();
void fdcan_send_message(FDCAN_HandleTypeDef *fdhcan, uint16_t tx_id, uint8_t *tx_data, uint32_t DLC);
void fdcan1_rx_callback(void);
void FDCAN_Recieve_Message(FDCAN_HandleTypeDef *hfdcan, uint16_t *rx_id, uint8_t *buff);