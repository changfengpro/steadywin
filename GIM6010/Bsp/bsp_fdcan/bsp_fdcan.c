#include "bsp_fdcan.h"

void fdcan_user_init()
{
    FDCAN_FilterTypeDef fdcan1_filter;

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    fdcan1_filter.IdType = FDCAN_STANDARD_ID;
    fdcan1_filter.FilterIndex = 0;
    fdcan1_filter.FilterType = FDCAN_FILTER_RANGE;
    fdcan1_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan1_filter.FilterID1 = 0x0000;
    fdcan1_filter.FilterID2 = 0x07ff;

    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan1_filter) != HAL_OK)
        Error_Handler();

    HAL_FDCAN_Start(&hfdcan1);
}

void fdcan_send_message(FDCAN_HandleTypeDef *fdhcan, uint16_t tx_id, uint8_t *tx_data, uint32_t DLC)
{
    // 创建发送数据的头部和帧
    FDCAN_TxHeaderTypeDef txHeader;
    // uint8_t txData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    // 配置 FDCAN 发送消息的头部
    txHeader.Identifier = tx_id;  // 标准模式 CAN ID 为 0
    txHeader.IdType = FDCAN_STANDARD_ID;  // 标准ID
    txHeader.TxFrameType = FDCAN_DATA_FRAME;  // 数据帧
    txHeader.DataLength = DLC;  // 数据长度为 8 字节
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;;  // 错误状态指示符
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;  // 比特率切换
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;  // 设置为经典CAN模式
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS,	// 不需要，禁用事件FIFO
    txHeader.MessageMarker = 0;                     	// 不使用消息标记

    // // 发送消息到 FDCAN 发送队列
    // if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) != HAL_OK)
    // {
    //     // 发送失败时处理
    //     Error_Handler();
    // }

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) 
    {
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, tx_data);
    }

}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
    {
        fdcan1_rx_callback();
    }
}

void FDCAN_Recieve_Message(FDCAN_HandleTypeDef *hfdcan, uint16_t *rx_id, uint8_t *buff)
{
    FDCAN_RxHeaderTypeDef rx_header;
    if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, buff) == HAL_OK)
    {
        
    }
}


/**
************************************************************************
* @brief:      	fdcan1_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	供用户调用的接收弱函数
************************************************************************
**/
__weak void fdcan1_rx_callback(void)
{

}