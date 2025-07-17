#include "GIM6010_8.h"

GIM6010_Measure_s GIM6010_Measure;

/**
************************************************************************
* @brief:      	GIMMotorSetMode: 设置电机模式函数
* @param[in]:   fdhcan:     指向FDCAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   cmd:    指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
void GIMCANMotorSetMode(FDCAN_HandleTypeDef* fdhcan, uint16_t motor_id, GIMCANMotor_Mode_e cmd)
{
    uint8_t tx_buff;
    tx_buff = (uint8_t)cmd; // 最后一位是命令id
    fdcan_send_message(&hfdcan1, motor_id, &tx_buff, sizeof(tx_buff));
} 

void fdcan1_rx_callback()
{
    uint16_t rx_id;
    uint8_t rx_data[8];
    FDCAN_Recieve_Message(&hfdcan1, &rx_id, rx_data);
    switch (rx_data[0])
    {
         case 0xA3:  // 角度数据命令码
        {
            /* 1. 解析单圈角度（2字节无符号整数） */
            uint16_t single_angle_raw = ((uint16_t)rx_data[2] << 8) | rx_data[1];
            GIM6010_Measure.angle_single_round = single_angle_raw * (360.0f / 16384.0f);
            
            /* 2. 解析多圈角度（4字节有符号整数） */
            int32_t multi_angle_raw = (int32_t)(rx_data[3]       | 
                                              (rx_data[4] << 8)  |
                                              (rx_data[5] << 16) |
                                              (rx_data[6] << 24));
             GIM6010_Measure.multi_angle_deg = multi_angle_raw * (360.0f / 16384.0f);

            break;
        }
    }


    
}