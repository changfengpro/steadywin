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

/**
************************************************************************
* @brief      GIMCANMotorSetCurrent: 设置电机Q轴电流
* @param[in]  fdhcan:     FDCAN句柄指针
* @param[in]  motor_id:   电机ID (设备地址或0x100|设备地址)
* @param[in]  current_A:  目标电流值(单位：A)
* @retval     void
* @details    通过CAN总线向电机发送Q轴电流控制命令
************************************************************************
**/
void GIMCANMotorSetCurrent(FDCAN_HandleTypeDef* fdhcan, uint16_t motor_id, float current_A)
{
    uint8_t tx_data[5];
    
    // 命令码
    tx_data[0] = 0xC0;  // Q轴电流控制命令码
    
    // 电流值转换：A → 0.001A
    int32_t current_ma = (int32_t)(current_A * 1000.0f);
    
    // 小端序编码（低字节在前）
    tx_data[1] = current_ma & 0xFF;         // 最低字节
    tx_data[2] = (current_ma >> 8) & 0xFF;  // 次低字节
    tx_data[3] = (current_ma >> 16) & 0xFF; // 次高字节
    tx_data[4] = (current_ma >> 24) & 0xFF; // 最高字节
    
    // 发送CAN消息（DLC=5）
    fdcan_send_message(fdhcan, motor_id, tx_data, 5);
}


/**
************************************************************************
* @brief      GIMCANMotorSetAbsPosition: 设置电机绝对位置
* @param[in]  fdhcan:     FDCAN句柄指针
* @param[in]  motor_id:   电机ID (设备地址或0x100|设备地址)
* @param[in]  angle_deg:  目标角度值(单位：度)
* @retval     void
* @details    通过CAN总线向电机发送绝对位置控制命令
*             角度值将转换为电机计数值(Count)，电机一圈(360°)对应16384个计数值
************************************************************************
**/
void GIMCANMotorSetAbsPosition(FDCAN_HandleTypeDef* fdhcan, uint16_t motor_id, float angle_deg)
{
    uint8_t tx_data[5];
    
    // 命令码
    tx_data[0] = 0xC2;  // 绝对位置控制命令码
    
    // 角度值转换：度 → Count (360°=16384 Count)
    // 公式：Count = angle_deg × (16384 / 360.0)
    int32_t position_count = (int32_t)(angle_deg * (16384.0f / 360.0f));
    
    // 小端序编码（低字节在前）
    tx_data[1] = position_count & 0xFF;         // 最低字节
    tx_data[2] = (position_count >> 8) & 0xFF;  // 次低字节
    tx_data[3] = (position_count >> 16) & 0xFF; // 次高字节
    tx_data[4] = (position_count >> 24) & 0xFF; // 最高字节
    
    // 发送CAN消息（DLC=5）
    fdcan_send_message(fdhcan, motor_id, tx_data, 5);
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
        case 0xC0:  // Q轴电流控制响应
        {
            int32_t current_ma = (int32_t)(rx_data[1]   |
                                  (rx_data[2] << 8)     |
                                  (rx_data[3] << 16)    |
                                  (rx_data[4] << 24));
            GIM6010_Measure.current = current_ma * 0.001;
            
            break;
        }
        case 0xA2:  // 速度响应
        {
            int32_t speed_raw = (int32_t)(rx_data[1]        | 
                                (rx_data[2] << 8)  |
                                (rx_data[3] << 16) |
                                (rx_data[4] << 24));
            GIM6010_Measure.speed_raw = speed_raw * 0.01;
        }
    }
}

GIM6010_Measure_s *GIM6010_Measure_Ptr()
{
    return &GIM6010_Measure;
}