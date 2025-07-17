#include "bsp_fdcan.h"


#pragma pack(1)

typedef enum
{
    GIMCAN_CMD_RESET_MODE = 0x00, //重启主机
    GIMCAN_CMD_READ_RPM_MODE = 0xA2, // 读取实时旋转速度
    GIMCAN_CMD_READ_ANGLE_MODE =  0xA3, // 读取实时角度
} GIMCANMotor_Mode_e;

void GIMCANMotorSetMode(FDCAN_HandleTypeDef* fdhcan, uint16_t motor_id, GIMCANMotor_Mode_e cmd);

typedef struct 
{
   float angle_single_round; // 单圈角度
   float multi_angle_deg; // 多圈角度
} GIM6010_Measure_s;


#pragma pack()
