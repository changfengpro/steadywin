#include "bsp_fdcan.h"
#include "stdbool.h"

#pragma pack(1)

typedef enum
{
    GIMCAN_CMD_RESET_MODE = 0x00, //重启主机
    GIMCAN_CMD_READ_RPM_MODE = 0xA2, // 读取实时旋转速度
    GIMCAN_CMD_READ_ANGLE_MODE =  0xA3, // 读取实时角度
    GIMCAN_CMD_DISABLE_MODE = 0xCF, // 电机失能
} GIMCANMotor_Mode_e;


typedef struct 
{
   float angle_single_round; // 单圈角度
   float multi_angle_deg; // 多圈角度
   float current; // 电流
   float speed_raw; // 速度 rpm
   float set_angle;
   float set_current;
} GIM6010_Measure_s;


// 系统状态
typedef struct {
    float current_angle;        // 当前实际角度 (rad)
    float target_angle;         // 目标锁定位置 (rad)
    float virtual_angle;        // 虚拟平衡位置 (rad)
    float velocity;             // 当前角速度 (rad/s)
    float acceleration;         // 当前角加速度 (rad/s²)
    float gravity_comp_offset;  // 重力补偿偏移 (rad)
} PositionState;

// 控制状态
typedef struct {
    bool external_force;   // 外力检测标志
    bool position_locked;  // 位置锁定标志
    bool transition_state; // 过渡状态标志
    float settle_timer;    // 稳定计时器
} ControlState;

#pragma pack()


void GIMCANMotorSetMode(FDCAN_HandleTypeDef* fdhcan, uint16_t motor_id, GIMCANMotor_Mode_e cmd);
void GIMCANMotorSetCurrent(FDCAN_HandleTypeDef* fdhcan, uint16_t motor_id, float current_A);
void GIMCANMotorSetAbsPosition(FDCAN_HandleTypeDef* fdhcan, uint16_t motor_id, float angle_deg);
GIM6010_Measure_s *GIM6010_Measure_Ptr();