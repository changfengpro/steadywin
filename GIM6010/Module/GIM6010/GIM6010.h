#include "bsp_fdcan.h"

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define P_MIN  (-12.5f)
#define P_MAX  12.5f
#define V_MIN  (-45.0f)
#define V_MAX  45.0f
#define T_MIN  (-18.0f)
#define T_MAX   18.0f

#pragma pack(1)

typedef enum
{
    GIM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    GIM_CMD_RESET_MODE = 0xfd,   // 停止
    GIM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    GIM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}GIMMotor_Mode_e;

// 电机回传信息结构体
typedef struct
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} motor_fbpara_t;

#pragma pack()

void GIMMotorSetMode(FDCAN_HandleTypeDef* fdhcan, uint16_t motor_id, GIMMotor_Mode_e cmd);
void mit_ctrl(FDCAN_HandleTypeDef* hfdcan, uint16_t *id, float pos, float vel,float kp, float kd, float tor);
