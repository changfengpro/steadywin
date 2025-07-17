#include "GIM6010.h"

motor_fbpara_t motor_t;

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

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
void GIMMotorSetMode(FDCAN_HandleTypeDef* fdhcan, uint16_t motor_id, GIMMotor_Mode_e cmd)
{
    uint8_t tx_buff[8];
    memset(tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    fdcan_send_message(&hfdcan1, motor_id, tx_buff, sizeof(tx_buff));
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(FDCAN_HandleTypeDef* hfdcan, uint16_t *id, float pos, float vel,float kp, float kd, float tor)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;

	pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(vel, V_MIN, V_MAX, 12);
	tor_tmp = float_to_uint(tor, T_MIN, T_MAX, 12);
	kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	fdcan_send_message(hfdcan, id, data, sizeof(data));
}

// /**
// ************************************************************************
// * @brief:      	dim_motor_fbdata: 获取DIM6010电机反馈数据函数
// * @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
// * @param[in]:   rx_data:  指向包含反馈数据的数组指针
// * @retval:     	void
// * @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
// *               状态、位置、速度、扭矩以及相关温度参数
// ************************************************************************
// **/
// void dim_motor_fbdata(FDCAN_HandleTypeDef* hfdcan, motor_fbpara_t motor_t, uint8_t *rx_data)
// {
// 	motor_t.id = (rx_data[0])&0x0F;
// 	motor_t.state = (rx_data[0])>>4;
// 	motor_t.p_int=(rx_data[1]<<8)|rx_data[2];
// 	motor_t.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
// 	motor_t.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
// 	motor_t.pos = uint_to_float(motor_t.p_int, P_MIN, motor->tmp.PMAX, 16); // (-12.5,12.5)
// 	motor_t.vel = uint_to_float(motor_t.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12); // (-45.0,45.0)
// 	motor_t.tor = uint_to_float(motor_t.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12); // (-18.0,18.0)
// 	motor_t.Tmos = (float)(rx_data[6]);
// 	motor_t.Tcoil = (float)(rx_data[7]);
// }