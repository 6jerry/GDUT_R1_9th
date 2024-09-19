#ifndef __MIT_H_
#define __MIT_H_
#include "stm32f4xx.h"
#include "PID.h"
/*相关参数*/
float fmaxf(float a,float b);//a，b取最大

float fminf(float a,float b);//a，b取最小

int float_to_uint(float x1,float x1_min,float x1_max,int bits);//浮点转整型

float uint_to_float(int x1_int,float x1_min,float x1_max,int bits);//整型转浮点型

#define DM43_ID1 0x01//要使用的ID
#define DM43_ID2 0x02
#define DM43_ID3 0x03
#define DM43_ID4 0x04

//此时是设置电机模式的，当然也可以用枚举
#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03


void MOTOR_Speed_Control(uint16_t ID,float vel);///速度模式控制

//CAN返回的电机真实数据
typedef struct MIT_REAL_INFO
{
	float  ANGLE;   		        //采样角度	(rad)			
	float  V_angle;						  //采样角速度(rad/s	)	
	float  CURRENT;
	float  TARGET_CURRENT;
	float  TARGET_POS;
	float  TARGET_SPEED;
	float  REAL_ANGLE;         //处理过的真实角度（必须用float）
}MIT_REAL_INFO;

extern MIT_REAL_INFO MIT_DRIVER_REAL_INFO[4];


void DM43_control_cmd(uint16_t ID,uint8_t cmd);//模式控制 

void DM43_update_info(CAN_RxHeaderTypeDef *msg,uint8_t can2_RxData[8]);//不断更新数据 

extern void DM43_Init(void);
void ctrl_motor2(uint16_t id, float _pos, float _vel );

//此时是设置电机模式的，当然也可以用枚举
#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

//根据相关参数进行限幅
#define P_MIN   -12.5f//position电机位置
#define P_MAX	 12.5f
#define V_MIN	-500.0f//电机速度
#define V_MAX	 500.0f
#define T_MIN 	-18.0f//电机扭矩
#define T_MAX 	 18.0f
#define Kp_MIN 	 0//Kp范围
#define Kp_MAX   500.0f
#define Kd_MIN 	 0//Kd范围
#define Kd_MAX   5.0f
#define I_MAX  	 18.0f

/*函数AK80_motion_control用**/
#define PITCH_MAX		90.0f
#define PITCH_MIN		-90.0f

//此参数的作用暂时未知
#define MIT_P_MIN						-12.5f
#define MIT_P_MAX						 12.5f
#define MIT_V_MIN						-500.0f
#define MIT_V_MAX						 500.0f
#endif

