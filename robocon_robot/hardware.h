
#ifndef HARDWARE_H
#define HARDWARE_H

#include "stm32f4xx_hal.h"
#include "motor.h"

// ��ң����������ҡ�˺�PPM_buf��

// ��ҡ�˿��ƽ��˺������ƶ�
#define ROCK_L_Y PPM_buf[1] // ROLL 1000-1500-2000//δ֪bug
#define ROCK_L_X PPM_buf[0] // PITCH 1000-1500-2000P

// ��ҡ�˿�����ת
#define ROCK_R_X PPM_buf[3] // YAW  1000-1500-2000
#define ROCK_R_Y PPM_buf[2] // THR  1000-1500-2000

// ��ң���������ļ���PPM_buf��
#define SWA PPM_buf[4] // AUX4 1000~2000//û��
#define SWB PPM_buf[5] // AUX2 1000-1500-2000
#define SWD PPM_buf[7] // AUX1 1000~2000
#define SWC PPM_buf[6] // AUX3 1000-1500-2000

#define Rotary_L PPM_buf[8]

#define ZONE_1 0x01
#define ZONE_2 0x02

static uint16_t PPM_Databuf[10] = {0};

extern uint16_t PPM_buf[10];

extern uint8_t shoot_flag;

extern uint8_t ppm_update_flag;

typedef struct
{
	struct // ң��ԭʼ���ݣ�8ͨ��
	{
		uint16_t roll;	// ��ҡ��
		uint16_t pitch; //
		uint16_t thr;
		uint16_t yaw;
		uint16_t AUX1;
		uint16_t AUX2;
		uint16_t AUX3;
		uint16_t AUX4;
		uint16_t BUX1;
		uint16_t BUX2;
	} Remote;

} Air_Contorl;

typedef struct KEY_Type // ��������
{

	int KEY_armtop;	   // armtop
	int KEY_armbottom; // armbottom
	int KEY_push;	   // push
} KEY_Type;

/* Action���������� */
// ����ȫ����λģ�鶨λ��λ��
typedef struct ACTION_GL_POS
{
	float ANGLE_Z;
	float ANGLE_X;
	float ANGLE_Y;
	float POS_X;
	float POS_Y;
	float W_Z;

	float LAST_POS_X;
	float LAST_POS_Y;
	float LAST_POS_Z;

	float DELTA_POS_X;
	float DELTA_POS_Y;
	float DELTA_POS_Z;

	// ������ʵλ��
	float REAL_X;
	float REAL_Y;
	float REAL_Z;
} ACTION_GL_POS;

/* �����˵���ʵλ�� */
typedef struct ROBOT_CHASSIS_s
{

	float World_V[3];  // Y , X , W
	float Robot_V[3];  // Y , X , W
	float Position[2]; // ��ǰ����
	float Motor_RPM[3];
	float expect_angle;
	float Angle;
} ROBOT_CHASSIS;

// �����˵���ʵλ��
typedef struct ROBOT_REAL_POS
{
	float POS_X;
	float POS_Y;
	float POS_YAW;
	float POS_YAW_RAD;
	int robot_location;
} ROBOT_REAL_POS;

/*��������*/
typedef struct Laser_Data
{
	float Laser_X;
	float Laser_Y;
	unsigned char Error_Flag1;
	unsigned char Error_Flag2;
} Laser_Data;

extern struct ROBOT_REAL_POS ROBOT_REAL_POS_DATA;
extern struct ACTION_GL_POS ACTION_GL_POS_INFO;
extern ROBOT_CHASSIS ROBOT_REAL_POS_INFO;
extern Laser_Data Laser_Real_Data;
extern int BallPoint;
extern uint32_t TIME_ISR_CNT;
extern uint32_t LAST_TIME_ISR_CNT;
extern uint16_t Time_Sys[4];
extern uint16_t Microsecond_Cnt;
extern ROBOT_CHASSIS Robot_Chassis;
void remote_control(void);
void reduce_jitter(void);
void shoot_control(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Update_Action_gl_position(float value[6]);
void Laser_ReadData2(float *Laser_Data);
void Laser_ReadData(float *Laser_Data);
void processData(uint8_t *data, Laser_Data *laser, int uart);
void Adjust_Countrol(void);
void Remote_Process(void);
void ZONE2_Adjust(void);
void action_relocate(void);
#endif
