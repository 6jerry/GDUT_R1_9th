/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 Keten. All Rights Reserved.
 *
 * \file      motor.c
 * \author    Keten
 * \version   1.0
 * \date      2024年4月29日
 * \brief     m3508电机驱动文件
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history:
	1.date  : 2024年4月29日
	  author: Keten
	  change: create file

*****************************************************************************/
#include "motor.h"
#include "main.h"
#include "pid.h"
#include "can.h"
#include "pid_adjust.h"

MOTO_REAL_INFO MOTOR_REAL_INFO[8] = {0}; // 1-4分别对应顺时针方向的底盘电机
MOTO_REAL_INFO SHOOT_MOTOR_INFO[8] = {0};
pid MOTOR_PID_RPM[8] = {0}; // 速度pid信息 1-4底盘电机 5-6发射电机
pid MOTOR_PID_POS[8] = {0}; // 位置pid信息

MOTOR_RPM MOTOR_TARGET_RPM;
MOTOR_POS MOTOR_TARGET_POS;

PID_Controller motorleft_r, motorright_r, motorfront_r, shoot_down_left, shoot_down_right, shoot_up_left, shoot_up_right, heading_lock;

/*!
 * \fn     m3508_Init
 * \brief  电机结构体成员初始化
 *          初始化电机的同时，也初始化了pid参数，直接在主函数里调用这一个足矣
 *
 * \retval void
 */
void m3508_Init()
{
	/* 电机类型 */
	MOTOR_REAL_INFO[0].type = RM_3508;
	MOTOR_REAL_INFO[1].type = RM_3508;
	MOTOR_REAL_INFO[2].type = RM_3508;

	/* 电机模式 */
	/* 速度模式 */
	MOTOR_REAL_INFO[0].unitMode = SPEED_CONTROL_MODE;
	MOTOR_REAL_INFO[1].unitMode = SPEED_CONTROL_MODE;
	MOTOR_REAL_INFO[2].unitMode = SPEED_CONTROL_MODE;

	/* 位置模式 */
	// MOTOR_REAL_INFO[0].unitMode=POSITION_CONTROL_MODE;
	// MOTOR_REAL_INFO[1].unitMode=POSITION_CONTROL_MODE;
	// MOTOR_REAL_INFO[2].unitMode=POSITION_CONTROL_MODE;
	PID_Init(&motorleft_r, 16.2, 0.17, 2.87, 1000000, 20000, 5, 960);
	PID_Init(&motorright_r, 16.7, 0.21, 2.4, 10000000, 20000, 5, 960);
	PID_Init(&motorfront_r, 16.5, 0.18, 2.6, 1000000, 20000, 5, 960);
	PID_Init(&shoot_down_left, 16.6, 0.25, 1.97, 1000000, 20000, 5, 750);
	PID_Init(&shoot_down_right, 16.5, 0.25, 1.96, 1000000, 20000, 5, 750);
	PID_Init(&shoot_up_left, 16, 0.22, 1.86, 1000000, 20000, 5, 750);
	PID_Init(&shoot_up_right, 16.8, 0.23, 2.1, 1000000, 20000, 5, 750);
	PID_Init(&heading_lock, 7, 0, 0.7, 100000, 5, 0.01f, 0.5f);
	/* pid参数初始化 */
	//	/* 速度模式初始化 */
	PID_parameter_init(&MOTOR_PID_RPM[0], 12.0f, 1.4f, 0.1, 20000, 20000, -0.5);
	PID_parameter_init(&MOTOR_PID_RPM[1], 11.96f, 1.1f, 0.1, 20000, 20000, -0.5);
	PID_parameter_init(&MOTOR_PID_RPM[2], 12.2f, 1.5f, 0.1, 20000, 20000, -0.5);
	//	PID_parameter_init(&MOTOR_PID_RPM[0],12.0f,0.0f,0.0f,20000,20000,-0.5);
	//	PID_parameter_init(&MOTOR_PID_RPM[1],12.0f,0.0f,0.0f,20000,20000,-0.5);
	//	PID_parameter_init(&MOTOR_PID_RPM[2],12.0f,0.0f,0.0f,20000,20000,-0.5);
	/* 位置模式初始化 */
	PID_parameter_init(&MOTOR_PID_POS[0], 0.0f, 0.0f, 0.0f, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[1], 0.0f, 0.0f, 0.0f, 7000, 7000, 0.05);
	PID_parameter_init(&MOTOR_PID_POS[2], 0.0f, 0.0f, 0.0f, 7000, 7000, 0.05);
}

/*!
 * \fn     m3508_update_m3508_info
 * \brief  更新CAN上m3508电机的报文并且根据id存到不同的包
		   里
 *
 * \param  [in] CAN_RxHeaderTypeDef *msg   #
 * \param  [in] uint8_t	can1_RxData[8]     #
 *
 * \retval void
 */
void m3508_update_info(CAN_RxHeaderTypeDef *msg, uint8_t can1_RxData[8])
{
	switch (msg->StdId) // 检测标准ID
	{
	case M3508_CHASSIS_MOTOR_ID_1:
	{
		MOTOR_REAL_INFO[0].ANGLE = (can1_RxData[0] << 8) | can1_RxData[1];	 // 转子机械角度
		MOTOR_REAL_INFO[0].RPM = (can1_RxData[2] << 8) | can1_RxData[3];	 // 实际转子转速
		MOTOR_REAL_INFO[0].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5]; // 实际转矩电流
	};
	break;

	case M3508_CHASSIS_MOTOR_ID_2:
	{
		MOTOR_REAL_INFO[1].ANGLE = (can1_RxData[0] << 8) | can1_RxData[1];	 // 转子机械角度
		MOTOR_REAL_INFO[1].RPM = (can1_RxData[2] << 8) | can1_RxData[3];	 // 实际转子转速
		MOTOR_REAL_INFO[1].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5]; // 实际转矩电流
	};
	break;

	case M3508_CHASSIS_MOTOR_ID_3:
	{
		MOTOR_REAL_INFO[2].ANGLE = (can1_RxData[0] << 8) | can1_RxData[1];	 // 转子机械角度
		MOTOR_REAL_INFO[2].RPM = (can1_RxData[2] << 8) | can1_RxData[3];	 // 实际转子转速
		MOTOR_REAL_INFO[2].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5]; // 实际转矩电流
	};
	break;
	case M3508_CHASSIS_MOTOR_ID_4:
	{
		MOTOR_REAL_INFO[3].ANGLE = (can1_RxData[0] << 8) | can1_RxData[1];	 // 转子机械角度
		MOTOR_REAL_INFO[3].RPM = (can1_RxData[2] << 8) | can1_RxData[3];	 // 实际转子转速
		MOTOR_REAL_INFO[3].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5]; // 实际转矩电流
	};
	break;

	case M3508_CHASSIS_MOTOR_ID_5:
	{
		MOTOR_REAL_INFO[4].ANGLE = (can1_RxData[0] << 8) | can1_RxData[1];	 // 转子机械角度
		MOTOR_REAL_INFO[4].RPM = (can1_RxData[2] << 8) | can1_RxData[3];	 // 实际转子转速
		MOTOR_REAL_INFO[4].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5]; // 实际转矩电流
	};
	break;
	case M3508_CHASSIS_MOTOR_ID_6:
	{
		MOTOR_REAL_INFO[5].ANGLE = (can1_RxData[0] << 8) | can1_RxData[1];	 // 转子机械角度
		MOTOR_REAL_INFO[5].RPM = (can1_RxData[2] << 8) | can1_RxData[3];	 // 实际转子转速
		MOTOR_REAL_INFO[5].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5]; // 实际转矩电流
	};
	break;

	default:
		break;
	}
}
void shoot_motor_update(CAN_RxHeaderTypeDef *msg, uint8_t can1_RxData[8])
{

	switch (msg->StdId) // 检测标准ID
	{
	case M3508_CHASSIS_MOTOR_ID_1:
	{
		SHOOT_MOTOR_INFO[0].ANGLE = (can1_RxData[0] << 8) | can1_RxData[1];	  // 转子机械角度
		SHOOT_MOTOR_INFO[0].RPM = (can1_RxData[2] << 8) | can1_RxData[3];	  // 实际转子转速
		SHOOT_MOTOR_INFO[0].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5]; // 实际转矩电流
	};
	break;

	case M3508_CHASSIS_MOTOR_ID_2:
	{
		SHOOT_MOTOR_INFO[1].ANGLE = (can1_RxData[0] << 8) | can1_RxData[1];	  // 转子机械角度
		SHOOT_MOTOR_INFO[1].RPM = (can1_RxData[2] << 8) | can1_RxData[3];	  // 实际转子转速
		SHOOT_MOTOR_INFO[1].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5]; // 实际转矩电流
	};
	break;

	case M3508_CHASSIS_MOTOR_ID_3:
	{
		SHOOT_MOTOR_INFO[2].ANGLE = (can1_RxData[0] << 8) | can1_RxData[1];	  // 转子机械角度
		SHOOT_MOTOR_INFO[2].RPM = (can1_RxData[2] << 8) | can1_RxData[3];	  // 实际转子转速
		SHOOT_MOTOR_INFO[2].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5]; // 实际转矩电流
	};
	break;
	case M3508_CHASSIS_MOTOR_ID_4:
	{
		SHOOT_MOTOR_INFO[3].ANGLE = (can1_RxData[0] << 8) | can1_RxData[1];	  // 转子机械角度
		SHOOT_MOTOR_INFO[3].RPM = (can1_RxData[2] << 8) | can1_RxData[3];	  // 实际转子转速
		SHOOT_MOTOR_INFO[3].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5]; // 实际转矩电流
	};
	break;

	case M3508_CHASSIS_MOTOR_ID_5:
	{
		SHOOT_MOTOR_INFO[4].ANGLE = (can1_RxData[0] << 8) | can1_RxData[1];	  // 转子机械角度
		SHOOT_MOTOR_INFO[4].RPM = (can1_RxData[2] << 8) | can1_RxData[3];	  // 实际转子转速
		SHOOT_MOTOR_INFO[4].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5]; // 实际转矩电流
	};
	break;
	case M3508_CHASSIS_MOTOR_ID_6:
	{
		SHOOT_MOTOR_INFO[5].ANGLE = (can1_RxData[0] << 8) | can1_RxData[1];	  // 转子机械角度
		SHOOT_MOTOR_INFO[5].RPM = (can1_RxData[2] << 8) | can1_RxData[3];	  // 实际转子转速
		SHOOT_MOTOR_INFO[5].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5]; // 实际转矩电流
	};
	break;

	default:
		break;
	}
}
/* 发送电流 */
/*!
 * \fn     chassis_m3508_send_motor_currents
 * \brief  将经过pid计算后的控制值打包发送
 *
 * \param  [in] void   #
 *
 * \retval void
 */

// 发送电流
uint8_t send_buf1[8] = {0};
uint8_t error_flag = 0;
uint8_t send_buf2[8] = {0};

void chassis_m3508_send_motor_currents(void)
{

	/***********************************用于ID为 1 2 3 4 的电机*********************************/
	CAN_TxHeaderTypeDef tx_message_1;
	// uint8_t send_buf1[8] = {0};
	uint32_t msg_box1;
	// 配置控制段
	tx_message_1.IDE = CAN_ID_STD;	 // 报文的11位标准标识符CAN_ID_STD表示本报文是标准帧
	tx_message_1.RTR = CAN_RTR_DATA; // 报文类型标志RTR位CAN_ID_STD表示本报文的数据帧
	tx_message_1.DLC = 0x08;		 // 数据段长度
	tx_message_1.TransmitGlobalTime = DISABLE;
	// 配置仲裁段和数据段
	tx_message_1.StdId = 0x200; // 用于ID为 1 2 3 4 的电机

	send_buf1[0] = (uint8_t)(MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8);
	send_buf1[1] = (uint8_t)MOTOR_REAL_INFO[0].TARGET_CURRENT;
	send_buf1[2] = (uint8_t)(MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8);
	send_buf1[3] = (uint8_t)MOTOR_REAL_INFO[1].TARGET_CURRENT;
	send_buf1[4] = (uint8_t)(MOTOR_REAL_INFO[2].TARGET_CURRENT >> 8);
	send_buf1[5] = (uint8_t)MOTOR_REAL_INFO[2].TARGET_CURRENT;
	send_buf1[6] = (uint8_t)(MOTOR_REAL_INFO[3].TARGET_CURRENT >> 8);
	send_buf1[7] = (uint8_t)MOTOR_REAL_INFO[3].TARGET_CURRENT;

	if (HAL_CAN_AddTxMessage(&hcan1, &tx_message_1, send_buf1, &msg_box1) != HAL_OK)
	{
		error_flag = 1;
		// Failed to add message to the transmit mailbox
	}
	/***********************************用于ID为 5 6 7 8 的电机*********************************/
	CAN_TxHeaderTypeDef tx_message_2;
	uint32_t msg_box2;
	// 配置控制段
	tx_message_2.IDE = CAN_ID_STD;	 // 报文的11位标准标识符CAN_ID_STD表示本报文是标准帧
	tx_message_2.RTR = CAN_RTR_DATA; // 报文类型标志RTR位CAN_ID_STD表示本报文的数据帧
	tx_message_2.DLC = 0x08;		 // 数据段长度
	tx_message_2.TransmitGlobalTime = DISABLE;

	// 配置仲裁段和数据段
	tx_message_2.StdId = 0x200; // 用于ID为 5 6 7 8 的电机(不论是3508还是2006)
	send_buf2[0] = (uint8_t)(SHOOT_MOTOR_INFO[0].TARGET_CURRENT >> 8);
	send_buf2[1] = (uint8_t)SHOOT_MOTOR_INFO[0].TARGET_CURRENT; // ID 5
	send_buf2[2] = (uint8_t)(SHOOT_MOTOR_INFO[1].TARGET_CURRENT >> 8);
	send_buf2[3] = (uint8_t)SHOOT_MOTOR_INFO[1].TARGET_CURRENT; // ID 6 cast
	send_buf2[4] = (uint8_t)(SHOOT_MOTOR_INFO[2].TARGET_CURRENT >> 8);
	send_buf2[5] = (uint8_t)SHOOT_MOTOR_INFO[2].TARGET_CURRENT; // ID 7
	send_buf2[6] = (uint8_t)(SHOOT_MOTOR_INFO[3].TARGET_CURRENT >> 8);
	send_buf2[7] = (uint8_t)SHOOT_MOTOR_INFO[3].TARGET_CURRENT; // ID 8

	if (HAL_CAN_AddTxMessage(&hcan2, &tx_message_2, send_buf2, &msg_box2) != HAL_OK)
	{
		error_flag = 1;
		// Failed to add message to the transmit mailbox
	}
}

int itemp;
void MotorCtrl(void)
{
	/*for (int i = 0; i < 8; i++)
	{
		if (MOTOR_REAL_INFO[i].type == NONE)
			break;

		// 速度位置计算
		switch (MOTOR_REAL_INFO[i].unitMode)
		{
		case POSITION_CONTROL_MODE:																					  // 位置模式
			PID_incremental_PID_calculation(&MOTOR_PID_RPM[i], MOTOR_REAL_INFO[i].RPM, MOTOR_PID_POS[i].output);	  // 速度环
			PID_position_PID_calculation(&MOTOR_PID_POS[i], MOTOR_REAL_INFO[i].ANGLE, MOTOR_REAL_INFO[i].TARGET_POS); // 位置环

			break;

		case SPEED_CONTROL_MODE:																					   // 速度模式
			PID_incremental_PID_calculation(&MOTOR_PID_RPM[i], MOTOR_REAL_INFO[i].RPM, MOTOR_REAL_INFO[i].TARGET_RPM); // 速度环
			itemp++;
			break;

		case CURRENT_MODE: // 电流模式
			// MOTOR_REAL_INFO[i].TARGET_CURRENT = Set_target_current;//电流赋值
			// 什么都不执行，直接电流赋值
			break;

		case MOTO_OFF:								  // 电机关闭
			MOTOR_REAL_INFO[i].TARGET_CURRENT = 0.0f; // 电流赋值
			break;
		default:
			break;
		}
	}

	for (int i = 0; i < 8; i++)
	{
		if (MOTOR_REAL_INFO[i].type == NONE)
			break;
	}

	for (int i = 0; i < 8; i++)
	{

		if (MOTOR_REAL_INFO[i].unitMode == CURRENT_MODE)
		{
			// 电流模式下的特殊情况
		}

		else
		{
			if (MOTOR_REAL_INFO[i].type == RM_3508)
				MOTOR_REAL_INFO[i].TARGET_CURRENT = MOTOR_PID_RPM[i].output * 16384.0f / 20000.0f; // M3508单位毫安

			else if (MOTOR_REAL_INFO[i].type == M_2006)
				MOTOR_REAL_INFO[i].TARGET_CURRENT = MOTOR_PID_RPM[i].output * 10000.0f / 10000.0f; // M2006单位毫安
			else
				MOTOR_REAL_INFO[i].TARGET_CURRENT = 0.0f;
		}
	}*/
	motorleft_r.setpoint = MOTOR_REAL_INFO[1].TARGET_RPM;
	motorright_r.setpoint = MOTOR_REAL_INFO[0].TARGET_RPM;
	motorfront_r.setpoint = -MOTOR_REAL_INFO[2].TARGET_RPM;
	MOTOR_REAL_INFO[1].TARGET_CURRENT = rcurrent_to_vcurrent(PID_Compute(&motorleft_r, MOTOR_REAL_INFO[1].RPM));
	MOTOR_REAL_INFO[0].TARGET_CURRENT = rcurrent_to_vcurrent(PID_Compute(&motorright_r, MOTOR_REAL_INFO[0].RPM));
	MOTOR_REAL_INFO[2].TARGET_CURRENT = rcurrent_to_vcurrent(PID_Compute(&motorfront_r, MOTOR_REAL_INFO[2].RPM));

	SHOOT_MOTOR_INFO[0].TARGET_CURRENT = rcurrent_to_vcurrent(PID_Compute(&shoot_down_left, SHOOT_MOTOR_INFO[0].RPM));
	SHOOT_MOTOR_INFO[1].TARGET_CURRENT = rcurrent_to_vcurrent(PID_Compute(&shoot_down_right, SHOOT_MOTOR_INFO[1].RPM));
	SHOOT_MOTOR_INFO[2].TARGET_CURRENT = rcurrent_to_vcurrent(PID_Compute(&shoot_up_left, SHOOT_MOTOR_INFO[2].RPM));
	SHOOT_MOTOR_INFO[3].TARGET_CURRENT = rcurrent_to_vcurrent(PID_Compute(&shoot_up_right, SHOOT_MOTOR_INFO[3].RPM));
	// SHOOT_MOTOR_INFO[0].real_current = vcurrent_to_rcurrent(SHOOT_MOTOR_INFO[0].CURRENT);
	// SHOOT_MOTOR_INFO[0].real_target_current = vcurrent_to_rcurrent(SHOOT_MOTOR_INFO[0].TARGET_CURRENT);
	chassis_m3508_send_motor_currents();
}
void robot_speed_control(void)
{
	Robot_Chassis.Robot_V[2] = PID_Compute(&heading_lock, ROBOT_REAL_POS_DATA.POS_YAW_RAD);
}
float VelCrl(MOTO_REAL_INFO *MOTOR_REAL_INFO, float target_vel)
{
	MOTOR_REAL_INFO->unitMode = SPEED_CONTROL_MODE;
	MOTOR_REAL_INFO->TARGET_RPM = target_vel;

	return 0;
}

float CurrentCrl(MOTO_REAL_INFO *MOTOR_REAL_INFO, float target_current)
{
	MOTOR_REAL_INFO->unitMode = CURRENT_MODE;
	MOTOR_REAL_INFO->TARGET_CURRENT = target_current;

	return 0;
}

float vcurrent_to_rcurrent(int16_t vc)
{
	return ((float)vc / 16384.0f) * 20000; // mA
}

int16_t rcurrent_to_vcurrent(float rc)
{
	return (rc / 20000.0f) * 16384;
}