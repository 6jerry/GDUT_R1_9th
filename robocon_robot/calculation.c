#include "calculation.h"
#include "main.h"
#include "math.h"
#include "motor.h"
#include "hardware.h"
#include "MoveBase.h"
#include "FSM.h"
///*!
// * \fn     pos_calculate
// * \brief  �����ٶ����
// *         ���ݵ���ڷ�λ�ø���
// * \param  [in] int num    #
// * \param  [in] float vx   #
// * \param  [in] float vy   #
// * \param  [in] float w    #
// *
// * \retval float
// */
// float pos_calculate(int num,float vx,float vy,float Vw)
//{
//	float va=Vw*RADIUS;
//	if(num==1)
//	{
//		float out1=(float)(-vx*0.5-vy*sqrt(3)/2+va);
//		return out1;
//	}
//
//	else if(num==0)
//	{
//		float out2=(float)(vx+va);
//		return out2;
//
//	}
//
//	else if(num==2)
//	{
//		float out3=(float)(-vx*0.5+vy*sqrt(3)/2+va);
//		return out3;
//	}
//
//	else
//		return 0;
//}

/*!
 * \fn     v_to_rpm
 * \brief  ���������ٶ�ת��Ϊת��
 *
 * \param  ��float��v
 *
 * \retval float
 */

float v_to_rpm(float v)
{
	float rpm = v / Rwheel / (2 * PI) * 60 * 19;
	return rpm;
}

/*!
 * \fn     rpm_to_v
 * \brief  ������ת��ת��Ϊ���ٶ�
 *
 * \param  ��float��rpm
 *
 * \retval float
 */

float rpm_to_v(float rpm)
{
	float v = rpm * 2 * PI / 60 * Rwheel;
	return v;
}

float one_yaw = 45.0 * PI / 180.0f;
/*!
 * \fn     Kinematic_Analysis1
 * \brief  һ����̬��ȫ�����˶���� ��Ŀ�����������ٶ��ٶ�ת��Ϊÿ������ת��
 *
 * \param  [in] float Vx   #
 * \param  [in] float Vy   #
 *
 * \retval void
 */
void Kinematic_Analysis1(ROBOT_CHASSIS motor_target)
{

	float now_yaw = 0;
	if (Yaw_Clock == 0)
		now_yaw = -ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f;
	float Vx = cos(one_yaw + now_yaw) * (motor_target.World_V[1]) + sin(one_yaw + now_yaw) * (motor_target.World_V[0]);
	float Vy = -sin(one_yaw + now_yaw) * (motor_target.World_V[1]) + cos(one_yaw + now_yaw) * (motor_target.World_V[0]);
	float W = -motor_target.World_V[2];

	MOTOR_REAL_INFO[1].TARGET_RPM = -v_to_rpm((W * RADIUS + 0.6712 * Vx - Vy * 0.7412));

	MOTOR_REAL_INFO[2].TARGET_RPM = -v_to_rpm(W * RADIUS + 0.6712 * Vx + Vy * 0.7412);

	MOTOR_REAL_INFO[0].TARGET_RPM = v_to_rpm(-Vx + RADIUS * W);
}
void robot_tf()
{

	MOTOR_REAL_INFO[0].TARGET_RPM = v_to_rpm(Robot_Chassis.Robot_V[2] * 0.3149 + 0.6712 * Robot_Chassis.Robot_V[1] - Robot_Chassis.Robot_V[0] * 0.7412);

	MOTOR_REAL_INFO[1].TARGET_RPM = v_to_rpm(Robot_Chassis.Robot_V[2] * 0.3149 + 0.6712 * Robot_Chassis.Robot_V[1] + Robot_Chassis.Robot_V[0] * 0.7412); // y,x,w

	MOTOR_REAL_INFO[2].TARGET_RPM = v_to_rpm(-Robot_Chassis.Robot_V[1] + 0.20024 * Robot_Chassis.Robot_V[2]);
}
void world_tf()
{
	Robot_Chassis.Robot_V[1] = cos(ROBOT_REAL_POS_DATA.POS_YAW_RAD) * Robot_Chassis.World_V[1] + sin(ROBOT_REAL_POS_DATA.POS_YAW_RAD) * Robot_Chassis.World_V[0];
	Robot_Chassis.Robot_V[0] = cos(ROBOT_REAL_POS_DATA.POS_YAW_RAD) * Robot_Chassis.World_V[0] - sin(ROBOT_REAL_POS_DATA.POS_YAW_RAD) * Robot_Chassis.World_V[1];
}
/*!
 * \fn     Kinematic_Analysis
 * \brief  ������ȫ�����˶���� ��Ŀ�����������ٶ��ٶ�ת��Ϊÿ������ת��
 *
 * \param  [in] float Vx   #
 * \param  [in] float Vy   #
 *
 * \retval void
 */
void Kinematic_Analysis2(ROBOT_CHASSIS motor_target)
{

	world_to_robot(&motor_target);

	MOTOR_REAL_INFO[1].TARGET_RPM = v_to_rpm((motor_target.Robot_V[2] * RADIUS + 0.6712 * motor_target.Robot_V[1] - motor_target.Robot_V[0] * 0.7412));

	MOTOR_REAL_INFO[2].TARGET_RPM = v_to_rpm(motor_target.Robot_V[2] * RADIUS + 0.6712 * motor_target.Robot_V[1] + motor_target.Robot_V[0] * 0.7412);

	MOTOR_REAL_INFO[0].TARGET_RPM = -v_to_rpm(-motor_target.Robot_V[1] + RADIUS * motor_target.Robot_V[2]);
}

/*!
 * \fn     world_to_robot
 * \brief  ����������ϵ�ٶȻ���ɻ���������ϵ�ٶ�
 *
 * \param  ROBOT_REAL_POS robot_now_pos
 *
 * \retval void
 */
void world_to_robot(ROBOT_CHASSIS *robot_data)
{
	float now_yaw = -ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f;

	// x�����ϵ�ת��
	robot_data->Robot_V[1] = cos(now_yaw) * (robot_data->World_V[1]) + sin(now_yaw) * (robot_data->World_V[0]);
	// y�����ϵ�ת��
	robot_data->Robot_V[0] = -sin(now_yaw) * (robot_data->World_V[1]) + cos(now_yaw) * (robot_data->World_V[0]);
	// yaw�ϵ�ת��
	robot_data->Robot_V[2] = robot_data->World_V[2];

	return;
}

///*!
// * \fn     Kinematic_Analysis
// * \brief  ������ȫ�����˶���� ��Ŀ�����������ٶ��ٶ�ת��Ϊÿ������ת��
// *
// * \param  [in] float Vx   #
// * \param  [in] float Vy   #
// *
// * \retval void
// */
// void Kinematic_Analysis3(ROBOT_CHASSIS motor_target)
//{
//	float now_yaw=ROBOT_REAL_POS_DATA.POS_YAW* PI / 180.0f;
//
//	MOTOR_REAL_INFO[1].TARGET_RPM = v_to_rpm(motor_target.World_V[2]*RADIUS+0.6712*(motor_target.World_V[1]*cos(now_yaw)-motor_target.World_V[0]*sin(now_yaw));

//	MOTOR_REAL_INFO[2].TARGET_RPM = v_to_rpm(motor_target.Robot_V[2]*RADIUS+0.6712*motor_target.Robot_V[1]+motor_target.Robot_V[0]*0.7412);

//	MOTOR_REAL_INFO[0].TARGET_RPM= -v_to_rpm(motor_target.World_V[2]*RADIUS-motor_target.World_V[1]*cos(now_yaw)-motor_target.World_V[0]*sin(now_yaw));
//}

KalmanFilter Kalman_LaserX;
KalmanFilter Kalman_LaserY;

/*
 *  ��������Kalmanfiter_Init
 *  ����������������ϵ����ʼ��
 *  ���������KalmanFilter *EKF
 *  �����������
 *  ����ֵ����
 */
void Kalmanfiter_Init(KalmanFilter *EKF)
{
	EKF->LastP = 0.02;
	EKF->NewP = 0;
	EKF->Out = 0;
	EKF->Kg = 1;
	EKF->Q = 0.01;
	EKF->R = 5;
}

/*
 *  ��������kalmanfiter
 *  �����������������˲���
 *  ���������KalmanFilter *EKF float input
 *  �����������
 *  ����ֵ����
 */
void kalmanfiter(KalmanFilter *EKF, float input)
{
	EKF->NewP = EKF->LastP + EKF->Q;
	EKF->Kg = EKF->NewP / (EKF->NewP + EKF->R);
	EKF->Out = EKF->Out + EKF->Kg * (input - EKF->Out);
	EKF->LastP = (1 - EKF->Kg) * EKF->NewP;
}
