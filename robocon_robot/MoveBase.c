

#include "MoveBase.h"
#include "pid.h"
#include "hardware.h"
#include "math.h"
#include "calculation.h"
#include "hardware.h"
#include "driver_usart.h"
#include "communicate.h"
#include "FSM.h"
 
//ROBOT_CHASSIS ROBOT_TARGET_POS_INFO;// ������Ŀ��켣λ����Ϣ

ROBOT_CHASSIS Robot_Chassis;//�����ˣ�Ŀ�꣡�켣λ����Ϣ





int testflag=0;//���ڲ��Դ���Ľ���
float kp_x = 6;
float kd_x = 0;	//0.00011
float kp_y = 6;
float kd_y = 0;	//0.00011
float kp_yaw = 1;
float kd_yaw = 0;
float error_X;float error_Y;	// ����X��Yƫ��
float error_x;float error_y;	// ����x��yƫ��
float error_Yaw;							// ƫ����ƫ��
float now_yaw;								// ��ǰ������ƫ����
float u_output;								// ��������x�����ٶ����
float v_output;								// ��������y�����ٶ����
float w_ouput;	// ���ٶ����


pid point_traker_x_pid;
pid point_traker_y_pid;
pid point_traker_yaw_pid;

pid point_pid;
pid yaw_pid;

pid laser_take_pid;
pid laser_shoot_pid;

float error_X;float error_Y;	// ����X��Yƫ��
float error_x;float error_y;	// ����x��yƫ��



/*
 *  ��������Move_Init()
 *  ������������ʼ���˶�PID����
 *  �����������
 *  �����������
 *  ����ֵ����
*/
void Move_Init(void)
{
	//PD������
//	PID_parameter_init(&point_traker_x_pid, 3,0, 0.5, 2000, 0, 10);
//	PID_parameter_init(&point_traker_y_pid, 3,0, 0.5, 2000, 0, 10);
//	PID_parameter_init(&point_traker_yaw_pid, 30,0, 0.1,1000, 0, 1);
		
	// �Զ�·��PID
//	PID_parameter_init(&Vel_Track_pid, 12.0f,0,0.5, 3000, 3000, 10);
//	
//	PID_parameter_init(&Pos_Track_pid,1.92f, 0.0f, 0.5f, 3, 1, 0.01);	

	
	PID_parameter_init(&yaw_pid , 0.2f,0.02f,0.06f, 2.0f, 0.0f, 1.0f);
	
//	PID_parameter_init(&point_pid ,  1.0f,0.3f, 0.5f, 10, 0, 0.005);
	PID_parameter_init(&point_pid ,  1.5f,0.3f, 0.5f, 10, 0, 0.005);

	PID_parameter_init(&laser_take_pid , 4.2f,0.5f, 3.0f, 300, 0, 0.005);
	
//PID_parameter_init(&yaw_pid ,  30.0f,0.0f, 1.0f, 500.0f, 0.0f, 0.1f);
//	PID_parameter_init(&yaw_pid ,  0.1f,0.0f,0.0f, 1.0f, 0.0f, 1.0f);
	
	ROBOT_REAL_POS_DATA.robot_location=ZONE_1;
	
}

void PDController(PATH_TYPEDEF target_point, ROBOT_REAL_POS robot_now_pos);

int k;
float t;
float f1s;float f2s;float f3s;float f4s;
float last_X;float last_Y;float last_Yaw;
float Sx_error;float Sy_error;
float Hz;
int first_time_flag = 1;
PATH_TYPEDEF now_path_point;

/**
* @brief  PathPlan�滮+����
* @note		����B�����滮�����ֱ�Ӹ�ֵ�������յ㷵��1�����򷵻�0
* @param  t_real:��ʵ������ʱ�䣬t_target:Ŀ����ʱ�䣬num:���Ƶ���Ŀ+1��X��Y:���Ƶ�����
* @retval 
*/
int PathPlan(float t_real, float t_target, int num, float *X , float *Y, float *Yaw)
{ 
	float PathPlanerror_X;
	float PathPlanerror_Y;
	
	k = (int)(t_real * num / t_target);	// ��k��
	t = t_real - k * t_target / num;		// ��k��ʱ��
	t = t * num / t_target;							// ��һ��

	// λ����������
	f1s = (1 - t) * (1 - t) * (1 - t) / 6;
	f2s = (3 * t * t * t - 6 * t * t + 4) / 6;
	f3s = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6;
	f4s = (t * t * t) / 6;
	
	// ����Ŀ����ٵ�
	now_path_point.X = X[k] * f1s + X[k+1] * f2s + X[k+2] * f3s + X[k+3] * f4s;
	now_path_point.Y = Y[k] * f1s + Y[k+1] * f2s + Y[k+2] * f3s + Y[k+3] * f4s;
	now_path_point.Yaw = Yaw[k] * f1s + Yaw[k+1] * f2s + Yaw[k+2] * f3s + Yaw[k+3] * f4s;
	
	if(first_time_flag)
	{
		now_path_point.V_x = 0;
		now_path_point.V_y = 0;
		now_path_point.W = 0;
		first_time_flag = 0;
		Hz = 1 / t_real; 
		
	}
	else
	{
		now_path_point.V_x = (now_path_point.X - last_X) * Hz;
		now_path_point.V_y = (now_path_point.Y - last_Y) * Hz;
		now_path_point.W = (now_path_point.Yaw - last_Yaw) * Hz;
	}
	
	if(t_real < (t_target))
	{
	// PD������
	PDController(now_path_point, ROBOT_REAL_POS_DATA);
//	PathPlanerror_X=ABS(ROBOT_REAL_POS_DATA.POS_X-now_path_point.X);
//	PathPlanerror_Y=ABS(ROBOT_REAL_POS_DATA.POS_Y-now_path_point.Y);
	}	
	// ��������ֵ
	last_X = now_path_point.X;
	last_Y = now_path_point.Y;
	last_Yaw = now_path_point.Yaw;
	
	// �����յ�
	if(t_real > (t_target))
	{
		
//		if(moving_point_track(X[num+3], Y[num+3], Yaw[num+3],200))
//		{
		first_time_flag = 1;
		Robot_Chassis.World_V[1] = 0;//x��
		Robot_Chassis.World_V[0] = 0;//y��
		
			return 1;
//		}
	
	} 
	else	return 0;
}


/**
* @brief  AngleLimit�Ƕ��޷�
* @note	


���Ƕ�������-180�㵽180��
* @param  angle:Ҫ���Ƶ�ֵ
* @retval 
*/
void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(*angle>180.0f)
		{
			*angle-=360.0f;
			AngleLimit(angle);
		}
		else if(*angle<-180.0f)
		{
			*angle+=360.0f;
			AngleLimit(angle);
		}
	}
	
	recursiveTimes--;
}


/**
* @brief  YawAdjustƫ���ǿ���
* @note		��ƫ���ǿ�����Ŀ��Ƕ�
* @param  Target_angle:Ҫ���Ƶ�ֵ
* @retval 
*/
int YawAdjust(float Target_angle)
{
   float YawAdjust_error;
 
	 // �������
   if(ROBOT_REAL_POS_DATA.POS_YAW*Target_angle >= 0)
   {
      YawAdjust_error = Target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
   }
   
   else
   {
		 if(ABS(ROBOT_REAL_POS_DATA.POS_YAW)+ABS(Target_angle) <= 180) YawAdjust_error = Target_angle - ROBOT_REAL_POS_DATA.POS_YAW;
		 else 
		 {
				AngleLimit(&YawAdjust_error);
		 }
   }
   
   // ֱ������PID������ٶ�
   PID_position_PID_calculation_by_error(&yaw_pid, YawAdjust_error);
   
  Robot_Chassis.World_V[2]= yaw_pid.output;	// ���̽��ٶ� ��λ��rad/s
	 
	  if(ABS(YawAdjust_error)<1.0)return 1;
	 else 
	 {
		 return 0;
	 }
	 
}

pid point_pid;//��Ե�׷��PID
pid yaw_pid;//�Ƕ�PID
float error;
//�����
int moving_point_track(float POS_X, float POS_Y, float POS_YAW,float V_max)
{
	 YawAdjust(POS_YAW);
	
	  //�������
	error = sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X) * (ROBOT_REAL_POS_DATA.POS_X - POS_X) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y));  // �������
	point_pid.outputmax = ABS(V_max);
  	PID_position_PID_calculation_by_error(&point_pid, error);
	
		Robot_Chassis.World_V[1] =-(float)(point_pid.output * 1.0f*(ROBOT_REAL_POS_DATA.POS_X - POS_X) /error);//x��
		Robot_Chassis.World_V[0] =-(float)(point_pid.output * 1.0f*(ROBOT_REAL_POS_DATA.POS_Y - POS_Y) /error);//y��
	
	if(ABS(ROBOT_REAL_POS_DATA.POS_X - POS_X)<0.008 && ABS(ROBOT_REAL_POS_DATA.POS_Y- POS_Y)<0.008)
	{
		return 1;
	}
		return 0;
}

/**
* @brief  PDController������
* @note		���ٹ滮�õ�·��
* @param  target_point:��λʱ��Ҫ���ٵĵ㣨���ȹ滮���ٶȣ���robot_now_pos:�����˵�ǰ���������µ�λ��
* @retval 
*/
void PDController(PATH_TYPEDEF target_point,ROBOT_REAL_POS robot_now_pos)
{
	YawAdjust(target_point.Yaw);
	
	// �������
	error_X = target_point.X - robot_now_pos.POS_X;
	error_Y = target_point.Y - robot_now_pos.POS_X;
	error_Yaw = target_point.Yaw - robot_now_pos.POS_YAW;
	//�Ƕ���ת��Ϊ������
	now_yaw = robot_now_pos.POS_YAW * PI / 180.0f;
	// ���㵽��������
	error_x =  cos(now_yaw) * error_X + sin(now_yaw) * error_Y;
	error_y = -sin(now_yaw) * error_X + cos(now_yaw) * error_Y;
	
	// �����ٶ�
	w_ouput  = (kp_yaw * error_Yaw + kd_yaw * target_point.W) / (1 + kd_yaw);
	u_output = (kp_x*error_x + kd_x*( target_point.V_x  * cos(now_yaw) + \
																		target_point.V_y  * sin(now_yaw) + \
																		w_ouput * error_y * cos(now_yaw) - \
																		w_ouput * error_x * sin(now_yaw)))/(1 + kd_x);
	v_output = (kp_y*error_y + kd_y*(-target_point.V_x  * sin(now_yaw) + \
																		target_point.V_y  * cos(now_yaw) - \
																		w_ouput * error_y * sin(now_yaw) - \
																		w_ouput * error_x * cos(now_yaw)))/(1+kd_y);
																		 
	// ����Ϊ��������ϵ�µ��ٶ�
	Robot_Chassis.World_V[1] = -(u_output * cos(now_yaw) - v_output * sin(now_yaw));
	Robot_Chassis.World_V[0] = -(u_output * sin(now_yaw) + v_output * cos(now_yaw));
	Robot_Chassis.World_V[2]  = -w_ouput;
	PID_position_PID_calculation_by_error(&point_traker_x_pid, error_X);
	PID_position_PID_calculation_by_error(&point_traker_y_pid, error_Y);
	PID_position_PID_calculation_by_error(&point_traker_yaw_pid, error_Yaw);
//	
//	//��Ӹ���
//	Robot_Chassis.World_V[1] = point_traker_x_pid.output;
//	Robot_Chassis.World_V[0] = point_traker_y_pid.output;
	//Robot_Chassis.World_V[2]  = -point_traker_yaw_pid.output;
}







/*
 *  ��������chassis_TrapezoidPlaning
 *  ���������������ٶȹ滮
*  ���������POS_X_start:����λ�õĺ�����
			POS_Y_start,����λ�õ�������
			POS_X_end,Ŀ�ĵ�ĺ�����
			POS_Y_end,Ŀ�ĵ��������
			POS_YAW,Ŀ��ƫ���ǣ�
			V_start,��ʼ�ٶ�
			V_end,ĩ�ٶ�
			V_max,����ٶ�
			R_ac,����·�̱���
			R_de������·�̵ı���
 *  �����������
 *  ����ֵ��������������1/������0
*/
int chassis_TrapezoidPlaning(float POS_X_start,
							float POS_Y_start,
							float POS_X_end,
							float POS_Y_end,
							float POS_YAW,
							float V_start,
							float V_end,
							float V_max,
							float R_ac,
							float R_de)
{
//�����������
	float Ssu_chassis;   //��·��
	float Sac_chassis;   //����·��
	float Sde_chassis;   //����·��
	float Sco_chassis;   //����·��
	float Aac_chassis;   //���ټ��ٶ�
	float Ade_chassis;   //���ټ��ٶ�
	float S_chassis;     //��ǰ·��
	float output_V;//������ٶ�
	float real_error;//��ʵ���


	
	YawAdjust(POS_YAW);
		// �����������������ִ���ٶȹ滮		
	if((R_ac > 1) || (R_ac < 0) ||		//����·�̵ı���
		 (R_de > 1) || (R_de < 0) ||	//����·�̵ı���
		 (V_max < V_start) )			//�����ٶ�<��ʼ���ٶ� 
	{
		Robot_Chassis.World_V[1]=0;  // ���˶�
		Robot_Chassis.World_V[0]=0;
		return 1;
	}
	
	//�����г̱���
	Ssu_chassis=sqrt((POS_X_end-POS_X_start)*(POS_X_end-POS_X_start)+(POS_Y_end-POS_Y_start)*(POS_Y_end-POS_Y_start));
	
	Sac_chassis=Ssu_chassis*R_ac;
	Sde_chassis=Ssu_chassis*R_de;
	Sco_chassis=Ssu_chassis-Sac_chassis-Sde_chassis;
	
	Aac_chassis = (V_max * V_max - V_start * V_start) / (2.0f * Sac_chassis);	//���ټ��ٶ� (�����ٶ�*�����ٶ� - ��ʼ���ٶ� *��ʼ���ٶ� ) / (2.0f * ����·��)
//  	if(Aac_chassis>1800)
//		Aac_chassis=1200;//500mm/s
	Ade_chassis = (V_end * V_end - V_max *   V_max) / (2.0f * Sde_chassis);	//���ټ��ٶ�
//	  if(Ade_chassis>600)
//		Ade_chassis=600;//500mm/s
	real_error=sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X_end) * (ROBOT_REAL_POS_DATA.POS_X - POS_X_end) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y_end) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y_end));
		//�����쳣���
		if(Ssu_chassis<S_chassis)
		{
		output_V = -V_start;	//TARGET_RPM = ��ʼ���ٶ�
		}
		
			else
	{
		S_chassis = sqrt((ROBOT_REAL_POS_DATA.POS_X - POS_X_start) * (ROBOT_REAL_POS_DATA.POS_X - POS_X_start) + (ROBOT_REAL_POS_DATA.POS_Y - POS_Y_start) * (ROBOT_REAL_POS_DATA.POS_Y- POS_Y_start));   //��ʼλ��
		
		// �滮RPM
		if     (S_chassis < Sac_chassis)       output_V = sqrt(2.0f * Aac_chassis * S_chassis + V_start * V_start);               // ���ٽ׶�
		else if(S_chassis < (Sac_chassis+Sco_chassis)&&S_chassis>Sac_chassis) output_V = sqrt(2.0f * Aac_chassis * Sac_chassis + V_start * V_start);                                                        // ���ٽ׶�
		else                   output_V = sqrt(V_end * V_end - 2.0f * Ade_chassis * ABS(Ssu_chassis - S_chassis));  // ���ٽ׶�
	}
	
	//�ֽ��ٶȣ���������ʵ�������
		Robot_Chassis.World_V[1] =(output_V * 1.0f*(POS_X_end - ROBOT_REAL_POS_DATA.POS_X) /real_error);//x��
		Robot_Chassis.World_V[0] =(output_V * 1.0f*(POS_Y_end - ROBOT_REAL_POS_DATA.POS_Y) /real_error);//y��
	
	if(ABS(ROBOT_REAL_POS_DATA.POS_X - POS_X_end)<0.015&&ABS(ROBOT_REAL_POS_DATA.POS_Y - POS_Y_end)<0.015)//��ǰ����
	{
//		testflag=1;
		output_V=0;
		Robot_Chassis.World_V[1]=0;
		Robot_Chassis.World_V[0]=0;
		return 1;
		
	}
	else return 0;
	
	
}

int ZONE_Location_Flag=0;
/*
 *  ��������MoveCtrl
 *  �����������ٶȽ��������װ
 *  ���������
 *  �����������
 *  ����ֵ����
*/
void MoveCtrl(void)
{
	// if(!ZONE_Location_Flag)
	// {
	// 	if(ACTION_GL_POS_INFO.ANGLE_X>0)ROBOT_REAL_POS_DATA.robot_location=ZONE_1;//�Ƴ��⸩���ǣ�����
		
	// 	else 
	// 	{
	// 		ROBOT_REAL_POS_DATA.robot_location=ZONE_2;
	// 		ZONE_Location_Flag=1;
	// 	}

	// }
	switch(ROBOT_REAL_POS_DATA.robot_location)
	{
		case ZONE_1:
//			YawAdjust(0);
			if(Yaw_Clock)YawAdjust(0);
			Kinematic_Analysis1(Robot_Chassis);
		break;
		
		case ZONE_2:
//			if(Yaw_Clock)YawAdjust(-45);
			Kinematic_Analysis1(Robot_Chassis);
		break;
	}
}



/*---------------------------------------------------------����--------------------------------------------------------------*/



int laser_flag=0;

//�޸�����ȡ�綨λ
int Laser_calibration(float X, float Y,float yaw,float v_max,int location)
{

		float Laser_error_x,Laser_error_y,ERROR_SHOOTING;

	YawAdjust(yaw);
	//ת��ָ���Ƕ�
	Laser_error_x =  Laser_Real_Data.Laser_X - X;
	Laser_error_y =  Laser_Real_Data.Laser_Y - Y;
	ERROR_SHOOTING=sqrt(Laser_error_x*Laser_error_x+Laser_error_y*Laser_error_y);
//�жϾ����Ƿ����
//	if(Laser_error_x>-5&&Laser_error_y>-5)
//	{
		if(ABS(Laser_error_x)>0.015||ABS(Laser_error_y)>0.015)
		{
			//pidֱ�����
			
			laser_take_pid.outputmax = ABS(v_max);
			PID_position_PID_calculation_by_error(&laser_take_pid, ERROR_SHOOTING);
			if(Colour_Choice==Red)
			{
				if(ABS(Laser_error_y)>0.015)
				{
				Robot_Chassis.World_V[0] = laser_take_pid.output*Laser_error_y/sqrt(ERROR_SHOOTING);
				}
				else Robot_Chassis.World_V[0]=0;
				
				if(ABS(Laser_error_x)>0.015)
				{
				Robot_Chassis.World_V[1]= -laser_take_pid.output*Laser_error_x/sqrt(ERROR_SHOOTING);
				}
				else Robot_Chassis.World_V[1]= 0;
			}
			
			else if(Colour_Choice==Blue)
			{
				if(ABS(Laser_error_y)>0.015)
				{
				Robot_Chassis.World_V[0] =laser_take_pid.output*Laser_error_x/sqrt(ERROR_SHOOTING);
				}
				else Robot_Chassis.World_V[0]=0;
				
				if(ABS(Laser_error_x)>0.015)
				{
				Robot_Chassis.World_V[1]= laser_take_pid.output*Laser_error_y/sqrt(ERROR_SHOOTING);
				}
				else Robot_Chassis.World_V[1]= 0;
			}
			
		Laser_error_x =  Laser_Real_Data.Laser_X - X;
		Laser_error_y =  Laser_Real_Data.Laser_Y - Y;
		return 0;
		}
		else
		{
			if(Colour_Choice==Red)
			{
			switch(location)
			{
				
				case 1:
//					ACTION_GL_POS_INFO.REAL_X=-1.95f;
//					ACTION_GL_POS_INFO.REAL_Y=-0.25f;
				

					laser_flag++;
					ACTION_GL_POS_INFO.REAL_X=500*number2*(1.21f+0.4f);
					ACTION_GL_POS_INFO.REAL_Y=500*number2*(0.4f-1.21f);
				break;
				
				case 2:
					ACTION_GL_POS_INFO.REAL_X=500*number2*(2.15f+2.56f);			
					ACTION_GL_POS_INFO.REAL_Y=500*number2*(2.56f-2.15f);
				break;
				
				case 3:
					ACTION_GL_POS_INFO.REAL_X=500*number2*(1.94+0.25f);			
					ACTION_GL_POS_INFO.REAL_Y=500*number2*(0.25f-1.94f);
				break;
				
				case 7:
					ACTION_GL_POS_INFO.REAL_X=500*number2*(0);			
					ACTION_GL_POS_INFO.REAL_Y=500*number2*(0);
				break;
			}
				return 1;
		}
			if(Colour_Choice==Blue)
			{
			switch(location)
			{
				
				case 1:
//					ACTION_GL_POS_INFO.REAL_X=-1.95f;
//					ACTION_GL_POS_INFO.REAL_Y=-0.25f;
				

				
					ACTION_GL_POS_INFO.REAL_X=500*number2*(-1.21f-0.4f);
					ACTION_GL_POS_INFO.REAL_Y=500*number2*(0.4f-1.21f);
				break;
				
				case 2:
					ACTION_GL_POS_INFO.REAL_X=500*number2*(-2.15f-2.56f);			
					ACTION_GL_POS_INFO.REAL_Y=500*number2*(2.56f-2.15f);
				break;
				
				case 3:
					ACTION_GL_POS_INFO.REAL_X=500*number2*(-1.94-0.25f);			
					ACTION_GL_POS_INFO.REAL_Y=500*number2*(0.25f-1.94f);
				break;
				
				case 7:
					ACTION_GL_POS_INFO.REAL_X=500*number2*(0);			
					ACTION_GL_POS_INFO.REAL_Y=500*number2*(0);
				break;
			}
				return 1;
		}
			
		}
//	}
   
}



/*
 *  ��������Location_Adjust
 *  ����������ײǽ��λ����action����׼ȷֵ
 *  �����������ǰȡ���������finish��
 *  �����������
 *  ����ֵ����
*/
int location_flag=0;













void Location_Adjust(int ZONE1_finish)
{
	if(Colour_Choice==Red)
	{
		switch(ZONE1_finish)
		{
			//����ȡ��
			case 7:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Red_TakeCalibration_X[2]+Red_TakeCalibration_Y[2]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Red_TakeCalibration_Y[2]-Red_TakeCalibration_X[2]);
			break;
			
			//�ĺ�ȡ��
			case 10:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Red_TakeCalibration_X[3]+Red_TakeCalibration_Y[3]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Red_TakeCalibration_Y[3]-Red_TakeCalibration_X[3]);
			break;
			
			//���ȡ��
			case 13:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Red_TakeCalibration_X[4]+Red_TakeCalibration_Y[4]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Red_TakeCalibration_Y[4]-Red_TakeCalibration_X[4]);
			break;
			
			//����ȡ��
			case 16:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Red_TakeCalibration_X[5]+Red_TakeCalibration_Y[5]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Red_TakeCalibration_Y[5]-Red_TakeCalibration_X[5]);
			break;
			
			//�ߺ�ȡ��
			case 19:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Red_TakeCalibration_X[6]+Red_TakeCalibration_Y[6]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Red_TakeCalibration_Y[6]-Red_TakeCalibration_X[6]);
			break;
			
			//�˺�ȡ��
			case 22:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Red_TakeCalibration_X[7]+Red_TakeCalibration_Y[7]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Red_TakeCalibration_Y[7]-Red_TakeCalibration_X[7]);
			break;
			
			//�ź�ȡ��
			case 25:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Red_TakeCalibration_X[8]+Red_TakeCalibration_Y[8]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Red_TakeCalibration_Y[8]-Red_TakeCalibration_X[8]);
			break;
			
			//ʮ��ȡ��
			case 28:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Red_TakeCalibration_X[9]+Red_TakeCalibration_Y[9]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Red_TakeCalibration_Y[9]-Red_TakeCalibration_X[9]);
			break;
			
			//ʮһ��ȡ��
			case 31:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Red_TakeCalibration_X[10]+Red_TakeCalibration_Y[10]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Red_TakeCalibration_Y[10]-Red_TakeCalibration_X[10]);
			break;
			
	//		//ʮ����ȡ��
	//		case 34:
	//			ACTION_GL_POS_INFO.REAL_X=500*number2*(-2.45-0.25f);			
	//			ACTION_GL_POS_INFO.REAL_Y=500*number2*(-0.25f+2.45f);
	//		break;
		}
	}
	else if(Colour_Choice==Blue)
	{
		switch(ZONE1_finish)
		{
			//����ȡ��
			case 7:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Blue_TakeCalibration_X[2]-Blue_TakeCalibration_Y[2]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Blue_TakeCalibration_X[2]+Blue_TakeCalibration_Y[2]);
			break;
			
			//�ĺ�ȡ��
			case 10:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Blue_TakeCalibration_X[3]-Blue_TakeCalibration_Y[3]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Blue_TakeCalibration_X[3]+Blue_TakeCalibration_Y[3]);
			break;
			
			//���ȡ��
			case 13:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Blue_TakeCalibration_X[4]-Blue_TakeCalibration_Y[4]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Blue_TakeCalibration_X[4]+Blue_TakeCalibration_Y[4]);
			break;
			
			//����ȡ��
			case 16:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Blue_TakeCalibration_X[5]-Blue_TakeCalibration_Y[5]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Blue_TakeCalibration_X[5]+Blue_TakeCalibration_Y[5]);
			break;
			
			//�ߺ�ȡ��
			case 19:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Blue_TakeCalibration_X[6]-Blue_TakeCalibration_Y[6]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Blue_TakeCalibration_X[6]+Blue_TakeCalibration_Y[6]);
			break;
			
			//�˺�ȡ��
			case 22:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Blue_TakeCalibration_X[7]-Blue_TakeCalibration_Y[7]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Blue_TakeCalibration_X[7]+Blue_TakeCalibration_Y[7]);
			break;
			
			//�ź�ȡ��
			case 25:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Blue_TakeCalibration_X[8]-Blue_TakeCalibration_Y[8]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Blue_TakeCalibration_X[8]+Blue_TakeCalibration_Y[8]);
			break;
			
			//ʮ��ȡ��
			case 28:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Blue_TakeCalibration_X[9]-Blue_TakeCalibration_Y[9]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Blue_TakeCalibration_X[9]+Blue_TakeCalibration_Y[9]);
			break;
			
			//ʮһ��ȡ��
			case 31:
				ACTION_GL_POS_INFO.REAL_X=500*number2*(Blue_TakeCalibration_X[10]-Blue_TakeCalibration_Y[10]);			
				ACTION_GL_POS_INFO.REAL_Y=500*number2*(Blue_TakeCalibration_X[10]+Blue_TakeCalibration_Y[10]);
			break;
			
	//		//ʮ����ȡ��
	//		case 34:
	//			ACTION_GL_POS_INFO.REAL_X=500*number2*(-2.45-0.25f);			
	//			ACTION_GL_POS_INFO.REAL_Y=500*number2*(-0.25f+2.45f);
	//		break;
		}
	}
	location_flag=ZONE1_finish;
}


