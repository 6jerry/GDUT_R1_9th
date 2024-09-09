
#include "FSM.h"
#include "hardware.h"
#include "motor.h"
#include "MoveBase.h"
#include "calculation.h"
#include "communicate.h"
#include "MoveBase.h"
#include "movement.h"
/*移动状态机*/

float move_time_counter=0;

//上一次拨杆开关值

int Last_SWA=1000;
int Last_SWB=1000;
int Last_SWC=1000;
int Last_SWD=1000;

int Time_ms=0;
float angle_left=0;
float angle_right=0;
int plan_test=1;

int finish=0;//取苗动作完成标志位
int ZONE2_MoveFlag=0;//二区动作完成标志位

unsigned short MoveFlag=0;//总运动开关标志位
unsigned char Inspection_Flag=0;
	
unsigned char Remote_MoveFlag=0;//遥控启动标志位

int Absorb_pointColumn=0;//吸球列点位
int Absorb_pointRow=0;//吸球列点位
unsigned char R_X_State=1;//右侧摇杆选择状态位
unsigned char R_Y_State=1;//右侧摇杆选择状态位

unsigned char NowColumn=0;//当前列位置
unsigned char NowRow=0;//当前列位置

unsigned char Move_State=TrapezoidMove_Zone1;//运动状态标志
unsigned char Colour_Choice;//场地选择标志
uint8_t ZONE_State=ZONE1;
uint8_t Choice_Flag=0;
uint8_t PointChoice=0;
unsigned char Yaw_Clock;//是否锁航向角
int Last_State=TrapezoidMove_Zone1;

/*-------------------------------红方-----------------------------------*/
///* 红方                         1      2   3    4    5    6    7   8    9    10   11   12*/
//float Red_SeedlingTAKE_X[12]={1.213,1.46,1.72,1.95,2.18,2.45,2.67,2.95,3.18,3.45,3.67,3.92};
//float Red_SeedlingTAKE_Y[12]={-0.25,-0.237,-0.24,-0.25,-0.247,-0.249,-0.24,-0.24,-0.235,-0.235,-0.24,-0.24};



///*                               1 2   3    4      5     6    7      8     9    10    11  12*/
//float Red_TakeCalibration_X[12]={0,0,1.7,1.95,2.18,2.45,2.67,2.95,3.18,3.45,3.67,0};
//float Red_TakeCalibration_Y[12]={0,0,-0.26,-0.24,-0.24,-0.24,-0.24,-0.24,-0.24,-0.24,-0.24,0};




///*                              1      2     3     4     5     6     7     8     9    10   11    12*/
//float Red_SeedlingPUT_X[12]={-1.57,-1.57,-2.06,-2.06,-2.54,-2.54,-3.03,-3.03,-3.55,-3.55,-4.05,-4.05};
//float Red_SeedlingPUT_Y[12]={-2.336,-1.82,-2.32,-1.82,-2.32,-1.82,-2.32,-1.82,-2.32,-1.82,-2.32,-1.82};

/* 红方                         1    2   3    4    5    6    7   8    9    10   11   12*/
float Red_SeedlingTAKE_X[12]={1.22,1.47,1.72,1.95,2.18,2.45,2.67,2.95,3.18,3.45,3.67,3.92};
float Red_SeedlingTAKE_Y[12]={0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25};



/*                               1 2   3    4      5    6   7   8     9    10    11  12*/
float Red_TakeCalibration_X[12]={0,0,1.72,1.95,2.18,2.45,2.67,2.95,3.18,3.45,3.67,0};
float Red_TakeCalibration_Y[12]={0,0,0.26,0.24,0.24,0.24,0.24,0.24,0.24,0.24,0.24,0};




/*                              1      2     3     4     5     6     7     8     9    10   11    12*/
float Red_SeedlingPUT_X[12]={1.62,1.62,2.11,2.11,2.61,2.61,3.10,3.10,3.62,3.62,4.12,4.12};
float Red_SeedlingPUT_Y[12]={2.37,1.87,2.37,1.87,2.37,1.87,2.37,1.87,2.37,1.87,2.37,1.87};


/*-------------------------------蓝方-----------------------------------*/

///*                               1      2     3     4     5     6     7     8     9    10   11    12*/
//float Blue_SeedlingTAKE_X[12]={1.213,1.46,1.72,1.95,2.18,2.45,2.67,2.95,3.18,3.45,3.67,3.92};
//float Blue_SeedlingTAKE_Y[12]={0.239,0.237,0.248,0.25,0.247,0.249,0.24,0.24,0.235,0.235,0.24,0.24};


///*                                1 2   3    4      5     6    7      8     9    10    11  12*/
//float Blue_TakeCalibration_X[12]={0,0,1.7,1.95,2.18,2.45,2.67,2.95,3.18,3.45,3.67,0};
//float Blue_TakeCalibration_Y[12]={0,0,0.26,0.24,0.24,0.24,0.24,0.24,0.24,0.24,0.24,0};

///*                               1   2    3    4   5    6    7     8     9    10   11    12*/
//float Blue_SeedlingPUT_X[12]={1.32,1.32,1.80,1.80,2.29,2.29,2.80,2.80,3.32,3.32,3.91,3.91};
//float Blue_SeedlingPUT_Y[12]={2.54,2.00,2.54,2.0,2.54,2.0,2.54,2.0,2.54,2.0,2.52,2.0};

/*                               1      2     3     4     5     6     7     8     9    10   11    12*/
float Blue_SeedlingTAKE_X[12]={-1.20,-1.45,-1.70,-1.95,-2.20,-2.45,-2.70,-2.95,-3.20,-3.45,-3.70,-3.95};
float Blue_SeedlingTAKE_Y[12]={0.249,0.249,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25};


/*                                1 2   3    4      5     6    7      8     9    10    11  12*/
float Blue_TakeCalibration_X[12]={0,0,-1.70,-1.95,-2.20,-2.45,-2.70,-2.95,-3.20,-3.45,-3.70,0};
float Blue_TakeCalibration_Y[12]={0,0,0.24,0.24,0.24,0.24,0.24,0.24,0.24,0.24,0.24,0};

/*                               1    2      3     4    5     6     7      8      9     10   11    12*/
float Blue_SeedlingPUT_X[12]={-1.62,-1.62,-2.11,-2.11,-2.61,-2.61,-3.11,-3.11,-3.61,-3.61,-4.11,-4.11};
float Blue_SeedlingPUT_Y[12]={2.33,1.82,2.33,1.81,2.33,1.81,2.33,1.81,2.33,1.81,2.33,1.81};


//float Ball_X[6]={-1.9,-2.4,-2.915,-3.41,-3.90,-4.42};//取球X坐标

//float Ball_Y[3]={0.4,0.6,1.14};//取球Y坐标

/*----------------------------------------红区----------------------------*/

float Ball_X[6]={-1.9,-2.4,-2.915,-3.41,-3.90,-4.42};//取球X坐标

float Ball_Y[2]={-0.62,-1.1};//取球Y坐标 
					
// void Retry_FSM(void)
// {
// 	switch(No1_Data.flag)
// 	{
// 		case Retry:
// 			if(!No1_Data.WOrld_w)
// 			{
// 				MoveFlag=0;
// 				Robot_Chassis.World_V[0]=0;
// 				Robot_Chassis.World_V[0]=1;
// 			}
// 			else if(No1_Data.WOrld_w)MoveFlag=1;
// 		break;
		
// 		case Seedling:
// 			if(No1_Data.WOrld_w==1)finish=0;
// 			else if(No1_Data.WOrld_w==2)finish=4;
// 			else if(No1_Data.WOrld_w==3)finish=7;
// 			else if(No1_Data.WOrld_w==4)finish=10;
// 			else if(No1_Data.WOrld_w==5)finish=13;
// 			else if(No1_Data.WOrld_w==6)finish=16;
// 			else if(No1_Data.WOrld_w==7)finish=19;
// 			else if(No1_Data.WOrld_w==8)finish=22;
// 			else if(No1_Data.WOrld_w==9)finish=25;
// 			else if(No1_Data.WOrld_w==10)finish=28;
// 			else if(No1_Data.WOrld_w==11)finish=31;
// 			else if(No1_Data.WOrld_w==12)finish=34;
// 		break;
// 	}
// }

int remote_state=0;

/*
 *  函数名：
 *  功能描述：
 *  输入参数：
 *  输出参数：无
 *  返回值：无
*/
void remote_FSM(void)
{
	//自定义遥控
	

//航模遥控	
switch(Move_State)
{
	
	//纯手动模式
	case REMOTE:
		Yaw_Clock=0;
		if(MoveFlag)
		{
			if(SWA==1000)
			{
				remote_state=1;
				remote_control();
				Last_SWA=SWA;
			}
			if(SWA==2000)
			{
				remote_state = 2;
				Adjust_Countrol();
				Last_SWA=SWA;
			}
			
			switch(SWB)
			{
			case 1000:
				
				if(Last_SWB!=SWB)
				{
					HAL_Delay(10);
					if(SWB==1000)
					{
						Move_State=REMOTE;
						Last_SWB=SWB;
					}
				}
				
			break;
			
			case 1500:
				if(Last_SWB!=SWB)
				{
					HAL_Delay(10);
					if(SWB==1500)
					{
						Remote_MoveFlag=1;
//						Usart4_SendData(0,0,0,Inspection);
						Move_State=TrapezoidMove_Zone1;
//						Move_State=Remote_Adjust;
						Last_SWB=SWB;
					}
				}	
			break;
			
			case 2000:
				if(Last_SWB!=SWB)
				{
					HAL_Delay(10);
					if(SWB==2000)
					{
						Usart4_SendData(0,0,0,ZONE2_Inspection);
						Move_State=REMOTE;
						Last_SWB=SWB;
					}
				}
			break;
		}
			
		switch(SWC)
		{
			
			case 1000:
				if(Last_SWC!=SWC)
				{
					HAL_Delay(10);
					if(SWC==1000)
					{
						Usart4_SendData(0,0,0,Open);
						
						
						Last_SWC=SWC;
					}
				}
				
			break;
			
			case 1500:
				if(Last_SWC!=SWC)
				{
					HAL_Delay(10);
					if(SWC==1500)
					{
						Usart4_SendData(0,0,0,Absorb);
						Last_SWC=SWC;
					}
				}
			break;
			
			case 2000:
				if(Last_SWC!=SWC)
				{
					HAL_Delay(10);
					if(SWC==2000)
					{
						Usart4_SendData(1,0,6000,Shoot);
//						Cloud_TurnShoot();
//						if(Laser_Real_Data.Laser_Y<-0.82)
//						{
//							Usart4_SendData(0,0,6000,Shoot);
//						}
//						else 
//						{
//							Usart4_SendData(0,0,5000,Shoot);
//						}
						Last_SWC=SWC;
					}
				}
			break;
		}
		
//		switch(SWD)
//		{
//			case 1000:
//				if(Last_SWD!=SWD)
//				{
//					HAL_Delay(10);

//					if(SWD==1000)
//					{
//						if(ZONE_State==ZONE1)
//						{
//							Usart4_SendData(130,1,0,Rise);
//							Retry_Location();//位于取苗重定位处刷新坐标
//							HAL_Delay(400);
//							finish++;
//							Move_State=TrapezoidMove_Zone1;
//							MoveFlag=1;
//							Last_SWD=SWD;
//						}
//						else if(ZONE_State==ZONE2)
//						{
//							Usart4_SendData(0,0,0,ZONE2_Inspection);
//							MoveFlag=1;
//							Move_State=ZONE2_Remote;
//							Last_SWD=SWD;
//						}
//					}
//				}
//			break;

//			//板下右边拨杆，全车停
//			case 2000:
//			if(Last_SWD!=SWD)
//			{
//				HAL_Delay(10);
//				if(SWD==2000)
//				{
//					Last_SWD=SWD;
//				}
//			}
//			break;
//		}
		
		//��������
		if(SWB==1000)
		{
			switch(SWD)
			{
				case 1000:
					if(Last_SWD!=SWD)
					{
						HAL_Delay(10);

						if(SWD==1000)
						{
							Usart4_SendData(0,1,0,Rise);
							Last_SWD=SWD;
						}
					}
				break;
					
				case 2000:
				if(Last_SWD!=SWD)
				{
					HAL_Delay(10);
					if(SWD==2000)
					{
						Usart4_SendData(130,1,0,Rise);
						Last_SWD=SWD;
					}
				}
				break;
			}
		}
		
		else if(SWB==2000)
		{
			switch(SWD)
			{
				case 1000:
					if(Last_SWD!=SWD)
					{
						HAL_Delay(10);

						if(SWD==1000)
						{
							
							Last_SWD=SWD;
						}
					}
				break;
					
				case 2000:
				if(Last_SWD!=SWD)
				{
					HAL_Delay(10);
					if(SWD==2000)
					{
						Usart4_SendData(0,0,5000,Shoot);
						Last_SWD=SWD;
					}
				}
				break;
			}
		}
			
			if(ROCK_R_Y<1100)
			{
				HAL_Delay(10);
				if(ROCK_R_Y<1000)
				{
					Usart4_SendData(0,0,0,Inspection);
				}
			}
			
			
		}
		
	break;
	
		
			//手动微调
	case Remote_Adjust:	
			Yaw_Clock=0;
			Adjust_Countrol();
			if(!Left_Hand||!Right_Hand)
			{
				Robot_Chassis.World_V[0]=0;
				Robot_Chassis.World_V[1]=0;
				Usart4_SendData(130,1,0,Rise);
				
				Location_Adjust(finish);
				HAL_Delay(500);
				finish++;
				Move_State=TrapezoidMove_Zone1;
			}
			
			switch(SWA)
			{
				case 1000:
					if(Last_SWA!=SWA)
					{
						HAL_Delay(10);
						if(SWA==1000)
						{
							Usart4_SendData(0,0,0,Inspection);
							Move_State=REMOTE;
							Last_SWA=SWA;
						}
					}
					break;
				
				case 2000:
					if(Last_SWA!=SWA)
					{
						HAL_Delay(10);
						if(SWA==2000)
						{
							Move_State=Remote_Adjust;
							Last_SWA=SWA;
						}
					}
					break;
			}
			// if(SWC==2000)
			// {
			// 	HAL_Delay(10);
			// 	if(SWC==2000)
			// 	{
			// 		Robot_Chassis.World_V[0]=0;
			// 		Robot_Chassis.World_V[1]=0;
			// 		HAL_Delay(200);
			// 		Usart4_SendData(0,1,0,Rise);
			// 		HAL_Delay(200);
			// 		Location_Adjust(finish);
			// 		// finish++;
			// 		Move_State=TrapezoidMove_Zone1;
			// 	}
			// }
			
	break;
	
	//二区手动微调
	case ZONE2_Remote:
		Yaw_Clock=0;
		ZONE2_Adjust();
		switch(SWC)
		{
			
			case 1000:
				if(Last_SWC!=SWC)
				{
					HAL_Delay(10);
					if(SWC==1000)
					{
						Usart4_SendData(0,0,0,Open);
						Last_SWC=SWC;
					}
				}
				
			break;
			
			case 1500:
				if(Last_SWC!=SWC)
				{
					HAL_Delay(10);
					if(SWC==1500)
					{
						Usart4_SendData(0,0,0,Absorb);
						Last_SWC=SWC;
					}
				}
			break;
			
			case 2000:
				if(Last_SWC!=SWC)
				{
					HAL_Delay(10);
					if(SWC==2000)
					{
//						Cloud_TurnShoot();
						Usart4_SendData(0,0,0,Shoot);
						Last_SWC=SWC;
					}
				}
			break;
		}

	break;

	//放苗微调
	case PUT_Adjust:
		Yaw_Clock=0;
		Adjust_Countrol();

			
			if(SWD==2000&&Last_SWD!=SWD)
			{
				HAL_Delay(10);
				if(SWD==2000)
				{
					Last_SWD=SWD;
					Robot_Chassis.World_V[0]=0;
					Robot_Chassis.World_V[1]=0;
					HAL_Delay(200);
					Usart4_SendData(0,1,0,Rise);
					HAL_Delay(200);
					finish+=2;
					Move_State=TrapezoidMove_Zone1;
				}
			}
			
			else if(SWD==1000&&Last_SWD!=SWD)
			{
				HAL_Delay(10);
				if(SWD==1000)
				{
					Last_SWD=SWD;
				}
			}
			
	break;		
			
	default:
		//遥控一键启动
		switch(SWB)
		{
			case 1000:
				
				if(Last_SWB!=SWB)
				{
					HAL_Delay(100);
					if(SWB==1000)
					{
						Usart4_SendData(0,0,0,Inspection);
						Move_State=REMOTE;
						Last_SWB=SWB;
					}
				}
				
			break;
			
			case 1500:
				if(Last_SWB!=SWB)
				{
					HAL_Delay(10);
					if(SWB==1500)
					{
						Remote_MoveFlag=1;
//						Usart4_SendData(0,0,0,Inspection);
						Move_State=TrapezoidMove_Zone1;
						Last_SWB=SWB;
					}
				}	
			break;
			
			case 2000:
				if(Last_SWB!=SWB)
				{
					HAL_Delay(10);
					if(SWB==2000)
					{
						Move_State=TrapezoidMove_Zone2;
						Last_SWB=SWB;
					}
				}
			break;
		}

//		switch(SWD)
//		{
//			case 1000:
//			if(Last_SWD!=SWD)
//				{
//					HAL_Delay(10);
//					if(SWD==1000)
//					{
//						Last_SWD=SWD;
//					}
//				}
//			break;

//			//板下右边拨杆，全车停
//			case 2000:
//			if(Last_SWD!=SWD)
//				{
//					HAL_Delay(10);
//					if(SWD==2000)
//					{
//						Usart4_SendData(0,0,0,Retry);
//						Move_State=REMOTE;
//						Last_SWD=SWD;
//					}
//				}
//			break;
//		}
		
		break;
	}
}


void Activate_FSM(void)
{
	Action_Inspection();
//	if(No1_Data.flag==Inspection)Inspection_Flag=1;
	
	if(Remote_MoveFlag)MoveFlag=1;
}



float PUT1=0;
int Laser_Finish=0;
float x_start,y_start;
void move_Laser(float SeedlingTAKE_X[12],float SeedlingTAKE_Y[12],float SeedlingPUT_X[12],float SeedlingPUT_Y[12],float TakeCalibration_X[12],float TakeCalibration_Y[12])
{	
	if(MoveFlag)
	{
		switch(Move_State)
		{
			case TrapezoidMove_Zone1:
			Yaw_Clock=1;
		if(finish==0)
		{
			Usart4_SendData(0,0,0,Inspection);
			HAL_Delay(10);
			
			if(Colour_Choice==Red)
			{
				//һ��ȡ��
				if(moving_point_track(1.2130f, 0.4f, 0.0f,1.0f))finish=1;
			}
			else if(Colour_Choice==Blue)
			{
				//һ��ȡ��
				if(moving_point_track(-1.2050f, 0.4f, 0.0f,1.5f))finish=1;
//				if(chassis_TrapezoidPlaning(0,0,-1.205,0.4,0,0.2f,0,1.5f,0.3,0.4)
			
			}
				
//			finish=moving_point_track(-1.174f,-0.394f, 0,5);
//			if(Laser_calibration(-2.133f,-2.928f,0,5))
//			{
////				Laser_calibration(-2.133f,-2.07f,0,5);
//				Robot_Chassis.World_V[1]=0;
//				Robot_Chassis.World_V[0]=5;
//				HAL_Delay(1000);
//				Robot_Chassis.World_V[1]=0;
//				Robot_Chassis.World_V[0]=0;
//			}
		}
		
		if(finish==1)
		{
			//һ��ȡ��
					Move_State=Photogate;
				
//				Usart4_SendData(130,1,0,Rise);
//				while(No1_Data.flag<=move_flag)
//				{
//				}
//				move_flag=No1_Data.flag;
//				HAL_Delay(600);
//				finish=2;
			
			
		}
		
		if(finish==2)
		{
//			-1.649f, -2.067f, 0.0f,0.8f
			//-1.635，-2.534

			 
			//һ�ŷ���
			if(chassis_TrapezoidPlaning(SeedlingTAKE_X[0],SeedlingTAKE_Y[0],SeedlingPUT_X[0],SeedlingPUT_Y[0],0,0.1f,0,1.7f,0.3,0.4))
			{
				Move_State=PUT_Adjust;
//				 if(MOTOR_REAL_INFO[0].RPM==MOTOR_REAL_INFO[1].RPM==MOTOR_REAL_INFO[2].RPM==0)
//				{
//					HAL_Delay(200);
//				 	Usart4_SendData(0,1,0,Rise);
//				}
////				while(No1_Data.flag<=move_flag)
////				{
////				}
////				move_flag=No1_Data.flag;
//				HAL_Delay(300);
//			    finish=4;
			}
			
		
		}
		
		//һ�ŷ�����˿�
		if(finish==3)
		{	
//			if(chassis_TrapezoidPlaning(-1.656f, -2.563f,-1.656f, -2.650f,0,0.2f,0,0.5f,0.2,0.2))
			if(moving_point_track(SeedlingPUT_X[0]-0.4f, ROBOT_REAL_POS_DATA.POS_Y, 0.0f,1.0f))
			{
				Usart4_SendData(0,0,0,Inspection);
				finish=4;	
			}
		}
		
		//����ȡ��
		if(finish==4)
		{
			Usart4_SendData(0,0,0,Inspection);
			if(chassis_TrapezoidPlaning(SeedlingPUT_X[0]-0.4f,SeedlingPUT_Y[0],SeedlingTAKE_X[1],SeedlingTAKE_Y[1],0,0.2f,0.2,1.5f,0.3,0.4))
			{
				Move_State=Photogate;
//				Move_State=Photogate;
//				Usart4_SendData(130,1,0,Rise);
//				HAL_Delay(600);
//				finish=5;
			}
		}
		
		//���ŷ���
		if(finish==5)
		{			
			if(chassis_TrapezoidPlaning(SeedlingTAKE_X[1],SeedlingTAKE_Y[1],SeedlingPUT_X[1],SeedlingPUT_Y[1],0,0.2f,0,1.7f,0.4,0.4))
			{
				Move_State=PUT_Adjust;
//				if(MOTOR_REAL_INFO[0].RPM==MOTOR_REAL_INFO[1].RPM==MOTOR_REAL_INFO[2].RPM==0)
//				{
//					HAL_Delay(200);
//					Usart4_SendData(0,1,0,Rise);
//				}
//				HAL_Delay(300);
//				finish=7;
			}
		}
		
		

		//���ŷ�����˿�
		if(finish==6)
		{	
//			if(chassis_TrapezoidPlaning(-1.656f, -2.563f,-1.656f, -2.650f,0,0.2f,0,0.5f,0.2,0.2))
			if(moving_point_track(SeedlingPUT_X[1]-0.5f,SeedlingPUT_Y[1], 0.0f,1.0f))
			{
				Usart4_SendData(0,0,0,Inspection);
				finish=7;	
			}
		}
		
		
/*----------------------------------------------------------------------------------------------*/

		
		//����ȡ��
		if(finish==7)
		{
			Usart4_SendData(0,0,0,Inspection);
			if(chassis_TrapezoidPlaning(SeedlingPUT_X[1],SeedlingPUT_Y[1],SeedlingTAKE_X[2],SeedlingTAKE_Y[2],0,0.2f,0.2f,1.3f,0.3,0.5))
			{
				Move_State=Photogate;
//				Left_Take(finish);
//				Usart4_SendData(130,1,0,Rise);
//				while(No1_Data.flag<=move_flag)
//				{
//				}
//				move_flag=No1_Data.flag;
//				HAL_Delay(300);
//				finish=8;
//				while(No1_Data.flag<=move_flag)
//				{
//				}
//				move_flag=No1_Data.flag;
				
			}
		}
		
		//���ŷ���
		if(finish==8)
		{			
			if(chassis_TrapezoidPlaning(TakeCalibration_X[2],TakeCalibration_Y[2],SeedlingPUT_X[2],SeedlingPUT_Y[2],0,0.2f,0,2.0f,0.3,0.4))
			{
				Move_State=PUT_Adjust;
//				if(MOTOR_REAL_INFO[0].RPM==MOTOR_REAL_INFO[1].RPM==MOTOR_REAL_INFO[2].RPM==0)
//				{
//					HAL_Delay(200);
//					Usart4_SendData(0,1,0,Rise);
//				}

//				HAL_Delay(300);
//				finish=10;
					
			}
		}
		
		
		//���ŷ�����ƿ�
		if(finish==9)
		{	
//			if(chassis_TrapezoidPlaning(-1.656f, -2.563f,-1.656f, -2.650f,0,0.2f,0,0.5f,0.2,0.2))
			if(moving_point_track(SeedlingPUT_X[2]-0.5, ROBOT_REAL_POS_DATA.POS_Y, 0.0f,1.0f))
			{
//				//重定位
//				if(Laser_calibration(-3.105f, -0.78f,0,0.5f,1))
//				{
					Usart4_SendData(0,0,0,Inspection);
					finish=10;
//				}
					
			}
		}
		
		
		//�ĺ�ȡ��
		if(finish==10)
		{
			Usart4_SendData(0,0,0,Inspection);
			if(chassis_TrapezoidPlaning(SeedlingPUT_X[2], SeedlingPUT_Y[2],SeedlingTAKE_X[3], SeedlingTAKE_Y[3],0,0.2f,0.2f,1.5f,0.3,0.5))
			{
//				Left_Take(finish);
				Move_State=Photogate;
//				if(Laser_calibration(-2.63f, -3.06f,0,0.8f,1))
//				{
//					if(!Left_Hand)
//					{
//						Usart4_SendData(130,1,0,Rise);
//						HAL_Delay(400);
//						finish=11;
//					}
//					else Move_State=LaserMove;
//				}
				
//				while(No1_Data.flag<=move_flag)
//				{
//				}
//				move_flag=No1_Data.flag;
				
				
			}
		}
		
		//�ĺŷ���
		if(finish==11)
		{			
			if(chassis_TrapezoidPlaning(TakeCalibration_X[3], TakeCalibration_Y[3],SeedlingPUT_X[3], SeedlingPUT_Y[3],0,0.2f,0,1.7f,0.3,0.4))
			{
				Move_State=PUT_Adjust;
//				if(MOTOR_REAL_INFO[0].RPM==MOTOR_REAL_INFO[1].RPM==MOTOR_REAL_INFO[2].RPM==0)
//				{
//					HAL_Delay(200);
//					Usart4_SendData(0,1,0,Rise);
//				}

//				HAL_Delay(300);
//				finish=13;
			}
		}
		
		
		//�ĺŷ�����ƿ�
		if(finish==12)
		{	
//			if(chassis_TrapezoidPlaning(-1.656f, -2.563f,-1.656f, -2.650f,0,0.2f,0,0.5f,0.2,0.2))
			if(moving_point_track(SeedlingPUT_X[3]-0.5, ROBOT_REAL_POS_DATA.POS_Y, 0.0f,1.0f))
			{
				Usart4_SendData(0,0,0,Inspection);
				finish=13;	
			}
		}
		
		
/*------------------------------------------------------------------------------------------------------------*/
		
		//���ȡ��
		if(finish==13)
		{
			Usart4_SendData(0,0,0,Inspection);
			if(chassis_TrapezoidPlaning(SeedlingPUT_X[3], SeedlingPUT_Y[3],SeedlingTAKE_X[4], SeedlingTAKE_Y[4],0,0.2f,0.2f,1.3f,0.3,0.5))
			{
				Move_State=Photogate;
//				Left_Take(finish);
//				Usart4_SendData(130,1,0,Rise);
//				while(No1_Data.flag<=move_flag)
//				{
//				}
//				move_flag=No1_Data.flag;
//				HAL_Delay(400);
//				finish=14;
			}
		}
		
		//��ŷ���
		if(finish==14)
		{			
			if(chassis_TrapezoidPlaning(TakeCalibration_X[4], TakeCalibration_Y[4],SeedlingPUT_X[4], SeedlingPUT_Y[4],0,0.2f,0,2.0f,0.3,0.4))
			{
				Move_State=PUT_Adjust;
//				if(MOTOR_REAL_INFO[0].RPM==MOTOR_REAL_INFO[1].RPM==MOTOR_REAL_INFO[2].RPM==0)
//				{
//					HAL_Delay(200);
//					Usart4_SendData(0,1,0,Rise);
////				}
//				HAL_Delay(300);
//				finish=16;
			}
		}
		
		
		//��ŷ�����ƿ�
		if(finish==15)
		{	
//			if(chassis_TrapezoidPlaning(-1.656f, -2.563f,-1.656f, -2.650f,0,0.2f,0,0.5f,0.2,0.2))
			if(moving_point_track(SeedlingPUT_X[4]-0.5, ROBOT_REAL_POS_DATA.POS_Y, 0.0f,1.0f))
			{
				//重定位
//				if(Laser_calibration(-3.15f, -0.78f,0,0.5f,1))
//				{
					Usart4_SendData(0,0,0,Inspection);
					finish=16;
//				}
					
			}
		}
		
		
		//����ȡ��
		if(finish==16)
		{
			Usart4_SendData(0,0,0,Inspection);
			if(chassis_TrapezoidPlaning(SeedlingPUT_X[4],SeedlingPUT_Y[4],SeedlingTAKE_X[5],SeedlingTAKE_Y[5],0,0.2f,0.2f,1.5f,0.3,0.5))
			{
				Move_State=Photogate;
//				Usart4_SendData(130,1,0,Rise);
////				while(No1_Data.flag<=move_flag)
////				{
////				}
////				move_flag=No1_Data.flag;
//				HAL_Delay(400);
//				finish=17;
			}
		}
		
		//���ŷ���
		if(finish==17)
		{			
			if(chassis_TrapezoidPlaning(TakeCalibration_X[5],TakeCalibration_Y[5],SeedlingPUT_X[5],SeedlingPUT_Y[5],0,0.2f,0,1.7f,0.3,0.4))
			{
				Move_State=PUT_Adjust;
//				if(MOTOR_REAL_INFO[0].RPM==MOTOR_REAL_INFO[1].RPM==MOTOR_REAL_INFO[2].RPM==0)
//				{
//					HAL_Delay(200);
//					Usart4_SendData(0,1,0,Rise);
//				}
//				HAL_Delay(300);
//				finish=19;
			}
		}
		
		
		//���ŷ�����ƿ�
		if(finish==18)
		{	
//			if(chassis_TrapezoidPlaning(-1.656f, -2.563f,-1.656f, -2.650f,0,0.2f,0,0.5f,0.2,0.2))
			if(moving_point_track(SeedlingPUT_X[5]-0.5, ROBOT_REAL_POS_DATA.POS_Y, 0.0f,1.0f))
			{
				Usart4_SendData(0,0,0,Inspection);
				finish=19;	
			}
		}
		
		
		
		
		/*------------------------------------------------------------------------------------------------------------*/
		
		//�ߺ�ȡ��
		if(finish==19)
		{
			Usart4_SendData(0,0,0,Inspection);
			if(chassis_TrapezoidPlaning(SeedlingPUT_X[5],SeedlingPUT_Y[5],SeedlingTAKE_X[6],SeedlingTAKE_Y[6],0,0.1f,0.2f,1.3f,0.3,0.4))
			{
				Move_State=Photogate;
//				Usart4_SendData(130,1,0,Rise);
////				while(No1_Data.flag<=move_flag)
////				{
////				}
////				move_flag=No1_Data.flag;
//				HAL_Delay(400);
//				finish=20;
			}
		}
		
		//�ߺŷ���
		if(finish==20)
		{			
			if(chassis_TrapezoidPlaning(SeedlingTAKE_X[6],SeedlingTAKE_Y[6],SeedlingPUT_X[6],SeedlingPUT_Y[6],0,0.2f,0,2.0f,0.3,0.4))
//			if(moving_point_track(SeedlingPUT_X[6],SeedlingPUT_Y[6], 0.0f,1.0f))
			{
//				if(MOTOR_REAL_INFO[0].RPM==MOTOR_REAL_INFO[1].RPM==MOTOR_REAL_INFO[2].RPM==0)
//				{
//					HAL_Delay(200);
//					Usart4_SendData(0,1,0,Rise);
//				}

//				HAL_Delay(300);
//				finish=22;
				Move_State=PUT_Adjust;
			}
		}
		
		
		//�ߺŷ�����ƿ�
		if(finish==21)
		{	
//			if(chassis_TrapezoidPlaning(-1.656f, -2.563f,-1.656f, -2.650f,0,0.2f,0,0.5f,0.2,0.2))
			if(moving_point_track(SeedlingPUT_X[6]-0.5, ROBOT_REAL_POS_DATA.POS_Y, 0.0f,1.0f))
			{
				//重定位
//				if(Laser_calibration(-3.15f, -0.78f,0,0.5f,1))
//				{
					Usart4_SendData(0,0,0,Inspection);
					finish=22;
//				}
					
			}
		}
		
		
		//�˺�ȡ��
		if(finish==22)
		{
			Usart4_SendData(0,0,0,Inspection);
			if(chassis_TrapezoidPlaning(SeedlingPUT_X[6],SeedlingPUT_Y[6],SeedlingTAKE_X[7],SeedlingTAKE_Y[7],0,0.1f,0.2f,1.5f,0.3,0.5))
			{
				Move_State=Photogate;
//				Usart4_SendData(130,1,0,Rise);
////				while(No1_Data.flag<=move_flag)
////				{
////				}
////				move_flag=No1_Data.flag;
//				HAL_Delay(400);
//				finish=24;
			}
		}
		
		//�˺ŷ���
		if(finish==23)
		{			
			if(chassis_TrapezoidPlaning(TakeCalibration_X[7],TakeCalibration_Y[7],SeedlingPUT_X[7],SeedlingPUT_Y[7],0,0.2f,0,1.7f,0.4,0.4))
			{
				Move_State=PUT_Adjust;
//				if(MOTOR_REAL_INFO[0].RPM==MOTOR_REAL_INFO[1].RPM==MOTOR_REAL_INFO[2].RPM==0)
//				{
//					HAL_Delay(200);
//					Usart4_SendData(0,1,0,Rise);
//				}

//				HAL_Delay(300);
//				finish=25;
			}
		}
		
		
		//�˺ŷ�����˿�
		if(finish==24)
		{	
//			if(chassis_TrapezoidPlaning(-1.656f, -2.563f,-1.656f, -2.650f,0,0.2f,0,0.5f,0.2,0.2))
			if(moving_point_track(SeedlingPUT_X[7]-0.5, ROBOT_REAL_POS_DATA.POS_Y, 0.0f,1.0f))
			{
				Usart4_SendData(0,0,0,Inspection);
				finish=25;	
			}
		}
		
		
		
		/*------------------------------------------------------------------------------------------------------------*/
		
		//�ź�ȡ��
		if(finish==25)
		{
			Usart4_SendData(0,0,0,Inspection);
			if(chassis_TrapezoidPlaning(SeedlingPUT_X[7],SeedlingPUT_Y[7],SeedlingTAKE_X[8],SeedlingTAKE_Y[8],0,0.2f,0.2f,1.3f,0.3,0.4))
			{
				Move_State=Photogate;
//				Usart4_SendData(130,1,0,Rise);
////				while(No1_Data.flag<=move_flag)
////				{
////				}
////				move_flag=No1_Data.flag;
//				HAL_Delay(400);
//				finish=27;
			}
		}
		
		//�źŷ���
		if(finish==26)
		{			
			if(chassis_TrapezoidPlaning(SeedlingTAKE_X[8],SeedlingTAKE_Y[8],SeedlingPUT_X[8],SeedlingPUT_Y[8],0,0.2f,0,2.0f,0.3,0.4))
			{
				Move_State=PUT_Adjust;
//				if(MOTOR_REAL_INFO[0].RPM==MOTOR_REAL_INFO[1].RPM==MOTOR_REAL_INFO[2].RPM==0)
//				{
//					HAL_Delay(200);
//					Usart4_SendData(0,1,0,Rise);
//				}
//				HAL_Delay(300);
//				finish=28;
			}
		}
		
		
		//�źŷ�����ƿ�
		if(finish==27)
		{	
//			if(chassis_TrapezoidPlaning(-1.656f, -2.563f,-1.656f, -2.650f,0,0.2f,0,0.5f,0.2,0.2))
			if(moving_point_track(SeedlingPUT_X[8]-0.5, ROBOT_REAL_POS_DATA.POS_Y, 0.0f,1.0f))
			{
				//重定位
//				if(Laser_calibration(-3.15f, -0.78f,0,0.5f,1))
//				{
					Usart4_SendData(0,0,0,Inspection);
					finish=28;
//				}
					
			}
		}
		
		
		//ʮ��ȡ��
		if(finish==28)
		{
			Usart4_SendData(0,0,0,Inspection);
			if(chassis_TrapezoidPlaning(SeedlingPUT_X[8],SeedlingPUT_Y[8],SeedlingTAKE_X[9],SeedlingTAKE_Y[9],0,0.1f,0.2,1.5f,0.3,0.5))
			{
				Move_State=Photogate;
//				Usart4_SendData(130,1,0,Rise);
////				while(No1_Data.flag<=move_flag)
////				{
////				}
////				move_flag=No1_Data.flag;
//				HAL_Delay(400);
//				finish=30;
			}
		}
		
		//ʮ�ŷ���
		if(finish==29)
		{			
			if(chassis_TrapezoidPlaning(TakeCalibration_X[9],TakeCalibration_Y[9],SeedlingPUT_X[9],SeedlingPUT_Y[9],0,0.2f,0,1.7f,0.4,0.3))
			{
				Move_State=PUT_Adjust;
//				if(MOTOR_REAL_INFO[0].RPM==MOTOR_REAL_INFO[1].RPM==MOTOR_REAL_INFO[2].RPM==0)
//				{
//					HAL_Delay(200);
//					Usart4_SendData(0,1,0,Rise);
//				}

//				HAL_Delay(300);
//				finish=31;
			}
		}
		
		
		//ʮ�ŷ�����ƿ�
		if(finish==30)
		{	
//			if(chassis_TrapezoidPlaning(-1.656f, -2.563f,-1.656f, -2.650f,0,0.2f,0,0.5f,0.2,0.2))
			if(moving_point_track(SeedlingPUT_X[9]-0.5, ROBOT_REAL_POS_DATA.POS_Y, 0.0f,1.0f))
			{
				Usart4_SendData(0,0,0,Inspection);
				finish=31;	
			}
		}
		
		
		
		
		
		/*------------------------------------------------------------------------------------------------------------*/
		
		//ʮһ��ȡ��
		if(finish==31)
		{
			Usart4_SendData(0,0,0,Inspection);
			if(chassis_TrapezoidPlaning(SeedlingPUT_X[9],SeedlingPUT_Y[9],SeedlingTAKE_X[10],SeedlingTAKE_Y[10],0,0.1f,0.2,1.3f,0.3,0.4))
			{
				Move_State=Photogate;
//				Usart4_SendData(130,1,0,Rise);
////				while(No1_Data.flag<=move_flag)
////				{
////				}
////				move_flag=No1_Data.flag;
//				HAL_Delay(400);
//				finish=33;
			}
		}
		
		//ʮһ�ŷ���
		if(finish==32)
		{			
			if(chassis_TrapezoidPlaning(TakeCalibration_X[10],TakeCalibration_Y[10],SeedlingPUT_X[10],SeedlingPUT_Y[10],0,0.2f,0,2.0f,0.3,0.3))
			{
				Move_State=PUT_Adjust;
//				if(MOTOR_REAL_INFO[0].RPM==MOTOR_REAL_INFO[1].RPM==MOTOR_REAL_INFO[2].RPM==0)
//				{
//					HAL_Delay(200);
//					Usart4_SendData(0,1,0,Rise);
//				}
//				HAL_Delay(300);
//				finish=34;
			}
		}
		
		
		//ʮһ�ŷ�����ƿ�
		if(finish==33)
		{	
//			if(chassis_TrapezoidPlaning(-1.656f, -2.563f,-1.656f, -2.650f,0,0.2f,0,0.5f,0.2,0.2))
			if(moving_point_track(SeedlingPUT_X[10]-0.5, ROBOT_REAL_POS_DATA.POS_Y, 0.0f,1.0f))
			{
				//重定位
//				if(Laser_calibration(-3.15f, -0.78f,0,0.5f,1))
//				{
					Usart4_SendData(0,0,0,Inspection);
					finish=34;
//				}
					
			}
		}
		
		
		//ʮ����ȡ��
		if(finish==34)
		{
			Usart4_SendData(0,0,0,Inspection);
			if(chassis_TrapezoidPlaning(SeedlingPUT_X[10],SeedlingPUT_Y[10],SeedlingTAKE_X[11],SeedlingTAKE_Y[11],0,0.2f,0.2,1.5f,0.3,0.5))
			{
				Move_State=Photogate;
//				Usart4_SendData(130,1,0,Rise);
////				while(No1_Data.flag<=move_flag)
////				{
////				}
////				move_flag=No1_Data.flag;
//				HAL_Delay(400);
//				finish=36;
			}
		}
		
		//ʮ���ŷ���
		if(finish==35)
		{			
			if(chassis_TrapezoidPlaning(SeedlingTAKE_X[11],SeedlingTAKE_Y[11],SeedlingPUT_X[11],SeedlingPUT_Y[11],0,0.2f,0,1.5f,0.5,0.4))
			{
				Move_State=PUT_Adjust;
//				if(MOTOR_REAL_INFO[0].RPM==MOTOR_REAL_INFO[1].RPM==MOTOR_REAL_INFO[2].RPM==0)
//				{
//					HAL_Delay(200);
//					Usart4_SendData(0,1,0,Rise);
//				}
//				HAL_Delay(300);
//				finish=37;
			}
		}
		
		
		//ʮ���ŷ�����ƿ�
		if(finish==36)
		{	
//			if(chassis_TrapezoidPlaning(-1.656f, -2.563f,-1.656f, -2.650f,0,0.2f,0,0.5f,0.2,0.2))
			if(moving_point_track(SeedlingPUT_X[11]-0.5, ROBOT_REAL_POS_DATA.POS_Y, 0.0f,1.0f))
			{
				Usart4_SendData(0,0,0,Inspection);
//				finish=37;
				finish=37;
			}
		}
		
		if(finish==37)
		{
//			Move_State=REMOTE;
			
				if(chassis_TrapezoidPlaning(SeedlingPUT_X[11], SeedlingPUT_Y[11],SeedlingPUT_X[11], SeedlingPUT_Y[11]-0.9,0,0.2,0.1,1.5f,0.4,0.2))finish=38;
			
		}
		
		if(finish==38)
		{
			if(Colour_Choice==Blue)
			{
				if(chassis_TrapezoidPlaning(SeedlingPUT_X[11], SeedlingPUT_Y[11]-0.9,0.7f,0.9f,0,0.2f,0,2.0f,0.3,0.2))
				{
					finish=39;
					Move_State=REMOTE;
				}
			}
			else 
			{
				if(chassis_TrapezoidPlaning(SeedlingPUT_X[11], SeedlingPUT_Y[11]-0.9,-0.7f, 0.9f,0,0,0,2.0f,0.4,0.2))
				{
					finish=39;
					Move_State=REMOTE;
				}
			}

		}
		if(finish==39)
		{
//			if(chassis_TrapezoidPlaning(0.7f, 0.9f,0.7f, 5.2f,0,0,0,1.7f,0.4,0.2))Move_State=REMOTE;
		}
		break;
			}
		
			
	}
}


void Laser_FSM(void)
{
	if(1)
	{
		if(Move_State==LaserMove_Zone1)
		{
			Yaw_Clock=1;
			switch(finish)
			{
				//放三号苗
				case 8:
				if(Laser_calibration(3.07f, 0.75f,0,0.3f,2))
				{
					Usart4_SendData(0,1,0,Rise);
					HAL_Delay(300);
					finish=9;
					Move_State=TrapezoidMove_Zone1;
				}
				break;
				
				//取一号苗
				case 0:
				if(Colour_Choice==Red)
				{
					if(SWD==1000)Adjust_Countrol();
					else if(SWD==2000)
					{
						finish=1;
						Move_State=TrapezoidMove_Zone1;
//						__HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);//关闭USART2空闲中断
//						__HAL_UART_DISABLE_IT(&huart6, UART_IT_IDLE);//关闭USART2空闲中断
					}
				}
				
				else if(Colour_Choice==Blue)
				{
					if(Laser_calibration(2.10f,2.89f,0,0.8f,1))
					{
						finish=1;
						Move_State=TrapezoidMove_Zone1;
//						__HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);//关闭USART2空闲中断
//						__HAL_UART_DISABLE_IT(&huart6, UART_IT_IDLE);//关闭USART2空闲中断
					}
				}
				break;
				
				//取三号苗
				case 7:
					Left_Take(finish);
					if(Laser_calibration(2.60,3.01f,0,0.2f,100))
					{
						
						Usart4_SendData(130,1,0,Rise);
						HAL_Delay(300);
						finish=8;
						Move_State=TrapezoidMove_Zone1;
					}
				break;
				//取四号苗
				case 10:
					Left_Take(finish);
					if(Laser_calibration(2.85,3.02f,0,0.2f,100))
					{
						Usart4_SendData(130,1,0,Rise);
						HAL_Delay(300);
						finish=11;
						Move_State=TrapezoidMove_Zone1;
					}
				break;
				
				//取五号苗
				case 13:
					if(Laser_calibration(Laser_Real_Data.Laser_X,3.02f,0,0.5f,100))
					{
						Usart4_SendData(130,1,0,Rise);
						HAL_Delay(300);
						finish=14;
						Move_State=TrapezoidMove_Zone1;
					}
				break;
					
				}
		}
		
		else if(Move_State==LaserMove_Zone2)
		{

			if(BallPoint>0&&BallPoint<=6)
			{
				if(Laser_calibration(Ball_X[BallPoint-1],Ball_Y[0],0,0.5f,100))
				{
					Usart4_SendData(0,0,0,Shoot);
					Move_State=ZONE2_Remote;
				}
			}
			
			else if(BallPoint>6&&BallPoint<=12)
			{
				if(Laser_calibration(Ball_X[BallPoint-7],Ball_Y[1],0,0.5f,100))
				{
					Usart4_SendData(0,0,0,Shoot);
					Move_State=ZONE2_Remote;
				}
			}		
		}
	}
//	else if(Colour_Choice==Blue);
	
}

//�����״̬��
void Photogate_FSM(void)
{
		if(Move_State==Photogate)
		{
			Yaw_Clock=1;
			if(Colour_Choice==Red)
			{
			
				if(!Left_Hand)
				{
					Robot_Chassis.World_V[0]=0;
					Robot_Chassis.World_V[1]=0;
					Usart4_SendData(130,1,0,Rise);
//					Location_Adjust(finish);
		//			while(Time_ms<400);
					HAL_Delay(500);
					finish++;
					Move_State=TrapezoidMove_Zone1;
				}
				
				else 
				{
					Robot_Chassis.World_V[1]=0;
					Robot_Chassis.World_V[0]=-0.2f;//
					if(!Left_Wall)
					{
						Robot_Chassis.World_V[0]=0;
						Robot_Chassis.World_V[1]=0;
						Move_State=Remote_Adjust;
					}
				}
			}
			
			else if(Colour_Choice==Blue||SWD==2000)
			{
				if(!Right_Hand)
				{
					Last_SWD=2000;
					Robot_Chassis.World_V[0]=0;
					Robot_Chassis.World_V[1]=0;
					Usart4_SendData(130,1,0,Rise);
					Location_Adjust(finish);
		//			while(Time_ms<400);
					HAL_Delay(500);
					finish++;
					Move_State=TrapezoidMove_Zone1;
				}
				
				else 
				{
					Robot_Chassis.World_V[0]=-0.2f;//直行直到碰到苗
					if(!Right_Wall)
					{
						Robot_Chassis.World_V[0]=0;
						Robot_Chassis.World_V[1]=0;
						Move_State=Remote_Adjust;
					}
				}
			}
		}
}





void ZONE2_FSM(void)
{
	if(Colour_Choice==Red)
	{
		switch(Move_State)
		{
			case TrapezoidMove_Zone2:
				
				switch(ZONE2_MoveFlag)
				{
					
					case 0:
						
						Usart4_SendData(0,0,0,ZONE2_Inspection);
//						ZONE_State=ZONE2;
						Move_State=REMOTE;
					
					break;
					
					case 1:
						// if(moving_point_track(-1.9,0.4, 0.0f,0.8f))
						// {
						// 	Absorb_pointColumn=0;
						// 	Absorb_pointRow=0;
						// 	ZONE2_MoveFlag=2;
						// 	Move_State=ZONE2_Remote;
						// }
					break;
					
					case 2:
						
					break;
					
					case 3:
					break;
					
					case 4:
						
					break;
				}
			break;
				
			case ChooseBall:
					if(moving_point_track(Ball_X[Absorb_pointColumn],0.4f, 0.0f,1.0f))
					{
						NowColumn=Absorb_pointColumn;
						NowRow=0;
						Move_State=TrapezoidMove_Zone2;//返回选球
					}
			break;
						
		}
	}
	
	
}


//包含场地选择的运动状态机
void MOVE_FSM(void)
{
	if(Colour_Choice==Red)
	{
		move_Laser(Red_SeedlingTAKE_X,Red_SeedlingTAKE_Y,Red_SeedlingPUT_X,Red_SeedlingPUT_Y,Red_TakeCalibration_X,Red_TakeCalibration_Y);
	}
	else if(Colour_Choice==Blue)
	{
		move_Laser(Blue_SeedlingTAKE_X,Blue_SeedlingTAKE_Y,Blue_SeedlingPUT_X,Blue_SeedlingPUT_Y,Blue_TakeCalibration_X,Blue_TakeCalibration_Y);
	}

}



