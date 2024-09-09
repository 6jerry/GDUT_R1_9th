#include "hardware.h"
#include "string.h"
#include "driver_usart.h"
#include "calculation.h"
#include "motor.h"
#include "math.h"
#include "main.h"
#include "MoveBase.h"
#include "communicate.h"
#include "usart.h"
#include "FSM.h"
Air_Contorl  Device;

ACTION_GL_POS ACTION_GL_POS_INFO;

ROBOT_CHASSIS ROBOT_REAL_POS_INFO;

ROBOT_REAL_POS ROBOT_REAL_POS_DATA;

//�������

uint16_t Time_Sys[4]={0};
uint16_t Microsecond_Cnt=0;

uint16_t PPM_buf[10]={0};



uint8_t ppm_update_flag=0;
uint32_t now_ppm_time_send=0;
uint32_t TIME_ISR_CNT=0,LAST_TIME_ISR_CNT=0;


 

/**
  * ��������: �����ⲿ�жϻص�����
  * �������: GPIO_Pin���ж�����
  * �� �� ֵ: ��
  * ˵    ��: ��
 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	static uint32_t last_ppm_time=0,now_ppm_time=0;
	static uint8_t ppm_ready=0,ppm_sample_cnt=0;
	static uint16_t ppm_time_delta=0;//�õ����������½��ص�ʱ��
	
	if(GPIO_Pin==GPIO_PIN_6)//�ж��Ƿ�Ϊ�������������жϣ���������ΪPIN8
	{
		//ϵͳ����ʱ���ȡ����λus
		last_ppm_time=now_ppm_time;//��ȡ��һ�εĵ�ǰʱ����Ϊ�ϴ�ʱ��
		
		now_ppm_time_send=now_ppm_time=10000*TIME_ISR_CNT+TIM2->CNT;//us
		
		ppm_time_delta=now_ppm_time-last_ppm_time;//����õ�һ������ʱ��
		
		//PPM������ʼ
		if(ppm_ready==1)//�ж�֡����ʱ����ʼ�����µ�һ��PPM
		{
			if(ppm_time_delta>=2100)//֡������ƽ����2ms=2000us�����ڲ����ϰ汾ң������//���ջ����PPM�źŲ���׼�������ֽ����쳣ʱ�����Ը�С��ֵ�������������һ����ʹ����ط��ϰ汾ң����
			{
				//memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16));
				ppm_ready = 1;
				ppm_sample_cnt=0;//��Ӧ��ͨ��ֵ
				ppm_update_flag=1;
			} 
			else if(ppm_time_delta>=9&&ppm_time_delta<=2050)//����PWM������1000-2000us�������趨900-2100��Ӧ����Ϊ�������ݴ�
			{         
				PPM_buf[ppm_sample_cnt]=ppm_time_delta;//��Ӧͨ��д�뻺������cnt++�����ж��ٸ�Ԫ��
				ppm_sample_cnt++;
				
				if(ppm_sample_cnt>=9)//���ν�������0-7��ʾ8��ͨ���������������ʾ10��ͨ���������ֵӦ��Ϊ0-9�������޸�
				{
					memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16_t));
					ppm_ready=0;
					ppm_sample_cnt=0;
				}
			}
			
			else  ppm_ready=0;
			
		}
		
		else if(ppm_time_delta>=2100)//֡������ƽ����2ms=2000us
		{
			ppm_ready=1;
			ppm_sample_cnt=0;
			ppm_update_flag=0;
		}
		
		if(PPM_buf[3]>1450&&PPM_buf[3]<1550)PPM_buf[3]=1500;
		if(ROCK_L_Y>1450&&ROCK_L_Y<1550)ROCK_L_Y=1500;
		
		if(ROCK_R_X>1400&&ROCK_R_X<1600)ROCK_R_X=1500;
		if(ROCK_R_Y>1450&&ROCK_R_X<1550)ROCK_R_Y=1500;
		
		if(SWA>900&&SWA<1100)SWA=1000;
		if(SWA>1900&&SWA<2100)SWA=2000;
		
		if(SWD>900&&SWD<1100)SWD=1000;
		if(SWD>1900&&SWD<2100)SWD=2000;
		
		if(SWB>900&&SWB<1100)SWB=1000;
		if(SWB>1450&&SWB<1550)SWB=1500;
		if(SWB>1900&&SWB<2100)SWB=2000;
		
		if(SWC>900&&SWC<1100)SWC=1000;
		if(SWC>1450&&SWC<1550)SWC=1500;
		if(SWC>1900&&SWC<2100)SWC=2000;
		
		
	
	}
}
	
//	//�����½��ؾ�����
//	if(GPIO_Pin==GPIO_PIN_11)
//	{
//		KEY_DATA.KEY_armtop=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11);
//	}
//	if(GPIO_Pin==GPIO_PIN_12)
//	{
//		KEY_DATA.KEY_armbottom=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12);
//	}
//	if(GPIO_Pin==GPIO_PIN_13)
//	{
//		KEY_DATA.KEY_push=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13);
//	}
	


void remote_control(void)
{
	if(Colour_Choice==Red)
	{
		Robot_Chassis.World_V[1]=(ROCK_L_X-1500)*0.003f;
		Robot_Chassis.World_V[0]=(ROCK_L_Y-1500)*0.003f;
	//	Robot_Chassis.World_V[2]=(ROCK_R_X-1500)*0.003f;
	//	Robot_Chassis.World_V[1]=-(ROCK_L_X-1500)*0.0001328212f*(ROCK_L_X-1500)*0.0001328212f*(ROCK_L_X-1500)*0.0001328212f*10*(ROCK_L_X-1500);
	//	Robot_Chassis.World_V[0]=-(ROCK_L_Y-1500)*0.0001328212f*(ROCK_L_Y-1500)*0.0001328212f*(ROCK_L_Y-1500)*0.0001328212f*10*(ROCK_L_Y-1500);
		Robot_Chassis.World_V[2]=(ROCK_R_X-1500)*0.003f;
	}
	
	 else if(Colour_Choice==Blue)
	{
		Robot_Chassis.World_V[1]=(ROCK_L_X-1500)*0.002f;
		
		Robot_Chassis.World_V[0]=(ROCK_L_Y-1500)*0.003f;
	//	Robot_Chassis.World_V[2]=(ROCK_R_X-1500)*0.003f;
	//	Robot_Chassis.World_V[1]=-(ROCK_L_X-1500)*0.0001328212f*(ROCK_L_X-1500)*0.0001328212f*(ROCK_L_X-1500)*0.0001328212f*10*(ROCK_L_X-1500);
	//	Robot_Chassis.World_V[0]=-(ROCK_L_Y-1500)*0.0001328212f*(ROCK_L_Y-1500)*0.0001328212f*(ROCK_L_Y-1500)*0.0001328212f*10*(ROCK_L_Y-1500);
		Robot_Chassis.World_V[2]=(ROCK_R_X-1500)*0.003f;
	}
	
	
	if(ROCK_L_X==1500&&ROCK_L_Y==1500)
	{
		VelCrl(&MOTOR_REAL_INFO[0],0);
		VelCrl(&MOTOR_REAL_INFO[1],0);
		VelCrl(&MOTOR_REAL_INFO[2],0);
	}
	
//	Robot_Chassis.World_V[0]=vx;
//	Robot_Chassis.World_V[1]=vy;
//	Robot_Chassis.World_V[2]=W;
	
//	world_to_robot(&Robot_Chassis);
	
//	Kinematic_Analysis1(Robot_Chassis);
//	Kinematic_Analysis1(Robot_Chassis);
	
	
}


void Adjust_Countrol(void)
{
	
	Robot_Chassis.World_V[1]=(ROCK_L_X-1500)*0.0004f;
	Robot_Chassis.World_V[0]=(ROCK_L_Y-1500)*0.0004f;
	Robot_Chassis.World_V[2]=(ROCK_R_X-1500)*0.0003f;
	
	if( ROCK_L_X==ROCK_L_Y==0)
	{
		VelCrl(&MOTOR_REAL_INFO[0],0);
		VelCrl(&MOTOR_REAL_INFO[1],0);
		VelCrl(&MOTOR_REAL_INFO[2],0);
	}
	
//	Robot_Chassis.World_V[0]=vx;
//	Robot_Chassis.World_V[1]=vy;
//	Robot_Chassis.World_V[2]=W;
	
//	world_to_robot(&Robot_Chassis);
	
//	Kinematic_Analysis1(Robot_Chassis);
//	Kinematic_Analysis1(Robot_Chassis);
}

void ZONE2_Adjust(void)
{
	Robot_Chassis.World_V[1]=(ROCK_L_X-1500)*0.003f;
	Robot_Chassis.World_V[0]=(ROCK_L_Y-1500)*0.003f;
	Robot_Chassis.World_V[2]=(ROCK_R_X-1500)*0.003f;
	
	if(ROCK_L_X==ROCK_L_Y==0)
	{
		VelCrl(&MOTOR_REAL_INFO[0],0);
		VelCrl(&MOTOR_REAL_INFO[1],0);
		VelCrl(&MOTOR_REAL_INFO[2],0);
	}
	
	if(ROCK_R_X!=1500)
	{
		Usart4_SendData(0,0,ROCK_R_X-1500,Turn);
	}
}




void Update_Action_gl_position(float value[6])
{
	//������һ�ε�ֵ
	ACTION_GL_POS_INFO.LAST_POS_X = ACTION_GL_POS_INFO.POS_X;
	ACTION_GL_POS_INFO.LAST_POS_Y = ACTION_GL_POS_INFO.POS_Y;

	//��¼�˴ε�ֵ
	ACTION_GL_POS_INFO.ANGLE_Z = value[0]; // �Ƕȣ�-180~180
	ACTION_GL_POS_INFO.ANGLE_X = value[1];
	ACTION_GL_POS_INFO.ANGLE_Y = value[2];
	ACTION_GL_POS_INFO.POS_X = value[3]; // ����
	ACTION_GL_POS_INFO.POS_Y = value[4]; // ����
	ACTION_GL_POS_INFO.W_Z = value[5];//���ٶ�
	
	ROBOT_REAL_POS_INFO.Robot_V[w]=ACTION_GL_POS_INFO.W_Z ;

	// �������
	ACTION_GL_POS_INFO.DELTA_POS_X = ACTION_GL_POS_INFO.POS_X - ACTION_GL_POS_INFO.LAST_POS_X;
	ACTION_GL_POS_INFO.DELTA_POS_Y = ACTION_GL_POS_INFO.POS_Y - ACTION_GL_POS_INFO.LAST_POS_Y;
	
	
	//�ۼӵó�������ʵλ��
	ACTION_GL_POS_INFO.REAL_X += (ACTION_GL_POS_INFO.DELTA_POS_X);                       //action��װʱ����������ϵ��һ���任
	ACTION_GL_POS_INFO.REAL_Y += (ACTION_GL_POS_INFO.DELTA_POS_Y);
//	ACTION_GL_POS_INFO.REAL_X = ACTION_GL_POS_INFO.POS_X;
//	ACTION_GL_POS_INFO.REAL_Y = ACTION_GL_POS_INFO.POS_Y;
//�任����������
//	ROBOT_REAL_POS_INFO.Position[x] =  ACTION_GL_POS_INFO.REAL_X-161.86f * sin(ROBOT_REAL_POS_INFO.Angle* PI / 180) ;
//	ROBOT_REAL_POS_INFO.Position[y] =  ACTION_GL_POS_INFO.REAL_Y+161.86f * cos(ROBOT_REAL_POS_INFO.Angle* PI / 180) ;

	//ƫ����ֱ�Ӹ�ֵ
	ROBOT_REAL_POS_INFO.Angle = ACTION_GL_POS_INFO.ANGLE_Z;
	
	// ƫ����ֱ�Ӹ�ֵ����ʱ��Ϊ����˳ʱ��Ϊ����
	ROBOT_REAL_POS_DATA.POS_YAW = -ROBOT_REAL_POS_INFO.Angle;
	
	//������е���,��ֵX��Y
	ROBOT_REAL_POS_DATA.POS_X = (ACTION_GL_POS_INFO.REAL_X*cos(one_yaw)-(ACTION_GL_POS_INFO.REAL_Y*sin(one_yaw)))/1000; //+ INSTALL_ERROR_Y * sin(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f);
	ROBOT_REAL_POS_DATA.POS_Y = (ACTION_GL_POS_INFO.REAL_X*sin(one_yaw)+(ACTION_GL_POS_INFO.REAL_Y*cos(one_yaw)))/1000; //- INSTALL_ERROR_Y * (cos(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f)-1);
//	ROBOT_REAL_POS_DATA.POS_X = (ACTION_GL_POS_INFO.REAL_X*cos((-PI*45/180.0f))-(ACTION_GL_POS_INFO.REAL_Y*sin((-PI*45/180.0f))))/1000; //+ INSTALL_ERROR_Y * sin(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f);
//	ROBOT_REAL_POS_DATA.POS_Y = (ACTION_GL_POS_INFO.REAL_X*sin((-PI*45/180.0f))+(ACTION_GL_POS_INFO.REAL_Y*cos((-PI*45/180.0f))))/1000; //- INSTALL_ERROR_Y * (cos(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f)-1);
//	ROBOT_REAL_POS_DATA.POS_X = (ACTION_GL_POS_INFO.REAL_X*cos((PI*45/180.0f))-(ACTION_GL_POS_INFO.REAL_Y*sin((PI*45/180.0f))))/1000; //+ INSTALL_ERROR_Y * sin(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f);
//	ROBOT_REAL_POS_DATA.POS_Y = (ACTION_GL_POS_INFO.REAL_X*sin((PI*45/180.0f))+(ACTION_GL_POS_INFO.REAL_Y*cos((PI*45/180.0f))))/1000; //- INSTALL_ERROR_Y * (cos(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f)-1);
}


/*---------------------------------------------------------����ͨѶ--------------------------------------------------------------*/
Laser_Data Laser_Real_Data=0;
// float Last_Data1=0;//��һ�ε�����
/**
  * ��������: �������ݽ���
  * �������: �����������
  * �� �� ֵ: ���������
  * ˵    ��: ��
 */
float Laser_Resolution(uint8_t rx_Data[9])
{

		float Laser_Data=0;
		int Count_2E=0;
		Laser_Data=Laser_Data+(rx_Data[1]-48)*100;
		Laser_Data=Laser_Data+(rx_Data[2]-48)*10;
		Laser_Data=Laser_Data+(rx_Data[3]-48);
		
		Laser_Data=Laser_Data+(rx_Data[5]-48)*0.1f;
		Laser_Data=Laser_Data+(rx_Data[6]-48)*0.01f;
		Laser_Data=Laser_Data+(rx_Data[7]-48)*0.001f;
		Laser_Data=Laser_Data+(rx_Data[8]-48)*0.0001f;
		return Laser_Data;


}
float ID=0x80;
int Head[2]={0x06,0x83};

uint8_t Laser1[9]={0};
unsigned short count=0;
uint8_t Refer_ID=0;
uint8_t Refer1=0;
uint8_t Refer2=0;

uint8_t Laser2[9]={0};
unsigned short count2=0;
uint8_t Refer_ID2=0;
uint8_t Refer3=0;
uint8_t Refer4=0;
/**
  * ��������: 1�ż����ȡ����
  * �������: �����ֵ
  * �� �� ֵ: ��
  * ˵    ��: ��
 */
void Laser_ReadData(float* Laser_Data)
{
	
//	HAL_UART_Receive_IT(&huart2, &UART2_Receiver, 1); // ��������

	if(UART2_Receiver==0x80)Refer_ID=0x80;
	else if(UART2_Receiver==0x06)Refer1=0x06;
	else if(UART2_Receiver==0x83)Refer2=0x83;
	
	if(Refer_ID==0x80&&Refer1==0x06&&Refer2==0x83)
	{
		if(UART2_Receiver==0x45)//Error���
		{
			Laser_Real_Data.Error_Flag1=1;
			Refer_ID=0;
			Refer1=0;
			Refer2=0;
		}
		else 
		{
			Laser_Real_Data.Error_Flag1=0;
			Laser1[count]=UART2_Receiver;
			count++;
			if(count==9)
			{
//				kalmanfiter(&Kalman_LaserX,Laser_Resolution(Laser1));
//				*Laser_Data=-Kalman_LaserX.Out;
//				printf("%f,%f\n",*Laser_Data,-Laser_Resolution(Laser1));
				*Laser_Data=Laser_Resolution(Laser1);
				count=0;
				Refer_ID=0;
				Refer1=0;
				Refer2=0;
			}
		}
	}
}



/**
  * ��������: 2�ż����ȡ����
  * �������: �����ֵ
  * �� �� ֵ: ��
  * ˵    ��: ��
 */
void Laser_ReadData2(float* Laser_Data)
{
	
//	HAL_UART_Receive_IT(&huart6, &UART6_Receiver, 1); // ��������

	if(UART6_Receiver==0x80)Refer_ID2=0x80;
	else if(UART6_Receiver==0x06)Refer3=0x06;
	else if(UART6_Receiver==0x83)Refer4=0x83;
	if(Refer_ID2==0x80&&Refer3==0x06&&Refer4==0x83)
	{
		if(UART6_Receiver==0x45)
		{
			Laser_Real_Data.Error_Flag2=1;
			Refer_ID2=0;
			Refer3=0;
			Refer4=0;
		}
		else 
		{
			Laser_Real_Data.Error_Flag2=0;
			Laser2[count2]=UART6_Receiver;
			count2++;
			if(count2==9)
			{
				*Laser_Data=Laser_Resolution(Laser2);
				count2=0;
				Refer_ID2=0;
				Refer3=0;
				Refer4=0;
			}
		}
	}
}


 /**
   * ��������: �������ݴ�����
   * �������: Usart_struct* data,uint16_t len
   * �� �� ֵ: ��
   * ˵    ��: ��
  */
 void processData(uint8_t* data,Laser_Data* laser,int uart)
 {
	 float LaserBuf=0;
 	if(data[0]==0x80&&data[1]==0x06&&data[2]==0x83)
	{
		if(data[3]==0x45)Laser_Real_Data.Error_Flag2=1;
		else
		{
		LaserBuf=LaserBuf+(data[3]-48)*100;
		LaserBuf=LaserBuf+(data[4]-48)*10;
		LaserBuf=LaserBuf+(data[5]-48);
		
		LaserBuf=LaserBuf+(data[7]-48)*0.1f;
		LaserBuf=LaserBuf+(data[8]-48)*0.01f;
		LaserBuf=LaserBuf+(data[9]-48)*0.001f;
		if(uart==2)
		{
			if(Colour_Choice==Red)laser->Laser_X=LaserBuf;
			else if(Colour_Choice==Blue)laser->Laser_Y=LaserBuf;
		}
		else if(uart==6)
		{
			if(Colour_Choice==Blue)laser->Laser_X=-LaserBuf;
			else if(Colour_Choice==Red)laser->Laser_Y=-LaserBuf;
		}
		
		}
	}
	return;
	
 }
 
int BallPoint;
 /*
  *  ��������Remote_Process
  *  ����������ң�����ݴ���
  *  �����������
  *  �����������
  *  ����ֵ����
 */
void Remote_Process(void)
{
	if(ZONE_State==ZONE2)
	{
		BallPoint=No1_Data.WOrld_w;
		Move_State=LaserMove_Zone2;
//		if(No1_Data.WOrld_w<=6)Absorb_pointRow=1;
//		else Absorb_pointRow=2;
////		
//		if(No1_Data.WOrld_w==1.0f||No1_Data.WOrld_w==7.0f)Absorb_pointColumn=0;
//		else if(No1_Data.WOrld_w==2.0f||No1_Data.WOrld_w==8.0f)Absorb_pointColumn=1;
//		else if(No1_Data.WOrld_w==3.0f||No1_Data.WOrld_w==9.0f)Absorb_pointColumn=2;
//		else if(No1_Data.WOrld_w==4.0f||No1_Data.WOrld_w==10.0f)Absorb_pointColumn=3;
//		else if(No1_Data.WOrld_w==5.0f||No1_Data.WOrld_w==11.0f)Absorb_pointColumn=4;
//		else if(No1_Data.WOrld_w==6.0f||No1_Data.WOrld_w==12.0f)Absorb_pointColumn=5;
//		Move_State=TrapezoidMove_Zone2;
//		Choice_Flag=1;//��������ѡ��
	}
	
	if(No1_Data.flag==Clor_Choice)
	{
		Colour_Choice=No1_Data.WOrld_w;
		
		if(Colour_Choice==Red)
		{
			HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_RESET);
			one_yaw=PI*45/180.0f;
		}
		else if(Colour_Choice==Blue)
		{
			HAL_GPIO_WritePin(Blue_GPIO_Port,Blue_Pin,GPIO_PIN_RESET);
			one_yaw=-PI*45/180.0f;
		}
	}
}



