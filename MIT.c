#include "MIT.h"
#include "stm32f4xx_hal.h"
#include "can.h"

//MIT驱动

MIT_REAL_INFO MIT_DRIVER_REAL_INFO[4];         


void DM43_Init(void)
{	
	DM43_control_cmd(DM43_ID1, 0x01);
	HAL_Delay(5);
	DM43_control_cmd(DM43_ID2, 0x01);
  HAL_Delay(5);	
  DM43_control_cmd(DM43_ID3, 0x01);
	HAL_Delay(5);
  DM43_control_cmd(DM43_ID4, 0x01);
	HAL_Delay(5);
}

/*
速度模式发送函数
*/
void MOTOR_Speed_Control(uint16_t ID,float vel)
{
	uint8_t *vbuf;
	vbuf=(uint8_t*)&vel;
	uint8_t send_buf[8] = {0};
	uint32_t msg_box;
	
	CAN_TxHeaderTypeDef tx_message;
	
	tx_message.StdId = ID+0x200;
	
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x04;
	
	send_buf[0] = *vbuf; 
	send_buf[1] = *(vbuf+1); 
	send_buf[2] = *(vbuf+2); 
	send_buf[3] = *(vbuf+3); 
	
	if (HAL_CAN_AddTxMessage(&hcan2,&tx_message,send_buf,&msg_box)!= HAL_OK) {
        // Failed to add message to the transmit mailbox
    }
}


//速度位置控制模式
void ctrl_motor2(uint16_t id, float _pos, float _vel )
{ 
		  
	CAN_TxHeaderTypeDef tx_message;
		  
  uint8_t *pbuf,*vbuf; 
	pbuf=(uint8_t*)&_pos; 
	vbuf=(uint8_t*)&_vel; 
	uint32_t msg_box;
	uint8_t send_buf[8] = {0};
  
	tx_message.StdId = id+0x100; 
	tx_message.IDE = CAN_ID_STD; 
	tx_message.RTR = CAN_RTR_DATA; 
	tx_message.DLC = 0x08; 
	send_buf[0] = *pbuf; 
	send_buf[1] = *(pbuf+1); 
	send_buf[2] = *(pbuf+2); 
	send_buf[3] = *(pbuf+3); 
	send_buf[4] = *vbuf; 
	send_buf[5] = *(vbuf+1); 
	send_buf[6] = *(vbuf+2); 
	send_buf[7] = *(vbuf+3);
	  
	  if (HAL_CAN_AddTxMessage(&hcan2,&tx_message,send_buf,&msg_box)!= HAL_OK) {
        // Failed to add message to the transmit mailbox
    }
}
	  


/*
p_des 目标位置
v_des 目标速度
kp    位置环参数
kd    速度环参数
t_ff  目标扭矩
*/

/*
由使用说明书，其本身有特殊can代码，分别控制
1.进入电机控制模式
2.退出电机控制模式
3.设置电机当前位置为零点
即想要控制电机模式，应该先在此调用相关函数
*/
void DM43_control_cmd(uint16_t ID,uint8_t cmd)  
{
	CAN_TxHeaderTypeDef tx_message;
	
	tx_message.IDE = CAN_ID_STD;
	tx_message.RTR = CAN_RTR_DATA;
	tx_message.DLC = 0x08;
	
	uint32_t msg_box;
	uint8_t send_buf[8] = {0};
	
	// 配置仲裁段和数据段	
	tx_message.StdId = ID+0x100;  // 用于ID为 10X 的电机
	/// pack ints into the can buffer ///
	send_buf[0] = (uint8_t)(0xFF);
  send_buf[1] = (uint8_t)(0xFF);
	send_buf[2] = (uint8_t)(0xFF);
	send_buf[3] = (uint8_t)(0xFF);
	send_buf[4] = (uint8_t)(0xFF);
	send_buf[5] = (uint8_t)(0xFF);
	send_buf[6] = (uint8_t)(0xFF);
	
	switch(cmd)
	{
		case CMD_MOTOR_MODE:
			send_buf[7] = (uint8_t)(0xFC);break;
		case CMD_RESET_MODE:
			send_buf[7] = (uint8_t)(0xFD);break;
		case CMD_ZERO_POSITION:
      send_buf[7] = (uint8_t)(0xFE);break;
		default:
			return;
	}
	if (HAL_CAN_AddTxMessage(&hcan2,&tx_message,send_buf,&msg_box)!= HAL_OK) {
        // Failed to add message to the transmit mailbox
    }
}


