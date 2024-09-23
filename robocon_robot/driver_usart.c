#include "driver_usart.h"
#include "usart.h"
#include "main.h"
#include "ring_buffer.h"
#include "pid_adjust.h"
#include "pid.h"
#include <stdio.h>
#include "calculation.h"
#include "hardware.h"
#include "communicate.h"
#include <stm32f407xx.h>

static volatile uint8_t txcplt_flag = 0; // 发送完成标志，1完成0未完成
static volatile uint8_t rx_data = 0;

float pos_x = 0;
float pos_y = 0;
float zangle = 0;
float xangle = 0;
float yangle = 0;
float w_z = 0;
int xx = 0;
int yy = 0;
int Action_State = 0;
QueueHandle_t uartQueue;
Usart_struct Usart2;
Usart_struct Usart6;

uint8_t USART2_Rx_buf[12];
uint8_t USART6_Rx_buf[12];

uint8_t rx_buffer1[1] = {0};
uint8_t rx_buffer2[1] = {0};
/*
 *  函数名：EnableDebugIRQ
 *  功能描述：使能USART1的中断+使能UART4的中断
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
 */
void EnableDebugIRQ(void)
{
	/*HAL_NVIC_SetPriority(USART1_IRQn, 5, 1); // 设置USART1中断的优先级
	HAL_NVIC_EnableIRQ(USART1_IRQn);		 // 使能USART1的中断

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); // 使能USRAT1的发送和接收中断

	//	HAL_NVIC_SetPriority(USART2_IRQn, 0, 4);    // 设置USART2中断的优先级
	//    HAL_NVIC_EnableIRQ(USART2_IRQn);            // 使能USART2的中断
	//
	//    __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC | UART_IT_RXNE);   // 使能USART2的发送和接收中断

	HAL_NVIC_SetPriority(USART3_IRQn, 0, 2); // 设置USART3中断的优先级
	HAL_NVIC_EnableIRQ(USART3_IRQn);		 // 使能USART3的中断

	__HAL_UART_ENABLE_IT(&huart3, UART_IT_TC | UART_IT_RXNE); // 使能USART3的发送和接收中断

	HAL_UART_Receive_IT(&huart3, RxBuffer_for4, 1); // 每接收一个数据，就打开一次串口中断接收

	HAL_NVIC_SetPriority(UART4_IRQn, 5, 0); // 设置UART4中断的优先级
	HAL_NVIC_EnableIRQ(UART4_IRQn);			// 使能UART4的中断

	__HAL_UART_ENABLE_IT(&huart4, UART_IT_TC | UART_IT_RXNE); // 使能Uuart4的发送和接收中断

	//	HAL_NVIC_SetPriority(USART6_IRQn, 0, 3);    // 设置USART6中断的优先级
	//    HAL_NVIC_EnableIRQ(USART6_IRQn);            // 使能USART6的中断
	//
	//    __HAL_UART_ENABLE_IT(&huart6, UART_IT_TC | UART_IT_RXNE);   // 使能USART2的发送和接收中断

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); // 开启USART2空闲中断
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE); // 开启USART2空闲中断

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, USART2_Rx_buf, RX_BUF_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, USART6_Rx_buf, RX_BUF_SIZE);*/

	// HAL_UART_Receive_IT(&huart1, rx_buffer1, 1);
	HAL_UART_Receive_IT(&huart2, rx_buffer2, 1);
	// HAL_UART_Receive_IT(&huart3, RxBuffer_for4, 1);
	uartQueue = xQueueCreate(1, sizeof(UART_Message));
}

/*
 *  函数名：DisableDebugIRQ
 *  功能描述：失能USART1的中断
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
 */
void DisableDebugIRQ(void)
{
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_TC | UART_IT_RXNE); // 失能USRAT1的发送和接收中断

	HAL_NVIC_DisableIRQ(USART1_IRQn); // 失能USART1的中断
}

/*
 *  函数名：fputc
 *  功能描述：printf/putchar 标准输出函数的底层输出函数
 *  输入参数：ch --> 要输出的数据
 *  输出参数：无
 *  返回值：无
 */
int fputc(int ch, FILE *f)
{
	// txcplt_flag = 0;
	// HAL_UART_Transmit_IT(&huart1, (uint8_t*)&ch, 1);
	// while(txcplt_flag==0);
	uint8_t data = (uint8_t)ch;
	// HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
	return ch;
}
/*
 *  函数名：fgetc
 *  功能描述：scanf/getchar 标准输出函数的底层输出函数
 *  输入参数：
 *  输出参数：无
 *  返回值：接收到的数据
 */
// int fgetc(FILE *f)
//{
// uint8_t data = (uint8_t)ch;
// HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
// return ch;
//}

/*
 *  函数名：USART1_IRQHandler
 *  功能描述：USART1的中断服务函数
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
 */
// void USART1_IRQHandler(void)
//{
// unsigned char c = 0;
// if((USART1->SR &(1<<5)) != 0)   // 判断USART1的状态寄存器的第五位即RXNE位是否被置位
//{
// c = USART1->DR; // RXNE=1，表明DR寄存器有值，就将它读出来保存到临时变量中；
// ring_buffer_write(c, &test_buffer); // 将数据保存到环形缓冲区中
//}
// HAL_UART_IRQHandler(&huart1);   // HAL库中的UART统一中断服务函数，通过形参判断是要处理谁的中断
//}

/*
 *  函数名：HAL_UART_RxCpltCallback
 *  功能描述：HAL库中的UART接收完成回调函数
 *  输入参数：huart --> UART的设备句柄，用以指明UART设备是哪一个UART
 *  输出参数：无
 *  返回值：无
 */
static uint8_t ch;
static union
{
	uint8_t data[24];
	float ActVal[6];
} posture;
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UART_Message msg;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; // freertos任务调度标志位
	if (huart->Instance == USART1)
	{
		msg.uart_id = UART_ID_USART1;
		msg.data = rx_buffer1[0];
		HAL_UART_Receive_IT(&huart1, rx_buffer1, 1);
		// BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		// xQueueOverwriteFromISR(uartQueue, &msg, &xHigherPriorityTaskWoken);
		handle_serial_data_esp32(rx_buffer1[0]);
	}

	else if (huart->Instance == USART3)
	{
		Action_State++;
		static uint8_t count = 0;
		static uint8_t i = 0;
		// 如果接收数据寄存器非空，代表USART3接收到了数据，可以读取
		// 处理接收到的数据
		ch = RxBuffer_for4[0];
		// 此时RxBuffer中存储着接收到的数据

		switch (count)
		{
		case 0:
		{
			if (ch == 0x0d)
				count++;
			else
				count = 0;
		}
		break;

		case 1:
		{
			if (ch == 0x0a)
			{
				i = 0;
				count++;
			}
			else if (ch == 0x0d)
				;
			else
				count = 0;
		}
		break;

		case 2:
		{
			posture.data[i] = ch;
			i++;
			if (i >= 24)
			{
				i = 0;
				count++;
			}
		}
		break;

		case 3:
		{
			if (ch == 0x0a)
				count++;
			else
				count = 0;
		}
		break;

		case 4:
		{
			if (ch == 0x0d)
			{
				// 更新传感器数据
				Update_Action_gl_position(posture.ActVal);
				// zangle = posture.ActVal[0];
				// xangle = posture.ActVal[1];
				// yangle = posture.ActVal[2];
				// pos_x = posture.ActVal[3];
				// pos_y = posture.ActVal[4];
				// w_z = posture.ActVal[5];
			}
			count = 0;
		}
		break;

		default:
		{
			count = 0;
		}
		break;
		}
		RxBuffer_for4[0] = 0;
		HAL_UART_Receive_IT(&huart3, RxBuffer_for4, 1); // 每接收一个数据，就打开一次串口中断接收
	}

	else if (huart->Instance == USART2)
	{

		// uart4_ReceiveData(&No1_Data.WOrld_x, &No1_Data.WOrld_y, &No1_Data.WOrld_w, &No1_Data.flag);
		msg.uart_id = UART_ID_UART4;
		msg.data = rx_buffer2[0];
		uint32_t mat_id = handle_serial_data_mat(rx_buffer2[0]);
		if (mat_id == 0x01)
		{
			// heading_lock.setpoint = rx_frame_mat.data.msg_get[0];
			// PID_SetParameters(&heading_lock, rx_frame_mat.data.msg_get[1], rx_frame_mat.data.msg_get[2], rx_frame_mat.data.msg_get[3]);
		}
		// HAL_UART_Receive_IT(&huart2, rx_buffer2, 1);
		//  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		//  xQueueOverwriteFromISR(uartQueue, &msg, &xHigherPriorityTaskWoken);
		//  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		//		No1_Porcessing(No1_Data);
		/*Remote_Process();
		//	   __HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);

		__HAL_UNLOCK(&huart4);
		__HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_PE); // 清标志
		__HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_FE);
		__HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_NE);
		__HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_ORE);*/
//}
// portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}

/*
 *  函数名：HAL_UARTEx_RxEventCallback
 *  功能描述：DMA空闲回调函数
 *  输入参数：huart --> UART的设备句柄，用以指明UART设备是哪一个UART
 *  输出参数：无
 *  返回值：无
 */
/*void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	if (huart == &huart6)
	{

		processData(USART6_Rx_buf, &Laser_Real_Data, 6);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, USART6_Rx_buf, Max_BUFF_Len);
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT); // 关闭DMA半传输中断
	}

	else if (huart == &huart2)
	{
		processData(USART2_Rx_buf, &Laser_Real_Data, 2);

		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, USART2_Rx_buf, Max_BUFF_Len);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // 关闭DMA半传输中断
	}
}*/
