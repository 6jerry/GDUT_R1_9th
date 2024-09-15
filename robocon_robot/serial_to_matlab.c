#include "serial_to_matlab.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

// 数据帧结构体实例
serial_frame_mat_t tx_frame_mat;
serial_frame_mat_t rx_frame_mat;

// 状态机状态定义
typedef enum
{
    WAITING_FOR_HEADER_0_MAT,
    WAITING_FOR_HEADER_1_MAT,
    WAITING_FOR_ID_MAT,
    WAITING_FOR_LENGTH_MAT,
    WAITING_FOR_DATA_MAT,
    WAITING_FOR_CRC_0_MAT,
    WAITING_FOR_CRC_1_MAT,
    WAITING_FOR_END_0_MAT,
    WAITING_FOR_END_1_MAT
} rx_state_mat_t;

// 当前状态机状态
static rx_state_mat_t rx_state_mat = WAITING_FOR_HEADER_0_MAT;
// 数据索引
static uint8_t rx_index_mat = 0;
// 临时接收数据存储
static uint8_t rx_temp_data_mat[MAX_DATA_LENGTH_MAT * 4];

// 数据处理函数，使用状态机实现接收解包
uint8_t handle_serial_data_mat(uint8_t byte)
{
    switch (rx_state_mat)
    {
    case WAITING_FOR_HEADER_0_MAT:
        if (byte == FRAME_HEAD_0_MAT)
        {
            rx_frame_mat.frame_head[0] = byte;
            rx_state_mat = WAITING_FOR_HEADER_1_MAT;
        }
        break;

    case WAITING_FOR_HEADER_1_MAT:
        if (byte == FRAME_HEAD_1_MAT)
        {
            rx_frame_mat.frame_head[1] = byte;
            rx_state_mat = WAITING_FOR_ID_MAT;
        }
        else
        {
            rx_state_mat = WAITING_FOR_HEADER_0_MAT;
        }
        break;

    case WAITING_FOR_ID_MAT:
        rx_frame_mat.frame_id = byte;
        rx_state_mat = WAITING_FOR_LENGTH_MAT;
        break;

    case WAITING_FOR_LENGTH_MAT:
        rx_frame_mat.data_length = byte;
        rx_index_mat = 0;
        rx_state_mat = WAITING_FOR_DATA_MAT;
        break;

    case WAITING_FOR_DATA_MAT:
        rx_temp_data_mat[rx_index_mat++] = byte;
        if (rx_index_mat >= rx_frame_mat.data_length * 4)
        {
            rx_state_mat = WAITING_FOR_CRC_0_MAT;
        }
        break;

    case WAITING_FOR_CRC_0_MAT:
        rx_frame_mat.check_code.crc_buff[0] = byte;
        rx_state_mat = WAITING_FOR_CRC_1_MAT;
        break;

    case WAITING_FOR_CRC_1_MAT:
        rx_frame_mat.check_code.crc_buff[1] = byte;
        rx_state_mat = WAITING_FOR_END_0_MAT;
        break;

    case WAITING_FOR_END_0_MAT:
        if (byte == FRAME_END_0_MAT)
        {
            rx_frame_mat.frame_end[0] = byte;
            rx_state_mat = WAITING_FOR_END_1_MAT;
        }
        else
        {
            rx_state_mat = WAITING_FOR_HEADER_0_MAT;
        }
        break;

    case WAITING_FOR_END_1_MAT:
        if (byte == FRAME_END_1_MAT)
        {
            rx_frame_mat.frame_end[1] = byte;
            uint16_t received_crc = rx_frame_mat.check_code.crc_code;
            uint16_t calculated_crc = CRC16_Table(rx_temp_data_mat, rx_frame_mat.data_length * 4);
            rx_frame_mat.crc_calculated = calculated_crc;
            if (received_crc == calculated_crc)
            {
                for (uint8_t i = 0; i < rx_frame_mat.data_length * 4; i++)
                {
                    rx_frame_mat.data.buff_msg[i] = rx_temp_data_mat[i];
                }
                rx_state_mat = WAITING_FOR_HEADER_0_MAT;
                return rx_frame_mat.frame_id;
            }
        }
        rx_state_mat = WAITING_FOR_HEADER_0_MAT;
        break;

    default:
        rx_state_mat = WAITING_FOR_HEADER_0_MAT;
        break;
    }
    return 0;
}

// 发送数据帧的函数
void send_serial_frame_mat(UART_HandleTypeDef *huart, uint8_t frame_id, uint8_t data_length, float *data)
{
    uint8_t buff_msg[data_length * 4 + 8];
    tx_frame_mat.frame_head[0] = FRAME_HEAD_0_MAT;
    tx_frame_mat.frame_head[1] = FRAME_HEAD_1_MAT;
    tx_frame_mat.frame_id = frame_id;
    tx_frame_mat.data_length = data_length;

    for (int i = 0; i < data_length; i++)
    {
        tx_frame_mat.data.msg_get[i] = data[i];
    }

    buff_msg[0] = FRAME_HEAD_0_MAT;
    buff_msg[1] = FRAME_HEAD_1_MAT;
    buff_msg[2] = frame_id;
    buff_msg[3] = data_length;

    for (int q = 0; q < data_length * 4; q++)
    {
        buff_msg[4 + q] = tx_frame_mat.data.buff_msg[q];
    }

    tx_frame_mat.check_code.crc_code = CRC16_Table(tx_frame_mat.data.buff_msg, data_length * 4);
    buff_msg[4 + data_length * 4] = tx_frame_mat.check_code.crc_buff[0];
    buff_msg[5 + data_length * 4] = tx_frame_mat.check_code.crc_buff[1];
    buff_msg[6 + data_length * 4] = FRAME_END_0_MAT;
    buff_msg[7 + data_length * 4] = FRAME_END_1_MAT;

    // 使用UART发送数据
    HAL_UART_Transmit(huart, buff_msg, sizeof(buff_msg), HAL_MAX_DELAY);
}
