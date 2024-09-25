#include "serial_to_esp32.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <main.h>

// 数据帧结构体实例
serial_frame_esp32_t tx_frame_esp32;
serial_frame_esp32_t rx_frame_esp32;
XboxControllerData_t xbox_msgs;
float MAX_ROBOT_SPEED_Y = 1.50f;
float MAX_ROBOT_SPEED_X = 1.50f;
float locking_heading = 0.0f;
float MAX_ROBOT_SPEED_W = 3.60f;
int catch_ball_flag = 0;
int world_robot_flag = 0;
int robot_stop_flag = 0;
uint8_t speed_level = 1; // 0---低速，1---中速，2---高速
// 状态机状态定义
typedef enum
{
    WAITING_FOR_HEADER_0_ESP32,
    WAITING_FOR_HEADER_1_ESP32,
    WAITING_FOR_ID_ESP32,
    WAITING_FOR_LENGTH_ESP32,
    WAITING_FOR_DATA_ESP32,
    WAITING_FOR_CRC_0_ESP32,
    WAITING_FOR_CRC_1_ESP32,
    WAITING_FOR_END_0_ESP32,
    WAITING_FOR_END_1_ESP32
} rx_state_esp32_t;

// 当前状态机状态
static rx_state_esp32_t rx_state_esp32 = WAITING_FOR_HEADER_0_ESP32;
// 数据索引
static uint8_t rx_index_esp32 = 0;
// 临时接收数据存储
static uint8_t rx_temp_data_esp32[MAX_DATA_LENGTH_ESP32 * 4];

// 数据处理函数，使用状态机实现接收解包
uint8_t handle_serial_data_esp32(uint8_t byte)
{
    switch (rx_state_esp32)
    {
    case WAITING_FOR_HEADER_0_ESP32:
        if (byte == FRAME_HEAD_0_ESP32)
        {
            rx_frame_esp32.frame_head[0] = byte;
            rx_state_esp32 = WAITING_FOR_HEADER_1_ESP32;
        }
        break;

    case WAITING_FOR_HEADER_1_ESP32:
        if (byte == FRAME_HEAD_1_ESP32)
        {
            rx_frame_esp32.frame_head[1] = byte;
            rx_state_esp32 = WAITING_FOR_ID_ESP32;
        }
        else
        {
            rx_state_esp32 = WAITING_FOR_HEADER_0_ESP32;
        }
        break;

    case WAITING_FOR_ID_ESP32:
        rx_frame_esp32.frame_id = byte;
        rx_state_esp32 = WAITING_FOR_LENGTH_ESP32;
        break;

    case WAITING_FOR_LENGTH_ESP32:
        rx_frame_esp32.data_length = byte;
        rx_index_esp32 = 0;
        rx_state_esp32 = WAITING_FOR_DATA_ESP32;
        break;

    case WAITING_FOR_DATA_ESP32:
        rx_temp_data_esp32[rx_index_esp32++] = byte;
        if (rx_index_esp32 >= rx_frame_esp32.data_length)
        {
            rx_state_esp32 = WAITING_FOR_CRC_0_ESP32;
        }
        break;

    case WAITING_FOR_CRC_0_ESP32:
        rx_frame_esp32.check_code_e.crc_buff[0] = byte;
        rx_state_esp32 = WAITING_FOR_CRC_1_ESP32;
        break;

    case WAITING_FOR_CRC_1_ESP32:
        rx_frame_esp32.check_code_e.crc_buff[1] = byte;
        rx_state_esp32 = WAITING_FOR_END_0_ESP32;
        break;

    case WAITING_FOR_END_0_ESP32:
        if (byte == FRAME_END_0_ESP32)
        {
            rx_frame_esp32.frame_end[0] = byte;
            rx_state_esp32 = WAITING_FOR_END_1_ESP32;
        }
        else
        {
            rx_state_esp32 = WAITING_FOR_HEADER_0_ESP32;
        }
        break;

    case WAITING_FOR_END_1_ESP32:
        if (byte == FRAME_END_1_ESP32)
        {
            rx_frame_esp32.frame_end[1] = byte;
            uint16_t received_crc = rx_frame_esp32.check_code_e.crc_code;
            // uint16_t calculated_crc = CRC16_Table(rx_temp_data_esp32, rx_frame_esp32.data_length);
            // rx_frame_esp32.crc_calculated = calculated_crc;
            if (1)
            {
                for (uint8_t i = 0; i < rx_frame_esp32.data_length; i++)
                {
                    rx_frame_esp32.data_buff.buff_msg[i] = rx_temp_data_esp32[i];
                }
                parseXboxData(rx_frame_esp32.data_buff.buff_msg, &xbox_msgs);
                rx_state_esp32 = WAITING_FOR_HEADER_0_ESP32;
                return rx_frame_esp32.frame_id;
            }
        }
        rx_state_esp32 = WAITING_FOR_HEADER_0_ESP32;
        break;

    default:
        rx_state_esp32 = WAITING_FOR_HEADER_0_ESP32;
        break;
    }
    return 0;
}

// 发送数据帧的函数
void send_serial_frame_esp32(UART_HandleTypeDef *huart, uint8_t frame_id, uint8_t data_length, float *data)
{
    uint8_t buff_msg[data_length * 4 + 8];
    tx_frame_esp32.frame_head[0] = FRAME_HEAD_0_ESP32;
    tx_frame_esp32.frame_head[1] = FRAME_HEAD_1_ESP32;
    tx_frame_esp32.frame_id = frame_id;
    tx_frame_esp32.data_length = data_length;

    for (int i = 0; i < data_length; i++)
    {
        // tx_frame_esp32.data.msg_get[i] = data[i];
    }

    buff_msg[0] = FRAME_HEAD_0_ESP32;
    buff_msg[1] = FRAME_HEAD_1_ESP32;
    buff_msg[2] = frame_id;
    buff_msg[3] = data_length;

    for (int q = 0; q < data_length * 4; q++)
    {
        buff_msg[4 + q] = tx_frame_esp32.data_buff.buff_msg[q];
    }

    tx_frame_esp32.check_code_e.crc_code = CRC16_Table(tx_frame_esp32.data_buff.buff_msg, data_length * 4);
    buff_msg[4 + data_length * 4] = tx_frame_esp32.check_code_e.crc_buff[0];
    buff_msg[5 + data_length * 4] = tx_frame_esp32.check_code_e.crc_buff[1];
    buff_msg[6 + data_length * 4] = FRAME_END_0_ESP32;
    buff_msg[7 + data_length * 4] = FRAME_END_1_ESP32;

    // 使用UART发送数据
    HAL_UART_Transmit(huart, buff_msg, sizeof(buff_msg), HAL_MAX_DELAY);
}
void parseXboxData(uint8_t *xbox_datas, XboxControllerData_t *controllerData)
{
    // 解析按键数据 (bool 值)
    controllerData->btnY = xbox_datas[0];
    controllerData->btnB = xbox_datas[1];
    controllerData->btnA = xbox_datas[2];
    controllerData->btnX = xbox_datas[3];
    controllerData->btnShare = xbox_datas[4];
    controllerData->btnStart = xbox_datas[5];
    controllerData->btnSelect = xbox_datas[6];
    controllerData->btnXbox = xbox_datas[7];
    controllerData->btnLB = xbox_datas[8];
    controllerData->btnRB = xbox_datas[9];
    controllerData->btnLS = xbox_datas[10];
    controllerData->btnRS = xbox_datas[11];
    controllerData->btnDirUp = xbox_datas[12];
    controllerData->btnDirLeft = xbox_datas[13];
    controllerData->btnDirRight = xbox_datas[14];
    controllerData->btnDirDown = xbox_datas[15];

    // 解析霍尔传感器值（16位数据，高8位和低8位拼接）
    controllerData->joyLHori = ((uint16_t)xbox_datas[16] << 8) | xbox_datas[17];
    controllerData->joyLVert = ((uint16_t)xbox_datas[18] << 8) | xbox_datas[19];
    controllerData->joyRHori = ((uint16_t)xbox_datas[20] << 8) | xbox_datas[21];
    controllerData->joyRVert = ((uint16_t)xbox_datas[22] << 8) | xbox_datas[23];
    controllerData->trigLT = ((uint16_t)xbox_datas[24] << 8) | xbox_datas[25];
    controllerData->trigRT = ((uint16_t)xbox_datas[26] << 8) | xbox_datas[27];
}
int head_locking_flag = 0;
void xbox_remote_control()
{
    detectButtonEdgeRb(xbox_msgs.btnRB, &xbox_msgs.btnRB_last, &head_locking_flag, 1);
    detectButtonEdge(xbox_msgs.btnLS, &xbox_msgs.btnLS_last, &robot_stop_flag, 1);
    detectButtonEdge(xbox_msgs.btnRS, &xbox_msgs.btnRS_last, &world_robot_flag, 1);
    detectButtonEdge(xbox_msgs.btnLB, &xbox_msgs.btnLB_last, &catch_ball_flag, 1);
    detectButtonEdgeD(xbox_msgs.btnX, &xbox_msgs.btnX_last);
    detectButtonEdgeI(xbox_msgs.btnB, &xbox_msgs.btnB_last);
    if (speed_level == 1)
    {
        MAX_ROBOT_SPEED_X = 1.20f;
        MAX_ROBOT_SPEED_Y = 1.20f;
        MAX_ROBOT_SPEED_W = 3.20f;
    }
    if (speed_level == 0)
    {
        MAX_ROBOT_SPEED_X = 0.40f;
        MAX_ROBOT_SPEED_Y = 0.40f;
        MAX_ROBOT_SPEED_W = 1.10f;
    }
    if (speed_level == 2)
    {
        MAX_ROBOT_SPEED_X = 1.96f;
        MAX_ROBOT_SPEED_Y = 1.96f;
        MAX_ROBOT_SPEED_W = 3.98f;
    }
    if (xbox_msgs.btnXbox == 1)
    {
        action_relocate();
    }
    if (xbox_msgs.joyLHori > 31000 && xbox_msgs.joyLHori < 350000)
    {
        xbox_msgs.joyLHori_map = 0.0f;
    }
    if (xbox_msgs.joyLHori <= 31000)
    {
        xbox_msgs.joyLHori_map = (31000.0f - (float)xbox_msgs.joyLHori) / 31000.0f;
    }
    if (xbox_msgs.joyLHori >= 35000)
    {
        xbox_msgs.joyLHori_map = (35000.0f - (float)xbox_msgs.joyLHori) / 30535.0f;
    }

    if (xbox_msgs.joyLVert > 31000 && xbox_msgs.joyLVert < 350000)
    {
        xbox_msgs.joyLVert_map = 0.0f;
    }
    if (xbox_msgs.joyLVert <= 31000)
    {
        xbox_msgs.joyLVert_map = (31000.0f - (float)xbox_msgs.joyLVert) / 31000.0f;
    }
    if (xbox_msgs.joyLVert >= 35000)
    {
        xbox_msgs.joyLVert_map = (35000.0f - (float)xbox_msgs.joyLVert) / 30535.0f;
    }

    if (xbox_msgs.joyRHori > 31000 && xbox_msgs.joyRHori < 35000)
    {
        xbox_msgs.joyRHori_map = 0.0f;
    }
    if (xbox_msgs.joyRHori <= 31000)
    {
        xbox_msgs.joyRHori_map = (31000.0f - (float)xbox_msgs.joyRHori) / 31000.0f;
    }
    if (xbox_msgs.joyRHori >= 35000)
    {
        xbox_msgs.joyRHori_map = (35000.0f - (float)xbox_msgs.joyRHori) / 30535.0f;
    }

    if (xbox_msgs.trigRT == 0)
    {
        shoot_down_left.setpoint = 0;
        shoot_down_right.setpoint = 0;
        shoot_up_left.setpoint = 0;
        shoot_up_right.setpoint = 0;
    }
    if (xbox_msgs.trigRT > 0)
    {
        xbox_msgs.trigRT_map = (float)xbox_msgs.trigRT / 1023.0f;
        xbox_msgs.trigLT_map = (float)xbox_msgs.trigLT / 1023.0f;
        shoot_down_left.setpoint = MAX_SHOOT_RPM_DOWN * xbox_msgs.trigRT_map;
        shoot_down_right.setpoint = -shoot_down_left.setpoint;

        shoot_up_left.setpoint = MAX_SHOOT_RPM_UP * xbox_msgs.trigRT_map;
        shoot_up_right.setpoint = -shoot_up_left.setpoint;
    }
    if (catch_ball_flag == 0)
    {
        HAL_GPIO_WritePin(shoot_key_GPIO_Port, shoot_key_Pin, GPIO_PIN_RESET);
    }
    if (catch_ball_flag == 1)
    {
        HAL_GPIO_WritePin(shoot_key_GPIO_Port, shoot_key_Pin, GPIO_PIN_SET);
    }
    if (world_robot_flag == 0 && robot_stop_flag == 0)
    {
        Robot_Chassis.Robot_V[1] = MAX_ROBOT_SPEED_X * xbox_msgs.joyLHori_map;
        Robot_Chassis.Robot_V[0] = MAX_ROBOT_SPEED_Y * xbox_msgs.joyLVert_map;
    }
    if (world_robot_flag == 1 && robot_stop_flag == 0)
    {
        Robot_Chassis.World_V[1] = MAX_ROBOT_SPEED_X * xbox_msgs.joyLHori_map;
        Robot_Chassis.World_V[0] = MAX_ROBOT_SPEED_Y * xbox_msgs.joyLVert_map;
        world_tf();
    }
    if (robot_stop_flag == 1)
    {
        Robot_Chassis.Robot_V[1] = 0.0f;
        Robot_Chassis.Robot_V[0] = 0.0f;
    }
    if (head_locking_flag == 0)
    {
        Robot_Chassis.Robot_V[2] = -MAX_ROBOT_SPEED_W * xbox_msgs.joyRHori_map;
    }
    if (head_locking_flag == 1)
    {
        if (xbox_msgs.btnDirUp == 1)
        {
            heading_lock.setpoint = 0.0f;
        }
        if (xbox_msgs.btnDirUp != 1)
        {
            heading_lock.setpoint = locking_heading;
        }
        else if (xbox_msgs.btnDirLeft == 1)
        {
            // heading_lock.setpoint = -1.5705f;
        }
        else if (xbox_msgs.btnDirRight == 1)
        {
            // heading_lock.setpoint = 1.5705f;
        }
        else if (xbox_msgs.btnDirDown == 1)
        {
            // heading_lock.setpoint = 3.0f;
        }
    }
    // Robot_Chassis.Robot_V[2] = -MAX_ROBOT_SPEED_W * xbox_msgs.joyRHori_map;
}

void detectButtonEdge(bool currentBtnState, bool *lastBtnState, int *toggleState, int maxState)
{
    if (currentBtnState && !(*lastBtnState))
    { // 检测到上升沿
        *toggleState = (*toggleState + 1) % (maxState + 1);
        // locking_heading = ROBOT_REAL_POS_DATA.POS_YAW_RAD;
    }
    *lastBtnState = currentBtnState;
}
void detectButtonEdgeRb(bool currentBtnState, bool *lastBtnState, int *toggleState, int maxState)
{
    if (currentBtnState && !(*lastBtnState))
    { // 检测到上升沿
        *toggleState = (*toggleState + 1) % (maxState + 1);
        locking_heading = ROBOT_REAL_POS_DATA.POS_YAW_RAD;
    }
    *lastBtnState = currentBtnState;
}
void detectButtonEdgeD(bool currentBtnState, bool *lastBtnState)
{

    if (currentBtnState && !(*lastBtnState))
    { // 检测到上升沿
        //*toggleState = (*toggleState + 1) % (maxState + 1);
        // locking_heading = ROBOT_REAL_POS_DATA.POS_YAW_RAD;
        if (speed_level > 0)
        {
            speed_level--;
        }
    }
    *lastBtnState = currentBtnState;
}
void detectButtonEdgeI(bool currentBtnState, bool *lastBtnState)
{
    if (currentBtnState && !(*lastBtnState))
    { // 检测到上升沿
        //*toggleState = (*toggleState + 1) % (maxState + 1);
        // locking_heading = ROBOT_REAL_POS_DATA.POS_YAW_RAD;
        if (speed_level < 2)
        {
            speed_level++;
        }
    }
    *lastBtnState = currentBtnState;
}
