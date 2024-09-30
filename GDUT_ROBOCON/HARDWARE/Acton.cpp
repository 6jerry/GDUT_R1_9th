#include "Action.h"

action::action(UART_HandleTypeDef *huart, float install_delta_x, float install_delta_y) : SerialDevice(huart), state_(WAITING_FOR_HEADER_0), rxIndex_(0)
{
    action_install_pos.delta_x = install_delta_x;
    action_install_pos.delta_y = install_delta_y;
}

void action::handleReceiveData(uint8_t byte)
{
    switch (state_)
    {
    case WAITING_FOR_HEADER_0:
        if (byte == FRAME_HEAD_ACTION_0) // 检查包头的第一个字节
        {
            state_ = WAITING_FOR_HEADER_1;
        }
        break;

    case WAITING_FOR_HEADER_1:
        if (byte == FRAME_HEAD_ACTION_1) // 检查包头的第二个字节
        {
            state_ = RECEIVING_DATA;
            rxIndex_ = 0; // 重置接收索引
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0; // 不是正确的包头，重新开始
        }
        break;

    case RECEIVING_DATA:
        rx_action_frame.temp_buff[rxIndex_++] = byte; // 保存接收到的字节
        if (rxIndex_ >= DATA_LENGTH_ACTION)           // 接收到24字节数据
        {
            state_ = WAITING_FOR_TAIL_0;
        }
        break;

    case WAITING_FOR_TAIL_0:
        if (byte == FRAME_TAIL_ACTION_0) // 检查包尾的第一个字节
        {
            state_ = WAITING_FOR_TAIL_1;
        }
        else
        {
            state_ = WAITING_FOR_HEADER_0; // 包尾不匹配，重新开始
        }
        break;

    case WAITING_FOR_TAIL_1:
        if (byte == FRAME_TAIL_ACTION_1) // 检查包尾的第二个字节
        {
            // 包接收完毕，解析数据
            for (uint8_t i = 0; i < DATA_LENGTH_ACTION; i++)
            {
                rx_action_frame.data.buff_msg[i] = rx_action_frame.temp_buff[i];
            }
            Update_Action_gl_position(rx_action_frame.data.msg_get);
        }
        state_ = WAITING_FOR_HEADER_0; // 处理完毕后重新等待新包
        break;

    default:
        state_ = WAITING_FOR_HEADER_0;
        break;
    }
}

void action::Update_Action_gl_position(float value[6])
{
    action_info.last_pos_x = action_info.now_pos_x;
    action_info.last_pos_y = action_info.now_pos_y;
    action_info.last_pos_z = action_info.now_pos_z;

    action_info.now_pos_z = value[0];
    action_info.now_pos_x = value[3];
    action_info.now_pos_y = value[4];

    action_info.delta_pos_x = action_info.now_pos_x - action_info.last_pos_x;
    action_info.delta_pos_y = action_info.now_pos_y - action_info.last_pos_y;
    action_info.delta_pos_z = action_info.now_pos_z - action_info.last_pos_z;

    action_info.pos_z_sum += action_info.delta_pos_z;
    pose_data.yaw_angle = -action_info.pos_z_sum;
    pose_data.yaw_rad = pose_data.yaw_angle * 0.01745f;
    action_info.pos_x_sum += action_info.delta_pos_x;
    action_info.pos_y_sum += action_info.delta_pos_y;
}

void action::restart()
{
    action_info.pos_z_sum = 0.0f;
    action_info.pos_x_sum = 0.0f;
    action_info.pos_y_sum = 0.0f;
}
void relocate(float x, float y)
{
    
}