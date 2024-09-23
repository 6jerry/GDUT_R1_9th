#ifndef RC9_PROTOCOL_H
#define RC9_PROTOCOL_H

#include "Serial_device.h"
#include "crc_util.h"
#include <cstring>

#define FRAME_HEAD_0_RC9 0xFC
#define FRAME_HEAD_1_RC9 0xFB
#define FRAME_END_0_RC9 0xFD
#define FRAME_END_1_RC9 0xFE
#define MAX_DATA_LENGTH_RC9 64
#ifdef __cplusplus
// 数据帧结构体定义，包含联合体
typedef struct serial_frame_mat
{
    uint8_t data_length; // 数据载荷的字节数
    uint8_t frame_head[2];
    uint8_t frame_id;
    uint16_t crc_calculated;
    uint8_t rx_temp_data_mat[MAX_DATA_LENGTH_RC9];
    union data
    {
        float msg_get[MAX_DATA_LENGTH_RC9];    // 用于浮点数的接收
        uint8_t buff_msg[MAX_DATA_LENGTH_RC9]; // 用于字节流的接收
    } data;
    union check_code
    {
        uint16_t crc_code;
        uint8_t crc_buff[2]; // CRC 校验的字节形式
    } check_code;
    uint8_t frame_end[2];
} serial_frame_mat_t;

class RC9Protocol : public SerialDevice
{
public:
    // 构造函数，传入 UART 句柄，是否启用发送任务，是否启用 CRC 校验
    RC9Protocol(UART_HandleTypeDef *huart, bool enableSendTask = false, bool enableCrcCheck = true);

    // 实现接收数据的处理逻辑
    void handleReceiveData(uint8_t byte) override;

    // 实现获取待发送的数据
    uint8_t *getSendData(size_t *length) override;

private:
    serial_frame_mat_t rx_frame_mat; // 接收数据的数据帧结构体
    serial_frame_mat_t tx_frame_mat; // 发送数据的数据帧结构体
    uint8_t sendBuffer_[MAX_DATA_LENGTH_RC9 + 8];
    uint8_t rxIndex_; // 当前接收到的字节的索引

    bool enableCrcCheck_; // 是否启用 CRC 校验

    // 状态机
    enum rxState
    {
        WAITING_FOR_HEADER_0,
        WAITING_FOR_HEADER_1,
        WAITING_FOR_ID,
        WAITING_FOR_LENGTH,
        WAITING_FOR_DATA,
        WAITING_FOR_CRC_0,
        WAITING_FOR_CRC_1,
        WAITING_FOR_END_0,
        WAITING_FOR_END_1
    } state_;
};
#endif
#endif // RC9_PROTOCOL_H
