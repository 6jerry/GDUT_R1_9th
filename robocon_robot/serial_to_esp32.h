#ifndef __SERIAL_TO_ESP32_H__
#define __SERIAL_TO_ESP32_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "crc_util.h"
#include <stdbool.h>
#include "MoveBase.h"
#define FRAME_HEAD_0_ESP32 0xFC
#define FRAME_HEAD_1_ESP32 0xFB
#define FRAME_ID_ESP32 0x01 // 示例数据帧ID
#define FRAME_END_0_ESP32 0xFD
#define FRAME_END_1_ESP32 0xFE
#define MAX_DATA_LENGTH_ESP32 28

typedef struct serial_frame_esp32
{
    uint8_t data_length;
    uint8_t frame_head[2];
    uint8_t frame_id;
    uint16_t crc_calculated;
    union data_buff
    {
        // float msg_get[MAX_DATA_LENGTH_ESP32];
        uint8_t buff_msg[MAX_DATA_LENGTH_ESP32];
    } data_buff;
    union check_code_esp
    {
        uint16_t crc_code;
        uint8_t crc_buff[2];
    } check_code_e;
    uint8_t frame_end[2];
} serial_frame_esp32_t;
typedef struct
{
    // 按键数据（bool类型）
    bool btnY;
    bool btnB;
    bool btnA;
    bool btnX;
    bool btnShare;
    bool btnStart;
    bool btnSelect;
    bool btnXbox;
    bool btnLB;
    bool btnRB;
    bool btnLS;
    bool btnRS;
    bool btnDirUp;
    bool btnDirLeft;
    bool btnDirRight;
    bool btnDirDown;

    // 霍尔值（16位数值）
    uint16_t joyLHori;
    uint16_t joyLVert;
    uint16_t joyRHori;
    uint16_t joyRVert;
    uint16_t trigLT;
    uint16_t trigRT;
} XboxControllerData_t;

uint8_t handle_serial_data_esp32(uint8_t byte);
void send_serial_frame_esp32(UART_HandleTypeDef *huart, uint8_t frame_id, uint8_t data_length, float *data);
void parseXboxData(uint8_t *xbox_datas, XboxControllerData_t *controllerData);
void xbox_remote_control();
extern XboxControllerData_t xbox_msgs;
extern serial_frame_esp32_t rx_frame_esp32;

#endif // __SERIAL_TO_ESP32_H__
