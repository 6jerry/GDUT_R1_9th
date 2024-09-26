#ifndef __SERIAL_TO_ESP32_H__
#define __SERIAL_TO_ESP32_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "crc_util.h"
#include <stdbool.h>
#include "MoveBase.h"
#include "motor.h"
#include "hardware.h"
#include "main.h"
#include "calculation.h"

#define FRAME_HEAD_0_ESP32 0xFC
#define FRAME_HEAD_1_ESP32 0xFB
#define FRAME_ID_ESP32 0x01 // 示例数据帧ID
#define FRAME_END_0_ESP32 0xFD
#define FRAME_END_1_ESP32 0xFE
#define MAX_DATA_LENGTH_ESP32 28

#define MAX_SHOOT_RPM_UP 4600.0f
#define MAX_SHOOT_RPM_DOWN 3600.0f

extern float MAX_ROBOT_SPEED_X;
extern float MAX_ROBOT_SPEED_Y;
extern float MAX_ROBOT_SPEED_W;
extern int head_locking_flag;   // 0是不锁死，1是锁死
extern int catch_ball_flag;     // 0是松开，1是夹紧
extern int world_robot_flag;    // 0是机器人坐标系控制，1是世界坐标系控制
extern int robot_stop_flag;     // 0是正常运行，1是触发急停
extern XboxControllerData_t xbox_msgs;
extern serial_frame_esp32_t rx_frame_esp32;

// 串口数据帧结构体
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

// Xbox手柄数据结构体
typedef struct
{
    // 按键数据（bool类型）
    bool btnY;
    bool btnY_last;
    bool btnB;
    bool btnB_last;
    bool btnA;
    bool btnA_last;
    bool btnX;
    bool btnX_last;
    bool btnShare;
    bool btnShare_last;
    bool btnStart;
    bool btnStart_last;
    bool btnSelect;
    bool btnSelect_last;
    bool btnXbox;
    bool btnXbox_last;
    bool btnLB;
    bool btnLB_last;
    bool btnRB;
    bool btnRB_last;
    bool btnLS;
    bool btnLS_last;
    bool btnRS;
    bool btnRS_last;
    bool btnDirUp;
    bool btnDirup_last;
    bool btnDirLeft;
    bool btnDirLeft_last;
    bool btnDirRight;
    bool btnDirRight_last;
    bool btnDirDown;
    bool btnDirDown_last;

    // 霍尔值（16位数值）
    uint16_t joyLHori;
    uint16_t joyLVert;
    uint16_t joyRHori;
    uint16_t joyRVert;
    uint16_t trigLT;
    uint16_t trigRT;
    // 映射百分比
    float joyLHori_map;
    float joyLVert_map;
    float joyRHori_map;
    float joyRVert_map;
    float trigLT_map;
    float trigRT_map;
} XboxControllerData_t;

// 按键控制模式枚举
typedef enum {
    FLAG_CHANGE,
    LOCK_HEAD,
    DECREASE_SPEED,
    INCREASE_SPEED
} ButtonAction;

uint8_t handle_serial_data_esp32(uint8_t byte);
void send_serial_frame_esp32(UART_HandleTypeDef *huart, uint8_t frame_id, uint8_t data_length, float *data);
void parseXboxData(uint8_t *xbox_datas, XboxControllerData_t *controllerData);
void xbox_remote_control();
float joydata_to_map(float joydata);
void detectButtonEdge(bool currentBtnState, bool *lastBtnState, int *toggleState, ButtonAction action);
void setRobotSpeed(uint8_t speed_level_);

#endif // __SERIAL_TO_ESP32_H__
