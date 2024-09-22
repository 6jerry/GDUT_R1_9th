#ifndef SERIAL_DEVICE_H
#define SERIAL_DEVICE_H

extern "C"
{

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <cmsis_os.h>
}

#ifdef __cplusplus
#define MAX_INSTANCES 10 // 最多支持 10 个串口实例

#define RX_BUFFER_SIZE 1 // 接收缓冲区大小

class SerialDevice
{
public:
    // 构造函数：传入 UART 句柄，自动启动接收中断
    SerialDevice(UART_HandleTypeDef *huart, bool enableSendTask = false);

    // 注册当前实例到全局实例数组中
    static void registerInstance(SerialDevice *instance);

    // 静态成员函数：处理 UART 接收中断
    static void handleRxCallback(UART_HandleTypeDef *huart, uint8_t rxByte);

    // 虚函数：派生类必须实现该方法，用于处理接收到的数据
    virtual void handleReceiveData(uint8_t byte) = 0;

    // 虚函数：派生类必须实现该方法，用于获取待发送的数据
    virtual const uint8_t *getSendData(size_t *length) = 0;

    // 全局 UART 中断回调函数（C 语言的函数，便于 HAL 调用）
    static void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

protected:
    UART_HandleTypeDef *huart_;        // 保存 UART 句柄
    uint8_t rxBuffer_[RX_BUFFER_SIZE]; // 接收缓冲区
    bool enableSendTask_;              // 是否启用 FreeRTOS 任务

    static SerialDevice *instances_[MAX_INSTANCES]; // 保存所有实例
    static int instanceCount_;

    static TaskHandle_t sendTaskHandle_; // 静态变量：发送任务句柄
    static bool sendTaskCreated_;        // 静态变量：是否已经创建了发送任务

    // FreeRTOS 任务函数，处理发送需求
    static void sendTaskFunction(void *params);

    // 自动启动发送任务（在构造函数中调用）
    static void autoStartSendTask();

    // 启用 UART 接收中断（在构造函数中调用）
    void startUartReceiveIT();
};
#endif

#endif // SERIAL_DEVICE_H
