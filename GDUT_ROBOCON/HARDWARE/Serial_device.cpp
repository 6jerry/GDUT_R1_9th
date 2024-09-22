#include "Serial_device.h"

// 静态变量初始化
SerialDevice *SerialDevice::instances_[MAX_INSTANCES] = {nullptr};
int SerialDevice::instanceCount_ = 0;
TaskHandle_t SerialDevice::sendTaskHandle_ = nullptr;
bool SerialDevice::sendTaskCreated_ = false;

// 构造函数：传入 UART 句柄，自动启动接收中断
SerialDevice::SerialDevice(UART_HandleTypeDef *huart, bool enableSendTask)
    : huart_(huart), enableSendTask_(enableSendTask)
{
    // 注册当前实例到全局数组
    if (instanceCount_ < MAX_INSTANCES)
    {
        registerInstance(this);
    }

    // 自动启动 UART 接收中断
    startUartReceiveIT();

    // 自动启动发送任务
    autoStartSendTask();
}

// 注册当前实例
void SerialDevice::registerInstance(SerialDevice *instance)
{
    instances_[instanceCount_++] = instance;
}

// 启用 UART 接收中断
void SerialDevice::startUartReceiveIT()
{
    // 启用 UART 接收中断
    HAL_UART_Receive_IT(huart_, rxBuffer_, RX_BUFFER_SIZE);
}

// 静态成员函数：处理 UART 接收中断
void SerialDevice::handleRxCallback(UART_HandleTypeDef *huart, uint8_t rxByte)
{
    // 遍历所有实例，找到对应的 UART 实例
    for (int i = 0; i < instanceCount_; i++)
    {
        if (instances_[i]->huart_ == huart)
        {
            // 调用该实例的接收处理逻辑
            instances_[i]->handleReceiveData(rxByte);
        }
    }
}

// 全局回调函数：HAL 库调用该函数
extern "C" void SerialDevice::HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t rxByte;
    // 获取 UART 接收的字节
    for (int i = 0; i < instanceCount_; i++)
    {
        if (instances_[i]->huart_ == huart)
        {
            rxByte = instances_[i]->rxBuffer_[0]; // 读取接收缓冲区中的字节
            // 调用静态成员函数来处理接收数据
            SerialDevice::handleRxCallback(huart, rxByte);
            // 再次启用 UART 接收中断
            HAL_UART_Receive_IT(huart, instances_[i]->rxBuffer_, RX_BUFFER_SIZE);
        }
    }
}

// 自动启动发送任务
void SerialDevice::autoStartSendTask()
{
    if (!sendTaskCreated_)
    {
        sendTaskCreated_ = true;
        // 创建 FreeRTOS 任务（只创建一次）
        xTaskCreate(sendTaskFunction, "SendTask", 128 * 4, nullptr, osPriorityNormal, &sendTaskHandle_);
    }
}

// FreeRTOS 任务函数：遍历所有启用了发送任务的实例，处理发送逻辑
void SerialDevice::sendTaskFunction(void *params)
{
    const uint16_t delayTime = pdMS_TO_TICKS(5); // 固定任务延时为 5ms

    while (1)
    {
        // 遍历所有启用了发送任务的实例
        for (int i = 0; i < instanceCount_; i++)
        {
            if (instances_[i]->enableSendTask_)
            {
                size_t dataLength = 0;
                // 获取待发送的数据
                const uint8_t *data = instances_[i]->getSendData(&dataLength);

                // 如果有数据则发送
                if (data != nullptr && dataLength > 0)
                {
                    HAL_UART_Transmit(instances_[i]->huart_, (uint8_t *)data, dataLength, HAL_MAX_DELAY);
                }
            }
        }

        // 按照 5ms 延时进行任务调度
        vTaskDelay(delayTime);
    }
}