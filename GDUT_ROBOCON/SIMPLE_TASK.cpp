#include "SIMPLE_TASK.h"

// 全局变量：两个任务实例，初始化时设置不同的执行频率
 SimpleTask task1("Task1", 128, tskIDLE_PRIORITY + 1, pdMS_TO_TICKS(100)); // 100ms 自增一次
SimpleTask task2("Task2", 128, tskIDLE_PRIORITY + 1, pdMS_TO_TICKS(1000));         // 500ms 自增一次

// 构造函数：初始化类时创建任务，并传入任务的执行频率
SimpleTask::SimpleTask(const char *taskName, uint16_t stackSize, UBaseType_t priority, TickType_t tickDelay)
    : taskName_(taskName), stackSize_(stackSize), priority_(priority), taskHandle_(nullptr), counter_(0), tickDelay_(tickDelay)
{
    // 创建任务，传递 this 指针作为参数
    xTaskCreate(taskEntry, taskName_, stackSize_, this, priority_, &taskHandle_);
}

// 析构函数：删除任务
SimpleTask::~SimpleTask()
{
    if (taskHandle_ != nullptr)
    {
        vTaskDelete(taskHandle_);
    }
}

// 静态任务入口函数，兼容 FreeRTOS 的任务入口
void SimpleTask::taskEntry(void *params)
{
    SimpleTask *taskInstance = static_cast<SimpleTask *>(params);
    taskInstance->taskFunction(); // 调用实际的任务函数
}

// 实际任务函数，根据传入的 tickDelay_ 控制任务频率
void SimpleTask::taskFunction()
{
    while (1)
    {
        counter_++;             // 自增计数器
        vTaskDelay(tickDelay_); // 根据 tickDelay_ 控制任务执行频率
    }
}

// C 的接口函数，调用 C++ 类来创建 FreeRTOS 任务
extern "C" void create_tasks(void)
{
    // 全局变量已经定义，任务实例已经创建，这里不需要再重复创建

    // 启动 FreeRTOS 调度器
    osKernelStart(); // 开始任务调度，不会返回
}