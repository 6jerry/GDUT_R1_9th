#ifndef SIMPLE_TASK_H
#define SIMPLE_TASK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "task.h"
#include <cstdint>
#include <cmsis_os.h>
#include "RC9Protocol.h"
#include "usart.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class SimpleTask
{
public:
    // 构造函数：初始化时创建任务，并传入任务的执行频率
    SimpleTask(const char *taskName, uint16_t stackSize, UBaseType_t priority, TickType_t tickDelay);

    // 析构函数：删除任务
    ~SimpleTask();

    // 获取任务自增计数器的值，用于调试
    uint32_t getCounter() const { return counter_; }

private:
    const char *taskName_;    // 任务名称
    uint16_t stackSize_;      // 堆栈大小
    UBaseType_t priority_;    // 优先级
    TaskHandle_t taskHandle_; // 任务句柄
    uint32_t counter_;        // 自增计数器
    TickType_t tickDelay_;    // 任务执行的延迟时间（影响任务频率）

    // 静态任务入口函数
    static void taskEntry(void *params);

    // 实际的任务函数
    void taskFunction();
};

// C 的接口函数，用于在 main.c 中调用
extern "C" void create_tasks(void);

#endif // SIMPLE_TASK_H

#endif
