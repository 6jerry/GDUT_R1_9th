#include "SIMPLE_TASK.h"

RC9Protocol ros_serial(&huart2, true);
action Action(&huart3, 0, 0);
TaskManager task_core;

// C 的接口函数，调用 C++ 类来创建 FreeRTOS 任务
extern "C" void create_tasks(void)
{
    // 全局变量已经定义，任务实例已经创建，这里不需要再重复创建
    ros_serial.startUartReceiveIT();
    Action.startUartReceiveIT();
    task_core.registerTask(0, &ros_serial);
    osKernelStart(); // 开始任务调度，不会返回
    // ros_serial.startUartReceiveIT();
    // 启动 FreeRTOS 调度器
}