#include "SIMPLE_TASK.h"

RC9Protocol ros_serial(&huart2, true); // 九期的通用协议类，可用于ros调参，ros通讯,与esp32通讯等
action Action(&huart3, 0, 0);          // 还未封装完全，明天继续搞
TaskManager task_core;                 // 这是整个架构的地基

// C 的接口函数，调用 C++ 类来创建 FreeRTOS 任务
extern "C" void create_tasks(void)
{

    ros_serial.startUartReceiveIT();
    Action.startUartReceiveIT();            // 这两个都是基于串口设备基类的，需要用户调用一下startit，不然会有bug
    task_core.registerTask(0, &ros_serial); // 将实例注册到freertos任务0中，一个任务可以注册多个实例,action实例没有串口发送的需求，无需注册到任务中，只有有定期数据处理需求的实例，比如电机pid计算，滤波计算等实例才需要注册到任务管理核心中，action实例可以在中断中完成数据处理，不需依赖freertos任务
    osKernelStart();
} // 这几行代码就能搞定任务管理，ros收发,action读取和解算了，action还没封装完
