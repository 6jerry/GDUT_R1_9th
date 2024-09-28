#include "SIMPLE_TASK.h"

RC9Protocol ros_serial(&huart2, true); // 九期的通用协议类，可用于ros调参，ros通讯,与esp32通讯等
action Action(&huart3, 0, 0);
TaskManager task_core; // 这是整个架构的地基
CanManager can_core;
m3508 m3508_right(1, &hcan1, 19, 16.7f, 0.21f, 2.4f), m3508_front(3, &hcan1, 19, 16.5f, 0.18f, 2.6f), m3508_left(2, &hcan1, 19, 16.2f, 0.17f, 2.87f);

extern "C" void create_tasks(void)
{

    ros_serial.startUartReceiveIT();
    Action.startUartReceiveIT();
    can_core.init();
    m3508_right.target_rpm = 60;
    task_core.registerTask(4, &ros_serial);
    task_core.registerTask(0, &can_core);
    osKernelStart();
}
