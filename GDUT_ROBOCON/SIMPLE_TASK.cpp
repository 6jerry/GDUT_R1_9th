#include "SIMPLE_TASK.h"

RC9Protocol ros_serial(&huart2, true); // 九期的通用协议类，可用于ros调参，ros通讯,与esp32通讯等
action Action(&huart3, 0, 0);
TaskManager task_core; // 这是整个架构的地基
CanManager can_core;
m3508 m3508_right(1, &hcan1, 19, 16.7f, 0.21f, 2.4f), m3508_front(3, &hcan1, 19, 16.5f, 0.18f, 2.6f), m3508_left(2, &hcan1, 19, 16.2f, 0.17f, 2.87f);

omni3_unusual r1_chassis(&m3508_front, &m3508_right, &m3508_left, &Action, 7.0f, 0.0f, 0.7f); // 组装r1的底盘

extern "C" void create_tasks(void)
{

    ros_serial.startUartReceiveIT();
    Action.startUartReceiveIT();
    can_core.init();
    r1_chassis.switch_chassis_mode(remote_robotv);
    r1_chassis.unlock();
    r1_chassis.setrobotv(0.0f, 0.4f, 0.0f);
    task_core.registerTask(4, &ros_serial);
    task_core.registerTask(0, &can_core);
    task_core.registerTask(4, &r1_chassis);
    osKernelStart();
}
