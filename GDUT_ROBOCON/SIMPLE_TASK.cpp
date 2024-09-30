#include "SIMPLE_TASK.h"

RC9Protocol ros_serial(&huart2, true), esp32_serial(&huart1, false); // 九期的通用协议类，可用于ros调参，ros通讯,与esp32通讯等
action Action(&huart3, 0, 0);
TaskManager task_core; // 在这里初始化任务核心不会创建任何任务，只有后面首次注册指定的实例到指定的任务中才会创建任务
CanManager can_core;
m3508 m3508_right(1, &hcan1, 19, 16.7f, 0.21f, 2.4f), m3508_front(3, &hcan1, 19, 16.5f, 0.18f, 2.6f), m3508_left(2, &hcan1, 19, 16.2f, 0.17f, 2.87f);
// 以上是基本模块，相当于乐高积木中的最小零件

// 以下为较大的模块，由基本模块拼接而成，开始拼乐高制作整车！你只需了解一些模块的基本拼接规则即可！！

// 库中提供一些常用的底盘，其中omni3_unusual是极为特殊的一款，只能九期r1用！
omni3_unusual r1_chassis(&m3508_front, &m3508_right, &m3508_left, &Action, 7.0f, 0.0f, 0.7f); // 组装r1的底盘，只需简单将底盘和数个动力电机和一个定位模块拼接在一起，这里的动力电机可不止3508哦，只要派生自动力电机的接口就行，特别的，有些底盘比如说舵轮底盘还需要同时将动力电机和伺服电机组装起来，在库看来，所有电机都分为两类：动力电机和伺服电机，位置控制的3508属于伺服电机

// 组装r1的远程遥控器
xbox_r1n r1_remote(&Action, &r1_chassis);

extern "C" void create_tasks(void)
{

    ros_serial.startUartReceiveIT();
    esp32_serial.startUartReceiveIT();
    esp32_serial.addsubscriber(&r1_remote);
    Action.startUartReceiveIT();
    can_core.init();
    task_core.registerTask(4, &ros_serial); // 首次将ros_serial注册到任务4，创建任务4
    task_core.registerTask(0, &can_core);
    task_core.registerTask(3, &r1_chassis); // 又将另一个实例注册到了任务4，不会重复创建任务4
    task_core.registerTask(2, &r1_remote);
    osKernelStart();
}
