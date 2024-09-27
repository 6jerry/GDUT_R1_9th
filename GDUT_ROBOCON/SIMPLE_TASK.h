#ifndef SIMPLE_TASK_H
#define SIMPLE_TASK_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "task.h"
// #include <stdint.h>
#include <cmsis_os.h>
#include "RC9Protocol.h"
#include "usart.h"
#include "TaskManager.h"
#include "Action.h"
#include "can_device.h"
void create_tasks(void);
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
// C 的接口函数，用于在 main.c 中调用

#endif
#endif // SIMPLE_TASK_H
