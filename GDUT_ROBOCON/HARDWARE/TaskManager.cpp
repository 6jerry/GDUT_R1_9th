#include "TaskManager.h"
#include "RC9Protocol.h"

TaskInfo TaskManager::tasks[MAX_TASKS];

TaskManager::TaskManager()
{
    for (int i = 0; i < MAX_TASKS; i++)
    {
        tasks[i].name[0] = 't';
        tasks[i].name[1] = 'a';
        tasks[i].name[2] = 's';
        tasks[i].name[3] = 'k';
        tasks[i].name[4] = '_';
        tasks[i].name[5] = '0' + i;
        tasks[i].name[6] = '\0';
        // 生成一波任务名字，便于后续debug
    }
    // 默认的任务参数设置，如有不满可以走客制化
    tasks[0].Priority = 5;
    tasks[0].delay_ms = 1;
    tasks[1].Priority = 4;
    tasks[1].delay_ms = 1;
    tasks[2].Priority = 4;
    tasks[2].delay_ms = 5;
    tasks[3].Priority = 3;
    tasks[3].delay_ms = 5;
    tasks[4].Priority = 3;
    tasks[4].delay_ms = 10;
    tasks[5].Priority = 2;
    tasks[5].delay_ms = 10;
    tasks[6].Priority = 2;
    tasks[6].delay_ms = 15;
    tasks[7].Priority = 1;
    tasks[7].delay_ms = 15;
    tasks[8].Priority = 1;
    tasks[8].delay_ms = 20;
    tasks[9].Priority = 0;
    tasks[9].delay_ms = 20;
}

void TaskManager::registerTask(int taskID, ITaskProcessor *instance)
{
    if (taskID < 0 || taskID >= MAX_TASKS || tasks[taskID].instanceCount >= MAX_CLASSES_PER_TASK)
    {
        return;
    }
    // 注册
    tasks[taskID].instances[tasks[taskID].instanceCount] = instance;
    tasks[taskID].instanceCount++;

    // 如果任务还未创建，则创建任务
    if (tasks[taskID].handle == nullptr)
    {
        xTaskCreate(TaskFunction, tasks[taskID].name, tasks[taskID].stack_size, (void *)taskID, tasks[taskID].Priority, &tasks[taskID].handle);
    }
}

void TaskManager::TaskFunction(void *parameters)
{

    int run_taskID = (int)(uintptr_t)parameters;
    // TaskInfo &taskInfo = tasks[taskID];

    while (true)
    {
        for (int i = 0; i < tasks[run_taskID].instanceCount; ++i)
        { // 执行所有实例
            if (tasks[run_taskID].instances[i])
            {
                tasks[run_taskID].instances[i]->process_data();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(tasks[run_taskID].delay_ms)); // 延时
    }
}

void TaskManager::customize(int taskID, uint8_t priority, uint32_t delay_ms, uint32_t stack_size)
{
    if (tasks[taskID].handle != nullptr)
    {
        return; // 已创建的任务无法客制化
    }
    tasks[taskID].Priority = priority;
    tasks[taskID].delay_ms = delay_ms;
    tasks[taskID].stack_size = stack_size;
}
