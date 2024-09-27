#ifndef CAN_DEVICE_H
#define CAN_DEVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "can.h"
#include "TaskManager.h"
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
#define MAX_INSTANCES 4 // 一条can上最多挂四个3508
enum class CanDeviceType
{
    M3508,
    M2006,
    DM43,
    U8,
    M6020
};

class CanDevice
{
public:
    CAN_HandleTypeDef *hcan_;  // CAN 句柄
    CanDeviceType deviceType_; // 设备类型
    uint8_t can_id = 0;
    virtual uint16_t m3508_process(); // 给m3508用的接口，其他电机不要管
    virtual uint16_t m2006_process();

    virtual void m3508_update();
    virtual void m2006_update();

    static CanDevice *m3508_instances_can1[MAX_INSTANCES]; // 保存所有实例,供can管理者使用
    static CanDevice *m2006_instances_can1[MAX_INSTANCES];
    static int instanceCount_m3508_can1;

    static CanDevice *m3508_instances_can2[MAX_INSTANCES];
    static int instanceCount_m3508_can2;

    CanDevice(CanDeviceType deviceType_, CAN_HandleTypeDef *hcan_, uint8_t can_id);
};

class CanManager : public ITaskProcessor
{
private:
    void CAN1_Filter_Init(void);
    void CAN2_Filter_Init(void);

public:
    void process_data();
};

#endif
#endif