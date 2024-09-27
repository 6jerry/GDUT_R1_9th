#include "can_device.h"

uint16_t CanDevice::m3508_process()
{
    return;
}

uint16_t CanDevice::m2006_process()
{
    return;
}
void CanDevice::m3508_update()
{
    return;
}
void CanDevice::m2006_update()
{
    return;
}
// 以上都是虚函数，不用在基类写具体的东西

CanDevice::CanDevice(CanDeviceType deviceType_, CAN_HandleTypeDef *hcan_, uint8_t can_id) : deviceType_(deviceType_), hcan_(hcan_), can_id(can_id)
{
    // 注册该can设备
    if (hcan_ == &hcan1)
    {
        switch (deviceType_)
        {
        case CanDeviceType::M3508:
            if (instanceCount_m3508_can1 < MAX_INSTANCES)
            {
                // 使用插入排序方法，根据 can_id 插入到合适位置
                int insertPos = instanceCount_m3508_can1;
                for (int i = 0; i < instanceCount_m3508_can1; ++i)
                {
                    if (m3508_instances_can1[i]->can_id > can_id)
                    {
                        insertPos = i;
                        break;
                    }
                }

                // 从插入位置开始，依次向后移动元素
                for (int i = instanceCount_m3508_can1; i > insertPos; --i)
                {
                    m3508_instances_can1[i] = m3508_instances_can1[i - 1];
                }

                // 在插入位置放入当前设备
                m3508_instances_can1[insertPos] = this;
                instanceCount_m3508_can1++;
            }
            break;

        default:
            break;
        }
    }
    if (hcan_ == &hcan2)
    {
        switch (deviceType_)
        {
        case CanDeviceType::M3508:
            if (instanceCount_m3508_can2 < MAX_INSTANCES)
            {
                // 使用插入排序方法，根据 can_id 插入到合适位置
                int insertPos = instanceCount_m3508_can2;
                for (int i = 0; i < instanceCount_m3508_can2; ++i)
                {
                    if (m3508_instances_can2[i]->can_id > can_id)
                    {
                        insertPos = i;
                        break;
                    }
                }

                // 从插入位置开始，依次向后移动元素
                for (int i = instanceCount_m3508_can2; i > insertPos; --i)
                {
                    m3508_instances_can2[i] = m3508_instances_can2[i - 1];
                }

                // 在插入位置放入当前设备
                m3508_instances_can2[insertPos] = this;
                instanceCount_m3508_can2++;
            }
            break;

        default:
            break;
        }
    }
}