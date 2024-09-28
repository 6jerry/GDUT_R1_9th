#include "M3508.h"

m3508::m3508(CanDeviceType deviceType_, uint8_t can_id, CAN_HandleTypeDef *hcan_, uint8_t gear_ratio) : CanDevice(deviceType_, hcan_, can_id), gear_ratio(gear_ratio)
{
}

void m3508::can_update(uint8_t can_RxData[8])
{
    uint16_t vangle = (can_RxData[0] << 8) | can_RxData[1];
    rangle = vangle_to_rangle(vangle);

    rpm = (can_RxData[2] << 8) | can_RxData[3];

    int16_t vcurrent = (can_RxData[4] << 8) | can_RxData[5];
    rcurrent = vcurrent_to_rcurrent(vcurrent);
}

uint16_t m3508::m3508_process()
{
    return 0;
}

float m3508::vcurrent_to_rcurrent(int16_t vc)
{

    return ((float)vc / 16384.0f) * 20000; // mA
}

int16_t m3508::rcurrent_to_vcurrent(float rc)
{
    return (rc / 20000.0f) * 16384;
}

float m3508::vangle_to_rangle(uint32_t va)
{
    return ((float)va / 8191.0f) * 360.0f;
}
