#include "M3508.h"

m3508::m3508(uint8_t can_id, CAN_HandleTypeDef *hcan_, uint8_t gear_ratio, float kp_, float ki_, float kd_) : CanDevice(M3508, hcan_, can_id), gear_ratio(gear_ratio), pid(kp_, ki_, kd_, 1000000.0f, 20000.0f, 5.0f, 960.0f) // 选择使用积分分离的话积分限幅就是无意义的，随便给个爆大的值就行
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

int16_t m3508::m3508_process()
{
    setpoint = target_rpm * (float)gear_ratio;
    vtarget_current = rcurrent_to_vcurrent(PID_Compute(rpm));
    return vtarget_current;
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
