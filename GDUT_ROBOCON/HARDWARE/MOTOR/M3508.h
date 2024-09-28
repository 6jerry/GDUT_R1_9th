#ifndef M3508_H
#define M3508_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "can_device.h"
#include "pid.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class m3508 : public CanDevice, public pid
{
private:
    uint8_t gear_ratio = 1;
    float vcurrent_to_rcurrent(int16_t vc);
    int16_t rcurrent_to_vcurrent(float rc);
    float vangle_to_rangle(uint32_t va);

public:
    m3508(CanDeviceType deviceType_, uint8_t can_id, CAN_HandleTypeDef *hcan_, uint8_t gear_ratio = 1);
    void can_update(uint8_t can_RxData[8]);
    uint16_t m3508_process() override;
    float rangle = 0;
    uint32_t rpm = 0;
    float rcurrent = 0;
    uint16_t rtarget_current = 0;
    float rtarget_angle = 0;
    float target_rpm = 0;
};

#endif

#endif