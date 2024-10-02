#ifndef M3508_H
#define M3508_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "can_device.h"
#include "pid.h"
#include "motor.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class m3508 : public CanDevice, public pid, public power_motor // 动力电机版本的m3508
{
private:
    uint8_t gear_ratio = 1;
    float vcurrent_to_rcurrent(int16_t vc);
    int16_t rcurrent_to_vcurrent(float rc);
    float vangle_to_rangle(uint32_t va);

public:
    m3508(uint8_t can_id, CAN_HandleTypeDef *hcan_, uint8_t gear_ratio = 19, float kp_ = 16.5f, float ki_ = 0.19f, float kd_ = 2.6f);
    void can_update(uint8_t can_RxData[8]);
    int16_t m3508_process() override;
    float rangle = 0;
    int16_t rpm = 0;
    float rcurrent = 0;
    int16_t vtarget_current = 0;
    float rtarget_angle = 0;
    float target_rpm = 0;

    // 动力电机通用接口
    float get_rpm();
    void set_rpm(float power_motor_rpm);
};

#endif

#endif