#ifndef M6020_H
#define M6020_H
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

class m6020s : public CanDevice, public servo_motor, public dji_motor // 动力电机版本的m3508
{
private:
    uint8_t gear_ratio = 1;

public:
    m6020s(uint8_t can_id, CAN_HandleTypeDef *hcan_, float kp_r = 270.0f, float ki_r = 1.8f, float kd_r = 6.0f, float kp_p = 0.0f, float ki_p = 0.0f, float kd_p = 0.0f);

    int16_t motor_process() override;
    void can_update(uint8_t can_RxData[8]);
    float rtarget_angle = 0;
    float target_rpm = 0;
    int16_t target_v = 0;

    pid rpm_pid;
    pid pos_pid;

    float get_relative_pos();
    float get_absolute_pos();
    void set_relative_pos(float relative_pos_);
    void set_absolute_pos_multi(float absolute_pos_multi_);
    void set_absolute_pos_single(float absolute_pos_single_);
    void relocate(float new_zero_point);
};

#endif

#endif