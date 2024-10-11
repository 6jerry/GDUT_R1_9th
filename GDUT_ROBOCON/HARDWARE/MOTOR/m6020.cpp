#include "m6020.h"

m6020s::m6020s(uint8_t can_id, CAN_HandleTypeDef *hcan_, float kp_r, float ki_r, float kd_r, float kp_p, float ki_p, float kd_p) : CanDevice(M6020, hcan_, can_id), rpm_pid(kp_r, ki_r, kd_r, 1000000.0f, 25000.0f, 1.0f, 90.0f), pos_pid(kp_p, ki_p, kd_p, 0, 0, 0, 0), dji_motor(3000.0f, 16384, 8191)
{
}

int16_t m6020s::motor_process()
{
    rpm_pid.setpoint = target_rpm * (float)gear_ratio;
    target_v = (int16_t)rpm_pid.PID_Compute(rpm);
    return target_v;
}
void m6020s::can_update(uint8_t can_RxData[8])
{
    uint16_t vangle = (can_RxData[0] << 8) | can_RxData[1];
    rangle = vangle_to_rangle(vangle);

    rpm = (can_RxData[2] << 8) | can_RxData[3];

    int16_t vcurrent = (can_RxData[4] << 8) | can_RxData[5];
    rcurrent = vcurrent_to_rcurrent(vcurrent);
}

float m6020s::get_relative_pos()
{
    return 0;
}
float m6020s::get_absolute_pos()
{
    return 0;
}
void m6020s::set_relative_pos(float relative_pos_)
{
}
void m6020s::set_absolute_pos_multi(float absolute_pos_multi_)
{
}
void m6020s::set_absolute_pos_single(float absolute_pos_single_)
{
}
void m6020s::relocate(float new_zero_point)
{
}