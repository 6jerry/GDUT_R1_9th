#include "chassis.h"

void chassis::switch_chassis_mode(Chassis_mode target_mode)
{
    chassis_mode = target_mode;
}
Chassis_mode chassis::get_mode()
{
    return chassis_mode;
}
bool chassis::setrobotv(float rx, float ry, float w)
{ 
    input_w = w;
    if (chassis_mode == remote_robotv)
    {
        input_rvx = rx;
        input_rvy = ry;
        return true;
    }
    else
    {
        return false; // 模式错误，设置失败
    }
}
bool chassis::setworldv(float wx, float wy, float w)
{
    input_w = w;
    if (chassis_mode == remote_worldv)
    {
        input_wvx = wx;
        input_wvy = wy;
        return true;
    }
    else
    {
        return false;
    }
}
bool chassis::setpoint(float x, float y)
{
    if (chassis_mode == point_track_standby)
    {
        target_wx = x;
        target_wy = y;
        chassis_mode = point_tracking;
        return true;
    }
    else
    {
        return false;
    }
}
void chassis::lock_to(float heading)
{
    if_lock_heading = true;
    target_heading_rad = heading;
}
void chassis::lock()
{
    if_lock_heading = true;
    target_heading_rad = ACTION->pose_data.yaw_rad;
}
void chassis::unlock()
{
    if_lock_heading = false;
}
float chassis::get_track_state()
{
    return 0;
}
void chassis::worldv_to_robotv()
{
    target_rvx = cos(ACTION->pose_data.yaw_rad) * input_wvx + sin(ACTION->pose_data.yaw_rad) * input_wvy;
    target_rvy = cos(ACTION->pose_data.yaw_rad) * input_wvy - sin(ACTION->pose_data.yaw_rad) * input_wvx;
}
chassis::chassis(ChassisType chassistype_, action *ACTION_, float headingkp, float headingki, float headingkd) : chassistype(chassistype_), heading_pid(headingkp, headingki, headingkd, 100000.0f, 5.0f, 0.01f, 0.5f), ACTION(ACTION_)
{
}

float omni3_unusual ::v_to_rpm(float v)
{
    float rpm = v / Rwheel / (2 * PI) * 60;
    return rpm;
}

omni3_unusual::omni3_unusual(power_motor *front_motor, power_motor *right_motor, power_motor *left_motor, action *ACTION_, float headingkp, float headingki, float headingkd) : chassis(omni3_unusual_, ACTION_, headingkp, headingki, headingkd)
{
    motors[0] = front_motor;
    motors[1] = right_motor;
    motors[2] = left_motor; // 顺时针安装
}
void omni3_unusual::process_data()
{
    // 底盘内置小型状态机

    switch (chassis_mode)
    {
    case chassis_standby:
        target_rvx = 0.0f;
        target_rvy = 0.0f;
        break;
    case remote_robotv:
        target_rvx = input_rvx;
        target_rvy = input_rvy;
        break;
    case remote_worldv:
        worldv_to_robotv();
        break;
    case point_track_standby:

        break;
    case point_tracking:

        break;
    default:

        break;
    }
    if (if_lock_heading)
    {
        heading_pid.setpoint = target_heading_rad;
        target_w = heading_pid.PID_Compute(ACTION->pose_data.yaw_rad);
    }
    else
    {
        target_w = input_w;
    }

    motors[0]->set_rpm(v_to_rpm(-target_rvx + 0.20024f * target_w));
    motors[1]->set_rpm(v_to_rpm(target_w * 0.3149f + 0.6712f * target_rvx - target_rvy * 0.7412f));
    motors[2]->set_rpm(v_to_rpm(target_w * 0.3149f + 0.6712f * target_rvx + target_rvy * 0.7412f));
}
