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

    point_track_info.target_x = x;
    point_track_info.target_y = y;

    // chassis_mode = point_tracking;
}
void chassis::lock_to(float heading)
{
    if_lock_heading = true;
    target_heading_rad = heading;
}
void chassis::lock()
{
    if_lock_heading = true;
    if (if_first_lock == 0)
    {
        target_heading_rad = ACTION->pose_data.yaw_rad;
        if_first_lock = 1;
    }
}
void chassis::unlock()
{
    if_lock_heading = false;
    if_first_lock = 0;
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
void chassis::point_track_compute()
{
    float dis_vector_x = ACTION->pose_data.world_pos_x - point_track_info.target_x;
    float dis_vector_y = ACTION->pose_data.world_pos_y - point_track_info.target_y;

    point_track_info.distan_error = sqrt(dis_vector_x * dis_vector_x + dis_vector_y * dis_vector_y); // 求模

    // 单位化
    if (point_track_info.distan_error != 0)
    {
        point_track_info.direct_vector_x = dis_vector_x / point_track_info.distan_error;
        point_track_info.direct_vector_y = dis_vector_y / point_track_info.distan_error;

        distan_pid.setpoint = point_track_info.target_distan;
        float target_speed_vector = distan_pid.PID_Compute(point_track_info.distan_error);

        point_track_info.target_speed_x = target_speed_vector * point_track_info.direct_vector_x;
        point_track_info.target_speed_y = target_speed_vector * point_track_info.direct_vector_y;
    }
    else
    {
        point_track_info.target_speed_x = 0.0f;
        point_track_info.target_speed_y = 0.0f;
    }
}
void chassis::line_track_compute()
{
    // line_track_info.cur_to_target.X = ;
    float target_line_dis = sqrt(line_track_info.target_line.X * line_track_info.target_line.X + line_track_info.target_line.Y * line_track_info.target_line.Y);
    line_track_info.tangent_dis = (line_track_info.target_line.X * line_track_info.cur_to_target.X + line_track_info.target_line.Y * line_track_info.cur_to_target.Y) / target_line_dis;
    float temp = line_track_info.tangent_dis / target_line_dis;

    line_track_info.tangent_proj.X = temp * line_track_info.target_line.X;
    line_track_info.tangent_proj.Y = temp * line_track_info.target_line.Y;

    line_track_info.proj_point.X = line_track_info.tangent_proj.X + line_track_info.target_point.X;
    line_track_info.proj_point.Y = line_track_info.tangent_proj.Y + line_track_info.target_point.Y;
    float normal_vector_x = ACTION->pose_data.world_pos_x - line_track_info.proj_point.X;
    float normal_vector_y = ACTION->pose_data.world_pos_y - line_track_info.proj_point.Y;
    line_track_info.normal_dis = sqrt(normal_vector_x * normal_vector_x + normal_vector_y * normal_vector_y);

    line_track_info.tangent_dir.X = line_track_info.target_line.X / target_line_dis;
    line_track_info.tangent_dir.Y = line_track_info.target_line.Y / target_line_dis;

    line_track_info.normal_dir.X = normal_vector_x / line_track_info.normal_dis;
    line_track_info.normal_dir.Y = normal_vector_y / line_track_info.normal_dis;
}

chassis::chassis(ChassisType chassistype_, float Rwheel_, action *ACTION_, float headingkp, float headingki, float headingkd, float kp_, float ki_, float kd_) : chassistype(chassistype_), heading_pid(headingkp, headingki, headingkd, 100000.0f, 5.0f, 0.01f, 0.5f), ACTION(ACTION_), Rwheel(Rwheel_), distan_pid(kp_, ki_, kd_, 1000000.0f, 1.4f, 50.0f, 600.0f)
{
}

float chassis ::v_to_rpm(float v)
{
    float rpm = v / Rwheel / (2 * PI) * 60;
    return rpm;
}

omni3_unusual::omni3_unusual(power_motor *front_motor, power_motor *right_motor, power_motor *left_motor, float Rwheel_, action *ACTION_, float headingkp, float headingki, float headingkd, float point_kp, float point_ki, float point_kd) : chassis(omni3_unusual_, Rwheel_, ACTION_, headingkp, headingki, headingkd, point_kp, point_ki, point_kd)
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
    case line_tracking:

        break;
    case point_tracking:
        point_track_compute();
        input_wvy = point_track_info.target_speed_y;
        input_wvx = point_track_info.target_speed_x;
        worldv_to_robotv();
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

    motors[0]->set_rpm(-v_to_rpm(-target_rvx + 0.20024f * target_w));
    motors[1]->set_rpm(v_to_rpm(target_w * 0.3149f + 0.6712f * target_rvx - target_rvy * 0.7412f));
    motors[2]->set_rpm(v_to_rpm(target_w * 0.3149f + 0.6712f * target_rvx + target_rvy * 0.7412f));
}

omni3::omni3(power_motor *front_motor, power_motor *right_motor, power_motor *left_motor, float Rwheel_, float CHASSIS_R_, action *ACTION_, float headingkp, float headingki, float headingkd, float point_kp, float point_ki, float point_kd) : chassis(omni3_unusual_, Rwheel_, ACTION_, headingkp, headingki, headingkd, point_kp, point_ki, point_kd)
{
    motors[0] = front_motor;
    motors[1] = right_motor;
    motors[2] = left_motor; // 顺时针安装

    CHASSIS_R = CHASSIS_R_;
}
void omni3::process_data()
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
    case line_tracking:

        break;
    case point_tracking:
        point_track_compute();
        input_wvx = point_track_info.target_speed_x;
        input_wvy = point_track_info.target_speed_y;
        worldv_to_robotv();

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

    motors[0]->set_rpm(v_to_rpm(-target_rvx + CHASSIS_R * target_w));
    motors[1]->set_rpm(v_to_rpm(target_w * CHASSIS_R + 0.5f * target_rvx - target_rvy * 0.866f));
    motors[2]->set_rpm(v_to_rpm(target_w * CHASSIS_R + 0.5f * target_rvx + target_rvy * 0.866f));
}
omni4::omni4(power_motor *right_front_motor, power_motor *right_back_motor, power_motor *left_back_motor, power_motor *left_front_motor, float Rwheel_, float CHASSIS_R_, action *ACTION_, float headingkp, float headingki, float headingkd, float point_kp, float point_ki, float point_kd) : chassis(omni4_, Rwheel_, ACTION_, headingkp, headingki, headingkd, point_kp, point_ki, point_kd)
{
    motors[0] = right_front_motor;
    motors[1] = right_back_motor;
    motors[2] = left_back_motor;
    motors[3] = left_front_motor;

    CHASSIS_R = CHASSIS_R_;
}
void omni4::process_data()
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
    case line_tracking:

        break;
    case point_tracking:
        point_track_compute();
        input_wvx = point_track_info.target_speed_x;
        input_wvy = point_track_info.target_speed_y;
        worldv_to_robotv();
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

    motors[0]->set_rpm(v_to_rpm(-target_rvx * 0.70710678f - target_rvy * 0.70710678f + CHASSIS_R * target_w));
    motors[1]->set_rpm(v_to_rpm(target_w * CHASSIS_R + 0.70710678f * target_rvx - target_rvy * 0.70710678f));
    motors[2]->set_rpm(v_to_rpm(target_w * CHASSIS_R + 0.70710678f * target_rvx + target_rvy * 0.70710678f));
    motors[3]->set_rpm(v_to_rpm(target_w * CHASSIS_R - 0.70710678f * target_rvx + target_rvy * 0.70710678f));
}