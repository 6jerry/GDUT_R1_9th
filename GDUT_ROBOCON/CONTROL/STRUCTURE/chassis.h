#ifndef CHASSIS_H
#define CHASSIS_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "TaskManager.h"
#include "motor.h"
#include "Action.h"
#include "pid.h"
#include "simple_math_tool.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#define PI 3.1415926f
// 底盘属于机构，由多个动力电机和一个定位器(action)构成，或由多个动力电机和伺服电机组合而成(舵轮底盘)
enum ChassisType
{
    omni3_,
    omni3_unusual_,
    omni4_,
    mecanum4_,
    swerve4_, // 四舵轮
    swerve3_
};

enum Chassis_mode
{
    chassis_standby,
    remote_robotv,
    remote_worldv,
    point_track_standby,
    point_tracking
};
typedef struct point_track_info_
{
    float target_x = 0.0f;
    float target_y = 0.0f;
    float target_distan = 0.0f;
    float target_speed_x = 0.0f;
    float target_speed_y = 0.0f;

    float distan_error = 0.0f;
    float direct_vector_x = 0.0f;
    float direct_vector_y = 0.0f;
};

class chassis // 基类，也是底盘的通用控制接口
{
public:
    ChassisType chassistype;
    Chassis_mode chassis_mode = chassis_standby;
    float input_rvx = 0.0f, input_rvy = 0.0f, input_wvx = 0.0f, input_wvy = 0.0f, input_w = 0.0f, target_heading_rad = 0.0f, target_rvx = 0.0f, target_rvy = 0.0f, target_w = 0.0f;
    uint8_t if_first_lock = 0;
    pid heading_pid;
    action *ACTION = nullptr;
    bool if_lock_heading = false; // 是否锁死朝向
    void unlock();
    void lock();
    void lock_to(float heading);
    float Rwheel = 0.0719f;
    float CHASSIS_R = 0.0f;

    point_track_info_ point_track_info;
    pid distan_pid;
    void point_track_compute();

public:
    void
    switch_chassis_mode(Chassis_mode target_mode);
    Chassis_mode get_mode();
    bool setrobotv(float rx, float ry, float w); // 机器人坐标遥控函数,只有当前底盘处于机器人坐标遥控模式才有效
    bool setworldv(float wx, float wy, float w);
    bool setpoint(float x, float y);
    float get_track_state();
    chassis(ChassisType chassistype_, float Rwheel_, action *ACTION_, float headingkp, float headingki, float headingkd, float kp_, float ki_, float kd_);
    void worldv_to_robotv();
    float v_to_rpm(float v);
};
// 钻石三轮全向轮底盘，通常以钻石那个尖角为车头,典型车体:九期r1
class omni3_unusual : public ITaskProcessor, public chassis
{
private:
    power_motor *motors[3] = {nullptr};

    // float Rwheel = 0.0719;

public:
    omni3_unusual(power_motor *front_motor, power_motor *right_motor, power_motor *left_motor, float Rwheel_, action *ACTION_, float headingkp = 7.0f, float headingki = 0.0f, float headingkd = 0.7f, float point_kp = 0.0f, float point_ki = 0.0f, float point_kd = 0.0f);
    void process_data();
};

// 常规三轮全向轮底盘，通常以一个电机为车头的朝向，典型车体：九期r2
class omni3 : public ITaskProcessor, public chassis
{
private:
    power_motor *motors[3] = {nullptr};

    // float v_to_rpm(float v);
    // float Rwheel = 0.0719;

public:
    omni3(power_motor *front_motor, power_motor *right_motor, power_motor *left_motor, float Rwheel_, float CHASSIS_R_, action *ACTION_, float headingkp, float headingki, float headingkd, float point_kp, float point_ki, float point_kd);
    void process_data();
};

// 常规四轮全向轮底盘，典型车体：八期r1

class omni4 : public ITaskProcessor, public chassis
{
private:
    power_motor *motors[4] = {nullptr};

public:
    omni4(power_motor *right_front_motor, power_motor *right_back_motor, power_motor *left_back_motor, power_motor *left_front_motor, float Rwheel_, float CHASSIS_R_, action *ACTION_, float headingkp, float headingki, float headingkd, float point_kp, float point_ki, float point_kd);
    void process_data();
};

#endif
#endif