#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdbool.h>
#ifdef __cplusplus
}
#endif

// 通用电机接口，便于底盘和各类机构调用的，总体来说分为动力电机和伺服电机,使用位置控制的m3508也属于伺服电机类
#ifdef __cplusplus
class power_motor
{

public:
    virtual float get_rpm() = 0;
    virtual void set_rpm(float power_motor_rpm) = 0; // 获取当前转速和设置目标转速的通用接口
};

// 待开发，因为现在r1上没有伺服电机
class servo_motor
{
public:
};

#endif

#endif
