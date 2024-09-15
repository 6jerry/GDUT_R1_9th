#include "new_pid.h"
// 初始化 PID 控制器
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float integral_limit, float output_limit, float deadzone, float integral_separation_threshold)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->i_out = 0.0f;
    pid->p_out = 0.0f;
    pid->d_out = 0.0f;
    pid->error_sum = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    pid->deadzone = deadzone;
    pid->integral_separation_threshold = integral_separation_threshold;
}

// 调整 PID 参数
void PID_SetParameters(PID_Controller *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

// PID 计算函数
float PID_Compute(PID_Controller *pid, float input)
{
    pid->error = pid->setpoint - input;

    // 死区处理
    if (pid->error < pid->deadzone && pid->error > 0)
    {
        pid->error = 0.0f;
    }
    if (pid->error > -pid->deadzone && pid->error < 0)
    {
        pid->error = 0.0f;
    }
    pid->p_out = pid->kp * pid->error;

    pid->error_sum += pid->error;

    // 积分分离
    if (pid->error > pid->integral_separation_threshold)
    {
        pid->error_sum = 0.0f;
    }
    if (pid->error < -pid->integral_separation_threshold)
    {
        pid->error_sum = 0.0f;
    }
    // 积分限幅
    if (pid->error_sum > pid->integral_limit)
    {
        pid->error_sum = pid->integral_limit;
    }
    if (pid->error_sum < -pid->integral_limit)
    {
        pid->error_sum = -pid->integral_limit;
    }
    pid->i_out = pid->ki * pid->error_sum;

    pid->d_out = pid->kd * (pid->error - pid->previous_error);
    pid->previous_error = pid->error;

    pid->output = pid->p_out + pid->i_out + pid->d_out;

    // 输出限幅
    if (pid->output > pid->output_limit)
    {
        pid->output = pid->output_limit;
    }
    else if (pid->output < -pid->output_limit)
    {
        pid->output = -pid->output_limit;
    }

    return pid->output;
}
