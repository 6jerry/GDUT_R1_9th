#ifndef NEW_PID_H
#define NEW_PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;
    float setpoint;
    float i_out;
    float d_out;
    float p_out;
    float error_sum;
    float previous_error;
    float output;
    float integral_limit;
    float output_limit;
    float deadzone;
    float integral_separation_threshold;
    float error;

} PID_Controller;

// 初始化 PID 控制器
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float integral_limit, float output_limit, float deadzone, float integral_separation_threshold);

// 调整 PID 参数
void PID_SetParameters(PID_Controller *pid, float kp, float ki, float kd);

// PID 计算函数
float PID_Compute(PID_Controller *pid, float input);

#endif // PID_CONTROLLER_H
