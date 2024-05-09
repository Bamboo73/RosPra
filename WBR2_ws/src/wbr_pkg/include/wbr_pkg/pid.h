#ifndef __PID_H_
#define __PID_H_

// 定义一个pid 的类，之后所有的pid控制，都使用这些内容完成；
// 非常值得重复使用

class PID_Class{
    public:
    // 控制参数
    float kp;
    float kd;
    float ki;
    
    // 误差记录
    float target_value;
    float error;
    float last_error;   // 微分控制上一步
    float sum_error;    // 积分控制积累量
    float sum_error_ub; // up bound
    float sum_error_lb; // low bound

    // 输出与前馈
    float output;
    float feedforward_value;

    // 初始化与运行中的判断
    bool is_first_loop; //bool用得少，就直接is 前缀说明一下好了。

    // 下面是各种方法
    void Set_feedforward_value(float ffv);
    void Set_target_value(float target);
    void Set_pid_param(float p,float i,float d, float sub, float slb); // 设置 kp kd ki 参数
    void Init_pid();      // 初始化 pid, 将error以及所有 erro相关归零。 前馈输出也归零。
    float Calculate_pid_output(float current_value); //计算，输入当前值，输出控制值。
};

//设置PID前馈控制量
void PID_Class::Set_feedforward_value(float ffv){
    feedforward_value = ffv;
}

//设置目标值
void PID_Class::Set_target_value(float target){ 
    target_value = target;
}

//设置PID控制的参数
void PID_Class::Set_pid_param(float p,float i,float d, float sub, float slb){
    kp = p;
    ki = i;
    kd = d;
    sum_error_ub = sub; //积分上界
    sum_error_lb = slb; //积分下界
}

//初始化PID控制
void PID_Class::Init_pid()
{
    kp = 0;
    kd = 0;
    ki = 0;
    // 误差记录
    target_value = 0;
    error = 0;
    last_error = 0;
    sum_error = 0;
    sum_error_ub = 0; //积分上界
    sum_error_lb = 0; //积分下界
    // 输出与前馈
    output = 0;
    feedforward_value = 0;
    // 初始化与运行中的判断
    is_first_loop = 1; //bool用得少，就直接is 前缀说明一下好了。
}


float PID_Class::Calculate_pid_output(float current_value)
{
    error = target_value - current_value;
    if(is_first_loop == 1) //PID是否为初次的判断，感觉还是有必要的，如果只是简单的将lasterro=0，那么第一次计算微分项可能导致一个大脉冲，不好。
        {
            last_error = error;
            is_first_loop = 0;
        }
    output = kp * error + kd * (error - last_error) + ki * sum_error;
    sum_error += error;
    last_error = error; // 更新上次误差
    return output;
}

#endif