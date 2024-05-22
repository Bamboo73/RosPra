#ifndef __LQR_H_
#define __LQR_H_

#define STATE_LENGTH 6 
// TODO : 先使用长队为6 的LQR，日后可能还是需要改成使用数据结构来实现的方案。
// 暂且为了快速推进WBR的快速验证，只能


class Lqr_Class{
    public:
    int State_Length  = STATE_LENGTH;  // TODO 之后可以修改成动态分配
    
    float Target_States[STATE_LENGTH]; // 目标状态

    float System_States[STATE_LENGTH]; // 系统状态，不过后面的函数不一定使用这个，根据需要取舍即可。

    bool  IsTargetSetted = 0;
    float K_Matrix[STATE_LENGTH];

    // 设置状态反馈矩阵的参数
    void SetKMatrix(float *k){
        for(int i = 0;i<STATE_LENGTH;i++)
        {
            K_Matrix[i] = k[i];
        }
    }

    //设置LQR控制器的目标状态
    void SetTarget_States(float * t_states){
        for(int i = 0;i<STATE_LENGTH;i++){
            Target_States[i] = t_states[i];
        }
        IsTargetSetted = 1; // 标注，已经手动设置的目标状态，之后的LQR求解K，不再是调节问题。
    }

    // 计算反馈控制量的数值
    float LqrCalculate(float *states){
        float feedback_ctrl = 0.0f;
        if(!IsTargetSetted) //没有手动设置过目标状态，默认是一个调节问题。
        {
            for(int i=0;i<STATE_LENGTH;i++)
            {
                feedback_ctrl += (0 - states[i]) * K_Matrix[i];
            }
        }
        else if(IsTargetSetted) // 已经手动设置过目标状态，选择对目标状态进行跟随。
        {
            for(int i=0;i<STATE_LENGTH;i++)
            {
                feedback_ctrl += (Target_States[i] - states[i]) * K_Matrix[i];
            }
        }
        return feedback_ctrl; //反馈求得的控制量。
    }  
};



#endif
