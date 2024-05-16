#ifndef __LQR_H_
#define __LQR_H_

#define STATE_LENGTH 4


class Lqr_Class{
    public:
    float K_Matrix[STATE_LENGTH];
    // 设置状态反馈矩阵的参数
    void SetKMatrix(float *k){
        for(int i = 0;i<STATE_LENGTH;i++)
        {
            K_Matrix[i] = k[i];
        }
    }

    // 计算反馈控制量的数值
    float LqrCalculate(float *states){
        float feedback_ctrl = 0.0f;
        for(int i=0;i<STATE_LENGTH;i++)
        {
            feedback_ctrl += (0 - states[i]) * K_Matrix[i];
        }
        return feedback_ctrl;
    }  
};



#endif
