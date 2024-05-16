%% Basic_LQR 
%% 本文件来自RMer的模型，知乎上的“平衡步兵”以及相关论文中的推导结果。直接在WIP上使用，看看控制效果如何。

m_c = 0.8;
m_l = 0.5;
m_b = 10;
R = 0.05;
l = 0.5;
l_c = 0.25;
l_b = 0.25;

I_c = 0.5* m_c * R*R;




pkg load control;