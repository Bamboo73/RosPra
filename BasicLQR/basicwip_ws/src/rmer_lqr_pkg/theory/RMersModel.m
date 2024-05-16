%% Basic_LQR 
%% 本文件来自RMer的模型，知乎上的“平衡步兵”以及相关论文中的推导结果。直接在WIP上使用，看看控制效果如何。

pkg load control;
m_c = 0.8;
m_l = 0.5;
m_b = 8;
R = 0.1;
l = 0.5;
l_c = 0.25;
l_b = 0.25;

I_c = 0.5* m_c * R*R;

m = m_l; 
w = 0.02;
k = 0.5;
I_l = m*(w*w + k*k)/12;

m = m_b; 
w = 0.2;
k = 0.3;
I_b = m*(w*w + k*k)/12;
g = 9.8;

a1 = ((m_l + m_b)*l_c* R*R)/(I_c + (m_c + m_l + m_b)*R*R );
a2 = R/(I_c + (m_c + m_l + m_b)*R*R );
a3 = g*(m_l*l_c + m_b*l_c + m_b *l_b) / (I_l + l_c*(m_l*l_c + m_b*l_c + m_b *l_b));
a4 = (m_l*l_c + m_b*l_c + m_b *l_b) / (I_l + l_c*(m_l*l_c + m_b*l_c + m_b *l_b));
a5 = 1/(I_l + l_c*(m_l*l_c + m_b*l_c + m_b *l_b));
a6 = 1/I_b;

A1 = a1*a3/(1 + a1*a4);
A2 = a3 / (1+ a1*a4);
B1 = (a2 - a1*a5)/(1+ a1*a4);
B2 = (a1*a5)/(1+ a1*a4);
B3 = -(a5 + a2*a4)/(1+ a1*a4);
B4 = a5/(1+ a1*a4);


A = [0 1 0 0 0 0;
     0 0 A1 0 0 0;
     0 0 0 1 0 0 ;
     0 0 A2 0 0 0;
     0 0 0 0 0 1;
     0 0 0 0 0 0];
     
B = [0 0;
     B1 B2;
     0 0 ;
     B3 B4;
     0 0 ;
     0 0];



C = [0 1 0 0 0 0];

D = [0];

wip =ss(A,B,C,D);

Q = diag([10 10 1 1 1 1]);
R = diag([1 1]);


%K = lqr(wip,Q,R)





