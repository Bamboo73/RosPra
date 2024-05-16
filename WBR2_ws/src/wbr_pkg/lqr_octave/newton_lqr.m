%% 使用牛顿欧拉法求解的WIP动力学模型的状态空间方程与LQR求解
%% Robot basic parameters
clear; 

m = 3.3134; %两个轮子的质量，单轮为 1.6567
M = 15.6369;     %上肢全体重量
g = 9.8;
l = 0.26006;
r = 0.08;

pkg load control;

Q = 1.5*m + M;

A = [ 0 0 1 0;
      0 0 0 1;
      0 -M*g/Q 0 0;
      0 g/l 0 0];


B = [0 ; 0 ; (l+r)/(Q*l*r) ; -1.0/(M*l*l) ] ;
C = [0 1 0 0 ];
D = [0];

wip =ss(A,B,C,D)

Q = diag([1 100 1 1]);
R = 1;

K_newton = lqr(wip,Q,R)

