%%% octave的问题？
%% 我怀疑Octave输出的K自带负号... 为了验证这个问题，我决定跑一下之前在B站看到的小车倒立摆的模型.
clear;
%M = 0.5;
%m = 0.5;
%l = 0.3;
%g = 9.81;

m = 3.3134; %两个轮子的质量，单轮为 1.6567
M = 15.6369;     %上肢全体重量
g = 9.8;
l = 0.26006;
r = 0.08;



%% state space matrices:
A = [0 0 1 0;
    0 0 0 1;
    0 m*g/M 0 0;
    0 (m*g + M*g)/(M*l) 0 0];
B = [0;0;1/M;1/(M*l)];
C = eye(4);
D = 0;

%% build ss system:
cartpole = ss(A,B,C,D);

%% LQR:
Q = diag([10 10 1 1]); % x q dx dq
R = 1; % fx
K = lqr(cartpole, Q, R)