% 使用B站上的LQR开源工程尝试进行复现
%
% 状态变量： x dx th dth delta d_delta x是轮水平位置，the是角度 delta是偏航角。
%
pkg load control;

% 这部分是我的WIP模型的参数，录入如下：

M = 10;
m = 2;
r = 0.1;
l = 0.5;
L = l;

wheel_length = 0.04;
D = 0.5 + wheel_length;

g = 9.8;



I = 0.5*m*r*r;
Jz = M*l*l;
Jy = D*D*0.5 * m;



a = r*(M + 2*m + 2*I/(r*r));
b = M*r*l;
c = Jz + M * l * l;
d = M * g * l;
e = M * l;
f = 1/(r* (m*D + I*D/(r*r) + 2 * Jy / D )   );

A23 = -b*d/(a*c - b*e);
A43 = a*d / (a*c - b*e);
B21  = (c+b)/(a*c - b*e);
B22 = B21;
B41 = -(e+a)/(a*c - b*e);
B42 = B41;
B61 = -f; % 这里和参考文档的原文不同，绕竖直方向，应该是向左旋转为正才对。B61 B62 调整符号
B62 = f;

A = [ 0 1 0   0 0 0 ;
      0 0 A23 0 0 0 ;
      0 0 0   1 0 0 ;
      0 0 A43 0 0 0 ;
      0 0 0   0 0 1 ;
      0 0 0   0 0 0 ];

B = [0   0  ;
     B21 B22;
     0   0  ;
     B41 B42;
     0   0  ;
     B61 B62];

C = [0 0 1 0 0 0];

D = [0 0];

wip =ss(A,B,C,D);
% 状态变量： x dx th dth delta d_delta x是轮水平位置，the是角度 delta是偏航角。
Q = diag([10 1 2 1 1 1]);
R = diag([1 1]);

K = lqr(wip,Q,R)


% 这份测试是成功的



