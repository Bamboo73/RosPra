%% 使用拉格朗日法
%% Robot basic parameters

clear;
m = 0.8;
M = 10;
g = 9.8;
l = 0.5;
L = 0.5;
r = 0.05;

pkg load control;


Q_k = 1 + (r*(1.5*m + M)/(M*l));

A1 = g/r  - r*g*(1.5*m + M)/(1.5*m*r*r);
A2 = g*(1.5*m + M)/(1.5*m*l);
B1 = (Q_k/(1.5*m*r*r) - 1/(M*l*r) );
B2 = -Q_k/(1.5*m*l*r);




A = [ 0 0 1 0;
      0 0 0 1;
      0 A1 0 0;
      0 A2 0 0];


B = [0 ; 0 ; B1  ; B2 ] ;
C = [0 1 0 0 ];
D = [0];

wip =ss(A,B,C,D);

Q = diag([1000 100 10 1]);
R = 1000;

K_lagrange = lqr(wip,Q,R)

