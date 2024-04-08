
%% Robot basic parameters
m = 0.78172;
M = 9.8910;
g = 9.8;
l = 0.4685;
r = 0.05;
pkg load control;

Q1 = M*g/(M + 1.5*m);
Q2 = (l+r)/(l*r*(M+1.5*m));

A = [ 0 0 1 0;
      0 0 0 1;
      0 -Q1 0 0;
      0 -g/l 0 0];

B = [0 ; 0 ; Q2 ; 1.0/(M*l*l) ] ;

C = [0 1 0 0 ];

D = [0];

wip =ss(A,B,C,D);

Q = diag([0.051 0.051 0.5 0.15]);
R = 5;


K = lqr(wip,Q,R)