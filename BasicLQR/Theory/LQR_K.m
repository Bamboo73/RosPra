%% Basic_LQR 
%% Robot basic parameters
m = 0.8;
M = 10;
g = 9.8;
l = 0.5;
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

Q = diag([5 80 0.51 0.51]);
R = 1;


K = lqr(wip,Q,R)