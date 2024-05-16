%% Basic_LQR 
%% Robot basic parameters
m = 0.8;
M = 10;
g = 9.8;
l = 0.5;
L = 0.5;
r = 0.05;
pkg load control;

Q1 = M*g/(M + 1.5*m);
Q2 = (l+r)/(l*r*(M+1.5*m));

A = [ 0 0 1 0;
      0 0 0 1;
      0 -Q1 0 0;
      0 g/l 0 0];

B = [0 ; 0 ; Q2 ; -1.0/(M*l*l) ] ;

C = [0 1 0 0 ];

D = [0];

wip =ss(A,B,C,D);

Q = diag([10 10 1 1]);
R = 1;


K = lqr(wip,Q,R)



Q3 = (1.5*m+M)*g/(1.5*m);
Q4 = (1.5*m+M)*g/(1.5*m);
Q5 = (1.5*m+M)/(1.5*m*M*L);
Q6 = 1/(1.5*m);


A = [ 0 0 1 0;
      0 0 0 1;
      0 -Q4 0 0;
      0 Q3 0 0];

B = [0 ; 0 ; -Q6 ; Q5 ] ;

C = [0 1 0 0 ];

D = [0];
wip2 =ss(A,B,C,D);

Q = diag([10 10 1 1]);
R = 1;


K2 = lqr(wip2,Q,R)


