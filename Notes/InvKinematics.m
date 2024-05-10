mb = 6.9465;
mt = 2*2.258;
ms = 2*2.0872;
mw = 1.6567*2;
l1 = 0.2;
l2 = 0.18;
M = mb + mt + ms;

h = 0.34;




theta1 = 1.178;
theta2 = -1.949;




test_x_wheel = l1*sin(theta1) + l2*sin(theta1 + theta2)
test_x_c = (1/M) *( mt*0.5* l1* sin(theta1) + ms*(l1*sin(theta1) + 0.5 * l2 * sin(theta1 + theta2))  )


h_a = l1*cos(theta1) + l2*cos(theta1 + theta2)
