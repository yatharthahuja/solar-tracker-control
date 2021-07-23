num_6 = [ 0 0 1 ];
den_6 = [ 2.91E10-5 6.18 1 ];

num_8 = [ 0 0 1 ];
den_8 = [ 2.91E10-5  6.18  1 ];

%sys_6 = tf(num_6, den_6);
sys_6 = 1/(3.24E10-5*s^2 + 22.59*s + 1);
sys_8 = 1/(2.91E10-5*s^2 + 6.18*s + 1);

%syms t;
%t=0:1:1000;
%step(sys_8)
dt = 0.01;
t = 0:dt:1;

cont_8 = 19.33 + 35.79/s + 29.67 * s;

cont_6 = 15.82 + 8.51/s + 7.36 * s;

e = 1 - step(feedback(sys_8*cont_8,1),t)
%S = stepinfo(feedback(plant*cont,1));
J = sum(t'.*abs(e)*dt)
%%syms s;
%F = 95665.01/( 3.14*s^2 + 216073.8*s );
%ilaplace(F);