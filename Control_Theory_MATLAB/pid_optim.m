function [J] = pid_optim(x)

num_6 = [ 0 0 95665.01 ];
den_6 = [ 3.10 2160738.84 95665.01 ];
num_8 = [ 0 106169.59 0 ];
den_8 = [ 3.09 655775.87 106169.59 ];
sys_6 = tf(num_6, den_6);
sys_8 = tf(num_8, den_8);

wn_6 = 309.30;
sigma_6 = 3492.95;

wn_8 = 325.84;
sigma_8 = 1006.29;


s = tf('s');

%plant = 1/(3.24E10-5*s^2 + 22.59*s + 1);%6
plant = 1/(2.91E10-5*s^2 + 6.18*s + 1);%8

%plant = 95665.01/(3.10*s^2 + 2160738.84*s + 95665.01);%6inch
%plant = 106169.59/(3.09*s^2 + 655775.87*s + 106169.59);%8inch

Kp = x(1)
Ki = x(2)
Kd = x(3)

cont = Kp + Ki/s + Kd * s;

step(feedback(plant*cont,1))

dt = 0.01;
t = 0:dt:1;

e = 1 - step(feedback(plant*cont,1),t)
%S = stepinfo(feedback(plant*cont,1));
J = sum(t'.*abs(e)*dt);
%+1.5*stepinfo(S).Overshoot+7*stepinfo(S).SettlingTime+1*stepinfo(S).RiseTime