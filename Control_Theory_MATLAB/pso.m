%Define the objective function.
%fun = @(x)x(1)*exp(-norm(x)^2); 


sys_6 = 1/(3.24E10-5*s^2 + 22.59*s + 1);
sys_8 = 1/(2.91E10-5*s^2 + 6.18*s + 1);
num_6 = [ 0 0 1 ];
den_6 = [ 2.91E10-5 6.18 1 ];
num_8 = [ 0 0 1 ];
den_8 = [ 2.91E10-5  6.18  1 ];

%sys_6 = tf(num_6, den_6);
%sys_8 = tf(num_8, den_8);

%syms t;
%t=0:1:1000;
%step(sys_8)
dt = 0.01;
t = 0:dt:1;

cont_8 = 19.33 + 35.79/s + 29.67 * s;
cont_6 = 15.82 + 8.51/s + 7.36 * s;

h = tf(1,1);
f = 1/(sys_8*cont_8+1); = tf( [] ,  [] ) ;

e = 1 - step(f,t)

%S = stepinfo(feedback(plant*cont,1));
fun = sum(t'.*abs(e)*dt)




%Callparticleswarmto minimize the function.
rng default 
% For reproducibility 
nvars= 3;
x = particleswarm(fun,nvars)