num_6 = [ 0 0 95665.01 ];
den_6 = [ 3.10 2160738.84 95665.01 ];

num_8 = [ 0 106169.59 0 ];
den_8 = [ 3.09 655775.87 106169.59 ];

sys_6 = tf(num_6, den_6)
sys_8 = tf(num_8, den_8);

syms s;
syms t;
t=0:1:1000;

%rlocus(num_6, den_6)
%rlocus(num_8, den_8)
%grid;

%bode(num_6, den_6)
%bode(num_8, den_8)
grid;

%nyquist(num_6, den_6)
nyquist(num_8, den_8)
grid;

