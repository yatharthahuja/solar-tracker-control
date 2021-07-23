s=tf('s');
sys_6 = 95665.01/(3.10*s^2 + 2160738.84*s + 95665.01)
sys_8 = 106169.59/(3.09*s^2 + 655775.87*s + 106169.59)
figure
step(sys_6)
%---values obtained from step response of sys--%
Kcr_8 = 3200000;
Pcr_8 = 0.002;

Kcr_6 = 250000;
Pcr_6 = 0.003;


K=0.01;
L=0.025;
T=0.3-L;

a=K*L/T;
Ti=2*L;
Td=L/2;

p=1.2/a;
Ki=Kp/Ti;
Kd=Kp*Td;
cont=pid(Kp,Ki,Kd)
figure
step(feedback(cont*sys_6,1))