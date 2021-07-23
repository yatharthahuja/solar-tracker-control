num_6 = [ 0 0 95665.01 ];
den_6 = [ 3.10 2160738.84 95665.01 ];
num_8 = [ 0 106169.59 0 ];
den_8 = [ 3.09 655775.87 106169.59 ];
sys_6 = tf(num_6, den_6)
sys_8 = tf(num_8, den_8);
%syms s;
%syms t;
%t=0:1:1000;
%rlocus(num_6, den_6);
%rlocus(num_8, den_8);
%grid;
%bode(num_6, den_6);
%bode(num_8, den_8);
%grid;
%nyquist(num_6, den_6);
%bode(num_8, den_8);
%grid;
t = 0:0.01:8; 
for  K = 5:-0.2:2; % Starts the outer loop to vary the K values 
    for  a = 1.5:-0.2:0.5; 
        % Starts the inner loop to vary the a values 
        num = [0 0 1.2*K 2.4*K*a 1 .2*K*a^2 ]; 
        den = [10.36 1.86 2.5+1.2*K 1 +2.4*K*a 1 .2*K*a^2];
        y = step(num,den,t); 
        m = max(y); 
        if m< 1.1 &m> 1.05 
            break; 
            % Breaks the inner loop 
        end
    end
    if m<1.1 &m> 1.05 
        break; 
        % Breaks the outer loop 
    end
end
plot(t,y) 
grid 
title('Unit-Step Response') 
xlabel('t Sec') 
ylabel('Output') 
KK = num2str(K); % String value of K to be printed on plot 
aa = num2str(a); % String value of a to be printed on plot 
text(4.25,0.54,'K = '), text(4.75,0.54,KK) 
text(4.25,0.46,'a = '), text(4.75,0.46,aa) 
sol = [K;a;m]
