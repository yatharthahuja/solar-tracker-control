function Wc  = ZieglerNicholasPID( Kc,Ti,Td )
% ZieglerNicholasPID function to generate the PID controller transfer 
%% Parameters
% Kc : Critical gain
% Ti : Reset time (minutes)
% Td : Derivative time (minutes)
% Wc : The laplace representation of Z-N
%% EXAMPLE
%    Kc=10;
%    Ti=0.83;
%    Td=2.5
%    Wc=ZieglerNicholasPID( Kc,Ti,Td )
%% Result is        
%               1 
% Wc= 10*(1+ -------- + 2.5*s
%             0.83*s 
%% Function implementation 
s=tf('s');
Wc=Kc*(1+(1/(Ti*s))+Td*s);
end
