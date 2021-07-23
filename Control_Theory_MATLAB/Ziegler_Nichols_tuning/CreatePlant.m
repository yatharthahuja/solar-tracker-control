function [ Wp ] = CreatePlant( num,den )
%CreatePlant Creates plant transfer function.
%   The returned value is the system in numerator/denomerator format
%% Parameters
% num : Numerator vector (starting from highest order of coefficients)
% den : Denomerator vector (starting from highest order of coefficients)
% plant : Plant transfer function 
%% EXAMPLE
%    num=[1];
%    den=[1 0 1];
%    sys=CreatePlant(num,den)
%% Result is        
%             1
% sys= ---------------
%           S^2+1
%% Function implementation
syms s;
Wp=tf(num,den);
end