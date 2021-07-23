function sys = CLS( Wp,Wc )
%CLS Closed loop system function
%% Parameters
% Wp : Plant transfer function
% Wc : Controller transfer function
% sys : Closed Loop transfer function with assuming unity feedback.
%% Function implementation
sys=feedback(series(Wp,Wc),1);
end