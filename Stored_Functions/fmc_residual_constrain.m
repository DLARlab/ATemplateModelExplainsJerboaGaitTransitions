%% Constrian function for fmincon
function [c,ceq] = fmc_residual_constrain(X)

% % scaling X values
% X = X.*1000;

[residual,T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset_optimization(X);
ceq = residual;
c = [];

end
