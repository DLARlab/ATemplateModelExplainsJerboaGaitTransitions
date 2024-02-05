%% This script create unconstrained optimization problem, called by fminsearch
function cost = fms_cost_fun(X)

% % scaling X if needed
% X = X.*1000;

global ob_data To footsequence
[residual,T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset_optimization(X);

[xr,yr,alphaLr,alphaRr] = ZFBA_resample2(T,Y,To);

% % resample function in case interp1 not working
% interval = fix(size(Y,1)/size(ob_data,1));
% inv = interval;
% for i = 1:1:size(ob_data,1)
%     xr(i,1) = (Y(1+inv*(i-1),1) + Y(1+inv*i,1))/2;
%     yr(i,1) = (Y(1+inv*(i-1),3) + Y(1+inv*i,3))/2;
%     alphaLr(i,1) = (Y(1+inv*(i-1),5) + Y(1+inv*i,5))/2;
%     alphaRr(i,1) = (Y(1+inv*(i-1),7) + Y(1+inv*i,7))/2;
% end

t_difference = norm([footsequence To(end)] - X(8:12));

% calculate norm of difference between simulated & observed data
cost = 5*norm(xr'-ob_data(:,1)) +50*norm(yr'-ob_data(:,3)) + 20*norm(alphaLr'-ob_data(:,5)) + 20*norm(alphaRr'-ob_data(:,7))...
    + 100*norm(residual) + 100*t_difference;
end