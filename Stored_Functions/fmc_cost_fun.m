%% This script create unconstrained optimization problem, called by fmincon
function cost = fmc_cost_fun(X)

% % scaling X if needed
% X = X.*1000;

global ob_data To
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

% calculate norm of difference between simulated & observed data
cost = 5*norm(xr'-ob_data(:,1)) + 50*norm(yr'-ob_data(:,3)) + 10*norm(alphaLr'-ob_data(:,5)) + 10*norm(alphaRr'-ob_data(:,7));

end