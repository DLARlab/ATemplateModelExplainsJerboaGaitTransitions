%% This script runs optimization problem with different initial guess in for loop.
% Since we are using single shooting method, the best way we escape from
% local minimun is keep varying the initial guess and searching for good
% combinations. (This script need to be revised)
%% Constrains and options for fmincon
A = [];
b = [];
Aeq = [];
beq = [];
%[0.7975 0.9138 0.4818 0.5982 1.688];
lb = [0.0000; 0.2900; -1.000; -pi/2; -5.1719;  -pi/2; -5.8848; 
      0.4089; 0.5948; 0.1972;  0.2977; 1.2373;
      10; 2.0360; -0.5; -0.5];
ub = [5.0000; 2.5000; 1.0000;  pi/2; 10.9719;   pi/2; 10.6848; 
      1.4089; 1.8948; 1.1173;  1.877; 2.4373;
      500; 6.0000;  0.5; 0.5];
  

stride_extractor(filename);
options = optimoptions('fmincon','Algorithm','sqp',...
                        'ConstraintTolerance',1e-1,...
                        'FunctionTolerance',1e-6,...
                        'StepTolerance',1e-3,...
                        'DiffMaxChange',1e-3,...
                        'FiniteDifferenceStepSize',1e-6,...
                        'MaxFunctionEvaluations',10000,...
                        'PlotFcn',@optimplotfval);
                    
%% Varying initial guess to get better optimal result
                    
stride_extractor(filename);
% options for fminsearch
options = optimset('MaxIter',5000,'TolFun',10,'MaxFunEvals',10000);
% 'PlotFcns',,@optimplotfval

% store initial guess
XI = X;

% initialize solution matrix 
X200 = zeros(16,200);
fval200 = zeros(200,1);

for i = 1:200
    
    X0 = XI;
        
    X0(1) = X0(1) +  (-0.05 + 0.10*rand);
    X0(2) = X0(2) +  (-0.02 + 0.04*rand);  
    X0(4) = X0(4) +  ( -0.05 + 0.10*rand);
    X0(5) = X0(5) +  (-0.1 + 0.2*rand);
    X0(6) = X0(6) +  ( -0.05 + 0.10*rand);
    X0(7) = X0(7) +  (-0.1 + 0.2*rand);
    X0(13) = 60 + (-10 + 20*rand);
    X0(14) = X0(14) + (-0.05 + 0.1*rand);
    X0(15) = X0(15) + ( -0.02 + 0.04*rand);
    X0(16) = X0(16) + ( -0.02 + 0.04*rand);
       a = 0;
%     X0(8) = (X0(8)-rand*a)*(0.98 + 0.04*rand);
%     X0(9) = (X0(9)-rand*0*rand*a)*(0.98 + 0.04*rand);
%     X0(10) = (X0(10)-rand*0*a)*(0.98 + 0.04*rand);
%     X0(11) = (X0(11)-rand*0.5*a)*(0.98 + 0.04*rand);
%     X0(12) = To(end)*(0.98 + 0.04*rand);


% [X,fval,grad] = fmincon(@fmc_cost_fun,X0,A,b,Aeq,beq,lb,ub,@residual_constrain,options);
[X,fval] = fminsearch(@(X) fms_cost_fun(X),X0,options);

X200(:,i) = X;
fval200(i,1) = fval;
save('X200_gp14_fms.mat','X200','fval200')
disp(i)
disp(fval)
% if fval<fval_pr
%   X0 = X;
%   fval_pr = fval;
% end
end

%% Save the results

% test min value
[M,I] = min(fval200);
X = X200(:,I);
                    
save('IC_gp6.mat','X1','X2')
save('Xr_gp1_fms.mat','Xr','fr')

%% 
filename = 'gp10_1802_j30_r.mat';
load(filename)

% X = X200(:,163);


% resample simulated data (match the dimension)
[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset(X);
[xr,yr,alphaLr,alphaRr] = ZFBA_resample2(T,Y,To);

figure(6)
plot(To,xr','Color',[0.6350 0.0780 0.1840],'LineWidth',3.5)
hold on
plot(To,yr','Color',[0.9290 0.6940 0.1250],'LineWidth',3.5)
hold on
plot(To,ob_data(:,1),'--','Color',[0.6350 0.0780 0.1840],'LineWidth',3.5)
hold on
plot(To,ob_data(:,3),'--','Color',[0.9290 0.6940 0.1250],'LineWidth',3.5)
legend('simulated x','simulated y','observed x','observed y');
grid on; box on
xlim([0 To(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
ylabel('Distance $[l_o]$','Interpreter','LaTex')
title('Trajectories of center of mass positions')


figure(7)
plot(To,alphaLr','Color',[0 0.4470 0.7410],'LineWidth',3.5)
hold on
plot(To,alphaRr','Color',[0.3010 0.7450 0.9330],'LineWidth',3.5)
hold on
plot(To,ob_data(:,5),'--','Color',[0 0.4470 0.7410],'LineWidth',3.5)
hold on
plot(To,ob_data(:,7),'--','Color',[0.3010 0.7450 0.9330],'LineWidth',3.5)
grid on; box on
xlim([0 To(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
legend('simulated left','simulated right','observed left','observed right');
ylabel('Leg angles $[rad]$','Interpreter','LaTex')
title('Trajectories of center of leg angles')