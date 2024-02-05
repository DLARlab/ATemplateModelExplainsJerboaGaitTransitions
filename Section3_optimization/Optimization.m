%% This is the main script running optimization problem. 
% We would like to find optimal solution that able to reproduce the bipedal
% locomotion which matches the trajectories of simulated data.
clc
clear all
close all

%% Extract one stride from the dataset 

%Load data from stored dataset
filename = 'gp11_1826_j30.mat';
load(filename)

figure(1)
plot(To,ob_data,'LineWidth',2)
grid on; 
box on
xlim([0 To(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
    ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);

offsetL = mean(ob_data(:,5));
offsetR = mean(ob_data(:,7));

%% Test previous model
% load('G12_v1_ALL.mat')
% k = 24;
% xCYC = config(:,k);
% [residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex(xCYC);
% ShowTrajectory(T,Y,P,'Test')
% figure(2)
% plot(T,Y,'LineWidth',2)
% grid on; box on
% xlim([0 T(end)]);
% xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
% 
% leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
%     ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
% set(leg1,'Interpreter','latex');
% set(leg1,'FontSize',10);

%% Test offset model

% create xCYC called by fslove in next part
xCYC(1:7) = ob_data(1,2:8);
% xCYC(3) = 0;
% xCYC(5) = 5;
% xCYC(7) = 5;
T_Apex = To(length(To));
xCYC(8:12) = [footsequence T_Apex];%sequence from video
xCYC(13:16) = [100 4.5 offsetL offsetR];
%xCYC(13:16) = X(13:16);


stride_extractor(filename);
[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset(xCYC);
%ShowTrajectory(T,Y,P,'Test')
figure(3)
plot(T,Y,'LineWidth',2)
grid on; box on
xlim([0 T(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');

leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
    ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);
%% Find Better Initial Condition for The Next Part(optional)

% % modifying xCYC
% xCYC = xFINAL;
% xCYC(1) = 1.86;
% xCYC(4) = -0.2761;
% xCYC(6) = 0.3391;
% 
% xCYC(15) = offsetL;
% xCYC(16) = offsetR;
% xCYC(15) = 0;
% xCYC(16) = 0;

stride_extractor(filename);
numOPTS = optimset('Algorithm','levenberg-marquardt',... 
                   'Display','iter',...
                   'MaxFunEvals',3000,...
                   'MaxIter',30000,...
                   'UseParallel', false,...
                   'TolFun',1e-15,...
                   'TolX',1e-12,'PlotFcn',@optimplotfirstorderopt);
[xFINAL, ~] = fsolve(@ZeroFunc_BipedApex_offset,xCYC, numOPTS);

[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset(xFINAL);
%ShowTrajectory(T,Y,P,'Test')
figure(4)
plot(T,Y,'LineWidth',2)
grid on; box on
xlim([0 T(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');

leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
    ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);

%% Find the solution matches observed data

% initialize X0
X0 = xCYC;
X0 = xFINAL;
X0 = X + (rand-0.5)*1e-1;
X0 = X;
 % modifying X0
    X0(1:7) = ob_data(1,2:8);
    X0(1) = X0(1) + ( -0.2);
    X0(2) = X0(2) + 0.3;  
    X0(4) = X0(4) - 0.15 ;
    X0(5) = X0(5) + 0.5;
    X0(6) = X0(6) -0.4;
    X0(7) = X0(7) + 0.5;
    %X0(13) = X0(13) + (-30 + 60*rand);
    X0(13) = 40;
    X0(14) = 7;
    X0(15) = X0(15)+0.1 ;
    X0(16) = X0(16)+0.1 ;
    X0(8) = X0(8)-0.1;
    X0(9) = X0(9)+0.1;
    X0(10) = X0(10)-0.2;
    X0(11) = X0(11)-0.2;
    X0(12) = X0(12) -0.3;
    
 
  
% lb = [0.0000; 0.1000; -1.000; -pi/2; -1.1719;  -pi/2; -1.8848; 
%       0.5089; 0.6948; 0.2972;  0.1977; 1.2373;
%       10; 2.0360; -0.10; -0.38];
% ub = [5.5000; 2.5000; 1.0000;  pi/2; 10.9719;   pi/2; 10.6848; 
%       1.2089; 1.6948; 0.9972;  1.477; 2.4373;
%       60; 4.9360;  0.12; 0.18];

% lb = [];
% ub = [];

% solve fmincon
A = [];
b = [];
Aeq = [];
beq = [];

lb = [0.0000; 0.2900; -1.000; -pi/2; -5.1719;  -pi/2; -5.8848; 
      0.2089; 0.2948; 0.1972;  0.2977; 0.5373;
      10; 2.0360; -0.5; -0.5];
ub = [5.0000; 2.5000; 1.0000;  pi/2; 10.9719;   pi/2; 10.6848; 
      1.4089; 1.8948; 1.1173;  1.877; 1.4373;
      500; 10.0000;  0.5; 0.5];
  
stride_extractor(filename);
options = optimoptions('fmincon','Algorithm','sqp',...
                        'ConstraintTolerance',1e-1,...
                        'FunctionTolerance',1e-3,...
                        'StepTolerance',1e-3,...
                        'DiffMaxChange',1e-1,...
                        'FiniteDifferenceStepSize',1e-6,...
                        'MaxFunctionEvaluations',10000,...
                        'PlotFcn',@optimplotfval);
X = fmincon(@fmc_cost_fun,X0,A,b,Aeq,beq,lb,ub,@residual_constrain,options);

% solve fminsearch
stride_extractor(filename);
options = optimset('MaxIter',5000,'TolFun',10,'MaxFunEvals',10000,'PlotFcns',@optimplotfval);
X = fminsearch(@(X) fms_cost_fun(X),X0,options);

% % plot result
% X = X0;

% [residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset(X);
% figure(5)
% plot(T,Y,'LineWidth',2)
% grid on; box on
% xlim([0 T(end)]);
% xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
% 
% leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
%     ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
% set(leg1,'Interpreter','latex');
% set(leg1,'FontSize',10);

%% If this part cannot give satisfying result, use For_Loop_Optimization.

%% Plot the comparison of ob_data & simulated_data

% X = X200(:,93);

% resample simulated data (match the dimension)
[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset_optimization(X);
[xsim,ysim,alphaLsim,alphaRsim] = ZFBA_resample2(T,Y,To);
xexp = ob_data(:,1);
yexp = ob_data(:,3);
alphaLexp = ob_data(:,5);
alphaRexp = ob_data(:,7);

figure(6)
plot(To,xsim','Color',[0.6350 0.0780 0.1840],'LineWidth',3.5)
hold on
plot(To,ysim','Color',[0.9290 0.6940 0.1250],'LineWidth',3.5)
hold on
plot(To,xexp,'--','Color',[0.6350 0.0780 0.1840],'LineWidth',3.5)
hold on
plot(To,yexp,'--','Color',[0.9290 0.6940 0.1250],'LineWidth',3.5)
legend('simulated x','simulated y','observed x','observed y');
grid on; box on
xlim([0 To(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
ylabel('Distance $[l_o]$','Interpreter','LaTex')
title('Trajectories of center of mass positions')


figure(7)
plot(To,alphaLsim','Color',[0 0.4470 0.7410],'LineWidth',3.5)
hold on
plot(To,alphaRsim','Color',[0.3010 0.7450 0.9330],'LineWidth',3.5)
hold on
plot(To,alphaLexp,'--','Color',[0 0.4470 0.7410],'LineWidth',3.5)
hold on
plot(To,alphaRexp,'--','Color',[0.3010 0.7450 0.9330],'LineWidth',3.5)
grid on; box on
xlim([0 To(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
legend('simulated left','simulated right','observed left','observed right');
ylabel('Leg angles $[rad]$','Interpreter','LaTex')
title('Trajectories of center of leg angles')



states_sim = X;
as_x_sim = mean(Y(:,2));
as_y_sim = mean(Y(:,4));
as_alphaL_sim = mean(Y(:,6));
as_alphaR_sim = mean(Y(:,8));
% states_sim(1) = as_x_sim;
% states_sim(3) = as_y_sim;
% states_sim(5) = as_alphaL_sim;
% states_sim(7) = as_alphaR_sim;
disp('states_sim = ')
disp(states_sim)


states_exp = ob_data(1,2:end)';
as_x_exp = mean(ob_data(:,2));
as_y_exp = mean(ob_data(:,4));
as_alphaL_exp = mean(ob_data(:,6));
as_alphaR_exp = mean(ob_data(:,8));
% states_exp(1) = as_x_exp;
% states_exp(3) = as_y_exp;
% states_exp(5) = as_alphaL_exp;
% states_exp(7) = as_alphaR_exp;
disp('states_exp = ')
disp(states_exp)



t_sim_normalized = X(8:11)/X(12);
disp('t_sim_normalized = ')
disp(t_sim_normalized)

t_exp_normalized = footsequence'/To(end);
disp('t_exp_normailized = ')
disp(t_exp_normalized)

duty_factor_sim = (t_sim_normalized(2)-t_sim_normalized(1) + t_sim_normalized(4)-t_sim_normalized(3))/2;
disp('duty_factor_sim = ')
disp(duty_factor_sim)

duty_factor_exp = (t_exp_normalized(2)-t_exp_normalized(1) + t_exp_normalized(4)-t_exp_normalized(3))/2;
disp('duty_factor_exp = ')
disp(duty_factor_exp)

stride_length_sim = Y(end,1);
disp('stride_length_sim = ')
disp(stride_length_sim)

stride_length_exp = ob_data(end,1);
disp('stride_length_exp = ')
disp(stride_length_exp)

duty_factor_sim = [duty_factor_sim;stride_length_sim];
duty_factor_exp = [duty_factor_exp;stride_length_exp];

% cftool(xsim,xexp)
% cftool(ysim,yexp)
% cftool(alphaLsim,alphaLexp)
% cftool(alphaRsim,alphaRexp)
%%
X(15) = -X(15);
X(16) = -X(16)
save('sim_1941_j61.mat','X')
