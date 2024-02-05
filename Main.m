%% Section1: initialization
close all
clear
clc

disp('Section 1 : Enviroment Initializtion')
disp('...')
pause(2)
restoredefaultpath
current_path = string(pwd);
% slash and back slash
if ispc
    slash = '\';
else
    slash = '/';
end
disp('Adding project path...')
disp('...')
addpath(genpath(current_path+slash+'Section2_solution_examples'))
addpath(genpath(current_path+slash+'Section3_optimization'))
addpath(genpath(current_path+slash+'Section4_continuation'))
addpath(genpath(current_path+slash+'Stored_Functions')) 
pause(2)
disp('Section complete')
disp('...')

screensize = get( groot, 'Screensize' );
screen_horizontal= screensize(3);
screen_vertical = screensize(4);
%% Section2: exemplary solutions of walk, run, hop, skip and asymmetrical run
% The code in this section is used to show 5 exemplary results of different gaits.
% The readers can simply follow the instruction to finish this section.
% To finish reading of the current gait, please press Enter.
close all
clc
disp('Section2: exemplary solutions of walk, run, hop, skip and asymmetrical run')
disp('...')
pause(2)
disp('The code in this section is used to show 5 exemplary results of different gaits')
disp('...')
pause(2)
disp('The readers can simply follow the instructions.')
disp('...')
pause(2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Part1: show trajectory and animation of walking gait
disp('...')
disp('Execution starts.')
pause(2)
disp('...')
disp('Current Gait:  Walking... ')
disp('...')
pause(2);
% load the initial state and NLSA from existing data
load('W1.mat')
index = 30;
X = results(1:12,index);
Offsets = results(13:14,index);


% Show the solution and the current branch on Poincare Section
% This figure is in the same style of Figure4 in the manuscript
roadmap = figure;
view([0 90]);  pbaspect([1 1 1])  % figure settings
xlim([0 40]);  ylim([-3 3])
hold on;       box on
set(roadmap,'position',[0.125*screen_horizontal 0.575*screen_vertical 0.45*screen_horizontal 0.35*screen_vertical])
xlabel('$\dot{x}$','Interpreter','LaTex','FontSize',15)
ylabel('$\alpha_l$','Interpreter','LaTex','FontSize',15)
zlabel('$y$','Interpreter','LaTex','FontSize',15)
title('Roadmap of Solution Branches')
% define line width on this plot
LineWidth = 2;
% plot walking branch
p1 = plot3(results(1,:),results(4,:),results(2,:),'LineWidth',LineWidth,'Color',[0.4940 0.1840 0.5560]); % 'left-advanced' branch
p2 = plot3(results(1,:),results(6,:),results(2,:),'LineWidth',LineWidth,'Color',[0.4940 0.1840 0.5560]); % 'right-advanced' branch
% plot current solution
sc_w = scatter3(X(1),X(4),X(2),60,[0 0 0],'*');
datatip(sc_w,X(1),X(4)); % in the data tip, X stands for x_dot, Y stands for alpha_l, Z stands for y
pause(2)

% reproduce the gait using our model
[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset(X,Offsets);
% show trajectories 
disp('Showing Walking Trajectories...')
disp('...')
walk = figure;
set(walk,'position',[0.575*screen_horizontal 0.575*screen_vertical 0.25*screen_horizontal 0.35*screen_vertical])
plot(T,Y,'LineWidth',2)
grid on; box on
xlim([0 T(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
    ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);
title('Trajetories of a Walking Gait')
pause(2)

% Show animation
disp('Showing Walking Animation...Please DONOT drag the figure windows before animation stop.')
disp('...')
ShowTrajectory_BipedalDemo(T,Y,P,'Test',screen_horizontal,screen_vertical);
pause(2)


% Press Enter to move on next gait
disp('Walking section complete. To move on, please press Enter')
disp('...')
pause(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Part2: show trajectory and animation of Running gait
disp('...')
disp('Current Gait:  Running...  ')
disp('...')
pause(2);
% load the initial state and NLSA from existing data
load('R1.mat')
index = 30;
X = results(1:12,index);
Offsets = results(13:14,index);

% Show the solution and the current branch on Poincare Section
% This figure is in the same style of Figure 4 in the manuscript
figure(roadmap); hold on
delete(findall(roadmap,'Type','hggroup'))% remove all the data tips
% plot running branch
p3 = plot3(results(1,:),results(4,:),results(2,:),'LineWidth',LineWidth,'Color',[0.9290 0.6940 0.1250]); % 'left-advanced' branch
p4 = plot3(results(1,:),results(6,:),results(2,:),'LineWidth',LineWidth,'Color',[0.9290 0.6940 0.1250]); % 'right-advanced' branch
% plot current solution
sc_w = scatter3(X(1),X(4),X(2),60,[0 0 0],'*');
datatip(sc_w,X(1),X(4)); % in the data tip, X stands for x_dot, Y stands for alpha_l, Z stands for y
%Plot walk-run transition point
p5 = scatter3(results(1,1),results(4,1),results(2,1),90,[0 0 0],'filled','d');
p6 = scatter3(results(1,1),results(6,1),results(2,1),90,[0 0 0],'filled','d');
pause(2)


% Reproduce the gait using our model
[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset(X,Offsets);
% show trajectories
disp('Showing Running Trajectories...')
disp('...')
run = figure;
set(run,'position',[0.575*screen_horizontal 0.575*screen_vertical 0.25*screen_horizontal 0.35*screen_vertical])
plot(T,Y,'LineWidth',2)
grid on; box on
xlim([0 T(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
    ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);
title('Trajetories of a Running Gait')
pause(2)

% show animation
disp('Showing Running Animation...Please DONOT drag the figure windows before animation stop')
disp('...')
ShowTrajectory_BipedalDemo(T,Y,P,'Test',screen_horizontal,screen_vertical);
pause(2)

% Press Enter to move on next gait
disp('Running section complete. To move on, please press Enter')
disp('...')
pause(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Part3: show trajectory and animation of Hopping gait
disp('...')
disp('Current Gait:  Hopping...')
disp('...')
pause(2);
% load the initial state and NLSA from existing data
load('HP1.mat')
index = 50;
X = results(1:12,index);
Offsets = results(13:14,index);
% reproduce the gait using our model
[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset(X,Offsets);


% Show the solution and the current branch on Poincare Section
% This figure is in the same style of Figure 4 in the manuscript
figure(roadmap); hold on
delete(findall(roadmap,'Type','hggroup')) % remove all the data tips
% plot hopping branch
p7 = plot3(results(1,:),results(4,:),results(2,:),'LineWidth',LineWidth,'Color',[0.8500 0.3250 0.0980]);
% plot current solution
sc_w = scatter3(X(1),X(4),X(2),60,[0 0 0],'*');
datatip(sc_w,X(1),X(4)); % in the data tip, X stands for x_dot, Y stands for alpha_l, Z stands for y
pause(2)


% show trajectories
disp('Showing Hopping Trajectories...')
disp('...')
hop = figure;
set(hop,'position',[0.575*screen_horizontal 0.575*screen_vertical 0.25*screen_horizontal 0.35*screen_vertical])
plot(T,Y,'LineWidth',2)
grid on; box on
xlim([0 T(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
    ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);
title('Trajetories of a Hopping Gait')
pause(2)

% show animation
disp('Showing Hopping Animation...Please DONOT drag the figure windows before animation stop')
disp('...')
ShowTrajectory_BipedalDemo(T,Y,P,'Test',screen_horizontal,screen_vertical);
pause(2)


% Press Enter to move on next gait
disp('Hopping section complete. To move on, please press Enter')
disp('...')
pause(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Part4: show trajectory and animation of Skipping gait
disp('...')
disp('Current Gait:  Skipping...')
disp('...')
pause(2);
% load the initial state and NLSA from existing data
load('SK1.mat')
index = 30;
X = results(1:12,index);
Offsets = results(13:14,index);
% reproduce the gait using our model
[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset(X,Offsets);


% Show the solution and the current branch on Poincare Section
% This figure is in the same style of Figure 4 in the manuscript
figure(roadmap); hold on
delete(findall(roadmap,'Type','hggroup')) % remove all the data tips
% plot skipping branch
p8 = plot3(results(1,:),results(4,:),results(2,:),'LineWidth',LineWidth,'Color',[0 0.4470 0.7410]); % 'left-advanced' branch
p9 = plot3(results(1,:),results(6,:),results(2,:),'LineWidth',LineWidth,'Color',[0 0.4470 0.7410]); % 'right-advanced' branch
% plot current solution
sc_w = scatter3(X(1),X(4),X(2),60,[0 0 0],'*');
datatip(sc_w,X(1),X(4)); % in the data tip, X stands for x_dot, Y stands for alpha_l, Z stands for y
% plot hop-skip transition
p10 = scatter3(results(1,1),results(4,1),results(2,1),90,[0 0 0],'filled','o');

load('SK2.mat') % another part of skipping branch
p11 = plot3(results(1,:),results(4,:),results(2,:),'LineWidth',LineWidth,'Color',[0 0.4470 0.7410]); % 'left-advanced' branch
p12 = plot3(results(1,:),results(6,:),results(2,:),'LineWidth',LineWidth,'Color',[0 0.4470 0.7410]); % 'right-advanced' branch
p13 = scatter3(results(1,end),results(4,end),results(2,end),90,[0 0 0],'filled','o');
pause(2)



% show trajectories
disp('Showing Skipping Trajectories...')
disp('...')
skip = figure;
set(skip,'position',[0.575*screen_horizontal 0.575*screen_vertical 0.25*screen_horizontal 0.35*screen_vertical])
plot(T,Y,'LineWidth',2)
grid on; box on
xlim([0 T(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
    ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);
title('Trajetories of a Skipping Gait')
pause(2)

% show animation
disp('Showing Skipping Animation...Please DONOT drag the figure windows before animation stop')
disp('...')
ShowTrajectory_BipedalDemo(T,Y,P,'Test',screen_horizontal,screen_vertical);
pause(2)


% Press Enter to move on next gait
disp('Skipping section complete. To move on, please press Enter')
disp('...')
pause(3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Part5: show trajectory and animation of Asymmetrical Running gait
disp('...')
disp('Current Gait:  Asymmetrical Running...')
disp('...')
pause(2);
% load the initial state and NLSA from existing data
load('AR1.mat')
index = 50;
X = results(1:12,index);
Offsets = results(13:14,index);
% reproduce the gait using our model
[~, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset(X,Offsets);


% Show the solution and the current branch on Poincare Section
% This figure is in the same style of Figure 4 in the manuscript
figure(roadmap); hold on
delete(findall(roadmap,'Type','hggroup')) % remove all the data tips
% plot asymmetrical running branch
p14 = plot3(results(1,:),results(4,:),results(2,:),'LineWidth',LineWidth,'Color',[0.4660 0.6740 0.1880]); % 'left-advanced' branch
p15 = plot3(results(1,:),results(6,:),results(2,:),'LineWidth',LineWidth,'Color',[0.4660 0.6740 0.1880]); % 'right-advanced' branch
% plot current solution
sc_w = scatter3(X(1),X(4),X(2),60,[0 0 0],'*');
datatip(sc_w,X(1),X(4)); % in the data tip, X stands for x_dot, Y stands for alpha_l, Z stands for y
% plot skip-asymmetrical run transition
p16 = scatter3(results(1,1),results(4,1),results(2,1),90,[0 0 0],'filled','^');
p17 = scatter3(results(1,end),results(4,end),results(2,end),90,[0 0 0],'filled','^'); % 'left-advanced'
p18 = scatter3(results(1,1),results(6,1),results(2,1),90,[0 0 0],'filled','v');
p19 = scatter3(results(1,end),results(6,end),results(2,end),90,[0 0 0],'filled','v'); % 'right-advanced'
pause(2)


% show trajectories
disp('Showing Asymmetrical Running Trajectories...')
disp('...')
asym_run = figure;
set(asym_run,'position',[0.575*screen_horizontal 0.575*screen_vertical 0.25*screen_horizontal 0.35*screen_vertical])
plot(T,Y,'LineWidth',2)
grid on; box on
xlim([0 T(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
    ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);
title('Trajetories of a Asymmetrical Running Gait')
pause(2)

% show animation
disp('Showing Asymmetrical Running Animation...Please DONOT drag the figure windows before animation stop')
disp('...')
ShowTrajectory_BipedalDemo(T,Y,P,'Test',screen_horizontal,screen_vertical);
pause(2)


% Section complete
disp('Asymmetrical Running section complete.')
disp('...')
disp('...')
pause(3)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Showing Figure.4 A & B...')
disp('...')
% Figure.4 B
figure(roadmap)
set(roadmap,'position',[0.15*screen_horizontal+0.7*screen_vertical 0.2*screen_vertical 0.7*screen_vertical 0.7*screen_vertical])
delete(findall(roadmap,'Type','hggroup')) % remove all the data tips
legend([p1 p3 p7 p8 p14  p5 p10 p16 sc_w],{'Walking Branch','Running Branch',...
        'Hopping Branch','Skipping Branch','Asymmetrical Running Branch',...
        'Walk-Run Transition','Skip-Hop Transition','Skip-Asymmetrical Run Transition',...
        'exemples of Each Gait'})
title('Figure.4 B')
% Figure.4 A 
savefig(roadmap,'roadmap_B.fig')
roadmap_A = openfig('roadmap_B.fig');
figure(roadmap_A)
set(roadmap_A,'position',[0.15*screen_horizontal 0.2*screen_vertical 0.7*screen_vertical 0.7*screen_vertical])
legend off; xlim([0 40]); ylim([-3 3]); zlim([0 1.05]);
view([45 45]); pbaspect([1 1 0.8])
title('Figure.4 A')
% clean path
delete *.fig
disp(' Section Complete. ')
disp('...')

%% Section 3: Optimization Process
% This section shows how we find an optimized solution given a experimental trajectory.
% The sim-exp comparison is also showing in this part, including the data
% in Table.1 and Figure.2 
close all
clc
disp('Section 3: Optimization and sim-exp plotting')
disp('...')
pause(2)
disp('This section shows how we find an optimized solution given a experimental trajectory.')
disp('...')
pause(3)
disp('The sim-exp comparison is also showing in this part, including the data in Table.1 and Figure.2') 
disp('...')
pause(3)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Part1: Optimization Process

disp('Part1: Optimization Process...')
disp('...')
pause(2)
% load experimental data
disp('Loading experimental dataset: exp_1802_j30.mat.')
disp('...')
pause(3)
filename_exp = 'exp_1802_j30.mat';
load(filename_exp)
xexp = ob_data(:,1);
yexp = ob_data(:,3);
alphaLexp = ob_data(:,5);
alphaRexp = ob_data(:,7);
pause(2)

% showing experimental trajectories of current trial
disp('Showing experimental dataset. This plot is analogous to Figure.2 in the manuscript.')
disp('...')
positions = figure; hold on % sim-exp: horizontal and vertical positions
set(positions,'position',[0.175*screen_horizontal 0.5*screen_vertical 0.55*screen_vertical 0.4*screen_vertical])
plot(To,xexp,'--','Color',[0.6350 0.0780 0.1840],'LineWidth',3.5)
plot(To,yexp,'--','Color',[0.9290 0.6940 0.1250],'LineWidth',3.5)
legend('experiment x','experiment y');
grid on; box on
xlim([0 To(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
ylabel('Distance $[l_o]$','Interpreter','LaTex')
title('Trajectories of center of mass positions')

leg_angles = figure; hold on % sim-exp: left and right leg angles
set(leg_angles,'position',[0.175*screen_horizontal+0.55*screen_vertical 0.5*screen_vertical 0.55*screen_vertical 0.4*screen_vertical])
plot(To,alphaLexp,'--','Color',[0 0.4470 0.7410],'LineWidth',3.5)
plot(To,alphaRexp,'--','Color',[0.3010 0.7450 0.9330],'LineWidth',3.5)
grid on; box on
xlim([0 To(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
legend('experiment left','experiment right');
ylabel('Leg angles $[rad]$','Interpreter','LaTex')
title('Trajectories of leg angles')
pause(5)


% loading initial guess of optimization and plot the sim trajectories
disp('Loading pre-saved initial guess and reproducing simulation results.')
disp('...')
filename_sim = 'sim_1802_j30.mat';
load(filename_sim)
% Tuning initial guess
xInitial = X;
xInitial(1) = xInitial(1) +  (-0.2 + 0.40*rand);                % revising states
xInitial(2) = xInitial(2) +  (-0.01 + 0.02*rand);  
xInitial(4) = xInitial(4) +  ( -0.02 + 0.04*rand);
xInitial(5) = xInitial(5) +  (-0.10 + 0.20*rand);
xInitial(6) = xInitial(6) +  ( -0.02 + 0.04*rand);
xInitial(7) = xInitial(7) +  (-0.10 + 0.20*rand);
% xInitial(13) = xInitial(13) + (-5 + 10*rand);                   % revising parameters
% xInitial(14) = xInitial(14) + (-0.10 + 0.20*rand);
% xInitial(15) = xInitial(15) + ( -0.02 + 0.04*rand);
% xInitial(16) = xInitial(16) + ( -0.02 + 0.04*rand);

[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset_optimization(xInitial); % reproduce sim results
pause(2)
% plotting simulation trajectories.
disp('Plotting simulation trajectories of initial guess.')
disp('...')
initial_guess = figure; grid on;
set(initial_guess ,'position',[0.175*screen_horizontal 0.125*screen_vertical 0.55*screen_vertical 0.3*screen_vertical]); grid on; box on
plot(T,Y,'LineWidth',2)
xlim([0 T(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
title('Trajectories of initial guess')

leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
    ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);
pause(5)
% plotting simulation of initial guess - experimental data
disp('Plotting simulation of initial guess - experimental data.')
disp('...')
sim_exp_plot(xInitial,ob_data,To,positions,leg_angles)
pause(3)

% optimization process
disp('Calling optimization function...')
disp('...')
% optimization start
stride_extractor(filename_exp); % globalize the experimental data, called in optimization functions later
% fminsearch function, switch to fmincon by comment the following lines
options = optimset('MaxIter',5000,'TolFun',10,'MaxFunEvals',1000,'PlotFcns',@optimplotfval); % options setup
xFinal = fminsearch(@(X) fms_cost_fun(X),xInitial,options); % predefined optimization function
% % % fmincon function
% % A = [];
% % b = [];
% % Aeq = [];
% % beq = [];
% % lb = [0.0000; 0.2900; -1.000; -pi/2; -5.1719;  -pi/2; -5.8848; 
% %       0.2089; 0.2948; 0.1972;  0.2977; 0.5373;
% %       10; 2.0360; -0.5; -0.5];
% % ub = [5.0000; 2.5000; 1.0000;  pi/2; 10.9719;   pi/2; 10.6848; 
% %       1.4089; 1.8948; 1.1173;  1.877; 1.4373;
% %       500; 10.0000;  0.5; 0.5];
% %   
% % stride_extractor(filename);
% % options = optimoptions('fmincon','Algorithm','sqp',...
% %                         'ConstraintTolerance',1e-1,...
% %                         'FunctionTolerance',1e-3,...
% %                         'StepTolerance',1e-3,...
% %                         'DiffMaxChange',1e-1,...
% %                         'FiniteDifferenceStepSize',1e-6,...
% %                         'MaxFunctionEvaluations',10000,...
% %                         'PlotFcn',@optimplotfval);
% % xFinal = fmincon(@fmc_cost_fun,xInitial,A,b,Aeq,beq,lb,ub,@fmc_residual_constrain,options);
pause(3)

% showing trajectories of optimized result
[residual, T,Y,~,Y_EVENT,TE] = ZeroFunc_BipedApex_offset_optimization(xFinal); % reproduce sim results
% plotting simulation trajectories.
disp('Plotting simulation trajectories of optimized result.')
disp('...')
optimized_result = figure; grid on;
set(optimized_result,'position',[0.175*screen_horizontal+0.55*screen_vertical 0.125*screen_vertical 0.55*screen_vertical 0.3*screen_vertical]); grid on; box on
plot(T,Y,'LineWidth',2)
xlim([0 T(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
title('Trajectories of optimized result')

leg1 = legend('$x$','$\dot{x}$','$y$','$\dot{y}$'...
    ,'$\alpha_l$','$\dot{\alpha_l}$','$\alpha_r$','$\dot{\alpha_r}$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);
pause(5)

disp('Showing sim-exp results')
disp('...')
% plotting optimization results: sim vs exp
sim_exp_plot(xFinal,ob_data,To,positions,leg_angles);
pause(3)

disp('Optimization complete.')
disp('...')
pause(5)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Part2: Showing saved optimization results and data in the table
% The results can be checked by modifying the file name in the following
% line, or directly go to folder 'saved_results' and double click them

% close all
close(optimized_result)
close(initial_guess)
disp('Part2: Showing the pre-storage optimization results and calculating the table values')
disp('...')
pause(3)
disp('The sim and exp dataset can be selected by modifying the file name in the following line,')
disp('...')
pause(3)
disp('or directly go to folder "saved_results" and double click them.')
disp('...')
pause(3)
disp('The 12 trials showing in Table.1 are stored in folder "saved_results". ')
disp('...')
pause(3)
disp('Showing the pre-storage optimization results: 1802_j30')
disp('...')

% load existing exp and sim data
filename_exp = 'exp_1802_j30.mat';
filename_sim = 'sim_1802_j30.mat';
load(filename_exp)
load(filename_sim)
% plotting optimization results: sim vs exp
[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset_optimization(X); % reproduce sim results
% positions = figure; set(positions,'position',[400 300 550 400])
% leg_angles = figure; set(leg_angles,'position',[950 300 550 400])
% sim_exp_plot(X,ob_data,To,positions,leg_angles);
[xsim,ysim,alphaLsim,alphaRsim] = ZFBA_resample2(T,Y,To); % resample exp data
xexp = ob_data(:,1);
yexp = ob_data(:,3);
alphaLexp = ob_data(:,5);
alphaRexp = ob_data(:,7);
pause(2)


% showing the values in the table
disp('Showing sim-exp values: states, normalized timing, duty factor, parameters, etc... ')
disp('...')
pause(2)
% showing values in the table
states_sim = X(1:7);                                % states of sim-exp data
disp('states_sim = ')
disp(states_sim)
states_exp = ob_data(1,2:end)';
disp('states_exp = ')
disp(states_exp)
pause(3)

as_x_sim = mean(Y(:,2));                       % average speed of sim-exp data
disp('average_speed_sim = ')
disp(as_x_sim)
as_x_exp = mean(ob_data(:,2));
disp('average_speed_exp = ')
disp(as_x_exp)
pause(3)

t_sim_normalized = X(8:11)/X(12);              % normalized timing of sim-exp data
disp('t_sim_normalized = ')
disp(t_sim_normalized)
t_exp_normalized = footsequence'/To(end);
disp('t_exp_normailized = ')
disp(t_exp_normalized)
pause(3)

duty_factor_sim = (t_sim_normalized(2)-t_sim_normalized(1) + t_sim_normalized(4)-t_sim_normalized(3))/2;
disp('duty_factor_sim = ')                     % duty factor of sim-exp data
disp(duty_factor_sim)
duty_factor_exp = (t_exp_normalized(2)-t_exp_normalized(1) + t_exp_normalized(4)-t_exp_normalized(3))/2;
disp('duty_factor_exp = ')
disp(duty_factor_exp)
pause(3)

stride_length_sim = Y(end,1);                  % stride length of sim-exp data
disp('stride_length_sim = ')
disp(stride_length_sim)
stride_length_exp = ob_data(end,1);
disp('stride_length_exp = ')
disp(stride_length_exp)
pause(3)

parameters_sim = X(13:16);
disp('parameters_sim = ')
disp(parameters_sim)
pause(3)


disp('...')
disp('Section Complete.')
disp('...')

%% Section4: Continuation Process and Gait Searching
% This section shows the code to run our continuation algorithm for gait searching.
% A plot analogous to Figure.8 in the manuscript is also shown in this part.
close all
clc
disp('Section4: Continuation Process and Gait Searching')
disp('...')
pause(2)

% Numerical Continuation
disp('Part1: Numerical Continuation.')
disp('...')
pause(3)
disp('This part only shows a demo running of our continuation algorithm.')
disp('...')
pause(3)
disp('To complete the search for a solution branch, please enlarge the input: "iterations" in "NumericalContinuation1D".')
disp('...')
pause(3)
% load two solutions of NLSW = [0;0]
load('GP_Initials.mat')
xFINAL1 = Initials(1:12,47);
xFINAL2 = Initials(1:12,48);
Offsets = Initials(13:14,47);
Offsets(1) = 0.03; 
numOPTS = optimset('Algorithm','levenberg-marquardt',... 
                   'Display','iter',...
                   'MaxFunEvals',8000,...
                   'MaxIter',3000,...
                   'UseParallel', false,...
                   'TolFun',1e-9,...
                   'TolX',1e-12);
% calculate two solution of current NLSW:[0.03; 0]
disp('Current NLSW: [0.03  0].')
disp('...')
pause(3)
disp('Finding the first two solutions...')
disp('...')
pause(2)
[X1, ~] = fsolve(@(X) ZeroFunc_BipedApex_offset(X,Offsets), xFINAL1, numOPTS);
[X2, ~] = fsolve(@(X) ZeroFunc_BipedApex_offset(X,Offsets), xFINAL2, numOPTS);
radius = 0.05;
iterations = 10;

disp('...')
disp('Finding the whole solution branch for current NLSW value...')
disp('...')
pause(3)
% searching for solution branch
results = NumericalContinuation1D(X1,X2,Offsets,radius,numOPTS,iterations);
disp('...')
disp('Numerical Search complete.')
disp('...')
pause(3)

% Part2: Show the solution branch (replot Figure.8)
% showing the roadmap
disp('Part2: Plotting solution branches.')
disp('...')
pause(3)
disp('In this part, we are plotting the pre-saved results (analogous to Figure.8 in manuscript).')
disp('...')
pause(3)
disp('The reader can also search for a complete branch using the code above and plot it.')
disp('...')
pause(3)


disp('Plotting starts...')
disp('...')
pause(2)
disp('Plotting the left advanced part of the roadmap (Figure.4, NLSW = [0; 0]) as reference...')
disp('...')
pause(3)
fig8_A = figure;
set(fig8_A,'position',[0.15*screen_horizontal 0.2*screen_vertical 0.7*screen_vertical 0.7*screen_vertical]);   hold on;   box on;
xlim([0 40]); ylim([-3 3]); zlim([0 1.05]);  % figure settings
view([45 45]); pbaspect([1 1 0.8]);
LineWidth = 2;

xlabel('$\dot{x}$','Interpreter','LaTex','FontSize',15)
ylabel('$\alpha_l$','Interpreter','LaTex','FontSize',15)
zlabel('$y$','Interpreter','LaTex','FontSize',15)
title('Figure.8 A')


cd(current_path+slash+'Section2_solution_examples')
fileinfo = dir('*.mat'); % load the solutions for the roadmap
fnames = {fileinfo.name};
for i = 1:size(fnames,2)
    filename = cell2mat(fnames(i));
    load(filename)
    switch filename(1) % plot the solution branches of each gait and the transition points respectively
        case 'W'
            plot3(results(1,:),results(4,:),results(2,:),'LineWidth',LineWidth,'Color',brighten([0.4940 0.1840 0.5560],0.8))
        case 'R'
            plot3(results(1,:),results(4,:),results(2,:),'LineWidth',LineWidth,'Color',brighten([0.9290 0.6940 0.1250],0.8))
            scatter3(results(1,1),results(4,1),results(2,1),90,[0 0 0],'filled','d')
        case 'H'
            plot3(results(1,:),results(4,:),results(2,:),'LineWidth',LineWidth,'Color',brighten([0.8500 0.3250 0.0980],0.8))
        case 'S'
            plot3(results(1,:),results(4,:),results(2,:),'LineWidth',LineWidth,'Color',brighten([0 0.4470 0.7410],0.8))
            if filename(3)=='1'
                scatter3(results(1,1),results(4,1),results(2,1),90,[0 0 0],'filled','o')
            else 
                scatter3(results(1,end),results(4,end),results(2,end),90,[0 0 0],'filled','o')
            end
        case 'A' 
            plot3(results(1,:),results(4,:),results(2,:),'LineWidth',LineWidth,'Color',brighten([0.4660 0.6740 0.1880],0.8))
            scatter3(results(1,1),results(4,1),results(2,1),90,[0 0 0],'filled','^')
            scatter3(results(1,end),results(4,end),results(2,end),90,[0 0 0],'filled','^')
    end
end
cd(current_path)
savefig(fig8_A,'fig8_A.fig')
fig8_B = openfig('fig8_A.fig');
figure(fig8_B)
set(fig8_B,'position',[0.15*screen_horizontal+0.7*screen_vertical 0.2*screen_vertical 0.7*screen_vertical 0.7*screen_vertical])
title('Figure.8 B')
delete *.fig % clean path
pause(3)

% plot solution branches
disp('Plotting the solution branches of:  A) NLSW = [0.06; 0]....')
disp('...')
% Figure.8 A
figure(fig8_A)
load('results_13_offsetL_0.06_offsetR_0.mat')
index_ar = find(results(11,:)<results(8,:));
plot3(results(1,1:index_ar(1)),results(4,1:index_ar(1)),results(2,1:index_ar(1)),...
    'LineWidth',LineWidth,'Color',[0 0.4470 0.7410])                                           % skipping part
plot3(results(1,index_ar(end):end),results(4,index_ar(end):end),results(2,index_ar(end):end),...
    'LineWidth',LineWidth,'Color',[0 0.4470 0.7410])                                           % skipping part
plot3(results(1,index_ar(1):index_ar(end)),results(4,index_ar(1):index_ar(end)),...
    results(2,index_ar(1):index_ar(end)),'LineWidth',LineWidth,'Color',[0.4660 0.6740 0.1880]) % asymmetrical running part
scatter3(results(1,index_ar(1)),results(4,index_ar(1)),results(2,index_ar(1)),90,[0 0 0],'filled','^')
scatter3(results(1,index_ar(end)),results(4,index_ar(end)),results(2,index_ar(end)),90,[0 0 0],'filled','^')
pause(3)

% Figure.8 B
disp('Plotting the solution branches of:  B) NLSW = [-0.03; 0]....')
disp('...')
figure(fig8_B)
load('results_4_offsetL_-0.03_offsetR_0.mat')
index_ar = find(results(11,:)<results(8,:));
plot3(results(1,1:index_ar(1)),results(4,1:index_ar(1)),results(2,1:index_ar(1)),...
    'LineWidth',LineWidth,'Color',[0 0.4470 0.7410])                                           % skipping part
plot3(results(1,index_ar(end):end),results(4,index_ar(end):end),results(2,index_ar(end):end),...
    'LineWidth',LineWidth,'Color',[0 0.4470 0.7410])                                           % skipping part
plot3(results(1,index_ar(1):index_ar(end)),results(4,index_ar(1):index_ar(end)),...
    results(2,index_ar(1):index_ar(end)),'LineWidth',LineWidth,'Color',[0.4660 0.6740 0.1880]) % asymmetrical running part
scatter3(results(1,index_ar(1)),results(4,index_ar(1)),results(2,index_ar(1)),90,[0 0 0],'filled','^')
scatter3(results(1,index_ar(end)),results(4,index_ar(end)),results(2,index_ar(end)),90,[0 0 0],'filled','^')
pause(2)


disp('Plotting complete.')
disp('...')
pause(2)
disp('Section Complete.')