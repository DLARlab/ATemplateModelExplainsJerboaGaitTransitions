function sim_exp_plot(xFinal,ob_data,To,fig1,fig2)
[residual, T,Y,P,Y_EVENT,TE] = ZeroFunc_BipedApex_offset_optimization(xFinal); % reproduce sim results
[xsim,ysim,alphaLsim,alphaRsim] = ZFBA_resample2(T,Y,To); % resample exp data
xexp = ob_data(:,1);
yexp = ob_data(:,3);
alphaLexp = ob_data(:,5);
alphaRexp = ob_data(:,7);

figure(fig1); clf; hold on
plot(To,xsim','Color',[0.6350 0.0780 0.1840],'LineWidth',3.5)
plot(To,ysim','Color',[0.9290 0.6940 0.1250],'LineWidth',3.5)
plot(To,xexp,'--','Color',[0.6350 0.0780 0.1840],'LineWidth',3.5)
plot(To,yexp,'--','Color',[0.9290 0.6940 0.1250],'LineWidth',3.5)
legend('simulated x','simulated y','observed x','observed y');
grid on; box on
xlim([0 To(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
ylabel('Distance $[l_o]$','Interpreter','LaTex')
title('Trajectories of center of mass positions')

figure(fig2); clf; hold on
plot(To,alphaLsim','Color',[0 0.4470 0.7410],'LineWidth',3.5)
plot(To,alphaRsim','Color',[0.3010 0.7450 0.9330],'LineWidth',3.5)
plot(To,alphaLexp,'--','Color',[0 0.4470 0.7410],'LineWidth',3.5)
plot(To,alphaRexp,'--','Color',[0.3010 0.7450 0.9330],'LineWidth',3.5)
grid on; box on
xlim([0 To(end)]);
xlabel('Time $[\sqrt{l_o/g}]$','Interpreter','LaTex');
legend('simulated left','simulated right','observed left','observed right');
ylabel('Leg angles $[rad]$','Interpreter','LaTex')
title('Trajectories of leg angles')
end