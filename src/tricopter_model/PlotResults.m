% Test name
name = 'inertia_REDUX_5';

% Plot time
Tstart = 5;
Tend = 7.5;
fontSize = 13;
test = load('untitled.mat');

%% Attitude / Input
figure
subplot(2,1,1)
plot(test.ans.Attitude,'linewidth',2);
xlim([Tstart,Tend]);
title('Attitude')
l1 = legend('\phi', '\theta', '\psi', 'location', 'eastoutside');
set(l1,'FontSize',fontSize);
ylabel('[deg]')
set(gca,'fontsize', fontSize);

% %% East position plot
% subplot(2,1,1)
% plot(test.ans.X_e,'linewidth',2);
% xlim([Tstart,Tend]);
% title('Earth Position')
% legend('X_e', 'Y_e', 'Z_e','location', 'eastoutside')
% ylabel('[deg]')
% 
% subplot(2,1,1)
% plot(test.ans.X_e.time,test.ans.X_e.Data(:,3),'linewidth',2);
% xlim([Tstart,Tend]);
% title('Altitude')
% legend('Z_e','location', 'eastoutside')
% ylabel('[deg]')

subplot(2,1,2)
[ax,p1,p2] = plotyy(test.ans.throttle.Time, test.ans.throttle.Data, test.ans.eta1.Time, test.ans.eta1.Data);
ylabel(ax(1),'Throttle') % label left y-axis
ylabel(ax(2),'\eta_1') % label right y-axis
xlabel(ax(2),'Time') % label x-axis
set(p1, 'LineWidth', 2)
set(p2, 'LineWidth', 2)
xlim(ax(1),[Tstart,Tend]);
xlim(ax(2),[Tstart,Tend]);
l2 = legend('\tau_1', '\tau_2', '\tau_3', '\eta_1', 'location', 'eastoutside');
set(l2,'FontSize',fontSize);
title('Input command')
set(ax,'fontsize', fontSize)

% %% Inversion error
% figure
% subplot(2,1,1)
% plot(test.ans.inv_error.time, test.ans.inv_error.Data(:,2),'linewidth',2);
% xlim([Tstart,Tend]);
% title('Attitude')
% % l1 = legend('\phi', '\theta', '\psi', 'location', 'eastoutside');
% set(l1,'FontSize',fontSize);
% ylabel('[deg]')
% set(gca,'fontsize', fontSize);
% 
% subplot(2,1,2)
% [ax,p1,p2] = plotyy(test.ans.v_ad.time, test.ans.v_ad.Data(:,2),test.ans.inv_error.time, test.ans.inv_error.Data(:,2));
% ylabel(ax(1),'Throttle') % label left y-axis
% ylabel(ax(2),'\eta_1') % label right y-axis
% xlabel(ax(2),'Time') % label x-axis
% set(p1, 'LineWidth', 2)
% set(p2, 'LineWidth', 2)
% xlim(ax(1),[Tstart,Tend]);
% xlim(ax(2),[Tstart,Tend]);
% l2 = legend('v_{aD}', '\Delta_{inv}', 'location', 'eastoutside');
% set(l2,'FontSize',fontSize);
% title('Input command')
% set(ax,'fontsize', fontSize)

%% Speed test plot
% Attitude plot
% subplot(3,1,1)
% plot(test.ans.Attitude,'linewidth',2);
% xlim([Tstart,Tend]);
% title('Attitude')
% l1 = legend('\phi', '\theta', '\psi', 'location', 'eastoutside');
% set(l1,'FontSize',fontSize);
% ylabel('[deg]')
% set(gca,'fontsize', fontSize);
% 
% % Earth position plot
% subplot(3,1,2)
% plot(test.ans.X_e,'linewidth',2);
% xlim([Tstart,Tend]);
% title('Earth Position')
% legend('X_e', 'Y_e', 'Z_e','location', 'eastoutside')
% ylabel('[deg]')
% 
% % Control input plot
% subplot(3,1,3)
% [ax,p1,p2] = plotyy(test.ans.throttle.Time, test.ans.throttle.Data, test.ans.eta1.Time, test.ans.eta1.Data);
% ylabel(ax(1),'Throttle') % label left y-axis
% ylabel(ax(2),'\eta_1') % label right y-axis
% xlabel(ax(2),'Time') % label x-axis
% set(p1, 'LineWidth', 2)
% set(p2, 'LineWidth', 2)
% xlim(ax(1),[Tstart,Tend]);
% xlim(ax(2),[Tstart,Tend]);
% l2 = legend('\tau_1', '\tau_2', '\tau_3', '\eta_1', 'location', 'eastoutside');
% set(l2,'FontSize',fontSize);
% title('Input command')
% set(ax,'fontsize', fontSize)



%% RMSE calculation
error = test.ans.att_com.Data([100*Tstart:100*Tend],:)*180/pi - test.ans.Attitude.Data([100*Tstart:100*Tend],:);
RMSE = sqrt(sum(error.^2)/length(error))


%% Save figure
print(sprintf('figures\\%s', name),'-dpng')
print(sprintf('figures/pdf\\%s', name),'-dpdf')
savefig(sprintf('figures/fig\\%s', name))
save(['test/' name '.mat'], 'test', 'error', 'RMSE');
name = '';


% %% Plot throttle and eta separately
% subplot(3,1,2)
% axis equal
% plot(test.ans.throttle,'linewidth',2);
% xlim([10,Tplot]);
% title('Throttle')
% legend('\tau_1', '\tau_2', '\tau_3')
% ylabel('[0,1]')
% 
% subplot(3,1,3)
% plot(test.ans.eta1,'linewidth',2);
% xlim([10,Tplot]);
% title('Tilting servo angle')
% legend('\eta_1')
% ylabel('[deg]')

% %% Gyroscopic vs aerodynamics moments
% % step of 5 deg to phi, theta, psi that forces the tilting servo to
% % saturate at mas speed providing maximum gyro moment. It is compared with
% % the throttle and aerodynamic moment created by the same arm.
% subplot(1,2,1)
% [ax1,p11,p22] = plotyy(test.ans.throttle.Time, test.ans.throttle.Data(:,1), test.ans.eta1.Time, test.ans.eta1.Data);
% ylabel(ax1(1),'Throttle') % label left y-axis
% ylabel(ax1(2),'\eta_1 [deg]') % label right y-axis
% xlabel(ax1(2),'Time') % label x-axis
% set(p11, 'LineWidth', 2)
% set(p22, 'LineWidth', 2)
% xlim(ax1(1),[9,Tplot]);
% xlim(ax1(2),[9,Tplot]);
% legend('\tau_1', '\eta_1')
% title('Input command')
% 
% subplot(1,2,2)
% [ax,p1,p2] = plotyy(gyro_comp.Time, gyro_comp.Data(:,1), gyro_comp.Time, gyro_comp.Data(:,2));
% ylabel(ax(1),'Aerodynamic moment') % label left y-axis
% ylabel(ax(2),'Gyro moment') % label right y-axis
% xlabel(ax(2),'Time') % label x-axis
% set(p1, 'LineWidth', 2)
% set(p2, 'LineWidth', 2)
% xlim(ax(1),[9,Tplot]);
% xlim(ax(2),[9,Tplot]);
% legend('Aerod moment', 'Gyro moment')
% title('Moment comparison [Nm]')

