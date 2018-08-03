fig1 = figure(1);
set(fig1,'Position',[-1854          20        1211         954]);  %[2400, 400,560,420]
% set(fig1,'Position',[ 6   552   560   420]);  %[2400, 400,560,420]

for i = 1:n_link
Link(i) = plot(0,0,'LineWidth',2); grid on; hold on;
end

xlim([-1 5]);
ylim([-1 4]);

xLineTerrainLim = xlim;
yLineTerrainLim = tan(slope) * (xLineTerrainLim);
LineTerrain = plot(xLineTerrainLim,yLineTerrainLim);
handleQuiver = quiver(0,0);


p1plot = plot(0,0,'o');
p2plot = plot(0,0,'o');

%===========================================
for i = 1:n_link
set(Link(i),'xdata',Links(i,1,:),'ydata',Links(i,2,:));
end
%===========================================

%==========CoM position=============================
for i = 1:n_link
    plot_CoM(i) = plot(0,0,'x'); hold on;
end
plot_total_CoM1 = plot(0,0,'ko','MarkerSize',7,'Linewidth',1); hold on; %2 plot to make cross inside circle
plot_total_CoM2 = plot(0,0,'kx','MarkerSize',7,'Linewidth',1); hold on;

for i = 1:n_link
set(plot_CoM(i),'xdata',kinematics.linksCoMPosition(1,:,i),'ydata', kinematics.linksCoMPosition(2,:,i));
end
set(plot_total_CoM1,'xdata',kinematics.CoM_position(1),'ydata', kinematics.CoM_position(2)); %2 plot to make cross inside circle
set(plot_total_CoM2,'xdata',kinematics.CoM_position(1),'ydata', kinematics.CoM_position(2));
%===================================================

fig2 = figure(2);
set(fig2,'Position', [  -609   503   560   420])

subplot(2,1,1)    
plot_q1 = plot(0,0); hold on;
plot_q1_dot = plot(0,0); hold on;
plot_q1_Ddot = plot(0,0); hold on;

plotq1_legend = legend('$q_1$','$\dot{q_1}$','$\ddot{q_1}$');
set(plotq1_legend, 'Interpreter', 'latex');

% xlim([0 10]);
ylim([-20 20]);

subplot(2,1,2)
plot_q2 = plot(0,0); hold on;
plot_q2_dot = plot(0,0); hold on;
plot_q2_Ddot = plot(0,0); hold on;

plotq2_legend = legend('$q_2$','$\dot{q_2}$','$\ddot{q_2}$');
set(plotq2_legend, 'Interpreter', 'latex');

% xlim([0 10]);
ylim([-20 20]);



q_record = zeros(length(q),1);
q_dot_record = zeros(length(q_dot),1);
q_Ddot_record = zeros(length(q_dot),1);
%==========mechanical energy========================
% fig3 = figure(3);
% set(fig3,'Position',[-601   525   560   420])
% energyPlot = plot(0,0);
% xlim([0 50]);
% % ylim([-5 100]);
% T_record = 0;
%===================================================

%==========check2 (K_dot = -q_dot' * G)=============
% fig4 = figure(4);
% set(fig4,'Position',[  -601     7   560   420])
% plotCheck2(1) = plot(0,0); hold on;
% plotCheck2(2) = plot(0,0);
% K_dot_record = 0;
% check2_record = 0;
%===================================================

%============p2position - impact line===============
% fig5 = figure(5);
% set(fig5,'Position',[  -601     7   560   420])
% plot_p2 = plot(0,0); hold on;
% xlim([0 10]);
% ylim([-5 5]);
% p2_record = 0;
%===================================================

%===========controller==============================
% fig6 = figure(6);
% set(fig6,'Position',[  -601     7   560   420])
% for i = 1:length(h)
% plot_controller(i) = plot(0,0); hold on;
% end
% xlim([0 10]);
% ylim([-1 1]);
% controller_record = zeros(n_link-1,1);
%===================================================

%===============xi==================================
% fig7 = figure(7);
% set(fig7,'Position',[  -601     7   560   420])
% for i = 1:5
% plot_xi(i) = plot(0,0); hold on;
% end
% xi_record = zeros(5,1);
% legend('xi(1)','xi(2)','xi(3)','xi(4)','xi(5)')
%===================================================
time_record = 0;