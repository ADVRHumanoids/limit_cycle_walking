for i = 1:n_link
set(Link(i),'xdata',Links(i,1,:),'ydata',Links(i,2,:));
end   

% set(p2plot,'xdata',phi(1),'ydata',phi(2));

buffer = 100000;
time_record = [time_record time];
q_record = [q_record q];
q_dot_record = [q_dot_record q_dot];

if size(time_record,2) >= buffer
    time_record = [time_record(:,2:end) time];
    q_record = [q_record(:,2:end) q];
    q_dot_record = [q_dot_record(:,2:end) q_dot];
%     T_record = [T_record(:,2:end) T];
end

% for loop = 1:size(q,1)
set(plot_q,'xdata',time_record,'ydata',q_record(1,:));
set(plot_q_dot,'xdata',time_record,'ydata',q_dot_record(1,:));
% end

%==========CoM position=============================
for i = 1:n_link
set(plot_CoM(i),'xdata',kinematics.linksCoMPosition(1,:,i),'ydata', kinematics.linksCoMPosition(2,:,i));
end
set(plot_total_CoM1,'xdata',kinematics.CoM_position(1),'ydata', kinematics.CoM_position(2)); %2 plot to make cross inside circle
set(plot_total_CoM2,'xdata',kinematics.CoM_position(1),'ydata', kinematics.CoM_position(2));
%===================================================

%=========mechanical energy=========================
% T_record = [T_record T];
% set(energyPlot, 'xdata', time_record,'ydata', T_record);

%======check2=====(K_dot = -q_dot' * G)=============
% K_dot_record = [K_dot_record K_dot];
% check2_record = [check2_record check2];
% set(plotCheck2(1), 'xdata', time_record, 'ydata', K_dot_record);
% set(plotCheck2(2), 'xdata', time_record, 'ydata', check2_record);
%===================================================

%============p2position - impact line===============
% p2_record = [p2_record, Links(n_link,2,2) - yLineTerrain];
% set(plot_p2, 'xdata', time_record, 'ydata', p2_record)
%===================================================

%===========controller==============================
% controller_record = [controller_record, h];
% for i = 1:length(h)
% set(plot_controller(i), 'xdata', time_record, 'ydata', controller_record(i,:));
% end
%===================================================

%===================================================

drawnow;