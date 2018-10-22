for i = 1:n_link
set(Link(i),'xdata',Links(i,1,:),'ydata',Links(i,2,:));
end   

xlabel(['t = ', num2str(round(time,1))]);
% set(p2plot,'xdata',phi(1),'ydata',phi(2));

buffer = 100000;
time_record = [time_record time];

if size(time_record,2) >= buffer
    time_record = [time_record(:,2:end) time];
end
%===================q q_dot q_Ddot=====================
if plot_q || plot_phasePort
    q_record = [q_record q];
    q_dot_record = [q_dot_record q_dot];
    q_Ddot_record = [q_Ddot_record q_Ddot];

    if size(time_record,2) >= buffer

        q_record = [q_record(:,2:end) q];
        q_dot_record = [q_dot_record(:,2:end) q_dot];
        q_Ddot_record = [q_Ddot_record(:,2:end) q_Ddot];
    %     T_record = [T_record(:,2:end) T];
    end
end
if plot_q
    % for loop = 1:size(q,1)
    set(plot_q1,'xdata',time_record,'ydata',q_record(1,:));
    set(plot_q1_dot,'xdata',time_record,'ydata',q_dot_record(1,:));
    set(plot_q1_Ddot,'xdata',time_record,'ydata',q_Ddot_record(1,:));
    % end
    set(plot_q2,'xdata',time_record,'ydata',q_record(2,:));
    set(plot_q2_dot,'xdata',time_record,'ydata',q_dot_record(2,:));
    set(plot_q2_Ddot,'xdata',time_record,'ydata',q_Ddot_record(2,:));
    
    set(plot_q3,'xdata',time_record,'ydata',q_record(3,:));
    set(plot_q3_dot,'xdata',time_record,'ydata',q_dot_record(3,:));
    set(plot_q3_Ddot,'xdata',time_record,'ydata',q_Ddot_record(3,:));
end
%==========CoM position=============================
if plot_CoM
    for i = 1:n_link
    set(plot_CoM(i),'xdata',kinematics.linksCoMPosition(1,:,i),'ydata', kinematics.linksCoMPosition(2,:,i));
    end
    set(plot_total_CoM1,'xdata',kinematics.CoM_position(1),'ydata', kinematics.CoM_position(2)); %2 plot to make cross inside circle
    set(plot_total_CoM2,'xdata',kinematics.CoM_position(1),'ydata', kinematics.CoM_position(2));
end
%===================================================

%=========mechanical energy=========================
if plot_check_model
    mechanicalEnergy_record = [mechanicalEnergy_record mechanicalEnergy];
    set(energyPlot, 'xdata', time_record,'ydata', mechanicalEnergy_record);
%======check2=====(K_dot = -q_dot' * G)=============
    kineticEnergy_dot_record = [kineticEnergy_dot_record kineticEnergy_dot];
    check2_record = [check2_record check2];
    set(plotCheck2(1), 'xdata', time_record, 'ydata', kineticEnergy_dot_record);
    set(plotCheck2(2), 'xdata', time_record, 'ydata', check2_record); %-q_dot' * G
end
%===================================================

%============p2position - impact line===============
% p2_record = [p2_record, Links(n_link,2,2) - yLineTerrain];
% set(plot_p2, 'xdata', time_record, 'ydata', p2_record)
%===================================================
% plotq2_legend = legend('$q_2$','$\dot{q_2}$','$\ddot{q_2}$');
%===========controller==============================
% controller_record = [controller_record, h];
% for i = 1:length(h)
% set(plot_controller(i), 'xdata', time_record, 'ydata', controller_record(i,:));
% end
%===================================================

%===============xi==================================
if plot_xi
    xi_record = [xi_record, xi];
    for i = 1:5
    set(plot_xi(i), 'xdata', time_record,'ydata', xi_record(i,:));
    end
end
%===================================================

%============phase portrait=========================
if plot_phasePort
    for i = 1:n_link
    set(plot_phasePortrait(i), 'xdata', abs(q_record(i,:)),'ydata', abs(q_dot_record(i,:)));
    end
end
%===================================================

%================q_d & q_dot_d======================
if plot_q
% q_d_record = [q_d_record, q_d];
% q_d1_record = [q_d1_record, q_d1];
q_dot_d_record = [q_dot_d_record, q_dot_d];
set(plot_q_d(1),'xdata',time_record,'ydata',q_dot_d_record(1,:));
set(plot_q_d(2),'xdata',time_record,'ydata',q_dot_record(1,:));
% set(plot_q_dot_d,'xdata',time_record,'ydata',q_dot_d_record(1,:));
end
% set(plot_q_d(1),'xdata',time_record,'ydata',q_d_record(2,:));
% set(plot_q_d(2),'xdata',time_record,'ydata',q_d1_record(2,:));
% set(plot_q_dot_d,'xdata',time_record,'ydata',q_dot_d_record(2,:));
%===================================================

%=====================================
if plot_CoM_pos
    CoM_record = [CoM_record; kinematics.CoM_position'];
    set(plot_CoM_position_x, 'xdata', time_record, 'ydata', CoM_record(:,1));
    set(plot_CoM_position_y, 'xdata', time_record, 'ydata', CoM_record(:,2));
end
%===================================================

%================tau=======================
if plot_tau
tau_record = [tau_record; tau'];
for i = 1:n_link
    set(plot_tau(i), 'xdata', time_record,'ydata', tau_record(:,i));
end
end
%===================================================
drawnow;