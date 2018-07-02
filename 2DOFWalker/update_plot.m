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
    E_record = [E_record(:,2:end) E];
end

% for loop = 1:size(q,1)
set(plot_q,'xdata',time_record,'ydata',q_record(1,:));
set(plot_q_dot,'xdata',time_record,'ydata',q_dot_record(1,:));
% end


E_record = [E_record E];
set(energyPlot, 'xdata', time_record,'ydata', E_record);

drawnow;