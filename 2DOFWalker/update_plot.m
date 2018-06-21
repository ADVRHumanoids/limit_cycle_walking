for i = 1:n_link
set(Link(i),'xdata',Links(i,1,:),'ydata',Links(i,2,:));
end   

set(p2plot,'xdata',phi(1),'ydata',phi(2));

buffer = 1000;
time_record = sym([time_record time]);
q_record = [q_record q];

if size(time_record,2) >= buffer
    time_record = [time_record(:,2:end) time];
    q_record = [q_record(:,2:end) q];
end

% for loop = 1:size(q,1)
% set(plot_q(loop),'xdata',time_record,'ydata',q_record(loop,:));
% end

drawnow;