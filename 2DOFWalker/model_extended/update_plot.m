set(Link1,'xdata',Links(1,1,:),'ydata',Links(1,2,:));
set(Link2,'xdata',Links(2,1,:),'ydata',Links(2,2,:));


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