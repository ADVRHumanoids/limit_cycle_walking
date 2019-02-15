%figure(1);
%plot(foot_trajectory');

%% zmp
figure(2);
plot(zmp');
hold on; plot(zmp_ref', 'r--');
plot(com_trajectory(2,:)')


% plot(zmp_y(20:end)');

% plot(shifted');
plot(delayed');

hold on; plot(zmp_y_fake_right');
plot(zmp_y_fake_left');


plot(zmp_sloped(:,1)');
plot(1 + foot_trajectory(3,:)');

%% window moving 
for i = 1:size(window_tot,2)
    plot(window_tot(:,i)')
    ylim([-.1,.1])
    drawnow;
    pause(0.01)
end
%% phase LAND 1/FLIGHT 0
figure(3);
plot(landed_left);
hold on; plot(landed_right, 'r--');

%% force sensors
figure(4);
hold on; plot(abs(ft_right(3,:))', 'r--')
plot(abs(ft_left(3,:))')

%% impact detector
figure(5);
plot(impact_detector');
hold on;plot(1 +foot_trajectory(3,:)','r');

%% zmp and foot trajectory
hold on; plot(zmp_ref')
plot(zmp')
hold on;plot(1 +foot_trajectory(3,:)','r');

%% zmp and phase LAND and FLIGHT
hold on; plot(zmp_ref')
plot(0.1*landed_left,'k--');
plot(0.1*landed_right, 'r--');

%% com trajectory
plot(q1)
plot(com_trajectory')
hold on;
plot(foot_trajectory');hold on;
% plot(delayed');hold on;
plot(current_side);hold on;
plot(entered_period_delay');hold on;
plot(lateral_step); hold on;
plot(reset_lateral); hold on;
plot(event);hold on;
% plot(entered_left);
% plot(entered_right);