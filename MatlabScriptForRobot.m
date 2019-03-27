figure(1);
plot(time, zmp');
hold on; plot(time, zmp_ref', 'r--');
plot(time, com_trajectory(2,:)'); hold on;
plot(time, foot_trajectory');

plot(time, entered_forward)
plot(time, entered_delay)
plot(time, flag_impact);


%% zmp x
plot(time, com_pos(1,:)'); hold on;
plot(time, com_trajectory(1,:)); hold on;
% plot(time, q1_cmd); hold on;
plot(time, delta_com'); hold on;
plot(time, initial_com);
plot(time, zmp_x);


plot(vel_q1); hold on;
plot(steepness); hold on;
plot(time, q1_cmd); hold on;
plot(time, q1_sensed); hold on;

plot(alpha);
plot(switched);
plot(time, current_spatial_zmp_y); hold on;
plot(time, current_spatial_zmp_y_cmd); hold on;

%% zmp x OFFLINE

plot(q1);hold on;
plot(zmp_x_offline);
plot(com_trajectory(1,:)); hold on;
com_x_offline = [-0.0876 * ones(98,1); com_x_offline];
plot(com_x_offline);
plot(com_pos(1,:));

for i = 1:20
    plot([1:1200],com_max(i,:) * ones(1,1200),'--');
end


%% zmp
figure(2);
plot(zmp');
hold on; plot(zmp_ref', 'r--');
plot(com_trajectory(2,:)')
plot(foot_trajectory');



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

%% com trajectory (realized by opensot) and steps

plot(time, q1_cmd);hold on;
plot(time, foot_trajectory'); hold on;
plot(time, com_trajectory');hold on;
plot(time, q1_sensed);hold on;

plot(time, com_pos(1,:)');hold on;
plot(time, foot_pos_left);hold on;
plot(time, foot_pos_right);hold on;
plot(time, left_foot_to_com(1,:)); hold on;
plot(time, right_foot_to_com(1,:)); hold on;

plot(time, com_trajectory(1,:)); hold on;
plot(time, foot_trajectory(1,:)); hold on;

plot(time, initial_pose_foot(1,:)); hold on;
plot(time, final_pose_foot(1,:)); hold on;

plot(time, foot_pos_left(1,:)); hold on;
plot(time, foot_pos_right(1,:)); hold on;

