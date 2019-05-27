figure(1);
plot(time, zmp');
hold on; plot(time, zmp_ref', 'r--');
plot(time, com_trajectory(2,:)'); hold on; %% commanded trajectory
plot(time, com_pos(2,:)'); hold on; %% real trajectory


plot(time, initial_com_position);
plot(time, final_com_position(2,:));

plot(time, initial_sole_position);
plot(time, final_sole_position);

plot(time, final_com_position(1,:)); hold on;
plot(time, com_trajectory(1,:), 'r--'); hold on;

plot(time, steep_coeff);
plot(time, alpha);

plot(time, com_trajectory'); hold on;

plot(time, foot_trajectory'); hold on;


plot(time, delta_com'); hold on;
plot(time, delta_com_rot'); hold on;

plot(time, foot_pos_left(1,:));hold on; % actual
plot(time, foot_pos_right(1,:));hold on; % actual
figure;

%% steer
plot(time, com_trajectory'); hold on;
plot(time, foot_trajectory'); hold on;

plot(com_trajectory(1,:), com_trajectory(2,:)); hold on;
plot(foot_trajectory(1,:), foot_trajectory(2,:)); hold on;

plot(com_pos(1,:),com_pos(2,:));hold on; % actual 
plot(foot_pos_left(1,:), foot_pos_left(2,:));hold on; % actual 
plot(foot_pos_right(1,:), foot_pos_right(2,:));hold on; % actual

plot(time, q1_sensed); hold on;

plot(dist_com'); hold on;
plot(left_ankle_to_com_z); hold on;
plot(world_to_com'); hold on;
plot(q1_sns); hold on;
plot(current_world_to_com');

%% stab

plot(time, zmp_stab);
%% something
plot(time, get_com_initial_position); hold on;
plot(time, stopped_received); hold on;
%% q1 and step
figure(2);


plot(time, com_trajectory(1,:)', 'r--'); hold on; % reference
plot(time, foot_trajectory(1,:)','b--');hold on; % reference

plot(time, com_pos(1,:)');hold on; % actual 
plot(time, foot_pos_left);hold on; % actual 
plot(time, foot_pos_right);hold on; % actual 
%% alpha and tilt   
plot(time, q1_sensed);hold on;
plot(time, q1_cmd);hold on;

plot(time, q1_start);
plot(time, q1_temp);
plot(time, q1_max);hold on;
plot(time, q1_min);hold on;

plot(zmp_y_rec)
%% impact conditions
plot(time, impact_detected, 'r--'); hold on;
plot(time, cond_step); hold on; %% step
plot(time, cond_q); hold on;
%% window zmp y
figure(2);
for i = 1:size(window_tot,2)
    plot(window_tot(:,i));
    ylim([-.2,.2])
    drawnow
end

for i = 1:size(window_preview,2)
    plot(window_preview(:,i));
    ylim([-.2,.2])
    drawnow;
end
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

plot(alpha_sensed);
plot(alpha_cmd);
% plot(switched);
plot(current_spatial_zmp_y); hold on;
plot(current_spatial_zmp_y_cmd); hold on;

%% zmp previewed
hold on; plot(zmp_ref', 'r--');
plot(current_spatial_zmp_y); hold on;

plot(alpha_sensed);hold on;
plot(side_value); hold on; %% if alpha is >0 and <1
plot(space);hold on; %% which is dependent on alpha

plot(switched_prev); hold on; %% when the zmp_y preview switch. it is dependent on space (space <= max_space)

plot(max_space);hold on;
plot(length_step);hold on;
plot(spatial_window_preview(1,:)')
plot(foot_position);

plot(q1_min);hold on;
plot(q1_max);

% lateral spatial zmp window preview based on current position
for i = 1:size(spatial_window_preview,2)
plot(spatial_window_preview(:,i)')
ylim([-.1,.1])
txt1 = ['alpha = ' num2str(alpha_sensed(i))];
txt2 = ['side = ' num2str(side_value(i))];
text(10,0.09,txt1);
% text(200,0.09,txt2);
drawnow;
    pause(0.01)
% pause;
end

%% zmp x OFFLINE

plot(q1);hold on;
plot(zmp_x_offline);
plot(com_trajectory(1,:)); hold on;
com_x_offline = [-0.0876 * ones(98,1); com_x_offline];
plot(com_x_offline);
plot(com_pos(1,:));

for i = 1:20
    plot([1:1200],com_max(i,:) * ones(1,1200),'--'); hold on;
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

%% com velocity
plot(time, com_pos(1,:)'); hold on;
plot(time, com_vel(1,:)'); hold on;

plot(time, com_trajectory(2,:)); hold on;
plot(time, com_vel_cmd(2,:)'); hold on;

plot(steepness);

plot(foot_vel_current(1,:)'); hold on;
plot(foot_vel_cmd(1,:)'); hold on;

