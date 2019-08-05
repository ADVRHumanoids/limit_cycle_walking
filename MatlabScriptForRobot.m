figure();
plot(time, zmp');
hold on; plot(time, zmp_ref', 'r--');
plot(time, com_trajectory(2,:)'); hold on; %% commanded trajectory
plot(time, com_pos(2,:)'); hold on; %% real trajectory

%% COM and FOOT trajectories in space (x-y plane)

% planned
figure()
plot(final_com_position(1,:), final_com_position(2,:), 'o'); hold on;
plot(initial_com_position(1,:), initial_com_position(2,:), 'o'); hold on;
plot(com_trajectory(1,:), com_trajectory(2,:)); hold on;
plot(foot_trajectory(1,:), foot_trajectory(2,:)); hold on;
% real
figure()
plot(com_pos(1,:),com_pos(2,:));hold on; % actual 
plot(foot_pos_left(1,:), foot_pos_left(2,:));hold on; % actual 
plot(foot_pos_right(1,:), foot_pos_right(2,:));hold on; % actual


plot(time, cp(2,:)); hold on;
plot(time, com_trajectory(2,:)); hold on;

plot(time, cp(1,:)); hold on;
plot(time, com_trajectory(1,:)); hold on;

%% 
plot(time, com_trajectory(1,:)); hold on;
plot(time, final_com_position(1,:)'); hold on;
plot(time, initial_com_position(1,:)'); hold on;

plot(time, com_trajectory_fake(1,:));
%% alpha and tilt   
% q (tilt)
figure()
plot(time, q1_sensed); hold on;
plot(time, q1_max); hold on;
plot(time, q1_min); hold on;

plot(time, q1_cmd); hold on;
%% COM and FOOT trajectories in time
figure();

plot(time, com_trajectory(1,:)); hold on; % reference
plot(time, foot_trajectory(1,:)); hold on; % reference

plot(time, com_pos(1,:)');hold on; % actual 
plot(time, foot_pos_left);hold on; % actual 
plot(time, foot_pos_right);hold on; % actual 

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
%% com real and fake
plot(com_trajectory_fake'); hold on;
plot(com_trajectory'); hold on;
%% stab
plot(time, zmp_ref); hold on;
plot(time, zmp_stab); hold on;

% ---------------------- %
figure()
plot(zmp_measured'); hold on;
plot(zmp_ref'); hold on;

plot(zmp_measured(1,:)'); hold on;
plot(zmp_ref(1,:)'); hold on;

plot(zmp_measured(2,:)'); hold on;
plot(zmp_ref(2,:)'); hold on;