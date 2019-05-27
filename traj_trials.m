clear all;
clc; close all;

alpha = 0.1;
q = 0;
dt = 0.01;
h = 1;
t = 0;
traj = [0, 0];

theta = 0;
R = [cos(theta), sin(theta); -sin(theta), cos(theta)];
q_record = 0;
t_record = 0;
traj_record = [0; 0];
curved_traj_record = [0; 0];
real_traj_record = [0; 0];

real_traj = [0; 0];
% R = [cos(theta), sin(theta)];
% curved_traj = R *traj';
d_traj_rot  = [0;0];
d_traj = [0;0];
old_direction = 0;
direction = 0;
while (q<2)
   
traj_old = h* tan(q);

t = t +dt;
q = q + (alpha * dt);

traj = h* tan(q);

d_traj = [traj - traj_old; traj - traj_old];

% rotate traj

if (q>0.2 && theta <= pi/2)
    theta = theta +pi/100;
end

R = [cos(theta), sin(theta); -sin(theta), cos(theta)];
d_traj_rot = R * d_traj;
% d_traj_rot = [d_traj(1) * cos(theta); d_traj(2) * sin(theta)];
direction = atan2(d_traj(2), d_traj(1));

real_traj = real_traj + d_traj_rot;

% direction = atan2(d_traj(2), d_traj(1));

% if (q>0.2 && theta <= pi/4)
%     theta = + pi/4;
% end
% R = [cos(theta), sin(theta)];

% curved_traj = R *traj;
% curved_traj = [traj(1) * cos(theta); traj(2) * sin(theta)];
%  curved_traj = R *traj;

q_record = [q_record, q];
t_record = [t_record, t];
% traj_record = [traj_record, traj];
% curved_traj_record = [curved_traj_record, curved_traj];
real_traj_record = [real_traj_record, real_traj];




% plot(traj_record(1,:), traj_record(2,:)); hold on;
% plot(curved_traj_record(1,:), curved_traj_record(2,:));
plot(real_traj_record(1,:), real_traj_record(2,:));
% plot(curved_traj_record(:,2), curved_traj_record(:,2));
xlim([0,1]);
ylim([0,1]);
drawnow;
% plot()


end



%% second version

% clear all;
% clc; close all;
% 
% alpha = 0.1;
% q = 0;
% q1 = 0;
% dt = 0.05;
% h = 1;
% t = 0;
% traj = [0, 0];
% 
% theta = 0;
% R = [cos(theta), sin(theta); -sin(theta), cos(theta)];
% q_record = 0;
% q1_record = 0;
% t_record = 0;
% traj_record = [0; 0];
% curved_traj_record = [0; 0];
% real_traj_record = [0; 0];
% traj_rerot_record = [0; 0];
% traj_rerot = [0; 0];
% 
% alpha_record = 0;
% 
% real_traj = [0; 0];
% % R = [cos(theta), sin(theta)];
% % curved_traj = R *traj';
% d_traj_rot  = [0;0];
% d_traj = [0;0];
% old_direction = 0;
% direction = 0;
% signy = 1;
% theta = 0;
% n_step = 0;
% traj_initial = [0; 0];
% offset = 2;
% 
% figure(1);
% plot_q = plot(0,0); hold on;
% plot_q1 = plot(0,0); hold on;
% figure(2);
% plot_traj = plot(0,0); hold on;
% xlim([-5,5]);
% ylim([-5,5]);
% figure(3);
% plot_traj1 = plot(0,0); hold on;
% plot_traj2 = plot(0,0); hold on;
% figure(4);
% plot_traj_rerot = plot(0,0); hold on;
% 
% 
% % figure(3)
% % plot_alpha = plot(0,0);
% 
% while (q<2)
%    
% traj_old = h* tan(q);
% 
% t = t +dt;
% q = q + (alpha * dt);
% 
% if (q > pi/10)
%     q = 0;
%     traj_initial = real_traj;
%     signy = signy * -1;
%     offset = offset * -1;
%     n_step = n_step+1;
% end
% 
% if (n_step > 3)
%     theta = pi/8;
% end
% if (n_step > 8)
%     theta = 0;
% end
% 
% if (n_step > 3 && n_step <= 8)
%     theta_q1 = - pi/8;
%     R_q1 = [cos(theta_q1), sin(theta_q1); -sin(theta_q1), cos(theta_q1)];
%     traj_rerot = R_q1 * real_traj;
%     q1 = atan(traj_rerot(1));
% end
% 
% traj1 = h* tan(q);
% % traj2 = offset + signy * sin(q*10);
% traj2 = offset + signy;
% delta_traj = [traj1; traj2];
% 
% % rotate traj
% R = [cos(theta), sin(theta); -sin(theta), cos(theta)];
% direction = atan2(d_traj(2), d_traj(1));
% delta_traj = R * delta_traj;
% 
% 
% q1_record = [q1_record, q1];
% q_record = [q_record, q];
% t_record = [t_record, t];
% 
% % % sensed angle
% q1 = atan(real_traj(1));
% 
% 
% 
% 
% % %
% % traj_record = [traj_record, traj];
% % curved_traj_record = [curved_traj_record, curved_traj];
% real_traj_record = [real_traj_record, real_traj];
% alpha_record = [alpha_record, alpha];
% % traj_rerot_record = [traj_rerot_record, traj_rerot];
% 
% % set(plot_traj_rerot, 'xdata', traj_rerot_record(1,:), 'ydata', traj_rerot_record(1,:));
% set(plot_traj1, 'xdata', t_record, 'ydata', real_traj_record(1,:));
% set(plot_traj2, 'xdata', t_record, 'ydata', real_traj_record(2,:));
% set(plot_traj, 'xdata', real_traj_record(1,:), 'ydata', real_traj_record(2,:));
% set(plot_q, 'xdata', t_record, 'ydata', q_record);
% set(plot_q1, 'xdata', t_record, 'ydata', q1_record);
% % set(plot_alpha, 'xdata', t_record, 'ydata', alpha_record);
% 
% 
% drawnow;
% % plot()
% 
% 
% end
% 
