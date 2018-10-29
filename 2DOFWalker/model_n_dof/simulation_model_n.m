%%  simulation of a n-link robot walker

    % author: Francesco Ruscelli
    % e-mail: francesco.ruscelli@iit.it
    
%==========================================================================
close all; clear all; clc;
                                                                                                                                                                                                                               
Folder = cd;
addpath(genpath(fullfile(Folder, '..')));
%==================simulation model of a n-link walker=====================
robotTree; %change here parameters
%=========================step_lenght======================================
initial_step = 0.3;
step_lenght = 0.3;
angle_impact = pi/17;
%========================plot settings=====================================
plot_phasePort = 1; 
plot_q = 0; %q, q_dot, q_ddot
plot_check_model = 0; %mechanical energy and kineticEnergy_dot = -q_dot * G
plot_CoM = 1;
plot_CoM_pos = 0;
plot_tau = 0;
%==========================================================================
%==========================================================================
robotData = getRobotData;
slope = 0;
n_link = length(parent_tree);
relabelingMatrices = getRelabelingMatrices(parent_tree, waist);
%==========================================================================
q1_des = -asin(initial_step^2/(2*robotData.link_length(1)*initial_step));
q2_des = (pi - 2 *q1_des);
%========================================================================== 
    startingParameters = [ q1_des,  0,    0;...  %q1     %pi/18
                           q2_des,  0,    0;...  %q2    pi-(pi-2*(pi/2-(1/18*pi)))
                           -q1_des + pi/2,  0,    0;...
                                0,  0,    0;...  %z1
                                0,  0,    0];    %z2
                            
    startingParameters = [ 0,  0,    0;...  %q1     %pi/18
                           pi,  0,    0;...  %q2    pi-(pi-2*(pi/2-(1/18*pi)))
                           0,  0,    0;...
                                0,  0,    0;...  %z1
                                0,  0,    0];    %z2
%=======================starting kinematics================================
q = startingParameters(1:n_link+2,1);
q_dot = startingParameters(1:n_link+2,2);
q_Ddot = startingParameters(1:n_link+2,3);

% ~~~~~~~~~~~~~
q_d = q(1:2);
q_dot_d = q_dot(1:2);
% ~~~~~~~~~~~~~
[Links,kinematics] = KinematicsLinks(q);  %update kinematics %link,axis,begin/end  

yLineTerrain = double(tan(slope) * Links(n_link,1,2));
yLineTerrain_old = yLineTerrain;

% distance = Links(2,1,2) - Links(1,1,1);
% distance = robotData.link_length(1) * cos(pi/2 - q(1)) + robotData.link_length(2) * sin(pi - q(1) - q(2));

%just needed for initialization of the output for the controller y = h(q)
h = zeros(n_link-1,1);
%=======
%=======
% v_record = 0;
% w_d_record = 0;
% fig10 = figure(10);
% set(fig10,'Position',[1341         446         560         420]);
% plot_error = plot(0,0); hold on;
% plot_error1 = plot(0,0); hold on;
% writerObj = VideoWriter('walker1.avi');
% writerObj.FrameRate = 60;
% open(writerObj);



j = 0;
jj = 0;
global time;
time = 0;
time_reset = 0;
dt = 0.005; %0.001

tau = zeros(length(q),1);
% k_p = 1;
% k_d = 0.1;
Base = [0,0];

%=======
%=======
set_plot;
%=======
%=======


disp('push a button to continue'); pause;

impact_detected = 0;
impact_counter = 0;

control_flag = 0;
first_impact = 0;
distance_legs = 0;

offset_leg = pi;
offset_waist = 0;
%===============
%===============
 while 1
%  for j = 1:250

j = j + 1;
time = (j-1)*dt;

jj = jj + 1;
time_reset = (jj-1)*dt;

[D_ext,C_ext,G_ext] = calcDynMatrices(q,q_dot,fileName);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~
D = D_ext(1:n_link,1:n_link);
C = C_ext(1:n_link,1:n_link);
G = G_ext(1:n_link);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Links_old = Links;


% [q_Ddot, q_dot, q] = integrator(dt,D,C,G, tau, q_dot, q);
% q_Ddot(n_link+1:end) = 0;
% q_dot(n_link+1:end) = 0;
% q(n_link+1:end) = Base;

[q_Ddot(1:n_link), q_dot(1:n_link), q(1:n_link)] = integrator(dt,D,C,G, tau(1:n_link), q_dot(1:n_link), q(1:n_link));

% for i  = 1:length(q)
%     if q(i) >= 2*pi
%         q(i) = q(i) - 2*pi;
%     end
% end


yLineTerrain_old = yLineTerrain;
yLineTerrain = double(tan(slope) * Links(swing_leg,1,2));
[Links,kinematics] = KinematicsLinks(q);  %update kinematics
%============impact========================================================
% control : F2_T <= mu*F2_N and F2_N > 0
%           stance leg p1_dot_vertical >= 0
step_condition = Links(swing_leg,1,2) > Base(1) + step_lenght; %link,axis,begin/end  
% step_condition = Links(swing_leg,1,2) > Base(1) + abs(distance);
if Links(swing_leg,2,2) <= yLineTerrain  && step_condition %&& Links_old(swing_leg,2,2) > yLineTerrain_old  %link,axis,begin/end  

                     
    [q,q_dot, deltaqDotBar] = impact_handler(q,q_dot,relabelingMatrices,fileName);
%     deltaqDotBar_previous = deltaqDotBar;
    %------stay inside 2*pi-----
    for i  = 1:length(q)
        if abs(q(i)) >= 2*pi
            q(i) = sign(q(i))*((q(i)*sign(q(i))) - 2*pi);
        end
    end
    %===========change base=====================================
    Base = [Links(swing_leg,1,2); yLineTerrain]; %TODOstep_lenght
    q(end-1:end) = Base;
    [Links,kinematics] = KinematicsLinks(q); %update kinematics
    %===========================================================

    
    jj = 0;
    control_flag = 1;
    impact_detected = 1;
    impact_counter = impact_counter + 1;


end

if impact_detected
    impact_detected = 0;
    offset_leg = -offset_leg;
    offset_waist =  2*pi;
    
end
%============controller===============
    [tau,h] = controllerWalker(q,q_dot, D,C,G, offset_leg, offset_waist, q1_des); %linearizing normal model robot system
%     tau = [0;0;0];
%=====================================

%===========disturbance===============
% if time >= 4 && time <=6
%     tau = tau + 50;
% end
%=====================================

%===========start_movement===============
if time >= .3
    q(3) = q(3) + 0.01;
end
%=====================================
if plot_check_model
    %=========mechanicalenergy============
    [mechanicalEnergy,kineticEnergy,potentialEnergy, kineticEnergy_dot] = calcEnergy(q,q_dot,q_Ddot,fileName);
    %=====================================

    %===============check2================
    check2 = -q_dot' * G_ext; %must be equal to kineticEnergy_dot
    %=====================================
end
%==========
%==========

update_plot;

% frame = getframe;
% writeVideo(writerObj,frame);
% v_record = [v_record, v];
% w_d_record = [w_d_record, w_d];
% set(plot_error,'xdata',time_record,'ydata', v_record);%- w_d_record
% set(plot_error1,'xdata',time_record,'ydata', w_d_record);%- w_d_record
%==========
%==========
 end
 
 
 
 
%  close(writerObj);