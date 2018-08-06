%%  simulation of a n-link robot walker

    % author: Francesco Ruscelli
    % e-mail: francesco.ruscelli@iit.it
    
%==========================================================================
close all; clear all; clc;

Folder = cd;
addpath(genpath(fullfile(Folder, '..')));
%==================simulation model of a n-link walker=====================
robotTree; %change here parameters

%==========================================================================
slope = 0;
n_link = length(parent_tree);
relabelingMatrices = getRelabelingMatrices(parent_tree, waist);
%==========================================================================
                      %pos / vel / acc
% startingParameters = [ pi/18,   0,    0;...  %q1    pi/18
%                       -pi/18,   0,    0;...  %q2     -pi/18
%                        3*pi/4,  0,    0;...  %q3     3*pi/4,
%                        pi/4,    0,    0;...  %q4      pi/4,
%                        0,       0,    0;...  %q5
%                        0,       0,    0;...  %z1
%                        0,       0,    0];    %z2
if n_link == 3
    
    startingParameters = [                      pi/18,  0,    0;...  %q1     %pi/18
                           pi-(pi-2*(pi/2-(1/18*pi))),  0,    0;...  %q2      3*pi/4,    50,    0,...  %pi-(pi-2*(pi/2-(1/18*pi)))
                                               -pi/18,  0,    0;...
                                                    0,  0,    0;...  %z1
                                                    0,  0,    0];    %z2
elseif n_link == 2
    
    cyclic_var_value =  pi/16;

    startingParameters = [cyclic_var_value,  0,    0;...  %q1     %pi/18
                          pi - 2 *cyclic_var_value, 0,    0;...  %q2      3*pi/4,    50,    0,...  %pi-(pi-2*(pi/2-(1/18*pi)))
                          0,  0,    0;...  %z1
                          0,  0,    0];    %z2
end
%=======================starting kinematics================================
q = startingParameters(1:n_link+2,1);
q_dot = startingParameters(1:n_link+2,2);
q_Ddot = startingParameters(1:n_link+2,3);

[Links,kinematics] = KinematicsLinks(q);  %update kinematics %link,axis,begin/end  

yLineTerrain = double(tan(slope) * Links(n_link,1,2));
yLineTerrain_old = yLineTerrain;

% distance = Links(2,1,2) - Links(1,1,1);
distance = robotData.link_length(1) * cos(pi/2 - q(1)) + robotData.link_length(2) * sin(pi - q(1) - q(2));
% mechanical energy 1 link
% T = (981*cos(q(1)))/200 + (333*q_dot(1)^2)/2000;
% mechanical energy 2 link
% T = (981*cos(q(1) + q(2)))/200 + (2943*cos(q(1)))/200 + (q_dot(1)^2*cos(q(2)))/2 + (333*q_dot(1)*q_dot(2))/1000 + (833*q_dot(1)^2)/1000 + (333*q_dot(2)^2)/2000 + (q_dot(1)*q_dot(2)*cos(q(2)))/2;

%just needed for initialization of the output for the controller y = h(q)
h = zeros(n_link-1,1);
%=======
%=======
set_plot;
%=======
%=======
%-------------initial conditions for walking and controller----------------
[controller, q_dot_0] = calculateInitialConditions(startingParameters,fileName, relabelingMatrices, distance);
q_dot(1:n_link) = q_dot_0;
%--------------------------------------------------------------------------
j = 0;
time = 0;
dt = 0.005; %0.001

tau = zeros(length(q),1);
k_p = 1;
k_d = 0.1;
Base = [0,0];
disp('push a button to continue'); pause;

impact_detected = 0;
control_flag = 0;
first_impact = 0;
distance_legs = 0;
PI = pi;

 while 1
     
j = j + 1;
time = (j-1)*dt;

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
% step_condition = Links(swing_leg,1,2) > Base(1); %link,axis,begin/end  
step_condition = Links(swing_leg,1,2) > Base(1) + abs(distance);
if Links(swing_leg,2,2) <= yLineTerrain  && step_condition %&& Links_old(swing_leg,2,2) > yLineTerrain_old  %link,axis,begin/end  
% if q(1) >= pi/18
%     if ~first_impact 
%         distance_legs = Links(swing_leg,1,2);
%         first_impact = 1;
%     end
    
    [q,q_dot] = impact_handler(q,q_dot,relabelingMatrices,fileName);
    
    %===========change base=====================================
    Base = [Links(swing_leg,1,2); yLineTerrain]; %TODO
    q(end-1:end) = Base;
    [Links,kinematics] = KinematicsLinks(q); %update kinematics
    %===========================================================
    
    control_flag = 1;
%     impact_detected = 1;

end
%============controller===============
w = (controller(1) + controller(2)* time);

% v = finiteTime_stabilizer(w,controller(2));
% v = -PD_controller(w,controller(2));
if control_flag == 1
    
%     [tau,h] = controllerWalker(q,q_dot, D,C,G);
    tau = controllerWalker_ver1(v,q,q_dot, D,C,G);
end

xi = variableXi(q,q_dot,q_Ddot); %seems that it gives -w



%=====================================
% impact_detected = 0;

%=====================================
%=========mechanicalenergy============
% mechanical energy 1 link
% T = (981*cos(q(1)))/200 + (333*q_dot(1)^2)/2000;
% mechanical energy 2 link
% T = (981*cos(q(1) + q(2)))/200 + (2943*cos(q(1)))/200 + (q_dot(1)^2*cos(q(2)))/2 + (333*q_dot(1)*q_dot(2))/1000 + (833*q_dot(1)^2)/1000 + (333*q_dot(2)^2)/2000 + (q_dot(1)*q_dot(2)*cos(q(2)))/2;
%=====================================

%===============check2================
% K_dot = ((sin(q(1))*q_dot(1))/2 + (sin(q(1) + q(2))*(q_dot(1)/2 + q_dot(2)/2))/2)*(sin(q(1))*q_Ddot(1) + sin(q(1) + q(2))*(q_Ddot(1)/2 + q_Ddot(2)/2) + cos(q(1))*q_dot(1)^2 + cos(q(1) + q(2))*(q_dot(1) + q_dot(2))*(q_dot(1)/2 + q_dot(2)/2)) + (sin(q(1))*q_dot(1) + sin(q(1) + q(2))*(q_dot(1)/2 + q_dot(2)/2))*((sin(q(1))*q_Ddot(1))/2 + (sin(q(1) + q(2))*(q_Ddot(1)/2 + q_Ddot(2)/2))/2 + (cos(q(1))*q_dot(1)^2)/2 + (cos(q(1) + q(2))*(q_dot(1) + q_dot(2))*(q_dot(1)/2 + q_dot(2)/2))/2) + (83*q_dot(1)*q_Ddot(1))/1000 + (q_dot(1) + q_dot(2))*((83*q_Ddot(1))/2000 + (83*q_Ddot(2))/2000) + (q_Ddot(1) + q_Ddot(2))*((83*q_dot(1))/2000 + (83*q_dot(2))/2000) + ((cos(q(1))*q_dot(1))/2 + (cos(q(1) + q(2))*(q_dot(1)/2 + q_dot(2)/2))/2)*(cos(q(1))*q_Ddot(1) + cos(q(1) + q(2))*(q_Ddot(1)/2 + q_Ddot(2)/2) - sin(q(1))*q_dot(1)^2 - sin(q(1) + q(2))*(q_dot(1) + q_dot(2))*(q_dot(1)/2 + q_dot(2)/2)) + (cos(q(1))*q_dot(1) + cos(q(1) + q(2))*(q_dot(1)/2 + q_dot(2)/2))*((cos(q(1))*q_Ddot(1))/2 + (cos(q(1) + q(2))*(q_Ddot(1)/2 + q_Ddot(2)/2))/2 - (sin(q(1))*q_dot(1)^2)/2 - (sin(q(1) + q(2))*(q_dot(1) + q_dot(2))*(q_dot(1)/2 + q_dot(2)/2))/2) + (cos(q(1))^2*q_dot(1)*q_Ddot(1))/4 + (sin(q(1))^2*q_dot(1)*q_Ddot(1))/4;
% check2 = -q_dot' * G;
%=====================================
%==========
%==========
update_plot
%==========
%==========
 end
 
 
 
 
 