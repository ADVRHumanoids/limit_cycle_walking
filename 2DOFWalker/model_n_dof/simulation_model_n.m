close all; clear all; clc;

Folder = cd;
addpath(genpath(fullfile(Folder, '..')));
%==================simulation model of a 2 link walker extended========================
flagSim = 1;
slope = 0; %-pi/28;
parent_tree = [0 1];
n_link = length(parent_tree);
Base = [0;0];

link_length = 1;
com_position = 0.8; %0.8
mass = 0.3; %0.3
inertia = [0.03 0.03];

link_length = link_length * ones(1,length(parent_tree));
com_position = [1-com_position, com_position * ones(1,length(parent_tree)-1)];
m = mass * ones(1,length(parent_tree));
% I = inertia * ones(1,length(parent_tree));
I = inertia;
g = 9.81;
%==========================================================================
robotData = struct('n_link',n_link,'link_length',link_length, 'com_position',com_position, 'mass',m, 'inertia',I,'gravity', g, 'flagSim', flagSim);




                      %pos / vel / acc
% startingParameters = [ pi/18,   0,    0;...  %q1    pi/18
%                       -pi/18,   0,    0;...  %q2     -pi/18
%                        3*pi/4,  0,    0;...  %q3     3*pi/4,
%                        pi/4,    0,    0;...  %q4      pi/4,
% %                      0,       0,    0;...  %q5
%                        0,       0,    0;...  %z1
%                        0,       0,    0];    %z2
startingParameters = [pi/100,   0,    0;...  %q1     %pi/18
                      pi - pi/18,  0,    0;...  %q2      3*pi/4,    50,    0,...  %q2 -2*(pi/18)+pi
                      0,       0,    0;...  %z1
                      0,       0,    0];    %z2
                  
%=======================starting kinematics================================
q = startingParameters(1:n_link+2,1);
q_dot = startingParameters(1:n_link+2,2);
q_Ddot = startingParameters(1:n_link+2,3);


Links = KinematicsLinks(q,parent_tree,robotData);  %update kinematics

yLineTerrain = double(tan(slope) * Links(n_link,1,2));
yLineTerrain_old = yLineTerrain;

% mechanical energy 1 link
% T = (981*cos(q(1)))/200 + (333*q_dot(1)^2)/2000;
% mechanical energy 2 link
% T = (981*cos(q(1) + q(2)))/200 + (2943*cos(q(1)))/200 + (q_dot(1)^2*cos(q(2)))/2 + (333*q_dot(1)*q_dot(2))/1000 + (833*q_dot(1)^2)/1000 + (333*q_dot(2)^2)/2000 + (q_dot(1)*q_dot(2)*cos(q(2)))/2;
%=======
%=======
set_plot;
%=======
%=======


j = 0;
time = 0;
dt = 0.005; %0.001

tau = zeros(length(q),1);
k_p = 1;
k_d = 0.1;
Base = [0,0];
disp('push a button to continue'); pause;

anglePi = pi;
 while 1
     
j = j + 1;
time = (j-1)*dt;

[D_ext,C_ext,G_ext] = calcDynMatrices(q,q_dot);
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

% 
for i  = 1:length(q)
    if q(i) >= 2*pi
        q(i) = q(i) - 2*pi;
    end
end

yLineTerrain_old = yLineTerrain;
yLineTerrain = double(tan(slope) * Links(n_link,1,2));
[Links,kinematics] = KinematicsLinks(q,parent_tree,robotData);  %update kinematics

%============impact========================================================
if Links(n_link,2,2) <= yLineTerrain && Links_old(n_link,2,2) > yLineTerrain_old   %link,axis,begin/end  
    
    [q,q_dot] = impact_handler(q,q_dot);

    Base = [Links(n_link,1,2); yLineTerrain+ 0.01]; %TODO
    q(end-1:end) = Base;
    Links = KinematicsLinks(q,parent_tree,robotData); %update kinematics
    anglePi = -anglePi;
    
end
%============controller==================================================
tau = controllerWalker(q,q_dot, D,C,G, anglePi);
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