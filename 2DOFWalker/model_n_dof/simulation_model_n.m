close all; clear all; clc;

Folder = cd;
addpath(genpath(fullfile(Folder, '..')));
%==================simulation model of a 2 link walker extended========================
slope = 0;
parent_tree = [0,1];
n_link = length(parent_tree);



link_length = 1;
com_position = 0.8;
mass = 0.3;
inertia = 0.03;
link_length = link_length * ones(1,length(parent_tree));
com_position = [1-com_position, com_position * ones(1,length(parent_tree)-1)];
m = mass * ones(1,length(parent_tree));
I = inertia * ones(1,length(parent_tree));
g = 9.81;

robotData = struct('n_link',n_link,'link_length',link_length, 'com_position',com_position, 'mass',m, 'inertia',I,'gravity',g);



                      %pos / vel / acc
% startingParameters = [ pi/18,   0,    0;...  %q1    pi/18
%                       -pi/18,   0,    0;...  %q2     -pi/18
%                        3*pi/4,  0,    0;...  %q3     3*pi/4,
%                        pi/4,    0,    0;...  %q4      pi/4,
% %                      0,       0,    0;...  %q5
%                        0,       0,    0;...  %z1
%                        0,       0,    0];    %z2
startingParameters = [pi/18,   0,    0;...  %q1     %pi/18
                      pi/2,  0,    0;...  %q2      3*pi/4,    50,    0,...  %q2
                      0,       0,    0;...  %z1
                      0,       0,    0];    %z2
                  
%=======================starting kinematics================================
q = startingParameters(1:n_link+2,1);
q_dot = startingParameters(1:n_link+2,2);
q_Ddot = startingParameters(1:n_link+2,3);

Base = [0;0];
Links = simKinematics_n(q,parent_tree,robotData,Base);  %update kinematics

yLineTerrain = double(tan(slope) * Links(n_link,1,2));
yLineTerrain_old = yLineTerrain;



%=======
%=======
set_plot;
%=======
%=======

j = 0;
time = 0;
dt = 0.001; %0.001

tau = zeros(length(q),1);
k_p = 1;
k_d = 1;

disp('push a button to continue'); pause;
 while 1
     
j = j + 1;
time = (j-1)*dt;

[D,C,G] = calcDynMatrices(q,q_dot);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
q_old = q;
q_dot_old = q_dot;
Links_old = Links;


q_Ddot =  D \ (tau - C*(q_dot) - G);
q_dot = q_dot + dt * q_Ddot;
q = q + dt * q_dot; 


for i  = 1:length(q)
    if q(i) >= 2*pi
        q(i) = q(i) - 2*pi;
    end
end

yLineTerrain_old = yLineTerrain;
yLineTerrain = double(tan(slope) * Links(n_link,1,2));
Links = simKinematics_n(q,parent_tree,robotData,Base);  %update kinematics

%============impact========================================================
if Links(n_link,2,2) <= yLineTerrain && Links_old(n_link,2,2) > yLineTerrain_old   %link,axis,begin/end  
[q,q_dot] = impact_handler(q,q_dot);
Base = [Links(n_link,1,2);
        yLineTerrain];
Links = simKinematics_n(q,parent_tree,robotData,Base); %update kinematics
end
% %============controller====================================================
% % q2d = 2/ (pi*atan(q_dot(1)));
% 
% q2d = 0;
% % q2d_Ddot =  ;
% % q2d_dot = q2d_dot + dt * q2d_Ddot;
% % q2d = q2d + dt * q2d_dot; 
% u = k_p*(q2d - q(2)) - k_d * q_dot(2);
% % u = q2d_Ddot + k_d*(q2d_dot - q_dot(2)) + k_p*(q2d - q(2));
% %============linearization=================================================
% CommonTerm = D(1,2) * inv(D(1,1));
% q2_Ddot_term =  D(2,2) - CommonTerm * D(1,2);
% q1_dot_term =   C(2,1) - CommonTerm * C(1,1);
% q2_dot_term =   C(2,2) - CommonTerm * C(1,2);
% 
% tau = q2_Ddot_term * u + q1_dot_term * q_dot(1) + q2_dot_term * q_dot(2) + G(2) - CommonTerm * G(1);
% 
% %===================================
%==========
%==========
update_plot
%==========
%==========
 end