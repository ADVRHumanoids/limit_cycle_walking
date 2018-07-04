close all; clear all; clc;

Folder = cd;
addpath(genpath(fullfile(Folder, '..')));
%==================simulation model of a 2 link walker extended========================
flagSim = 1;
slope = 0;%-pi/28;
parent_tree = [0 1];
n_link = length(parent_tree);
Base = [0;0];

link_length = 1;
com_position = 0.5; %0.8
mass = 1; %0.3
inertia = [0.083 0.083];

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
startingParameters = [pi/9,   0,    0;...  %q1     %pi/18
                      3*pi/4,  0,    0;...  %q2      3*pi/4,    50,    0,...  %q2
                      0,       0,    0;...  %z1
                      0,       0,    0];    %z2
                  
%=======================starting kinematics================================
q = startingParameters(1:n_link+2,1);
q_dot = startingParameters(1:n_link+2,2);
q_Ddot = startingParameters(1:n_link+2,3);


Links = KinematicsLinks(q,parent_tree,robotData);  %update kinematics

yLineTerrain = double(tan(slope) * Links(n_link,1,2));
yLineTerrain_old = yLineTerrain;

%just 1 link
% T = (981*cos(q(1)))/200 + (333*q_dot(1)^2)/2000;
T = (981*cos(q(1) + q(2)))/200 + (2943*cos(q(1)))/200 + (q_dot(1)^2*cos(q(2)))/2 + (333*q_dot(1)*q_dot(2))/1000 + (833*q_dot(1)^2)/1000 + (333*q_dot(2)^2)/2000 + (q_dot(1)*q_dot(2)*cos(q(2)))/2;
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

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% for i = 1:length(parent_tree)
%     
%     K_terms(i) =   1/2 * m(i) * kinematics.velocityCoM_absolute(:,:,i).'*kinematics.velocityCoM_absolute(:,:,i) ...
%                  + 1/2 * kinematics.angularVelocity_absolute(:,:,i).' * I(i) * kinematics.angularVelocity_absolute(:,:,i);
% end
% 
% K = sum(K_terms);
% 
% 
% p = cat(3,[0;0],kinematics.linksPosition);
% for i = 1:length(parent_tree)
% P_terms(i,:) = m(i) * g * (q(end) + p(2,:,i) + kinematics.positionCoM_absolute(2,:,i));
% end
% 
% P = sum(P_terms);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%============impact========================================================
if Links(n_link,2,2) <= yLineTerrain && Links_old(n_link,2,2) > yLineTerrain_old   %link,axis,begin/end  
    
    [q,q_dot] = impact_handler(q,q_dot);

    Base = [Links(n_link,1,2); yLineTerrain]; %TODO
    q(end-1:end) = Base;
    Links = KinematicsLinks(q,parent_tree,robotData); %update kinematics
    
end
% %============controller==================================================
alpha = 0.1;
epsilon = 0.1;

x1 = q(1) + q(2);
x2 = epsilon*(q_dot(1) + q_dot(2));

phi = x1 + (1/(2 - alpha))*sign(x2)*abs(x2)^(2-alpha);
psi = -sign(x2)*abs(x2)^alpha - sign(phi) * abs(phi)^(alpha/(2 - alpha));

v = 1/epsilon^2 * psi;
%==========================================================================
h_dq_dq = [0 0 2 1];
B = [0; 1];
L2fh  = -h_dq_dq(end-1:end)* inv(D) * C* q_dot(1:n_link) - h_dq_dq(end-1:end) * inv(D) * G;
LgLfh =  h_dq_dq(end-1:end) * inv(D) * B;

u = inv(LgLfh) * (v - L2fh);
tau = [0;u];
% %=========mechanicalenergy=============
% mechanical energy 1 link
% T = (981*cos(q(1)))/200 + (333*q_dot(1)^2)/2000;
% mechanical energy 2 link
T = (981*cos(q(1) + q(2)))/200 + (2943*cos(q(1)))/200 + (q_dot(1)^2*cos(q(2)))/2 + (333*q_dot(1)*q_dot(2))/1000 + (833*q_dot(1)^2)/1000 + (333*q_dot(2)^2)/2000 + (q_dot(1)*q_dot(2)*cos(q(2)))/2;
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
 
 
 
 % %============controller====================================================
% alpha = 6; 
% q2d = -2*alpha/ pi * atan(q_dot(1));
% % q2d = -2*alpha/pi * sign(q_dot(1));
% % q2d = 0;
% % q2d_Ddot =  ;
% % q2d_dot = q2d_dot + dt * q2d_Ddot;
% % q2d = q2d + dt * q2d_dot; 
% u = k_p*(q2d - q(2)) - k_d * q_dot(2);
% % u = q2d_Ddot + k_d*(q2d_dot - q_dot(2)) + k_p*(q2d - q(2));
% % %============linearization=================================================
% CommonTerm = D(1,2) * inv(D(1,1));
% q2_Ddot_term =  D(2,2) - CommonTerm * D(1,2);
% q1_dot_term =   C(2,1) - CommonTerm * C(1,1);
% q2_dot_term =   C(2,2) - CommonTerm * C(1,2);

% tau(2) = q2_Ddot_term * u + q1_dot_term * q_dot(1) + q2_dot_term * q_dot(2) + G(2) - CommonTerm * G(1);