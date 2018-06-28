close all; clear all; clc;

Folder = cd;
addpath(genpath(fullfile(Folder, '..')));
%==================simulation model of a 2 link walker extended========================
syms m1 m2 ...
q1(t) q2(t) q3(t) q4(t)  q5(t) ...
q1_dot(t) q2_dot(t) q3_dot(t) q4_dot(t) q5_dot(t) ...
q1_Ddot(t) q2_Ddot(t) q3_Ddot(t) q4_Ddot(t) q5_Ddot(t) ...
z1(t) z2(t) z1_dot(t) z2_dot(t) z1_Ddot(t) z2_Ddot(t) ...
I1 I2 ...
g ...
slope ...
I l lc

slope = 0;%-pi/8-pi/8;-pi/10 

parent_tree = [0,1,2,3];
% parent_tree = [0,1];
n_link = length(parent_tree);


% q = [q1(t), q2(t)];
% q_dot = [q1_dot(t), q2_dot(t)];
% q_Ddot = [q1_Ddot(t), q2_Ddot(t)];

% q = [q1(t), q2(t), q3(t)];
% q_dot = [q1_dot(t), q2_dot(t), q3_dot(t)];
% q_Ddot = [q1_Ddot(t), q2_Ddot(t), q3_Ddot(t)];

q = [q1(t), q2(t), q3(t), q4(t)];
q_dot = [q1_dot(t), q2_dot(t), q3_dot(t), q4_dot(t),];
q_Ddot = [q1_Ddot(t), q2_Ddot(t), q3_Ddot(t), q4_Ddot(t)];

% q = [q1(t), q2(t), q3(t), q4(t), q5(t)];
% q_dot = [q1_dot(t), q2_dot(t), q3_dot(t), q4_dot(t)  q5_dot(t)];
% q_Ddot = [q1_Ddot(t), q2_Ddot(t), q3_Ddot(t), q4_Ddot(t), q5_Ddot(t)];

z = [z1(t), z2(t)];
z_dot = [z1_dot(t), z2_dot(t)];
z_Ddot = [z1_Ddot(t), z2_Ddot(t)];

qe = [q, z];
qe_dot = [q_dot, z_dot];
qe_Ddot = [q_Ddot, z_Ddot];

dim_qe = size(qe,2);

rp_rel = sym(zeros(3,length(parent_tree)));
rp_rel(1,:) = sym('l',[length(parent_tree),1]);

rc_rel = sym(zeros(3,length(parent_tree)));
rc_rel(1,:) = sym('lc',[length(parent_tree),1]);

m = sym('m',[length(parent_tree),1]);
I = sym('I',[length(parent_tree),1]);


for i = 1:n_link
    rp_rel(1,i) = subs(rp_rel(1,i),1);
end

rc_rel(1,1) = 1-0.8;
for i = 2:n_link
    rc_rel(1,i) = subs(rc_rel(1,i),0.8);
end

for i = 1:n_link
    m(i) = subs(m(i),0.3);
end
for i = 1:n_link
    I(i) = subs(I(i),0.03);
end

g = subs(g,9.81);
%======================
robotData = struct('link_length',rp_rel, 'com_link_length',rc_rel, 'mass',m, 'inertia',I);

%======================
kinematics_n;
Lagrange_n;
dynamics_n; 
impact_n;
%======================
AngleSlope = 0;%-pi/4; 

                      %pos / vel / acc
startingParameters = [ pi/18,   0,    0;...  %q1    pi/18
                      -pi/18,   0,    0;...  %q2     -pi/18
                       3*pi/4,  0,    0;...  %q3     3*pi/4,
                       pi/4,    0,    0;...  %q4      pi/4,
%                      0,       0,    0;...  %q5
                       0,       0,    0;...  %z1
                       0,       0,    0];    %z2
% startingParameters = [pi/18,   0,    0;...  %q1     %pi/18
%                       pi/2,  0,    0;...  %q2      3*pi/4,    50,    0,...  %q2
%                       0,       0,    0;...  %z1
%                       0,       0,    0];    %z2
                  
%======================pi/18
numericVar = zeros(n_link+2,1,3);
symbolicVar = cat(3,qe.',qe_dot.',qe_Ddot.');

for i = 1:n_link
    for j = 1:3
        numericVar(i,:,j) = subs(symbolicVar(i,:,j),symbolicVar(i,:,j),startingParameters(i,j));
    end
end


symD = D;
symC = C;
symG = G;
symE2 = E2;
symPhi = phi;


symq = q;
symq_dot = q_dot;
symq_Ddot = q_Ddot;

% [matD, matC, matG, matE2] = replaceMatrices(symD, symC, symG, symE2);
% %when needed


q = numericVar(:,:,1);
q_dot = numericVar(:,:,2);
q_Ddot = numericVar(:,:,3);
         

phi = double(subs(symPhi,symbolicVar,numericVar));

Origin = [0;0]; 
Base = [0;0];
%=======================starting kinematics================================
robotData.link_length = eval(robotData.link_length);
robotData.com_link_length = eval(robotData.com_link_length);
robotData.mass = eval(robotData.mass);
robotData.inertia = eval(robotData.inertia);

Links = simKinematics_n(q,parent_tree,robotData,Base);  %update kinematics

yLineTerrain = double(tan(slope) * Links(n_link,1,2));
yLineTerrain_old = yLineTerrain;

%=======
%=======
set_plot;
%=======
%=======
j = 0;
tau = zeros(length(q),1);
time = 0;
dt = 0.001; %0.001


k_p = 1;
k_d = 1;

disp('push a button to continue'); pause;
 while 1
     
j = j + 1;
time = (j-1)*dt;

[D,C,G] = calcDynMatrices(q,q_dot);
% [D,C,G] = updatateDynMatrices(symD,symC,symG,symbolicVar,numericVar);

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

% numericVar = cat(3,q,q_dot,q_Ddot);
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