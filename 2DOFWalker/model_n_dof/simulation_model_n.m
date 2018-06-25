close all; clear all; clc;

Folder = cd;
addpath(genpath(fullfile(Folder, '..')));
%==================simulation model of a 2 link walker extended========================
syms m1 m2
syms q1(t) q2(t) q3(t) q4(t)  q5(t)
syms q1_dot(t) q2_dot(t) q3_dot(t) q4_dot(t) q5_dot(t)
syms q1_Ddot(t) q2_Ddot(t) q3_Ddot(t) q4_Ddot(t) q5_Ddot(t)
syms z1(t) z2(t) z1_dot(t) z2_dot(t) z1_Ddot(t) z2_Ddot(t)
syms I1 I2
syms g
syms alfa
syms m I l lc
alfa = 0;
parent_tree = [0,1,2,3];
% parent_tree = [0,1];
n_link = length(parent_tree);
MatrixVelocityRelabel = inv(flip(tril(ones(n_link)))) *tril(ones(n_link));
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
% for i = 1:n_link
%     rc_rel(1,i) = subs(rc_rel(1,i),0.8);
% end
rc_rel(1,1) = 1-0.8;
rc_rel(1,2) = 0.8;
rc_rel(1,3) = 0.8;
rc_rel(1,4) = 0.8;
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
%======================pi/18
numericVar = zeros(n_link+2,1,3);
symbolicVar = cat(3,qe.',qe_dot.',qe_Ddot.');

for i = 1:n_link
    for j = 1:3
        numericVar(i,:,j) = subs(symbolicVar(i,:,j),symbolicVar(i,:,j),startingParameters(i,j));
    end
end

%=====================
         
% D = subs(D, alfa, double(subs(alfa,AngleSlope)));
% C = subs(C, alfa, double(subs(alfa,AngleSlope)));
% G = subs(G, alfa, double(subs(alfa,AngleSlope)));
% deltaF2 = subs(deltaF2, alfa, double(subs(alfa,AngleSlope)));
% deltaqDotBar = subs(deltaqDotBar, alfa, double(subs(alfa,AngleSlope)));

% alfa = double(subs(alfa,AngleSlope));

symD = D;
symC = C;
symG = G;
symE2 = E2;
symPhi = phi;


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
%=======
%=======
set_plot;
%=======
%=======
j = 0;
F = zeros(length(q),1);
time = 0;
dt = 0.005; %0.001

disp('push a button to continue'); pause;
 while 1
     
j = j + 1;
time = (j-1)*dt;

[D,C,G] = updatateDynMatrices(symD,symC,symG,symbolicVar,numericVar);
% q = double(q);
% q_dot = double(q_dot);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
q_old = q;
q_dot_old = q_dot;
Links_old = Links;

q_Ddot =  D \ (F - C*(q_dot) - G);
q_dot = q_dot + dt * q_Ddot;
q = q + dt * q_dot; 


for i  = 1:length(q)
    if q(i) >= 2*pi
        q(i) = q(i) - 2*pi;
    end
end

yLineTerrain_old = yLineTerrain;
yLineTerrain = double(tan(alfa) * Links(n_link,1,2));
Links = simKinematics_n(q,parent_tree,robotData,Base);  %update kinematics

numericVar = cat(3,q,q_dot,q_Ddot);


%===========impact switch===========
if Links(n_link,2,2) <= yLineTerrain && Links_old(n_link,2,2) > yLineTerrain_old   %link,axis,begin/end
% flag_plot = 1;
% 
%====================impact model===============
phi = Base + double(subs(symPhi,symbolicVar,numericVar));
set(p2plot,'xdata',phi(1),'ydata',phi(2));
E2 = double(subs(symE2,symbolicVar,numericVar));
deltaF2 = -inv(E2 * inv(D) * E2.') * E2 * [eye(n_link); zeros(2,n_link)];
deltaqDotBar = inv(D) * E2.' * deltaF2 + [eye(n_link); zeros(2,n_link)];
%===================================
q_old = q;
q(1) = -(pi -q_old(1) -q_old(2) -q_old(3) -q_old(4)); %- 2*alfa; 
q(2) = -q_old(4);
q(3) = -q_old(3);
q(4) = -q_old(2);
%==================check========================
% q_dot_check1 = deltaqDotBar * q_dot(1:n_link);
%===============================================
% q_dot(1:n_link) = [eye(n_link) zeros(n_link,2)] * deltaqDotBar * q_dot(1:n_link); %q

q_dot = deltaqDotBar * q_dot(1:n_link);
q_dot(1:n_link) = MatrixVelocityRelabel*q_dot(1:n_link);
% ==================check=======================
% numericVar = cat(3,q,q_dot,q_Ddot);
% 
% E2 = double(subs(symE2,symbolicVar,numericVar));
% q_dot_check2 = q_dot;
% q_dot_check2(5:6) = 0;
% p2_check2 = E2*q_dot_check2;
%===============================================
Base = [Links(n_link,1,2);
        yLineTerrain];

numericVar = cat(3,q,q_dot,q_Ddot);   
phi = Base + double(subs(symPhi,symbolicVar,numericVar));
set(p2plot,'xdata',phi(1),'ydata',phi(2));

Links = simKinematics_n(q,parent_tree,robotData,Base); %update kinematics

% % F2 = deltaF2 * q_dot(1:2);
% % delete(handleQuiver); 
end
%===================================
%==========
%==========
update_plot
%==========
%==========
 end