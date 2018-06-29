syms m1 m2 ...
q1(t) q2(t) q3(t) q4(t)  q5(t) ...
q1_dot(t) q2_dot(t) q3_dot(t) q4_dot(t) q5_dot(t) ...
q1_Ddot(t) q2_Ddot(t) q3_Ddot(t) q4_Ddot(t) q5_Ddot(t) ...
z1(t) z2(t) z1_dot(t) z2_dot(t) z1_Ddot(t) z2_Ddot(t) ...
I1 I2 ...
g ...
slope ...
I l lc

flagSim = 0;
parent_tree = [0,1,2,3];
% parent_tree = [0,1];
n_link = length(parent_tree);
% 
% q = [q1(t), q2(t)];
% q_dot = [q1_dot(t), q2_dot(t)];
% q_Ddot = [q1_Ddot(t), q2_Ddot(t)];

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


generalizedVariables.qe = qe;
generalizedVariables.qe_dot = qe_dot;
generalizedVariables.qe_Ddot = qe_Ddot;



%==========================================================================

link_length = sym('l',[length(parent_tree),1]).';
com_position = sym('lc',[length(parent_tree),1]).';
m = sym('m',[length(parent_tree),1]);
I = sym('I',[length(parent_tree),1]);

%==========================================================================

robotData = struct('n_link',n_link,'link_length',link_length, 'com_position',com_position, 'mass',m, 'inertia',I,'gravity', g, 'flagSim', flagSim);

