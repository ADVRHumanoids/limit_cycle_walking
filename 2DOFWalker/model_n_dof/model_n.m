clear all; clc
%==================dynamic model of a 2 link walker========================
syms m1 m2
syms q1(t) q2(t) q3(t) q4(t)  q5(t)
syms q1_dot(t) q2_dot(t) q3_dot(t) q4_dot(t) q5_dot(t)
syms q1_Ddot(t) q2_Ddot(t) q3_Ddot(t) q4_Ddot(t) q5_Ddot(t)
syms z1(t) z2(t) z1_dot(t) z2_dot(t) z1_Ddot(t) z2_Ddot(t)
syms I1 I2
syms g
syms alfa

parent_tree = [0,1,2,2,4];
% parent_tree = [0,1,2];
n_link = length(parent_tree);

q = [q1(t), q2(t), q3(t), q4(t), q5(t)];
q_dot = [q1_dot(t), q2_dot(t), q3_dot(t), q4_dot(t)  q5_dot(t)];
q_Ddot = [q1_Ddot(t), q2_Ddot(t), q3_Ddot(t), q4_Ddot(t), q5_Ddot(t)];

z = [z1(t), z2(t)];
z_dot = [z1_dot(t), z2_dot(t)];
z_Ddot = [z1_Ddot(t), z2_Ddot(t)];

qe = [q, z];
qe_dot = [q_dot, z_dot];
qe_Ddot = [q_Ddot, z_Ddot];

d_GeneralizedCoordinates = diff_t(qe, qe, qe_dot);
Dd_GeneralizedCoordinates = diff_t(qe_dot, qe_dot, qe_Ddot);  


rp_rel = sym(zeros(3,length(parent_tree)));
rp_rel(1,:) = sym('l',[length(parent_tree),1]);

rc_rel = sym(zeros(3,length(parent_tree)));
rc_rel(1,:) = sym('lc',[length(parent_tree),1]);

m = sym('m',[length(parent_tree),1]);
I = sym('I',[length(parent_tree),1]);

kinematics_n;
Lagrange_n;
%==========================================================================
%=============================getting D matrix=============================
dynamics_n;


D_dot = diff_t(D,[q,q_dot], [q,q_dot]);

N = (D_dot - 2*C);
ThisShouldbeZero = simplify(qe_dot*N*qe_dot.'); %NOT ZERO!
%==========================================================================
%==========================================================================

impact_extended; %output deltaqDot deltaq
% A = [ D -E2.';
%      E2  zeros(2)];
% B = [D * d_GeneralizedCoordinates.'; zeros(2,1)];
% X = linsolve(A,B);
