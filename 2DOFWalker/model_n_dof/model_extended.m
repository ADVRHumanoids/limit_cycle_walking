clear all; clc
%==================dynamic model of a 2 link walker========================
syms m1 m2
syms vc1_0 vc2_0
syms w01_0 w12_0 w12_1
syms R1_0 R2_1 R2_0
syms rp1_0 rc1_0
syms rp2_0 rc2_0 
syms rp1_1 rc1_1 
syms rp2_2 rc2_2
syms l1 lc1 l2 lc2
syms q1(t) q2(t) q1_dot(t) q2_dot(t) q1_Ddot(t) q2_Ddot(t)
syms z1(t) z2(t) z1_dot(t) z2_dot(t) z1_Ddot(t) z2_Ddot(t)
syms I1 I2
syms g
syms theta1 theta2 theta3 theta4 theta5
syms q_dot_plus F2
syms alfa
nlink = 2;

q = [q1(t),q2(t)]; %q1(t),q2(t),q3(t),q4(t),q5(t)
q_dot = [q1_dot(t), q2_dot(t)];
q_Ddot = [q1_Ddot(t), q2_Ddot(t)];

z = [z1(t), z2(t)];
z_dot = [z1_dot(t), z2_dot(t)];
z_Ddot = [z1_Ddot(t), z2_Ddot(t)];

qe = [q, z];
qe_dot = [q_dot, z_dot];
qe_Ddot = [q_Ddot, z_Ddot];

d_GeneralizedCoordinates = diff_t(qe, qe, qe_dot);
Dd_GeneralizedCoordinates = diff_t(qe_dot, qe_dot, qe_Ddot);  

kinematics_extended;

Lagrange_extended;
%==========================================================================
DynamicEquations = [simplify(First_eq), simplify(Second_eq), simplify(Third_eq), simplify(Fourth_eq)];
%=============================getting D matrix=============================
dynamics_extended;


D_dot = diff_t(D,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);

N = (D_dot - 2*C);
ThisShouldbeZero = simplify(d_GeneralizedCoordinates*N*d_GeneralizedCoordinates.');
%==========================================================================
%==========================================================================

impact_extended; %output deltaqDot deltaq
% A = [ D -E2.';
%      E2  zeros(2)];
% B = [D * d_GeneralizedCoordinates.'; zeros(2,1)];
% X = linsolve(A,B);
