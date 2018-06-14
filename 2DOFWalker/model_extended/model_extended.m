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
nlink = 2;

GeneralizedCoordinates = [q1(t),q2(t),z1(t),z2(t)];
first_derivative_names = [q1_dot(t), q2_dot(t), z1_dot(t), z2_dot(t)];
second_derivative_names = [q1_Ddot(t), q2_Ddot(t), z1_Ddot(t), z2_Ddot(t)];
dimGC = length(GeneralizedCoordinates);

d_GeneralizedCoordinates = diff_t(GeneralizedCoordinates, GeneralizedCoordinates, first_derivative_names);
Dd_GeneralizedCoordinates = diff_t(d_GeneralizedCoordinates, d_GeneralizedCoordinates, second_derivative_names);  

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
