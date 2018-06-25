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
syms I1 I2
syms g
syms q_dot_plus F2
nlink = 2;

GeneralizedCoordinates = [q1(t),q2(t)];
first_derivative_names = [q1_dot(t), q2_dot(t)];
second_derivative_names = [q1_Ddot(t), q2_Ddot(t)];
dimGC = length(GeneralizedCoordinates);

d_GeneralizedCoordinates = diff_t(GeneralizedCoordinates, GeneralizedCoordinates, first_derivative_names);
Dd_GeneralizedCoordinates = diff_t(d_GeneralizedCoordinates, d_GeneralizedCoordinates, second_derivative_names);  

kinematics;
Lagrange;
%==========================================================================
DynamicEquations = [simplify(First_eq), simplify(Second_eq)];     
%=============================getting D matrix=============================
dynamics;


D_dot = diff_t(D,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);

N = (D_dot - 2*C);
ThisShouldbeZero = simplify(d_GeneralizedCoordinates*N*d_GeneralizedCoordinates.');
%==========================================================================
