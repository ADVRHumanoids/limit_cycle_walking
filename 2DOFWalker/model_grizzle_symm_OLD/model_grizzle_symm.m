clear all; clc
%=======dynamic model of a 2 link walker (grizzle book notation)===========
nlink = 2;
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

syms m I l lc
%======================
m1 = m;
m2 = m;
I1 = I;
I2 = I;
l1 = l;
l2 = l;
lc1 = l-lc;
lc2 = lc;
% q1(t) = pi - q1(t);
% q2(t) = 3*pi/2 - q2(t);
%======================
GeneralizedCoordinates = [q1(t),q2(t),z1(t),z2(t)];
% GeneralizedCoordinates = [q1(t),q2(t)];
first_derivative_names = [q1_dot(t), q2_dot(t), z1_dot(t), z2_dot(t)];
% first_derivative_names = [q1_dot(t), q2_dot(t)];
second_derivative_names = [q1_Ddot(t), q2_Ddot(t), z1_Ddot(t), z2_Ddot(t)];
% second_derivative_names = [q1_Ddot(t), q2_Ddot(t)];
dimGC = length(GeneralizedCoordinates);

d_GeneralizedCoordinates = diff_t(GeneralizedCoordinates, GeneralizedCoordinates, first_derivative_names);
Dd_GeneralizedCoordinates = diff_t(d_GeneralizedCoordinates, d_GeneralizedCoordinates, second_derivative_names);  

kinematics_grizzle_symm;
Lagrange_grizzle_symm;

%==========================================================================
DynamicEquations = [simplify(First_eq), simplify(Second_eq), simplify(Third_eq), simplify(Fourth_eq)];
% DynamicEquations = [simplify(First_eq), simplify(Second_eq)]; 
%==========================================================================

%==========================================================================
dynamics_grizzle_symm;
%===============================check N====================================
D_dot = diff_t(D,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);

N = (D_dot - 2*C);
ThisShouldbeZero = simplify(d_GeneralizedCoordinates*N*d_GeneralizedCoordinates.');
%==========================================================================
%==========================================================================
% phi = [l2 * cos(q2(t)) - l1 * cos(q1(t) + q2(t));
%        l2 * sin(q2(t)) + l1 * sin(q1(t) + q2(t))];
   
phi = rp2_0(1:2) + rp1_0(1:2);
% rc1_0 =
p2 = [z1(t);z2(t)] + phi;
% p2 = [z1(t) + l2 * sin(q2(t)) + l1 * sin(q1(t) + q2(t));
%        z2(t) + l2 * cos(q2(t)) + l1 * cos(q1(t) + q2(t))]; %tip of the swinging leg in [q, z]

% E2 = [l2 * cos(q2(t)) + l1 * cos(q1(t) + q2(t)), l1 * cos(q1(t) + q2(t)) 1 0;
%       - l2 * sin(q2(t)) - l1 * sin(q1(t) + q2(t)), - l1 * sin(q1(t) + q2(t)) 0 1];
E2 = sym(zeros(size(phi,1), dimGC));

for j = 1:size(phi,1)
  for i = 1:dimGC
        E2(j,i) = functionalDerivative(p2(j),GeneralizedCoordinates(i));
  end
end 
% m1 = [D, -E2.';1
%       E2, zeros(2)];
  
deltaF2 = -inv(E2 * inv(D) * E2.') * E2 * [eye(nlink); zeros(2)];
% 
deltaqDotBar = inv(D) * E2.' * deltaF2 + [eye(nlink); zeros(2)];
% 
R = [-1 0;
     -1 1];
%  
deltaqDot = [R zeros(nlink,2)] * deltaqDotBar;

% A = [ D -E2.';
%      E2  zeros(2)];
% B = [D * d_GeneralizedCoordinates.'; zeros(2,1)];
% X = linsolve(A,B);
