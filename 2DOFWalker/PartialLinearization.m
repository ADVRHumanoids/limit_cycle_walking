clear all;
clc;
% linearization of a walkin robot choosing the output %h as wanted


%partial linearization of order 2, since the input is explicitly function
%of the output after 2 derivations.

% model of robot:
% M * q_Ddot + C * q_dot + G = B * u
% 
syms d11 d12 d21 d22...
c11 c12 c21 c22...
g1 g2...
q1 q2 q1_dot q2_dot...
u v...
y;

q = [q1; q2];
q_dot = [q1_dot; q2_dot];


D = [d11 d12; d12 d22];
C = [c11 c12; c21 c22];
G = [g1; g2];
B = [0; 1];
% 
% 
h =  q(1) - (pi - q(1) - q(2)); %%output of the system

h_dq = jacobian(h,q);  % h_dq = functionalDerivative(h,q)';

Lfh = h_dq * q_dot;

h_dq_dq = [jacobian(Lfh, q) h_dq]; % h_dq_dq = [functionalDerivative(Lfh,q)'  h_dq];

L2fh  = simplify(-h_dq_dq(end-1:end)* inv(D) * C* q_dot - h_dq_dq(end-1:end) * inv(D) * G);
LgLfh = h_dq_dq(end-1:end) * inv(D) * B;

v = 0; % control law after linearization of the system
u = simplify(inv(LgLfh) * (v - L2fh)); %% input partially linearizing the system

u = collect(u, q_dot(1));
u = collect(u, q_dot(2));
u = collect(u, v); 


%check if it works
tau = [0;u];
q_Ddot =  simplify(D \ (tau - C*(q_dot) - G));
%=========================================================================
% controller said to be good for double integrator

% alpha = 0.9;
% epsilon = 0.1;
% 
% x1 = q1 + q2;
% x2 = epsilon*(q1_dot + q2_dot);
% 
% phi = x1 + (1/(2 - alpha))*sign(x2)*abs(x2)^(2-alpha);
% psi = -sign(x2)*abs(x2)^alpha - sign(phi) * abs(phi)^(alpha/(2 - alpha));
% 
% v = 1/epsilon^2 * psi;