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
q1 q2 q3 q1_dot q2_dot q3_dot...
u v...
y;

% q = [q1; q2; q3];
% q_dot = [q1_dot; q2_dot; q3_dot];
q = [q1; q2];
q_dot = [q1_dot; q2_dot];

D = [d11 d12; d12 d22];
C = [c11 c12; c21 c22];
G = [g1; g2];
B = [0; 1];
% 
% 
h(1) =  q(1) - (pi- q(1) - q(2)); %%output of the system
% h(2) = q(3);
h_dq = jacobian(h,q);  % h_dq = functionalDerivative(h,q)';

Lfh = h_dq * q_dot;

h_dq_dq = [jacobian(Lfh, q) h_dq]; % h_dq_dq = [functionalDerivative(Lfh,q)'  h_dq];

L2fh  = simplify(-h_dq_dq(end-1:end)* inv(D) * C* q_dot - h_dq_dq(end-1:end) * inv(D) * G);
LgLfh = h_dq_dq(end-1:end) * inv(D) * B;

v = 5; % control law after linearization of the system
u = simplify(inv(LgLfh) * (v - L2fh)); %% input partially linearizing the system

u = collect(u, q_dot(1));
u = collect(u, q_dot(2));
u = collect(u, v); 


%check if it works
tau = [0;u];
q_Ddot =  simplify(D \ (tau - C*(q_dot) - G));

should_be_v = simplify(q_Ddot(1) - (pi- q_Ddot(1) - q_Ddot(2)));
