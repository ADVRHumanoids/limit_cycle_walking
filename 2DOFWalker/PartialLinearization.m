clear all;
clc;
% linearization of a walkin robot choosing the output as the actuated
% joint, q2.
%partial linearization of order 2.

% model of robot:
% M * q_Ddot + C * q_dot + G = B * u

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
h = q(1) + ((pi -q(1)-q(2)));

h_dq = jacobian(h,q);
dy_dt = h_dq * q_dot;

h_dq_dq = [jacobian(dy_dt, q) h_dq];

L2fh  = simplify(-h_dq_dq(end-1:end)* inv(D) * C* q_dot - h_dq_dq(end-1:end) * inv(D) * G);
LgLfh = h_dq_dq(end-1:end) * inv(D) * B;

u = simplify(inv(LgLfh) * (v - L2fh));

u = collect(u, q_dot(1));
u = collect(u, q_dot(2));
u = collect(u, v);


tau = [0;u];

q_Ddot =  simplify(D \ (tau - C*(q_dot) - G));
%=========================================================================

alpha = 0.9;
epsilon = 0.1;

x1 = q1 + q2;
x2 = epsilon*(q1_dot + q2_dot);

phi = x1 + (1/(2 - alpha))*sign(x2)*abs(x2)^(2-alpha);
psi = -sign(x2)*abs(x2)^alpha - sign(phi) * abs(phi)^(alpha/(2 - alpha));

v = 1/epsilon^2 * psi;