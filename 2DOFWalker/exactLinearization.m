clear all; clc;
syms d11 d12 d22 ...
c11 c12 c21 c22 ...
G1 G2 ...
q1(t) q2(t) ...
q1_dot(t) q2_dot(t)...
q1_Ddot(t) q2_Ddot(t) ...
z1(t) z2(t) z1_dot(t) z2_dot(t) z1_Ddot(t) z2_Ddot(t) ...
I1 I2 ...
g ...
slope ...
m I l lc ...
x1 x2 x3 x4 u ...
x2_dot x2_dot x3_dot x4_dot ...
x2_Ddot x2_Ddot x3_Ddot x4_Ddot...
m1 m2 lc1 lc2 l1 l2...
theta1 theta2 theta3 theta4 theta5

% twoWalkerMatrices = load('dynMatrices.mat');
twoWalkerMatricesExtended = load('dynMatricesExtended.mat');

% D = simplify(twoWalkerMatrices.D);
% C = simplify(twoWalkerMatrices.C);
% G = simplify(twoWalkerMatrices.G);

% D = simplify(twoWalkerMatricesExtended.D(1:2,1:2));
% C = simplify(twoWalkerMatricesExtended.C(1:2,1:2));
% G = simplify(twoWalkerMatricesExtended.G(1:2));

parent_tree = [1,2];

q = [q1(t), q2(t)];
q_dot = [q1_dot(t), q2_dot(t)];
q_Ddot = [q1_Ddot(t), q2_Ddot(t)];

z = [z1(t), z2(t)];
z_dot = [z1_dot(t), z2_dot(t)];
z_Ddot = [z1_Ddot(t), z2_Ddot(t)];

qe = [q, z];
qe_dot = [q_dot, z_dot];
qe_Ddot = [q_Ddot, z_Ddot];

dim_qe = size(qe,2);

symbolicVar = cat(3,q.',q_dot.',q_Ddot.');
xVar(:,:,1) = [x1 x3].';
xVar(:,:,2) = [x2 x4].';
xVar(:,:,3) = [x2_dot x3_dot].';

D = [d11 d12; d12 d22];
C = [c11 c12; c21 c22];
G = [G1; G2];



%==============================================
% 
% theta1 = m1 * lc1^2 + m2 * l1^2 + I1;
% theta2 = m2 * lc2^2 + I2;
% theta3 = m2 * l1 * lc2;
% theta4 = m1 * lc1 + m2 * l1;
% theta5 = m2 * lc2;
% 
% D(1,1) = theta1 + theta2 + theta3 * 2 * cos(q2(t));
% D(1,2) = theta2 + theta3 * cos(q2(t));
% D(2,1) = theta2 + theta3 * cos(q2(t));
% D(2,2) = theta2;
% C(1,1) = -theta3 * sin(q2(t)) * q2_dot(t);
% C(1,2) = -(q1_dot(t) + q2_dot(t)) * theta3 * sin(q2(t));
% C(2,1) =  theta3 * sin(q2(t)) * q1_dot(t);
% C(2,2) = 0;
% G(1,1) = -theta4 * g * sin(q1(t)) - theta5 * g * sin(q1(t) + q2(t));
% G(2,1) = -theta5 * g * sin(q1(t) + q2(t));
%==============================================
x = [x1;x2;x3;x4];
xVar(:,:,1) = [x1 x3].';
xVar(:,:,2) = [x2 x4].';
xVar(:,:,3) = [x2_dot x4_dot].';

D = subs(D, symbolicVar, xVar);
C = subs(C, symbolicVar, xVar);
G = subs(G, symbolicVar, xVar);

f(1,:) = x2;
f(3,:) = x4;
x_dot = -inv(D)*C*[x2;x4] - inv(D)*G; %+ inv(D)*u;
f(2,:) = simplify(x_dot(1));
f(4,:) = simplify(x_dot(2));


gu = simplify(inv(D)) * [0;1];
g(1,:) = 0;
g(2,:) = gu(1);
g(3,:) = 0;
g(4,:) = gu(2);


%distributions involutive
jacf = jacobian(f,x);
jacg = jacobian(g,x);

% d_fg = jacg * f - jacf*g;
d_fg = liebracket(f,g,x,2);
d_fg = simplify(d_fg);