clear all;
clc;
% linearization of a walking robot choosing the output %h as wanted


%partial linearization of order 2, since the input is explicitly function
%of the output after 2 derivations.

% model of robot:
% M * q_Ddot + C * q_dot + G = B * u

syms d11 d12 d13 ... 
     d21 d22 d23 ...
     d31 d32 d33 ...
     c11 c12 c13 ... 
     c21 c22 c23 ...
     c31 c32 c33 ...
     g1 g2 g3...
     q1 q2 q3 ...
     q1_dot q2_dot q3_dot...
     u v...
     y;



q = [q1; q2; q3];
q_dot = [q1_dot; q2_dot; q3_dot];
% q = [q1; q2];
% q_dot = [q1_dot; q2_dot];

n_unactuated_variable = 1;
n_base_variables = 2;
n_controlled_variables = length(q)- n_unactuated_variable; %-base z1 z2 // -underactuated var
%==========================================================================

D = [d11 d12 d13; 
     d21 d22 d23;
     d31 d32 d33];
 
C = [c11 c12 c13; 
     c21 c22 c23;
     c31 c32 c33];
 
G = [g1; 
     g2;
     g3];
 
B = [0 0;
     1 0;
     0 1]; 
 

% 
h(1) = q(2)- (pi- q(1) - q(2)); %%output of the system  
h(2) = q(3);
h_dq = jacobian(h,q);  % h_dq = functionalDerivative(h,q)';

Lfh = h_dq * q_dot;

h_dq_dq = [jacobian(Lfh, q) h_dq]; % h_dq_dq = [functionalDerivative(Lfh,q)'  h_dq];

L2fh  = simplify(-h_dq_dq(:,end-n_controlled_variables:end)* inv(D) * C* q_dot - h_dq_dq(:,end-n_controlled_variables:end) * inv(D) * G);
LgLfh = h_dq * inv(D) * B;

v = [0;
     0;]; % control law after linearization of the system
 
u = simplify(inv(LgLfh) * (v - L2fh)); %% input partially linearizing the system

u = collect(u, q_dot(1));
u = collect(u, q_dot(2));
u = collect(u, q_dot(3));
u = collect(u, v);


%check if it works
tau = [0;u];

q_Ddot =  simplify(D \ (tau - C*(q_dot) - G));

should_be_v(1,:) = simplify(q_Ddot(2)- (- q_Ddot(1) - q_Ddot(2))); % 
should_be_v(2,:) = simplify(q_Ddot(3));