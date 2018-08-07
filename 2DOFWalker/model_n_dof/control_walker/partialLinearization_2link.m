
% linearization of a walking robot choosing the output %h as wanted


%partial linearization of order 2, since the input is explicitly function
%of the output after 2 derivations.

% model of robot:
% M * q_Ddot + C * q_dot + G = B * u

syms d11 d12 ... 
     d21 d22 ...
     c11 c12 ... 
     c21 c22 ...
     g1 g2 ...
     q1(t) q2(t) ...
     q1_dot(t) q2_dot(t) ...
     u v...
     y t;



q = [q1(t); q2(t)];
q_dot = [q1_dot(t); q2_dot(t)];


n_unactuated_variable = 1;
n_base_variables = 2;
n_controlled_variables = length(q)- n_unactuated_variable; %-base z1 z2 // -underactuated var
%==========================================================================
D = model.dynamicMatrices.D;
C = model.dynamicMatrices.C;
G = model.dynamicMatrices.G;
E2 = model.dynamicMatrices.E2;
% D = [d11 d12 ; 
%      d21 d22 ;];
%  
% C = [c11 c12; 
%      c21 c22;];
%  
% G = [g1; 
%      g2;];
%  
B = [0
     1]; 
 

% 
h = q(2); %%output of the system  
h_dq = functionalDerivative(h,q)';

Lfh = h_dq * q_dot;

h_dq_dq = [functionalDerivative(Lfh,q)'  h_dq];

L2fh  = simplify(-h_dq_dq(:,end-n_controlled_variables:end)* inv(D) * C* q_dot - h_dq_dq(:,end-n_controlled_variables:end) * inv(D) * G);
LgLfh = h_dq * inv(D) * B;

v = [0]; % control law after linearization of the system
 
u = simplify(inv(LgLfh) * (v - L2fh)); %% input partially linearizing the system

% u = collect(u, q_dot(1));
% u = collect(u, q_dot(2));
% u = collect(u, v);


%check if it works
tau = [0;u];

q_Ddot =  simplify(D \ (tau - C*(q_dot) - G));