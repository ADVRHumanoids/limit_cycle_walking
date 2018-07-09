function torque = controllerWalker(q,q_dot, D,C,G, anglePi)
% =========================================================================
n_link = length(q)-2;
%=====================
alpha = 0.9;
epsilon = 0.1;

y = q(1) - (anglePi - q(1) - q(2));
y_dot = 2* q_dot(1) + q_dot(2);%- q_dot(2); 


x1 = y;
x2 = epsilon*y_dot;
phi = x1 + (1/(2 - alpha))*sign(x2)*abs(x2)^(2-alpha);
psi = -sign(x2)*abs(x2)^alpha - sign(phi) * abs(phi)^(alpha/(2 - alpha));

v = 1/epsilon^2 * psi;
% ===================partial linearization=================================
h_dq_dq = [ 0, 0, 2, 1];
B = [0; 1];
L2fh  = -h_dq_dq(end-1:end)* inv(D) * C* q_dot(1:n_link) - h_dq_dq(end-1:end) * inv(D) * G;
LgLfh =  h_dq_dq(end-1:end) * inv(D) * B;

u = inv(LgLfh) * (v - L2fh);
torque = [0;u];
end









%% =========================================================================
% syms q1 q2 q1_dot q2_dot
% % 
% q = [q1; q2];
% q_dot = [q1_dot; q2_dot];
% h = - (q(1) - (anglePi - q(1) - q(2))); %%output of the system
% h_dq = jacobian(h,q);  % h_dq = functionalDerivative(h,q)';
% Lfh = h_dq * q_dot;
% h_dq_dq = [jacobian(Lfh, q) h_dq]; % h_dq_dq = [functionalDerivative(Lfh,q)'  h_dq];