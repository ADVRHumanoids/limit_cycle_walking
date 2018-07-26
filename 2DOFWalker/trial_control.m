syms m1 m2 lc1 lc2 l1 l2 I1 I2 g 

theta1 = m1 * lc1^2 + m2 * l1^2 + I1;
theta2 = m2 * lc2^2 + I2;
theta3 = m2 * l1 * lc2;
theta4 = m1 * lc1 + m2 * l1;
theta5 = m2 * lc2;
% 
D(1,1) = theta1 + theta2 + theta3 * 2 * cos(q(2));
D(1,2) = theta2 + theta3 * cos(q(2));
% D(2,1) = theta2 + theta3 * cos(q2(t));
% D(2,2) = theta2;
% C(1,1) = -theta3 * sin(q2(t)) * q2_dot(t);
% C(1,2) = -(q1_dot(t) + q2_dot(t)) * theta3 * sin(q2(t));
% C(2,1) =  theta3 * sin(q2(t)) * q1_dot(t);
% C(2,2) = 0;
% G(1,1) = -theta4 * g * sin(q1(t)) - theta5 * g * sin(q1(t) + q2(t));
% G(2,1) = -theta5 * g * sin(q1(t) + q2(t));

q = generalizedVariables.q;
q_dot = generalizedVariables.q_dot;
q_Ddot = generalizedVariables.q_Ddot;
cyclic_variable = 1;
P = model.dynamics.PotentialEnergy;
K = model.dynamics.KineticEnergy;
Lagrangian = K - P;
sigma = simplify(functionalDerivative(Lagrangian, q_dot(cyclic_variable)));

% sigma1 = simplify((theta1 + theta2 + 2*theta3*cos(q(2)))* q_dot(1) + (theta2 + theta3 * cos(q(2)))* q_dot(2));
% 
% eqAreEqual = isAlways(sigma == sigma1);
%==========================================================================
sigma_dot1 = simplify(functionalDerivative(Lagrangian, q(cyclic_variable)));
sigma_dot2 =  - theta4 * g * sin(q(1)) - theta5 * g * sin(q(1) + q(2)); %?????????????
sigma_dot3 =  - simplify(functionalDerivative(P, q(cyclic_variable)));
sigma_dot4 = simplify( model.kinematics.CoM_position(1)*(g*(m1 + m2)) );

eqAreEqual1 = isAlways(sigma_dot1 == sigma_dot2);
eqAreEqual2 = isAlways(sigma_dot2 == sigma_dot3);
eqAreEqual3 = isAlways(sigma_dot1 == sigma_dot3);
eqAreEqual4 = isAlways(sigma_dot1 == sigma_dot4);


int(D(1,2)/D(1,1), q(2));