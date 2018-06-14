

% 
theta1 = m1 * lc1^2 + m2 * l1^2 + I1;
theta2 = m2 * lc2^2 + I2;
theta3 = m2 * l1 * lc2;
theta4 = m1 * lc1 + m2 * l1;
theta5 = m2 * lc2;
% 



D1(1,1) = theta1 + theta2 + theta3 * 2 * cos(q2(t));
D1(1,2) = theta2 + theta3 * cos(q2(t));
D1(1,3) = -theta4 * cos(q1(t)) - theta5 * cos(q1(t) + q2(t));
D1(1,4) = theta4 * sin(q1(t)) + theta5 * sin(q1(t) + q2(t));
D1(2,1) = theta2 + theta3 * cos(q2(t));
D1(2,2) = theta2;
D1(2,3) = -theta5 * cos(q1(t)+q2(t));
D1(2,4) = theta5 * sin(q1(t)+q2(t));
D1(3,1) = -theta4 * cos(q1(t)) - theta5 * cos(q1(t) + q2(t));
D1(3,2) = -theta5 * cos(q1(t)+q2(t));
D1(3,3) = m1 + m2;
D1(3,4) = 0;
D1(4,1) = theta4 * sin(q1(t)) + theta5 * sin(q1(t) + q2(t));
D1(4,2) = theta5 * sin(q1(t)+q2(t));
D1(4,3) = 0;
D1(4,4) = m1 + m2;


C1(1,1) = -theta3 * sin(q2(t)) * q2_dot(t);
C1(1,2) = -(q1_dot(t) + q2_dot(t)) * theta3 * sin(q2(t));
C1(1,3) = 0;
C1(1,4) = 0;
C1(2,1) =  theta3 * sin(q2(t)) * q1_dot(t);
C1(2,2) = 0;
C1(2,3) = 0;
C1(2,4) = 0;
C1(3,1) = theta4 * sin(q1(t)) * q1_dot(t) + theta5 * sin(q1(t) + q2(t)) * (q1_dot(t) + q2_dot(t));
C1(3,2) = theta5 * sin(q1(t) + q2(t)) * (q1_dot(t) + q2_dot(t));
C1(3,3) = 0;
C1(3,4) = 0;
C1(4,1) = theta4 * cos(q1(t)) * q1_dot(t) + theta5 * cos(q1(t) + q2(t)) * (q1_dot(t) + q2_dot(t));
C1(4,2) = theta5 * cos(q1(t) + q2(t)) * (q1_dot(t) + q2_dot(t));
C1(4,3) = 0;
C1(4,4) = 0;

G1(1,1) = -theta4 * g * sin(q1(t)) - theta5 * g * sin(q1(t) + q2(t));
G1(2,1) = -theta5 * g * sin(q1(t) + q2(t));
G1(3,1) = 0;
G1(4,1) = g * (m1 + m2);


%==========================================================================
c111 = 1/2 *(functionalDerivative(D(1,1),q1(t)));
c211 = 1/2 *(functionalDerivative(D(1,1),q2(t)));
c311 = 1/2 *(functionalDerivative(D(1,1),z1(t)));
c411 = 1/2 *(functionalDerivative(D(1,1),z2(t)));

c112 = functionalDerivative(D(2,1),q1(t)) - 1/2 *(functionalDerivative(D(1,1),q2(t)));
c212 = functionalDerivative(D(2,2),q1(t));

c121 = 1/2 *(functionalDerivative(D(1,1),q2(t)));
c221 = functionalDerivative(D(1,2),q2(t)) - 1/2 *(functionalDerivative(D(2,2),q1(t)));

c122 = functionalDerivative(D(2,2),q1(t));
c222 = 0;



C2(1,1) = c111 * q1_dot(t) + c211 * q2_dot(t);
C2(2,1) = c112 * q1_dot(t) + c212 * q2_dot(t);
C2(1,2) = c121 * q1_dot(t) + c221 * q2_dot(t);
C2(2,2) = c122 * q1_dot(t) + c222 * q2_dot(t);
