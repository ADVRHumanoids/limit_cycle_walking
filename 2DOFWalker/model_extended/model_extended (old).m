clear all; clc

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

GeneralizedCoordinates = [q1(t),q2(t),z1(t),z2(t)];
first_derivative_names = [q1_dot(t), q2_dot(t), z1_dot(t), z2_dot(t)];
second_derivative_names = [q1_Ddot(t), q2_Ddot(t), z1_Ddot(t), z2_Ddot(t)];
dimGC = length(GeneralizedCoordinates);

d_GeneralizedCoordinates = diff_t(GeneralizedCoordinates, GeneralizedCoordinates, first_derivative_names);
Dd_GeneralizedCoordinates = diff_t(d_GeneralizedCoordinates, d_GeneralizedCoordinates, second_derivative_names);  

R1_0 = [sin(q1(t)) -cos(q1(t)) 0;
        cos(q1(t)) sin(q1(t))  0;
        0          0           1];

R2_1 = [ cos(q2(t)) sin(q2(t)) 0;
        -sin(q2(t)) cos(q2(t)) 0;
        0           0          1];
    
R2_0 = R1_0*R2_1;

w01_0 = [0; 0; q1_dot(t)];
w12_1 = [0; 0; q2_dot(t)];

w12_0 = w01_0 + R1_0 * w12_1; %[0 0 q1_dot+q2_dot];

rp1_1 = [l1; 0; 0];
rc1_1 = [lc1; 0; 0];

rp2_2 = [l2; 0; 0];
rc2_2 = [lc2; 0; 0];

rp1_0 = R1_0 * rp1_1;
rc1_0 = R1_0 * rc1_1;

rp2_0 = R2_0 * rp2_2;
rc2_0 = R2_0 * rc2_2;

v1_0 = [z1_dot(t); z2_dot(t); 0];
% v1_0 = [0; 0; 0];
v2_0 = v1_0 + cross(w01_0, rp1_0);

vc1_0 = v1_0 + cross(w01_0,rc1_0);
vc2_0 = v2_0 + cross(w12_0,rc2_0);

K =   1/2 * m1 * (vc1_0).' * vc1_0 ...
    + 1/2 * (w01_0).' * I1 * (w01_0) ...
    + 1/2 * m2 * (vc2_0).' * (vc2_0) ...
    + 1/2 * (w12_0).' * I2 *  (w12_0);

P =   m1 * g * (z2(t) + lc1 * cos(q1(t))) ...
      + m2 * g * (z2(t) + l1 * cos(q1(t)) + lc2 * cos(q1(t) + q2(t)));

% P =   m1 * g * (lc1 * cos(q1(t))) ...
%       + m2 * g * (l1 * cos(q1(t)) + lc2 * cos(q1(t) + q2(t)));

L = K - P;
%==========================================================================
dL_q1dot = functionalDerivative(L,q1_dot(t));
dL1_dt = diff_t(dL_q1dot,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
dL_q1 = functionalDerivative(L,q1(t));
First_eq = dL1_dt - dL_q1;
%==========================================================================
dL_q2dot = functionalDerivative(L,q2_dot(t));
dL2_dt = diff_t(dL_q2dot,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
dL_q2 = functionalDerivative(L,q2(t));
Second_eq = dL2_dt - dL_q2;
%==========================================================================
dL_z1dot = functionalDerivative(L,z1_dot(t));
dL3_dt = diff_t(dL_z1dot,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
dL_z1 = functionalDerivative(L,z1(t));
Third_eq = dL3_dt - dL_z1;
%==========================================================================
dL_z2dot = functionalDerivative(L,z2_dot(t));
dL4_dt = diff_t(dL_z2dot,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
dL_z2 = functionalDerivative(L,z2(t));
Fourth_eq = dL4_dt - dL_z2;

%==========================================================================
%==========================================================================
First_eq = simplify(First_eq);
Second_eq = simplify(Second_eq);
Third_eq = simplify(Third_eq);
Fourth_eq = simplify(Fourth_eq);
%==========================================================================
DynamicEquations = [First_eq, Second_eq, Third_eq, Fourth_eq];          
%==========================================================================
zeroTerms_D = [q1_dot(t), q2_dot(t), g];
D = sym(zeros(dimGC));
for i = 1:dimGC
    for j = 1:dimGC
        D(i,j) = calcDynModMatrices(DynamicEquations(i), zeroTerms_D, Dd_GeneralizedCoordinates(j));
    end
end
%==========================================================================
zeroTerms_C = [q1_Ddot(t), q2_Ddot(t), g];
C = sym(zeros(dimGC));
for i = 1:dimGC
    for j = 1:dimGC
        C(i,j) = calcDynModMatrices(DynamicEquations(i), zeroTerms_C, d_GeneralizedCoordinates(j));
    end
end
%==========================================================================
zeroTerms_G = [q1_Ddot(t), q2_Ddot(t), q1_dot(t), q2_dot(t), z1_Ddot(t), z2_Ddot(t), z1_dot(t), z2_dot(t)];
G = sym(zeros(dimGC,1));
for i = 1:dimGC
        G(i) = calcDynModMatrices(DynamicEquations(i), zeroTerms_G);
end
%==========================================================================



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
D1(4,1) = -theta4 * cos(q1(t)) - theta5 * cos(q1(t) + q2(t));
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
C1(4,1) = theta4 * sin(q1(t)) * q1_dot(t) + theta5 * sin(q1(t) + q2(t)) * (q1_dot(t) + q2_dot(t));
C1(4,2) = theta5 * sin(q1(t) + q2(t)) * (q1_dot(t) + q2_dot(t));
C1(4,3) = 0;
C1(4,4) = 0;

G1(1,1) = -theta4 * g * sin(q1(t)) - theta5 * g * sin(q1(t) + q2(t));
G1(2,1) = -theta5 * g * sin(q1(t) + q2(t));
G1(3,1) = 0;
G1(4,1) = g * (m1 + m2);


%==========================================================================
c111 = 1/2 *(functionalDerivative(D(1,1),q1(t)));
c211 = 1/2 *(functionalDerivative(D(1,1),q2(t)));

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



D_dot = diff_t(D,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
% 
D1_dot = diff_t(D1,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
% 
N = (D_dot - 2*C);
N1 = (D1_dot - 2*C1);
% 
% something = ones(dimGC,1)'*(D_dot - 2*C)*ones(dimGC,1);
% something1 = ones(dimGC,1)'*(D1_dot - 2*C1)*ones(dimGC,1);
% %==========================================================================
% 
something2 = d_GeneralizedCoordinates*(D_dot - 2*C)*d_GeneralizedCoordinates.';
something3 = d_GeneralizedCoordinates*(D1_dot - 2*C1)*d_GeneralizedCoordinates.';