clear all; clc
%=======dynamic model of a 2 link walker (grizzle book notation)===========
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

% GeneralizedCoordinates = [q1(t),q2(t),z1(t),z2(t)];
GeneralizedCoordinates = [q1(t),q2(t)];
% first_derivative_names = [q1_dot(t), q2_dot(t), z1_dot(t), z2_dot(t)];
first_derivative_names = [q1_dot(t), q2_dot(t)];
% second_derivative_names = [q1_Ddot(t), q2_Ddot(t), z1_Ddot(t), z2_Ddot(t)];
second_derivative_names = [q1_Ddot(t), q2_Ddot(t)];
dimGC = length(GeneralizedCoordinates);

d_GeneralizedCoordinates = diff_t(GeneralizedCoordinates, GeneralizedCoordinates, first_derivative_names);
Dd_GeneralizedCoordinates = diff_t(d_GeneralizedCoordinates, d_GeneralizedCoordinates, second_derivative_names);  

kinematics_grizzle;

K =   1/2 * m2 * (vc2_0).' * vc2_0 ...
    + 1/2 * (w02_0).' * I2 * (w02_0) ...
    + 1/2 * m1 * (vc21_0).' * (vc21_0) ...
    + 1/2 * (w12_0).' * I1 *  (w12_0);

% P =   m1 * g * (z2(t) + lc1 * cos(q1(t))) ...
%       + m2 * g * (z2(t) + l1 * cos(q1(t)) + lc2 * cos(q1(t) + q2(t)));

P =   m2 * g * (lc2 * sin(q2(t))) ...
      + m1 * g * (l2 * sin(q2(t)) + lc1 * sin(q1(t) + q2(t)));

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
% dL_z1dot = functionalDerivative(L,z1_dot(t));
% dL3_dt = diff_t(dL_z1dot,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
% dL_z1 = functionalDerivative(L,z1(t));
% Third_eq = dL3_dt - dL_z1;
%==========================================================================
% dL_z2dot = functionalDerivative(L,z2_dot(t));
% dL4_dt = diff_t(dL_z2dot,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
% dL_z2 = functionalDerivative(L,z2(t));
% Fourth_eq = dL4_dt - dL_z2;

%==========================================================================
First_eq = simplify(First_eq);
Second_eq = simplify(Second_eq);
% Third_eq = simplify(Third_eq);
% Fourth_eq = simplify(Fourth_eq);
%==========================================================================
% DynamicEquations = [First_eq, Second_eq, Third_eq, Fourth_eq];
DynamicEquations = [First_eq, Second_eq];    
%=============================getting D matrix=============================
zeroTerms_D = [q1_dot(t), q2_dot(t), g];
D = sym(zeros(dimGC));
for i = 1:dimGC
    for j = 1:dimGC
        D(i,j) = calcDynModMatrices(DynamicEquations(i), zeroTerms_D, Dd_GeneralizedCoordinates(j));
    end
end
%=============================getting C matrix=============================
% zeroTerms_C = [q1_Ddot(t), q2_Ddot(t), g];
% C = sym(zeros(dimGC));
% for i = 1:dimGC
%     for j = 1:dimGC
%         C(i,j) = calcDynModMatrices(DynamicEquations(i), zeroTerms_C, d_GeneralizedCoordinates(j));
%     end
% end
%=============================getting G matrix=============================================
zeroTerms_G = [q1_Ddot(t), q2_Ddot(t), q1_dot(t), q2_dot(t), z1_Ddot(t), z2_Ddot(t), z1_dot(t), z2_dot(t)];
G = sym(zeros(dimGC,1));
for i = 1:dimGC
        G(i) = calcDynModMatrices(DynamicEquations(i), zeroTerms_G);
end
%=============================getting C matrix with Christoffel============
for i = 1:dimGC
    for j = 1:dimGC
        for k = 1:dimGC
            c(i,j,k) = 1/2 * (functionalDerivative(D(k,j), GeneralizedCoordinates(i)) + ...
                       functionalDerivative(D(k,i), GeneralizedCoordinates(j)) - ...
                       functionalDerivative(D(i,j), GeneralizedCoordinates(k)));
        end
    end
end

C = sym(zeros(dimGC));

for j = 1:dimGC
    for k = 1:dimGC
        for i = 1:dimGC
            C(k,j) = C(k,j) + c(i,j,k)*d_GeneralizedCoordinates(i).';
        end
    end
end



D_dot = diff_t(D,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);

N = (D_dot - 2*C);
ThisShouldbeZero = simplify(d_GeneralizedCoordinates*N*d_GeneralizedCoordinates.');
%==========================================================================
%==========================================================================
% phi = [l1 * sin(q1(t)) + l2 * sin(q1(t) + q2(t));
%        l1 * cos(q1(t)) + l2 * cos(q1(t) + q2(t))];
%   
% p2 = [z1(t) + l1 * sin(q1(t)) + l2 * sin(q1(t) + q2(t));
%        z2(t) + l1 * cos(q1(t)) + l2 * cos(q1(t) + q2(t))]; %tip of the swinging leg in [q, z]
% 
% E2 = [l1 * cos(q1(t)) + l2 * cos(q1(t) + q2(t)), l2 * cos(q1(t) + q2(t)) 1 0;
%       - l1 * sin(q1(t)) - l2 * sin(q1(t) + q2(t)), - l2 * sin(q1(t) + q2(t)) 0 1]; %derivative of p2 w.r.t. generalized coordinates [q, z]
%   
% % m1 = [D, -E2.';
% %       E2, zeros(2)];
%   
% deltaF2 = -inv(E2 * inv(D) * E2.') * E2 * [eye(N); ];

