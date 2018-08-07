%run robotTree

syms m1 m2 lc1 lc2 l1 l2 I1 I2 g 

theta1 = m1 * lc1^2 + m2 * l1^2 + I1;
theta2 = m2 * lc2^2 + I2;
theta3 = m2 * l1 * lc2;
theta4 = m1 * lc1 + m2 * l1;
theta5 = m2 * lc2;

% ===================
q = generalizedVariables.q;
q_dot = generalizedVariables.q_dot;
q_Ddot = generalizedVariables.q_Ddot;

n_link = 2;
q = generalizedVariables.q;
q_dot = generalizedVariables.q_dot;
q_Ddot = generalizedVariables.q_Ddot;
cyclic_variable = 1;
swing_leg = 2;
P = model.dynamics.PotentialEnergy;
K = model.dynamics.KineticEnergy;
Lagrangian = K - P;
E2 = model.dynamicMatrices.E2;
D = model.dynamicMatrices.D;
C = model.dynamicMatrices.C;
G = model.dynamicMatrices.G;
B = [0; 1];
E2_ext = model.dynamicMatricesExtended.E2;
D_ext = model.dynamicMatricesExtended.D;
v = 0;
%==========================================================================


dG_dq = functionalDerivative(G(1,:),q).';

 for i = 1:size(functionalDerivative(G(1,:),q))
     dG_dDq(:,i) = functionalDerivative(dG_dq(:,i),q);
 end
 

% ========partial linearization======(different change of variables)=======
term1 = - q_dot * dG_dDq * q_dot.';
term2 = dG_dq * inv(D) * B;
term3 = dG_dq * inv(D) *[- C*q_dot.' - G];

u = (v - term1 -term3)/ term2;

% ~~~~~~~~~~~~sigma_DDot~~~~~~~~~~~~~~
% w = diff_t(sigma_Ddot,[q,q_dot],[q_dot,q_Ddot]);
% w1 = term1 - dG_dq * q_Ddot.';
% shouldbeEqual = isAlways(w == w1);
% ~~~~~~~~~~~~~~~~~~~~~~~~~~

% w = term1 + term2*u + term3; 

%==============transformation from T to eta================================
integralq = q(2)/2 + ...
            (theta2 - theta1)/sqrt((theta1 + theta2)^2 - 4*theta3^4) ...
            *atan(sqrt((theta1 + theta2 - 2*theta3)/(theta1 + theta2 + 2*theta3)) * tan(q(2)/2));

p = q(1) + integralq;
sigma = simplify(functionalDerivative(Lagrangian, q_dot(cyclic_variable)));
sigma_dot = simplify(functionalDerivative(Lagrangian, q(cyclic_variable)));
sigma_Ddot = - dG_dq * q_dot.';

%==========================sigma===========================================
% sigma1 = simplify((theta1 + theta2 + 2*theta3*cos(q(2)))* q_dot(1) + (theta2 + theta3 * cos(q(2)))* q_dot(2));
%  eqAreEqual = isAlways(sigma == sigma1);
%=======================sigma_dot==========================================
% sigma_dot1 = simplify(functionalDerivative(Lagrangian, q(cyclic_variable)));
% sigma_dot3 = - simplify(functionalDerivative(P, q(cyclic_variable)));
% sigma_dot4 = simplify( model.kinematics.CoM_position(1)*(g*(m1 + m2)) );
% eqAreEqual1 = isAlways(sigma_dot1 == sigma_dot3);
% eqAreEqual3 = isAlways(sigma_dot3 == sigma_dot4);
% eqAreEqual4 = isAlways(sigma_dot1 == sigma_dot4);
%==========================================================================

xi1 = p;
xi2 = sigma;
xi3 = sigma_dot;
xi4 = sigma_Ddot;


Phi1 = [                                             1,   (theta2 + theta3 * cos(q(2)))/(theta1 + theta2 + 2* theta3 * cos(q(2)));
          theta4*g*cos(q(1)) + theta5*g*cos(q(1)+q(2)),                                                  theta5 * g* cos(q(1)+q(2))];
      
      
% Phi2 = [  theta1 + theta2 + 2* theta3 * cos(q(2)),  theta2 + theta3 * cos(q(2))
%          theta4*g*cos(q(1)) + theta5*g*cos(q(1)+q(2)),     theta5 * g* cos(q(1)+q(2))];
Phi2 = [                           D(1,1),                           D(1,2);
         -functionalDerivative(G(1),q(1)), -functionalDerivative(G(1),q(2))];
     
T = sym(zeros(4,1));
T(1) = p;
T(3) = theta4 * g * sin(q(1)) + theta5 * g * sin(q(1)+q(2));
tempT = Phi2 * q_dot.';
T(2) = tempT(1);
T(4) = tempT(2);


%================checks====================================================
T_1(2) = D(1,1)*q_dot(1) + D(1,2)*q_dot(2);
T_1(3) = -G(1);
T_1(4) = -functionalDerivative(G(1),q(1)) * q_dot(1) - functionalDerivative(G(1),q(2)) * q_dot(2);

if isAlways(T(2) ==  T_1(2)) && isAlways(T(3) ==  T_1(3)) && isAlways(T(4) ==  T_1(4))
    disp('ok');
else
    disp('something is wrong');
end
%==========================================================================
dG_dq = [ - g*lc2*m2*cos(q(1) + q(2)) - g*l1*m2*cos(q(1)) - g*lc1*m1*cos(q(1)), ...
                                                    -g*lc2*m2*cos(q(1) + q(2))];
                                                  
dG_dDq = [ g*lc2*m2*sin(q(1) + q(2)) + g*l1*m2*sin(q(1)) + g*lc1*m1*sin(q(1)), g*lc2*m2*sin(q(1) + q(2));
                                                   g *lc2*m2*sin(q(1) + q(2)), g*lc2*m2*sin(q(1) + q(2))];
                                               
                                               
                                               
xi4_dot = simplify(diff_t(T(4),[q q_dot], [q_dot q_Ddot]));                                       
                                               
xi4_dot1 = simplify(- q_dot * dG_dDq * q_dot.' ...
                    - dG_dq * q_Ddot.');

if isAlways(xi4_dot ==  xi4_dot1)
    disp('ok');
else
    disp('something is wrong');
end

term1 = - q_dot * dG_dDq * q_dot.';
term2 = simplify(dG_dq * inv(D) * B);
term3 = dG_dq * inv(D) *(- C*q_dot.' - G);

u = (v - term1 - term3)/ term2;
u1 = (q_dot * dG_dDq * q_dot.' + dG_dq * inv(D) * (C*q_dot.' + G)) / (dG_dq * inv(D) * B);

if isAlways(u ==  u1)
    disp('ok');
else
    disp('something is wrong');
end


%>> xi4_dot1 = alfa* tau2 + beta;
% alfa = simplify(det(Phi2)/det(D));
% 
% 
% if isAlways(alfa ==  term2) || logical(simplify(alfa -  term2) == 0)
%     disp('ok');
% else
%     disp('something is wrong');
% end


% D1(1,1) = theta1 + theta2 + theta3 * 2 * cos(q(2));
% D1(1,2) = theta2 + theta3 * cos(q(2));
% D1(2,1) = theta2 + theta3 * cos(q(2));
% D1(2,2) = theta2;
% C1(1,1) = -theta3 * sin(q(2)) * q_dot(2);
% C1(1,2) = -(q_dot(1) + q_dot(2)) * theta3 * sin(q(2));
% C1(2,1) =  theta3 * sin(q(2)) * q_dot(1);
% C1(2,2) = 0;
% G1(1,1) = -theta4 * g * sin(q(1)) - theta5 * g * sin(q(1) + q(2));
% G1(2,1) = -theta5 * g * sin(q(1) + q(2));