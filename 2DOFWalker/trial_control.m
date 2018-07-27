syms m1 m2 lc1 lc2 l1 l2 I1 I2 g 

theta1 = m1 * lc1^2 + m2 * l1^2 + I1;
theta2 = m2 * lc2^2 + I2;
theta3 = m2 * l1 * lc2;
theta4 = m1 * lc1 + m2 * l1;
theta5 = m2 * lc2;
% 
n_link = 2;
q = generalizedVariables.q;
q_dot = generalizedVariables.q_dot;
q_Ddot = generalizedVariables.q_Ddot;
cyclic_variable = 1;
swing_leg = 2;
P = model.dynamics.PotentialEnergy;
K = model.dynamics.KineticEnergy;
Lagrangian = K - P;
sigma = simplify(functionalDerivative(Lagrangian, q_dot(cyclic_variable)));
E2 = model.dynamicMatrices.E2;
D = model.dynamicMatrices.D;
C = model.dynamicMatrices.C;
G = model.dynamicMatrices.G;

E2_ext = model.dynamicMatricesExtended.E2;
D_ext = model.dynamicMatricesExtended.D;

MatrixRelabel = [1     1
                 0    -1];
             
             
D1(1,1) = theta1 + theta2 + theta3 * 2 * cos(q(2));
D1(1,2) = theta2 + theta3 * cos(q(2));
D1(2,1) = theta2 + theta3 * cos(q(2));
D1(2,2) = theta2;
C1(1,1) = -theta3 * sin(q(2)) * q_dot(2);
C1(1,2) = -(q_dot(1) + q_dot(2)) * theta3 * sin(q(2));
C1(2,1) =  theta3 * sin(q(2)) * q_dot(1);
C1(2,2) = 0;
G1(1,1) = -theta4 * g * sin(q(1)) - theta5 * g * sin(q(1) + q(2));
G1(2,1) = -theta5 * g * sin(q(1) + q(2));



% sigma1 = simplify((theta1 + theta2 + 2*theta3*cos(q(2)))* q_dot(1) + (theta2 + theta3 * cos(q(2)))* q_dot(2));
% 
% eqAreEqual = isAlways(sigma == sigma1);
%==========================================================================
sigma_dot1 = simplify(functionalDerivative(Lagrangian, q(cyclic_variable)));
% sigma_dot2 =  - theta4 * g * sin(q(1)) - theta5 * g * sin(q(1) + q(2)); %?????????????
sigma_dot3 = - simplify(functionalDerivative(P, q(cyclic_variable)));
sigma_dot4 = simplify( model.kinematics.CoM_position(1)*(g*(m1 + m2)) );

eqAreEqual1 = isAlways(sigma_dot1 == sigma_dot3);
% eqAreEqual2 = isAlways(sigma_dot2 == sigma_dot3);
eqAreEqual3 = isAlways(sigma_dot3 == sigma_dot4);
eqAreEqual4 = isAlways(sigma_dot1 == sigma_dot4);

if eqAreEqual1 && eqAreEqual3 && eqAreEqual4

    sigma_dot = sigma_dot1;
    disp('ok');
else
    disp('nope!');
end






integralq = q(2)/2 + ...
            (theta2 - theta1)/sqrt((theta1 + theta2)^2 - 4*theta3^4) ...
            *atan(sqrt((theta1 + theta2 - 2*theta3)/(theta1 + theta2 + 2*theta3)) * tan(q(2)/2));

p = q(1) + integralq;


p_dot = inv(D(1,1)) * sigma;



Phi1 = [                                       1,   (theta2 + theta3 * cos(q(2)))/(theta1 + theta2 + 2* theta3 * cos(q(2)));
          theta4*g*cos(q(1)) + theta5*g*cos(q(1)+q(2)),                                                  theta5 * g* cos(q(1)+q(2))];
      
      
Phi2 = [  theta1 + theta2 + 2* theta3 * cos(q(2)),  theta2 + theta3 * cos(q(2))
         theta4*g*cos(q(1)) + theta5*g*cos(q(1)+q(2)),     theta5 * g* cos(q(1)+q(2))];
     
deltaF2 = -inv(E2_ext * inv(D_ext) * E2_ext.') * E2_ext * [eye(n_link); zeros(2,n_link)];
deltaqDotBar = inv(D_ext) * E2_ext.' * deltaF2 + [eye(n_link); zeros(2,n_link)];

MatrixImpact = [MatrixRelabel zeros(n_link,2)]* deltaqDotBar;

D = 0.3;
T = 1;
epsilon1_0 = p; %with q(1) = 0; q(2) = 0;
epsilon2_0 = 1; %!!! %chosen so that at t=T impact
epsilon3_0 = G(1); %%with q(1) = 0; q(2) = 0;

matrixA = [-T^3/12, -T^4/24;
               T/6,   T^2/3];
           
matrixB = [  0,     0;
           T/2, T^2/6];
       
a1 = inv(Phi2 * MatrixImpact * inv(Phi2)* matrixA + matrixB);

matrixC = [zeros(1,length(D*g*2*m/T));
                            D*g*2*m/T];
       
matrixD = [epsilon3_0*T + D*g*m*T;
                      D*g*2*m/T];
                  
a2 = matrixC - Phi2 * MatrixImpact * inv(Phi2) * matrixD + [eye(2) - Phi2 * MatrixImpact * inv(Phi2)] * [epsilon2_0; 0];
                                                                                                            
controller = a1 * a2;                                                                                                            
                                                                                                            
epsilon4_0 = (D * g * 2 * m - a* T^2/2 - b* T^3/6)* inv(T);


w_ref = a + b*t; %input of the partial feedback linearization