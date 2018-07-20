%% =====================zero dynamics (PART I)============================
% (to run this script you need a non-extended dynamic model of the robot, which can be
% created by using %model_n

if ~exist('model','var')
robotTree %with flagSimulation set to 0
end

q = generalizedVariables.q;
q_dot = generalizedVariables.q_dot;
q_Ddot = generalizedVariables.q_Ddot;
dynamics = model.dynamics;
D = model.dynamicMatrices.D;
C = model.dynamicMatrices.C;
G = model.dynamicMatrices.G;
% Script divided into 2 parts (part I and part II)
%This is a way to get the Zero Dynamics of a robot by setting h, the
%output, and theta, a suitable function such that [h theta(q)] is a
%diffeomorphism onto its image

% as you can see, the Zero Dynamics gotten in this way, with h = q1, are 
% exactly the same as the ones got in simulation_n (part two)

ksi = sym(zeros(2,1));
kappa = sym(zeros(2,1));
ksi_dot =  sym(zeros(2,1));

cyclic_variable = 1;
% h = 0;
% Lfh = 0;
theta = q(1);
h = q(2); % - (pi - q(1) - q(2))

ksi(1) = theta;
ksi(2) = functionalDerivative(dynamics.KineticEnergy, q_dot(cyclic_variable));

kappa(1) = functionalDerivative(theta,q).' * inv([functionalDerivative(h,q).';D(cyclic_variable,:)]) * [0;1];
kappa(2) = - functionalDerivative(dynamics.PotentialEnergy, q(cyclic_variable)).';


kappa = simplify(kappa);
ksi = simplify(ksi);

zeroElements = [q(2), q_dot(2)];
dim = length(zeroElements);
ksi_Z = simplify(subs(ksi, zeroElements, zeros(1,dim)));
kappa_Z = simplify(subs(kappa, zeroElements, zeros(1,dim)));


ksi_Z_dot(1) = kappa_Z(1) * ksi_Z(2);
ksi_Z_dot(2) = kappa_Z(2);

q1_Ddot_ZeroDyn =  simplify((kappa_Z(1) * kappa_Z(2)));
%% =====================zero dynamics (PART II)============================
% 
%zero dynamics from the simulation_n, given the input control to zero the
%output. u = inv(LgLfh) * (L2fh);
%Zero dynamics of the system is q1_Ddot_ZeroDyn1. q2 was chosen as the
%output, since I can fully control the variable.
g1 = G(1);
g2 = G(2);

c11 = C(1,1);
c12 = C(1,2);
c21 = C(2,1);
c22 = C(2,2);

d11 = D(1,1);
d12 = D(1,2);
d21 = D(2,1);
d22 = D(2,2);

%zero dynamics got from the simulation (q_Ddot)
q1_Ddot_ZeroDyn1 = simplify(-(g1 + c11*q_dot(1) + c12*q_dot(2))/d11);
 
zeroElements = [q(2), q_dot(2)];
dim = length(zeroElements);
q1_Ddot_ZeroDyn1 = simplify(subs(q1_Ddot_ZeroDyn1, zeroElements, zeros(1,dim)));

%==========================================================================
% eqAreEqual = isequal(q1_Ddot_ZeroDyn,q1_Ddot_ZeroDyn1); %shitty
% eqAreEqual1 = isAlways(q1_Ddot_ZeroDyn == q1_Ddot_ZeroDyn1); %good
% eqAreEqual2 = logical(simplify(q1_Ddot_ZeroDyn-q1_Ddot_ZeroDyn1) == 0); %good

