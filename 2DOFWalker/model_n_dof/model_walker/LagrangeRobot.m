function [eqMotion,dynamics] = LagrangeRobot(kinematics, generalizedVariables, robotData)



% if robotData.flagSym == 1
    K_terms = sym(zeros(robotData.n_link,1));
    P_terms = sym(zeros(robotData.n_link,1));
    pc = kinematics.CoMPosition;
    
% else
    
%     K_terms = zeros(robotData.n_link,1);
%     P_terms = zeros(robotData.n_link,1);
%     rp_abs_fromOrigin = zeros(3,1,robotData.n_link);

% end

g = robotData.gravity;

q =  generalizedVariables.q;
q_dot =  generalizedVariables.q_dot;
q_Ddot =  generalizedVariables.q_Ddot;

parent_tree = kinematics.parent_tree;
w_abs = kinematics.angularVelocity_absolute;
vc_abs = kinematics.velocityCoM_absolute;

m = robotData.mass;
I = robotData.inertia;



for i = 1:length(parent_tree)
    
    K_terms(i) =   1/2 * m(i) * vc_abs(:,:,i).'*vc_abs(:,:,i) ...
                 + 1/2 * w_abs(:,:,i).' * I(i) * w_abs(:,:,i);
end

K = sum(K_terms);



for i = 1:length(parent_tree)
P_terms(i,:) = m(i) * g * pc(2,:,i);
end

P = sum(P_terms);


L = K - P;
mechanicalEnergy = K + P;

dynamics.mechanicalEnergy = mechanicalEnergy;
dynamics.KineticEnergy = K;
dynamics.PotentialEnergy = P;
%==========================================================================
dL_q_dot = functionalDerivative(L,q_dot);
dL_q_dot_dt = diff_t(dL_q_dot,[q,q_dot], [q_dot, q_Ddot]);
dL_q = functionalDerivative(L,q);
eqMotion = dL_q_dot_dt - dL_q;
%==========================================================================
end