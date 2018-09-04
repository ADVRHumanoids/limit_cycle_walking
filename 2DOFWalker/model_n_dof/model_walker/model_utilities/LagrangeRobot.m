function [eqMotion,dynamics] = LagrangeRobot(kinematics, generalizedVariables)
%% uses Lagrange to obtain the equation of motion of a PLANAR robot walker (without floating base)
% It requires:
% >> kinematics --> all the robot kinematics (in a struct)
% >> generalizedVariables --> symbolic variables of the robot
% >> robotData --> struct with robot parameters

    % author: Francesco Ruscelli
    % e-mail: francesco.ruscelli@iit.it

    robotData = getRobotData;

% if robotData.flagSym == 1
    K_terms = sym(zeros(robotData.n_link,1));
    P_terms = sym(zeros(robotData.n_link,1));
    pc = kinematics.linksCoMPosition;
    
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

%====================derivative of kinetic energy==========================
K_dot = diff_t(K,[q,q_dot], [q_dot, q_Ddot]);
%==========================================================================

    dynamics.mechanicalEnergy = simplify(mechanicalEnergy);
    dynamics.kineticEnergy = simplify(K);
    dynamics.potentialEnergy = simplify(P);
    dynamics.kineticEnergy_dot = simplify(K_dot);
    


%==========================================================================
    dL_q_dot = functionalDerivative(L,q_dot);
    dL_q_dot_dt = diff_t(dL_q_dot,[q,q_dot], [q_dot, q_Ddot]);
    dL_q = functionalDerivative(L,q);
    eqMotion = dL_q_dot_dt - dL_q;
    
    eqMotion = simplify(eqMotion);
%==========================================================================
    
end