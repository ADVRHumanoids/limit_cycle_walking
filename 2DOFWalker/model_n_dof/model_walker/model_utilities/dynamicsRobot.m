function [D,C,G] = dynamicsRobot(eqMotion, generalizedVariables)
%% generate the dynamic model of the PLANAR robot walker (without floating base)
% It requires:
% >> eqMotion --> equation of motion of the model of the robot walker
% >> generalizedVariables --> symbolic variables of the robot
% >> robotData --> struct with robot parameters

    % author: Francesco Ruscelli
    % e-mail: francesco.ruscelli@iit.it
    
    robotData = getRobotData;
    g = robotData.gravity;

    q = generalizedVariables.q;
    q_dot = generalizedVariables.q_dot;
    q_Ddot = generalizedVariables.q_Ddot;
    dim_q = size(q,2);

    zeroTerms_D = [q_dot, g];
    D = sym(zeros(dim_q));
    for i = 1:dim_q
        for j = 1:dim_q
            D(i,j) = calcDynModMatrices(eqMotion(i), zeroTerms_D, q_Ddot(j));
        end
    end

    %=============================getting G matrix=============================================
    zeroTerms_G = [q_Ddot, q_dot];
    G = sym(zeros(dim_q,1));
    for i = 1:dim_q
            G(i) = calcDynModMatrices(eqMotion(i), zeroTerms_G);
    end
    %=============================getting C matrix with Christoffel============
    for i = 1:dim_q
        for j = 1:dim_q
            for k = 1:dim_q
                c(i,j,k) = 1/2 * (functionalDerivative(D(k,j), q(i)) + ...
                                  functionalDerivative(D(k,i), q(j)) - ...
                                  functionalDerivative(D(i,j), q(k)));
            end
        end
    end

    C = sym(zeros(dim_q));

    for j = 1:dim_q
        for k = 1:dim_q
            for i = 1:dim_q
                C(k,j) = C(k,j) + c(i,j,k)*q_dot(i).';
            end
        end
    end


    D = simplify(D);
    C = simplify(C);
    G = simplify(G);
    
end
