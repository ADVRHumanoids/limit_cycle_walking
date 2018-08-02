function [D,C,G] = dynamics_n(eqMotion, generalizedVariables)
%% generate the dynamic model of the PLANAR robot walker (with floating base, extended model)
% It requires:
% >> eqMotion --> equation of motion of the model of the robot walker
% >> generalizedVariables --> symbolic variables of the robot + floating base
% >> robotData --> struct with robot parameters

    % author: Francesco Ruscelli
    % e-mail: francesco.ruscelli@iit.it
    robotData = getRobotData;
    g = robotData.gravity;

    qe = generalizedVariables.qe;
    qe_dot = generalizedVariables.qe_dot;
    qe_Ddot = generalizedVariables.qe_Ddot;
    dim_qe = size(qe,2);

    zeroTerms_D = [qe_dot, g];
    D = sym(zeros(dim_qe));
    for i = 1:dim_qe
        for j = 1:dim_qe
            D(i,j) = calcDynModMatrices(eqMotion(i), zeroTerms_D, qe_Ddot(j));
        end
    end

    %=============================getting G matrix=============================================
    zeroTerms_G = [qe_Ddot, qe_dot];
    G = sym(zeros(dim_qe,1));
    for i = 1:dim_qe
            G(i) = calcDynModMatrices(eqMotion(i), zeroTerms_G);
    end
    %=============================getting C matrix with Christoffel============
    for i = 1:dim_qe
        for j = 1:dim_qe
            for k = 1:dim_qe
                c(i,j,k) = 1/2 * (functionalDerivative(D(k,j), qe(i)) + ...
                                  functionalDerivative(D(k,i), qe(j)) - ...
                                  functionalDerivative(D(i,j), qe(k)));
            end
        end
    end

    C = sym(zeros(dim_qe));

    for j = 1:dim_qe
        for k = 1:dim_qe
            for i = 1:dim_qe
                C(k,j) = C(k,j) + c(i,j,k)*qe_dot(i).';
            end
        end
    end

end
