function [D,C,G,kinematics,dynamics] = createModelWalker(generalizedVariables)
%% generates the model of a PLANAR n-link model (without floating base)
% It requires:
% >> parent_tree --> an array describing how the links are connected
% >> generalizedVariables --> symbolic variables of the robot
% >> robotData --> struct with robot parameters

    % author: Francesco Ruscelli
    % e-mail: francesco.ruscelli@iit.it
    kinematics = kinematicsRobot(generalizedVariables);
    [eqMotion,dynamics] = LagrangeRobot(kinematics,generalizedVariables);
    [D,C,G] = dynamicsRobot(eqMotion,generalizedVariables);

end