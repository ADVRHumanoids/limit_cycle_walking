function [D,C,G,kinematics,dynamics] = createModelExtendedWalker(parent_tree,generalizedVariables,robotData)
%% generates the EXTENDED model of a PLANAR n-link model (with floating base)
% It requires:
% >> parent_tree --> an array describing how the links are connected
% >> generalizedVariablesExtended --> symbolic variables of the robot + floating base
% >> robotData --> struct with robot parameters

    % author: Francesco Ruscelli
    % e-mail: francesco.ruscelli@iit.it
    
kinematics = kinematics_n(parent_tree,generalizedVariables,robotData);
[eqMotion,dynamics] = Lagrange_n(kinematics,generalizedVariables,robotData);
[D,C,G] = dynamics_n(eqMotion,generalizedVariables,robotData);

end