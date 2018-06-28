function [D,C,G,kinematics] = createModelWalker(parent_tree,generalizedVariables,robotData)

kinematics = kinematics_n(parent_tree,generalizedVariables,robotData);
eqMotion = Lagrange_n(kinematics,generalizedVariables,robotData);
[D,C,G] = dynamics_n(eqMotion,generalizedVariables,robotData);

end