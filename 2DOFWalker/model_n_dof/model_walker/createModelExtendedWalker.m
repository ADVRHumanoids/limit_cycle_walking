function [D,C,G,kinematics,dynamics] = createModelExtendedWalker(parent_tree,generalizedVariables,robotData)

kinematics = kinematics_n(parent_tree,generalizedVariables,robotData);
[eqMotion,dynamics] = Lagrange_n(kinematics,generalizedVariables,robotData);
[D,C,G] = dynamics_n(eqMotion,generalizedVariables,robotData);

end