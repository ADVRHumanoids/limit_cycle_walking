function [D,C,G,kinematics,mechanicalEnergy] = createModelWalker(parent_tree,generalizedVariables,robotData)

kinematics = kinematics_n(parent_tree,generalizedVariables,robotData);
[eqMotion,mechanicalEnergy] = Lagrange_n(kinematics,generalizedVariables,robotData);
[D,C,G] = dynamics_n(eqMotion,generalizedVariables,robotData);

end