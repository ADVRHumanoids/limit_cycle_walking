function [D,C,G,kinematics,dynamics] = createModelWalker(parent_tree,generalizedVariables,robotData)

kinematics = kinematicsRobot(parent_tree,generalizedVariables,robotData);
[eqMotion,dynamics] = LagrangeRobot(kinematics,generalizedVariables,robotData);
[D,C,G] = dynamicsRobot(eqMotion,generalizedVariables,robotData);

end