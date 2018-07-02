function [Links,kinematics] = KinematicsLinks(q,parent_tree,robotData)

generalizedVariables = q;
kinematics = kinematics_n(parent_tree, generalizedVariables, robotData);

p = kinematics.linksPosition;
%==========================================================================
Base = q(end-1:end);

for i = 1:robotData.n_link
        Links(i,:,:) = [Base p(:,:,i)];
end

for i = 2:robotData.n_link
        Links(i,:,:) = [p(:,:,parent_tree(i)) p(:,:,i)];
end
%==========================================================================
end