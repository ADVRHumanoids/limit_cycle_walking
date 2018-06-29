function Links = KinematicsLinks(q,parent_tree,robotData,Base)

generalizedVariables = q;
kinematics = kinematics_n(parent_tree, generalizedVariables, robotData);

rp_abs = kinematics.positionLink_absolute;
%==========================================================================
for j = 1:2
    Links(1,j,:) = Base(j) + [0 rp_abs(j,:,1)]; %link,axis,begin/end
end

for i = 2:robotData.n_link
    for j = 1:2
Links(i,j,:) = Links(parent_tree(i),j,2) + [0 rp_abs(j,:,i)]; %x of link 1 begin/end
    end
end
%==========================================================================
end