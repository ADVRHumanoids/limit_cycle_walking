function Links = simKinematics_n(q,parent_tree,robotData,Base)

n_link = length(q)-2;

rp_rel = zeros(3,length(parent_tree));
for i = 1:robotData.n_link
rp_rel(1,i) = robotData.link_length(i);
end
    
    
R_rel = zeros(3,3,length(parent_tree));
R_rel(:,:,1) = [sin(q(1)) -cos(q(1)) 0;
                cos(q(1)) sin(q(1))  0;
                0          0         1];
           
for i = 2:length(parent_tree)  
    
R_rel(:,:,i) = [ cos(q(i))  sin(q(i))  0;
                -sin(q(i))  cos(q(i))  0;
                0           0          1];  
end

R_abs = zeros(3,3,length(parent_tree));
R_abs(:,:,1) = R_rel(:,:,1); %from parent origin to last child
for i = 2:length(parent_tree)
R_abs(:,:,i) = R_rel(:,:,i) * R_abs(:,:,parent_tree(i));
end

rp_abs = zeros(3,1,length(parent_tree));
for i = 1:length(parent_tree)
rp_abs(:,:,i) = R_abs(:,:,i) * rp_rel(:,i);
end



for j = 1:2
    Links(1,j,:) = Base(j) + [0 rp_abs(j,:,1)]; %link,axis,begin/end
end

for i = 2:n_link
    for j = 1:2
Links(i,j,:) = Links(parent_tree(i),j,2) + [0 rp_abs(j,:,i)]; %x of link 1 begin/end
    end
end

end