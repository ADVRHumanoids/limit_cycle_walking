R_rel = sym(zeros(3,3,length(parent_tree)));

R_rel(:,:,1) = [sin(q(1)) -cos(q(1)) 0;
                cos(q(1)) sin(q(1))  0;
                0          0         1];
           
for i = 2:length(parent_tree)  
    
R_rel(:,:,i) = [ cos(q(i))  sin(q(i))  0;
                -sin(q(i))  cos(q(i))  0;
                0           0          1];  
end


R_abs = sym(zeros(3,3,length(parent_tree)));
R_abs(:,:,1) = R_rel(:,:,1); %from parent origin to last child
for i = 2:length(parent_tree)
R_abs(:,:,i) = R_rel(:,:,i) * R_abs(:,:,parent_tree(i));
end

w_rel = sym(zeros(3,1,length(parent_tree)));

for i = 1:length(parent_tree)
    w_rel(:,:,i) = [0; 0; q_dot(i)];
end

w_abs = sym(zeros(3,1,length(parent_tree)));
w_abs(:,:,1) = w_rel(:,:,1);
for i = 2:length(parent_tree)
    w_abs(:,:,i) = w_abs(:,:,(parent_tree(i))) + R_abs(:,:,i) * w_rel(:,:,i); %MAYBEERROR
end


rp_abs = sym(zeros(3,1,length(parent_tree)));
for i = 1:length(parent_tree)
rp_abs(:,:,i) = R_abs(:,:,i) * rp_rel(:,i);
end

rc_abs = sym(zeros(3,1,length(parent_tree)));
for i = 1:length(parent_tree)
rc_abs(:,:,i) = R_abs(:,:,i) * rc_rel(:,i);
end


%base~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
v_abs(:,:,1) = [z1_dot(t); z2_dot(t); 0];
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
for i = 2:length(parent_tree)
v_abs(:,:,i) = v_abs(:,:,parent_tree(i)) + cross(w_abs(:,:,i-1), rp_abs(:,:,i-1));
end


%velocities
for i = 1:length(parent_tree)
vc_abs(:,:,i) = v_abs(:,:,i) + cross(w_abs(:,:,i),rc_abs(:,:,i));
end

