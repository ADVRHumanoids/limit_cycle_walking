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
% R2_0 = R1_0*R2_1;

end

% w01_0 = [0; 0; q1_dot(t)];
% w12_1 = [0; 0; q2_dot(t)];
w_rel = sym(zeros(3,1,length(parent_tree)));

for i = 1:length(parent_tree)
    w_rel(:,:,i) = [0; 0; q_dot(i)];
end

w_abs = sym(zeros(3,1,length(parent_tree)));
w_abs(:,:,1) = w_rel(:,:,1);
for i = 2:length(parent_tree)
    w_abs(:,:,i) = w_abs(:,:,(parent_tree(i))) + R_abs(:,:,i) * w_rel(:,:,i); %MAYBEERROR
end
% w12_0 = w01_0 + R1_0 * w12_1; %[0 0 q1_dot+q2_dot];



% rp1_1 = [l1; 0; 0];
% rc1_1 = [lc1; 0; 0];
% 
% rp2_2 = [l2; 0; 0];
% rc2_2 = [lc2; 0; 0];
rp_abs = sym(zeros(3,1,length(parent_tree)));
for i = 1:length(parent_tree)
rp_abs(:,:,i) = R_abs(:,:,i) * rp_rel(:,i);
end

rc_abs = sym(zeros(3,1,length(parent_tree)));
for i = 1:length(parent_tree)
rc_abs(:,:,i) = R_abs(:,:,i) * rc_rel(:,i);
end

% rp1_0 = R1_0 * rp1_1;
% rc1_0 = R1_0 * rc1_1;
% 
% rp2_0 = R2_0 * rp2_2;
% rc2_0 = R2_0 * rc2_2;

%base
v_abs(:,:,1) = [z1_dot(t); z2_dot(t); 0];
for i = 2:length(parent_tree)
v_abs(:,:,i) = v_abs(:,:,parent_tree(i)) + cross(w_abs(:,:,i-1), rp_abs(:,:,i-1));
end

% v1_0 = [z1_dot(t); z2_dot(t); 0];
% v2_0 = v1_0 + cross(w01_0, rp1_0); %TODO
%velocities
for i = 1:length(parent_tree)
vc_abs(:,:,i) = v_abs(:,:,i) + cross(w_abs(:,:,i),rc_abs(:,:,i));
end

% vc1_0 = v1_0 + cross(w01_0,rc1_0);
% vc2_0 = v2_0 + cross(w12_0,rc2_0);