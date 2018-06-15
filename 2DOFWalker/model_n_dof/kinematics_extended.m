parent_tree = [0,1,2,2,4];


for i = 1:length(parent_tree)   
R_rel(:,:,i) = [sin(q(i)) -cos(q(i)) 0;
            cos(q(i)) sin(q(i))  0;
            0          0         1];  
end

R_abs(1) = R_rel(1);
for i = 2:length(parent_tree)
R_abs(i) = R_rel(i) * R_abs(parentTree(i));
% R2_0 = R1_0*R2_1;

end
w01_0 = [0; 0; q1_dot(t)];
w12_1 = [0; 0; q2_dot(t)];

w12_0 = w01_0 + R1_0 * w12_1; %[0 0 q1_dot+q2_dot];

rp1_1 = [l1; 0; 0];
rc1_1 = [lc1; 0; 0];

rp2_2 = [l2; 0; 0];
rc2_2 = [lc2; 0; 0];

rp1_0 = R1_0 * rp1_1;
rc1_0 = R1_0 * rc1_1;

rp2_0 = R2_0 * rp2_2;
rc2_0 = R2_0 * rc2_2;

v1_0 = [z1_dot(t); z2_dot(t); 0];
% v1_0 = [0; 0; 0];
v2_0 = v1_0 + cross(w01_0, rp1_0); %TODO

vc1_0 = v1_0 + cross(w01_0,rc1_0);
vc2_0 = v2_0 + cross(w12_0,rc2_0);