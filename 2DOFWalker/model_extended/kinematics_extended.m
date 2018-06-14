R1_0 = [sin(q1(t)) -cos(q1(t)) 0;
        cos(q1(t)) sin(q1(t))  0;
        0          0           1];

R2_1 = [ cos(q2(t)) sin(q2(t)) 0;
        -sin(q2(t)) cos(q2(t)) 0;
        0           0          1];
    
R2_0 = R1_0*R2_1;

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