% R2_0 = [cos(3*pi/2 - q2(t)) -sin(3*pi/2 - q2(t)) 0;     % [ -sin(q2(t)),  cos(q2(t)), 0]             
%         sin(3*pi/2 - q2(t)) cos(3*pi/2 - q2(t))  0;     % [ -cos(q2(t)), -sin(q2(t)), 0]
%         0                   0                    1];    % [           0,           0, 1]

R2_0 = [cos(q2(t)) -sin(q2(t))  0;     %[ -cos(q1(t)),  -sin(q1(t)), 0]
        sin(q2(t))  cos(q2(t))  0;     %[  sin(q1(t)),  -cos(q1(t)), 0]
        0                0      1];
    
% R1_2 = [cos(pi - q1(t)) -sin(pi - q1(t))  0;     %[ -cos(q1(t)),  -sin(q1(t)), 0]
%         sin(pi - q1(t))  cos(pi - q1(t))  0;     %[  sin(q1(t)),  -cos(q1(t)), 0]
%         0                0                1];    %[0              0            1];

R1_2 = [-cos(q1(t))  sin(q1(t))  0;
         sin(q1(t))  cos(q1(t))  0;
        0            0           1]; 
    
R1_0 = R2_0*R1_2;

w02_0 = [0; 0; q2_dot(t)];
w12_1 = [0; 0; q1_dot(t)];

w12_0 = w02_0 + R1_0 * w12_1; %[0 0 q1_dot+q2_dot];

rp1_1 = [l1; 0; 0];
rc1_1 = [lc1; 0; 0];

rp2_2 = [l2; 0; 0];
rc2_2 = [lc2; 0; 0];

rp1_0 = R1_0 * rp1_1;
rc1_0 = R1_0 * rc1_1;

rp2_0 = R2_0 * rp2_2;
rc2_0 = R2_0 * rc2_2;

v2_0 = [z1_dot(t); z2_dot(t); 0];
% v2_0 = [0; 0; 0];
v1_0 = v2_0 + cross(w02_0, rp2_0); %TODO

vc2_0 = v2_0 + cross(w02_0,rc2_0);
vc21_0 = v1_0 + cross(w12_0,rc1_0);