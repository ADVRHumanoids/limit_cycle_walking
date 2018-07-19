Number of links: 
Length of links: 
Length of links: 5.000000e-01
Position of CoM: 8.000000e-01
Position of CoM: 8.000000e-01
Position of CoM: 2.500000e-01
Mass of links: 3.000000e-01
Mass of links: 3.000000e-01
Mass of links: 
Inertia of links: 3.000000e-02
Inertia of links: 3.000000e-02
Inertia of links: 1.000000e-01
Gravity: 9.810000e+00
D = [ 
(12*cos(q(2)))/25 + 93/125, (6*cos(q(2)))/25 + 111/500, - (27*cos(q(1)))/50 - (6*cos(q(1) + q(2)))/25, (27*sin(q(1)))/50 + (6*sin(q(1) + q(2)))/25;
(6*cos(q(2)))/25 + 111/500, 111/500, -(6*cos(q(1) + q(2)))/25, (6*sin(q(1) + q(2)))/25;
- (27*cos(q(1)))/50 - (6*cos(q(1) + q(2)))/25, -(6*cos(q(1) + q(2)))/25, 3/5, 0;
(27*sin(q(1)))/50 + (6*sin(q(1) + q(2)))/25, (6*sin(q(1) + q(2)))/25, 0, 3/5;
];

C = [ 
-(6*sin(q(2))*q2_dot(t))/25, -(6*sin(q(2))*(q1_dot(t) + q2_dot(t)))/25, 0, 0;
(6*sin(q(2))*q1_dot(t))/25, 0, 0, 0;
q1_dot(t)*((27*sin(q(1)))/50 + (6*sin(q(1) + q(2)))/25) + (6*sin(q(1) + q(2))*q2_dot(t))/25, (6*sin(q(1) + q(2))*(q1_dot(t) + q2_dot(t)))/25, 0, 0;
q1_dot(t)*((27*cos(q(1)))/50 + (6*cos(q(1) + q(2)))/25) + (6*cos(q(1) + q(2))*q2_dot(t))/25, (6*cos(q(1) + q(2))*(q1_dot(t) + q2_dot(t)))/25, 0, 0;
];

