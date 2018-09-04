%Dynamic matrices for a 2-link walker.
%Robot Parameters: 

%Number of links: 2
%Length of links: 1  1  
%Position of CoM: 0.8  0.2  
%Mass of links: 0.3  0.3  
%Inertia of links: 0.03  0.03  
%Gravity: 9.81

D = [ 
(3*cos(q(2)))/25 + 141/250, (3*cos(q(2)))/50 + 21/500, - (27*cos(q(1)))/50 - (3*cos(q(1) + q(2)))/50, (27*sin(q(1)))/50 + (3*sin(q(1) + q(2)))/50;
(3*cos(q(2)))/50 + 21/500, 21/500, -(3*cos(q(1) + q(2)))/50, (3*sin(q(1) + q(2)))/50;
- (27*cos(q(1)))/50 - (3*cos(q(1) + q(2)))/50, -(3*cos(q(1) + q(2)))/50, 3/5, 0;
(27*sin(q(1)))/50 + (3*sin(q(1) + q(2)))/50, (3*sin(q(1) + q(2)))/50, 0, 3/5;
];

C = [ 
-(3*sin(q(2))*q_dot(2))/50, -(3*sin(q(2))*(q_dot(1) + q_dot(2)))/50, 0, 0;
(3*sin(q(2))*q_dot(1))/50, 0, 0, 0;
q_dot(1)*((27*sin(q(1)))/50 + (3*sin(q(1) + q(2)))/50) + (3*sin(q(1) + q(2))*q_dot(2))/50, (3*sin(q(1) + q(2))*(q_dot(1) + q_dot(2)))/50, 0, 0;
q_dot(1)*((27*cos(q(1)))/50 + (3*cos(q(1) + q(2)))/50) + (3*cos(q(1) + q(2))*q_dot(2))/50, (3*cos(q(1) + q(2))*(q_dot(1) + q_dot(2)))/50, 0, 0;
];

G = [ 
- (26487*sin(q(1)))/5000 - (2943*sin(q(1) + q(2)))/5000;
-(2943*sin(q(1) + q(2)))/5000;
0;
2943/500;
];

E2 = [ 
cos(q(1)) + cos(q(1) + q(2)), cos(q(1) + q(2)), 1, 0;
- sin(q(1)) - sin(q(1) + q(2)), -sin(q(1) + q(2)), 0, 1;
];

