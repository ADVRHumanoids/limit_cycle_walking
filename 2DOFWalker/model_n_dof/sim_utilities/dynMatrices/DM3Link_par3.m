%Dynamic matrices for a 3-link walker.
%Robot Parameters: 

%Number of links: 3
%Length of links: 1  1  1  
%Position of CoM: 0.5  0.5  0.8  
%Mass of links: 5  5  15  
%Inertia of links: 0.5  0.5  1.5  
%Gravity: 9.81

D = [ 
5*cos(q(2)) + 24*cos(q(3)) + 173/5, (5*cos(q(2)))/2 + 7/4, 12*cos(q(3)) + 111/10, - (45*cos(q(1)))/2 - (5*cos(q(1) + q(2)))/2 - 12*cos(q(1) + q(3)), (45*sin(q(1)))/2 + (5*sin(q(1) + q(2)))/2 + 12*sin(q(1) + q(3));
(5*cos(q(2)))/2 + 7/4, 7/4, 0, -(5*cos(q(1) + q(2)))/2, (5*sin(q(1) + q(2)))/2;
12*cos(q(3)) + 111/10, 0, 111/10, -12*cos(q(1) + q(3)), 12*sin(q(1) + q(3));
- (45*cos(q(1)))/2 - (5*cos(q(1) + q(2)))/2 - 12*cos(q(1) + q(3)), -(5*cos(q(1) + q(2)))/2, -12*cos(q(1) + q(3)), 25, 0;
(45*sin(q(1)))/2 + (5*sin(q(1) + q(2)))/2 + 12*sin(q(1) + q(3)), (5*sin(q(1) + q(2)))/2, 12*sin(q(1) + q(3)), 0, 25;
];

C = [ 
- (5*sin(q(2))*q_dot(2))/2 - 12*sin(q(3))*q_dot(3), -(5*sin(q(2))*(q_dot(1) + q_dot(2)))/2, -12*sin(q(3))*(q_dot(1) + q_dot(3)), 0, 0;
(5*sin(q(2))*q_dot(1))/2, 0, 0, 0, 0;
12*sin(q(3))*q_dot(1), 0, 0, 0, 0;
q_dot(1)*((45*sin(q(1)))/2 + (5*sin(q(1) + q(2)))/2 + 12*sin(q(1) + q(3))) + (5*sin(q(1) + q(2))*q_dot(2))/2 + 12*sin(q(1) + q(3))*q_dot(3), (5*sin(q(1) + q(2))*(q_dot(1) + q_dot(2)))/2, 12*sin(q(1) + q(3))*(q_dot(1) + q_dot(3)), 0, 0;
q_dot(1)*((45*cos(q(1)))/2 + (5*cos(q(1) + q(2)))/2 + 12*cos(q(1) + q(3))) + (5*cos(q(1) + q(2))*q_dot(2))/2 + 12*cos(q(1) + q(3))*q_dot(3), (5*cos(q(1) + q(2))*(q_dot(1) + q_dot(2)))/2, 12*cos(q(1) + q(3))*(q_dot(1) + q_dot(3)), 0, 0;
];

G = [ 
- (8829*sin(q(1)))/40 - (981*sin(q(1) + q(2)))/40 - (2943*sin(q(1) + q(3)))/25;
-(981*sin(q(1) + q(2)))/40;
-(2943*sin(q(1) + q(3)))/25;
0;
981/4;
];

E2 = [ 
cos(q(1)) + cos(q(1) + q(2)), cos(q(1) + q(2)), 0, 1, 0;
- sin(q(1)) - sin(q(1) + q(2)), -sin(q(1) + q(2)), 0, 0, 1;
];

