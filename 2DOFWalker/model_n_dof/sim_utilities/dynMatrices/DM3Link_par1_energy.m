%Energy for a 3-link walker.
%Robot Parameters: 

%Number of links: 3
%Length of links: 1  1  0.5  
%Position of CoM: 0.8  0.2  0.25  
%Mass of links: 0.3  0.3  1  
%Inertia of links: 0.03  0.03  0.1  
%Gravity: 9.81

mechanicalEnergy = ... 
(75537*cos(q(1)))/5000 + (2943*cos(q(1) + q(2)))/5000 + (981*cos(q(1) + q(3)))/400 + (521*q_dot(1)*q_dot(2))/500 + (13*q_dot(1)*q_dot(3))/80 + (3453*q_dot(1)^2)/4000 + (521*q_dot(2)^2)/1000 + (13*q_dot(3)^2)/160 + (q_dot(1)^2*cos(q(2) - q(3)))/4 + (3*cos(q(2))*q_dot(1)^2)/50 + (q_dot(1)*q_dot(2)*cos(q(2) - q(3)))/4 + (q_dot(1)*q_dot(3)*cos(q(2) - q(3)))/4 + (q_dot(2)*q_dot(3)*cos(q(2) - q(3)))/4 + (3*cos(q(2))*q_dot(1)*q_dot(2))/50;


kineticEnergy = ... 
(521*q_dot(1)*q_dot(2))/500 + (13*q_dot(1)*q_dot(3))/80 + (3453*q_dot(1)^2)/4000 + (521*q_dot(2)^2)/1000 + (13*q_dot(3)^2)/160 + (q_dot(1)^2*cos(q(2) - q(3)))/4 + (3*cos(q(2))*q_dot(1)^2)/50 + (q_dot(1)*q_dot(2)*cos(q(2) - q(3)))/4 + (q_dot(1)*q_dot(3)*cos(q(2) - q(3)))/4 + (q_dot(2)*q_dot(3)*cos(q(2) - q(3)))/4 + (3*cos(q(2))*q_dot(1)*q_dot(2))/50;


potentialEnergy = ... 
(75537*cos(q(1)))/5000 + (2943*cos(q(1) + q(2)))/5000 + (981*cos(q(1) + q(3)))/400;


