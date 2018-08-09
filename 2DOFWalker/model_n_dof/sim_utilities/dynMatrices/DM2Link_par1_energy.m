%Energy for a 2-link walker.
%Robot Parameters: 

%Number of links: 2
%Length of links: 1  1  
%Position of CoM: 0.8  0.2  
%Mass of links: 0.3  0.3  
%Inertia of links: 0.03  0.03  
%Gravity: 9.81

mechanicalEnergy = ... 
(26487*cos(q(1)))/5000 + (2943*cos(q(1) + q(2)))/5000 + (21*q_dot(1)*q_dot(2))/500 + (141*q_dot(1)^2)/500 + (21*q_dot(2)^2)/1000 + (3*cos(q(2))*q_dot(1)^2)/50 + (3*cos(q(2))*q_dot(1)*q_dot(2))/50;


kineticEnergy = ... 
(21*q_dot(1)*q_dot(2))/500 + (141*q_dot(1)^2)/500 + (21*q_dot(2)^2)/1000 + (3*cos(q(2))*q_dot(1)^2)/50 + (3*cos(q(2))*q_dot(1)*q_dot(2))/50;


potentialEnergy = ... 
(26487*cos(q(1)))/5000 + (2943*cos(q(1) + q(2)))/5000;


