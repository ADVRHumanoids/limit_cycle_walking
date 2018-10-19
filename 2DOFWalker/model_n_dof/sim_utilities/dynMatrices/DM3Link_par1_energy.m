%Energy for a 3-link walker.
%Robot Parameters: 

%Number of links: 3
%Length of links: 1  1  1  
%Position of CoM: 0.5  0.5  0.5  
%Mass of links: 5  5  5  
%Inertia of links: 0.5  0.5  0.5  
%Gravity: 9.81

mechanicalEnergy = ... 
(2943*cos(q(1)))/40 + (981*cos(q(1) + q(2) + q(3)))/40 + (2943*cos(q(1) + q(2)))/40 + (17*q_dot(1)*q_dot(2))/2 + (7*q_dot(1)*q_dot(3))/4 + (7*q_dot(2)*q_dot(3))/4 + (5*cos(q(2) + q(3))*q_dot(1)^2)/2 + (81*q_dot(1)^2)/8 + (17*q_dot(2)^2)/4 + (7*q_dot(3)^2)/8 + (15*cos(q(2))*q_dot(1)^2)/2 + (5*cos(q(3))*q_dot(1)^2)/2 + (5*cos(q(3))*q_dot(2)^2)/2 + (15*cos(q(2))*q_dot(1)*q_dot(2))/2 + 5*cos(q(3))*q_dot(1)*q_dot(2) + (5*cos(q(3))*q_dot(1)*q_dot(3))/2 + (5*cos(q(3))*q_dot(2)*q_dot(3))/2 + (5*cos(q(2) + q(3))*q_dot(1)*q_dot(2))/2 + (5*cos(q(2) + q(3))*q_dot(1)*q_dot(3))/2;


kineticEnergy = ... 
(17*q_dot(1)*q_dot(2))/2 + (7*q_dot(1)*q_dot(3))/4 + (7*q_dot(2)*q_dot(3))/4 + (5*cos(q(2) + q(3))*q_dot(1)^2)/2 + (81*q_dot(1)^2)/8 + (17*q_dot(2)^2)/4 + (7*q_dot(3)^2)/8 + (15*cos(q(2))*q_dot(1)^2)/2 + (5*cos(q(3))*q_dot(1)^2)/2 + (5*cos(q(3))*q_dot(2)^2)/2 + (15*cos(q(2))*q_dot(1)*q_dot(2))/2 + 5*cos(q(3))*q_dot(1)*q_dot(2) + (5*cos(q(3))*q_dot(1)*q_dot(3))/2 + (5*cos(q(3))*q_dot(2)*q_dot(3))/2 + (5*cos(q(2) + q(3))*q_dot(1)*q_dot(2))/2 + (5*cos(q(2) + q(3))*q_dot(1)*q_dot(3))/2;


potentialEnergy = ... 
(2943*cos(q(1)))/40 + (981*cos(q(1) + q(2) + q(3)))/40 + (2943*cos(q(1) + q(2)))/40;


kineticEnergy_dot = ... 
(81*q_dot(1)*q_Ddot(1))/4 + (17*q_dot(1)*q_Ddot(2))/2 + (17*q_dot(2)*q_Ddot(1))/2 + (7*q_dot(1)*q_Ddot(3))/4 + (17*q_dot(2)*q_Ddot(2))/2 + (7*q_dot(3)*q_Ddot(1))/4 + (7*q_dot(2)*q_Ddot(3))/4 + (7*q_dot(3)*q_Ddot(2))/4 + (7*q_dot(3)*q_Ddot(3))/4 - (5*sin(q(2) + q(3))*q_dot(1)*q_dot(2)^2)/2 - (5*sin(q(2) + q(3))*q_dot(1)^2*q_dot(2))/2 - (5*sin(q(2) + q(3))*q_dot(1)*q_dot(3)^2)/2 - (5*sin(q(2) + q(3))*q_dot(1)^2*q_dot(3))/2 + 15*cos(q(2))*q_dot(1)*q_Ddot(1) + (15*cos(q(2))*q_dot(1)*q_Ddot(2))/2 + (15*cos(q(2))*q_dot(2)*q_Ddot(1))/2 + 5*cos(q(3))*q_dot(1)*q_Ddot(1) + 5*cos(q(3))*q_dot(1)*q_Ddot(2) + 5*cos(q(3))*q_dot(2)*q_Ddot(1) + (5*cos(q(3))*q_dot(1)*q_Ddot(3))/2 + 5*cos(q(3))*q_dot(2)*q_Ddot(2) + (5*cos(q(3))*q_dot(3)*q_Ddot(1))/2 + (5*cos(q(3))*q_dot(2)*q_Ddot(3))/2 + (5*cos(q(3))*q_dot(3)*q_Ddot(2))/2 + 5*cos(q(2) + q(3))*q_dot(1)*q_Ddot(1) + (5*cos(q(2) + q(3))*q_dot(1)*q_Ddot(2))/2 + (5*cos(q(2) + q(3))*q_dot(2)*q_Ddot(1))/2 + (5*cos(q(2) + q(3))*q_dot(1)*q_Ddot(3))/2 + (5*cos(q(2) + q(3))*q_dot(3)*q_Ddot(1))/2 - (15*sin(q(2))*q_dot(1)*q_dot(2)^2)/2 - (15*sin(q(2))*q_dot(1)^2*q_dot(2))/2 - (5*sin(q(3))*q_dot(1)*q_dot(3)^2)/2 - (5*sin(q(3))*q_dot(1)^2*q_dot(3))/2 - (5*sin(q(3))*q_dot(2)*q_dot(3)^2)/2 - (5*sin(q(3))*q_dot(2)^2*q_dot(3))/2 - 5*sin(q(3))*q_dot(1)*q_dot(2)*q_dot(3) - 5*sin(q(2) + q(3))*q_dot(1)*q_dot(2)*q_dot(3);


