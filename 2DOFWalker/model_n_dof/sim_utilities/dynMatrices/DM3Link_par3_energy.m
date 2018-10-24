%Energy for a 3-link walker.
%Robot Parameters: 

%Number of links: 3
%Length of links: 1  1  1  
%Position of CoM: 0.5  0.5  0.8  
%Mass of links: 5  5  15  
%Inertia of links: 0.5  0.5  1.5  
%Gravity: 9.81

mechanicalEnergy = ... 
(8829*cos(q(1)))/40 + (981*cos(q(1) + q(2)))/40 + (2943*cos(q(1) + q(3)))/25 + (7*q_dot(1)*q_dot(2))/4 + (111*q_dot(1)*q_dot(3))/10 + (173*q_dot(1)^2)/10 + (7*q_dot(2)^2)/8 + (111*q_dot(3)^2)/20 + (5*cos(q(2))*q_dot(1)^2)/2 + 12*cos(q(3))*q_dot(1)^2 + (5*cos(q(2))*q_dot(1)*q_dot(2))/2 + 12*cos(q(3))*q_dot(1)*q_dot(3);


kineticEnergy = ... 
(7*q_dot(1)*q_dot(2))/4 + (111*q_dot(1)*q_dot(3))/10 + (173*q_dot(1)^2)/10 + (7*q_dot(2)^2)/8 + (111*q_dot(3)^2)/20 + (5*cos(q(2))*q_dot(1)^2)/2 + 12*cos(q(3))*q_dot(1)^2 + (5*cos(q(2))*q_dot(1)*q_dot(2))/2 + 12*cos(q(3))*q_dot(1)*q_dot(3);


potentialEnergy = ... 
(8829*cos(q(1)))/40 + (981*cos(q(1) + q(2)))/40 + (2943*cos(q(1) + q(3)))/25;


kineticEnergy_dot = ... 
(173*q_dot(1)*q_Ddot(1))/5 + (7*q_dot(1)*q_Ddot(2))/4 + (7*q_dot(2)*q_Ddot(1))/4 + (111*q_dot(1)*q_Ddot(3))/10 + (7*q_dot(2)*q_Ddot(2))/4 + (111*q_dot(3)*q_Ddot(1))/10 + (111*q_dot(3)*q_Ddot(3))/10 + 5*cos(q(2))*q_dot(1)*q_Ddot(1) + (5*cos(q(2))*q_dot(1)*q_Ddot(2))/2 + (5*cos(q(2))*q_dot(2)*q_Ddot(1))/2 + 24*cos(q(3))*q_dot(1)*q_Ddot(1) + 12*cos(q(3))*q_dot(1)*q_Ddot(3) + 12*cos(q(3))*q_dot(3)*q_Ddot(1) - (5*sin(q(2))*q_dot(1)*q_dot(2)^2)/2 - (5*sin(q(2))*q_dot(1)^2*q_dot(2))/2 - 12*sin(q(3))*q_dot(1)*q_dot(3)^2 - 12*sin(q(3))*q_dot(1)^2*q_dot(3);


