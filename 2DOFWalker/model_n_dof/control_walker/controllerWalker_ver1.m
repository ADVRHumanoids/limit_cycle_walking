function torque = controllerWalker_ver1(v,q,q_dot, D,C,G)

robotData = getRobotData;

n_link = length(q)-2;
B = [0;1];

q = q(1:n_link);
q_dot = q_dot(1:n_link);

m1 = robotData.mass(1);
m2 = robotData.mass(2);

l1 = robotData.link_length(1);
l2 = robotData.link_length(2);

lc1 = robotData.com_position(1);
lc2 = robotData.com_position(2);

g = robotData.gravity;



dG_dq = [ - g*lc2*m2*cos(q(1) + q(2)) - g*l1*m2*cos(q(1)) - g*lc1*m1*cos(q(1)), ...
                                                    -g*lc2*m2*cos(q(1) + q(2))];
                                                  
dG_dDq = [ g*lc2*m2*sin(q(1) + q(2)) + g*l1*m2*sin(q(1)) + g*lc1*m1*sin(q(1)), g*lc2*m2*sin(q(1) + q(2));
                                                   g *lc2*m2*sin(q(1) + q(2)), g*lc2*m2*sin(q(1) + q(2))];
                                                

                                              
term1 = - q_dot(1:n_link)' * dG_dDq * q_dot(1:n_link);
term2 = dG_dq * inv(D) * B;
term3 = dG_dq * inv(D) *(- C*q_dot - G);

u = inv(term2) * (v - term1 - term3);

torque = [0;u];

% w = term1 + term2*u + term3;

end

