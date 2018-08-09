function [xi, Phi2] = variableXi(q,q_dot,q_Ddot)

    robotData = getRobotData;
    n_link = robotData.n_link;
    g = robotData.gravity;


    q = q(1:n_link);
    q_dot = q_dot(1:n_link);
    q_Ddot = q_Ddot(1:n_link);
    
    m1 = robotData.mass(1);
    m2 = robotData.mass(2);
    m = m1;

    l1 = robotData.link_length(1);
    l2 = robotData.link_length(2);

    lc1 = robotData.com_position(1);
    lc2 = robotData.com_position(2);

    I1 = robotData.inertia(1);
    I2 = robotData.inertia(2);



    %`````````````````````````````````````
    theta(1) = m1 * lc1^2 + m2 * l1^2 + I1;
    theta(2) = m2 * lc2^2 + I2;
    theta(3) = m2 * l1 * lc2; 
    theta(4) = m1 * lc1 + m2 * l1;
    theta(5) = m2 * lc2;
    %`````````````````````````````````````
        dG_dq = [ - g*lc2*m2*cos(q(1) + q(2)) - g*l1*m2*cos(q(1)) - g*lc1*m1*cos(q(1)), ...
                                                    -g*lc2*m2*cos(q(1) + q(2))];
                                                  
        dG_dDq = [ g*lc2*m2*sin(q(1) + q(2)) + g*l1*m2*sin(q(1)) + g*lc1*m1*sin(q(1)), g*lc2*m2*sin(q(1) + q(2));
                                                   g *lc2*m2*sin(q(1) + q(2)), g*lc2*m2*sin(q(1) + q(2))];  
        integralq = q(2)/2 + ...
                    (theta(2) - theta(1))/sqrt((theta(1) + theta(2))^2 - 4*theta(3)^2) ...
                    *atan(sqrt((theta(1) + theta(2) - 2*theta(3))/(theta(1) + theta(2) + 2*theta(3))) * tan(q(2)/2));
% 
%         integralq = q(2)/2 + ...
%                     (theta(1) - theta(2))/sqrt(4*theta(3)^2 - (theta(1) + theta(2))^2) ...
%                     *atan((theta(1) + theta(2) - 2*theta(3))/sqrt(4*theta(3)^2 - (theta(1) + theta(2))^2) * tan(q(2)/2));

    p = q(1) + integralq;

    Phi2 = [  theta(1) + theta(2) + 2* theta(3) * cos(q(2)),  theta(2) + theta(3) * cos(q(2))
             theta(4)*g*cos(q(1)) + theta(5)*g*cos(q(1)+q(2)),     theta(5) * g* cos(q(1)+q(2))];

    xi(1,:) = p;
    xi(3,:) = theta(4) * g * sin(q(1)) + theta(5) * g * sin(q(1)+q(2));
    tempT = Phi2 * q_dot;
    xi(2,:) = tempT(1);
    xi(4,:) = tempT(2);
                                                                                                                             
    xi(5,:) = - q_dot' * dG_dDq * q_dot - dG_dq * q_Ddot; %derivative of xi4
    
end






