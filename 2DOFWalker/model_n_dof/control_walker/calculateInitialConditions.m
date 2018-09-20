function [controller, q_dot_0] = calculateInitialConditions(startingParameters,fileName,relabelingMatrices, distance)
    
    n_link = 2;
    MatrixRelabel = relabelingMatrices.MatrixRelabel;
    

    robotData = getRobotData;
    m = robotData.mass(1);
    g = robotData.gravity;
    
    q_0 = startingParameters(1:n_link+2,1);
    q_dot = startingParameters(1:n_link+2,2);
    
    q_T = relabelingMatrices.MatrixRelabel*q_0(1:n_link) - relabelingMatrices.piMatrix;
    
    [~,~,G_0,~] = calcDynMatrices(q_0,q_dot,fileName);
    [D_T,~,~,E2_T] = calcDynMatrices(q_T,q_dot,fileName);

    B = [0;1];


    p = calc_p(q_0);
    Phi2_0 = calc_Phi2(q_0);
    Phi2_T = calc_Phi2(q_T);
         
    
    deltaF2 = -inv(E2_T * inv(D_T) * E2_T.') * E2_T * [eye(n_link); zeros(2,n_link)];
    deltaqDotBar = inv(D_T) * E2_T.' * deltaF2 + [eye(n_link); zeros(2,n_link)];

    MatrixImpact = [MatrixRelabel zeros(n_link,2)]* deltaqDotBar;

    d = abs(distance);
    T = 0.25; %0.515;
    xi_0(1) = p; %with q(1) = 0; q(2) = 0;
    xi_0(2) = .581; %!!! %chosen so that at t=T impact 1.569 1.139 .48115 .57
    xi_0(3) = -G_0(1); %%with q(1) = 0; q(2) = 0;

    matrixA = [-T^3/12, -T^4/24;
                   T/2,   T^2/3];

    matrixB = [  0,     0;
               T/2, T^2/6];

    matrixC = [zeros(1,length(d*g*2*m/T));
                                d*g*2*m/T];

    matrixD = [xi_0(3)*T + d*g*m*T;
                         d*g*2*m/T];

                     
    

    phi = Phi2_0 * MatrixImpact * inv(Phi2_T);
    
    term1 = phi * matrixA + matrixB;
    term2 = matrixC - phi * matrixD;
    term3 = [eye(2) - phi] * [xi_0(2); 0];
                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
    controller = inv(term1) * (term2 + term3);                                                                                                            

    a = controller(1);
    b = controller(2);

    xi_0(4) = (d * g * 2 * m - a* T^2/2 - b* T^3/6)* inv(T);
    q_dot_0 = inv(Phi2_0) * [xi_0(2); xi_0(4)];
end
    


function Phi2 = calc_Phi2(q)

    theta = getTheta;
    robotData = getRobotData;
    g = robotData.gravity;
    
    Phi2 = [     theta(1) + theta(2) + 2* theta(3) * cos(q(2)), theta(2) + theta(3) * cos(q(2))
              theta(4)*g*cos(q(1)) + theta(5)*g*cos(q(1)+q(2)),    theta(5) * g* cos(q(1)+q(2))];
          
end

function p = calc_p(q)

   theta = getTheta;
    integralq = q(2)/2 + ...
                (theta(2) - theta(1))/sqrt((theta(1) + theta(2))^2 - 4*theta(3)^2) ...
                *atan(sqrt((theta(1) + theta(2) - 2*theta(3))/(theta(1) + theta(2) + 2*theta(3))) * tan(q(2)/2));

%     integralq = q(2)/2 + ...
%                 (theta(1) - theta(2))/sqrt(4*theta(3)^2 - (theta(1) + theta(2))^2) ...
%                 *atan((theta(1) + theta(2) - 2*theta(3))/sqrt(4*theta(3)^2 - (theta(1) + theta(2))^2) * tan(q(2)/2));
            
    p = q(1) + integralq;

end

function theta = getTheta

    robotData = getRobotData;

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
end
% w_ref = a + b*t; %input of the partial feedback linearization

% function gravity = getGravity
% global g
% gravity = g;
% end
%===============check=======================
%     q_check = relabelingMatrices.MatrixRelabel*q_T(1:n_link) - relabelingMatrices.piMatrix;
% 
%     for i  = 1:length(q_check)
%         if abs(q_check(i)) >= 2*pi
%             if abs(q_check(i)) == q_check(i)
%                 q_check(i) = abs(q_check(i)) - 2*pi;
%             else
%                 q_check(i) = -(abs(q_check(i)) - 2*pi);
%             end
%         end
%     end
    %q_check should be equal to q_0
%===========================================