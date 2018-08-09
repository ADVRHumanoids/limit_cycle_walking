syms m1 m2 lc1 lc2 I1 I2 l1 l2 g q1 q2 y1 y2 A B C D

%`````````````````````````````````````
    theta(1) = m1 * lc1^2 + m2 * l1^2 + I1;
    theta(2) = m2 * lc2^2 + I2;
    theta(3) = m2 * l1 * lc2;
    theta(4) = m1 * lc1 + m2 * l1;
    theta(5) = m2 * lc2;
%`````````````````````````````````````

%     integralq = q2/2 + ...
%                 (theta(2) - theta(1))/sqrt((theta(1) + theta(2))^2 - 4*theta(3)^2) ...
%                 *atan(sqrt((theta(1) + theta(2) - 2*theta(3))/(theta(1) + theta(2) + 2*theta(3))) * tan(q2/2));

%     integralq = q2/2 + ...
%                 (theta(1) - theta(2))/sqrt(4*theta(3)^2 - (theta(1) + theta(2))^2) ...
%                 *atan((theta(1) + theta(2) - 2*theta(3))/sqrt(4*theta(3)^2 - (theta(1) + theta(2))^2) * tan(q2/2));
            
% eq1 = q1 + integralq == y1;

eq1 = q1 + q2/2 + A * atan(B * tan(q2/2));
eq2 = C * sin(q1) + D * sin(q1+q2) == y2;
% eq2 =theta(4) * g * sin(q1) + theta(5) * g * sin(q1+q2) == y2;


sol = solve([eq1 eq2],[q1 q2]);
x = ...
    5;
