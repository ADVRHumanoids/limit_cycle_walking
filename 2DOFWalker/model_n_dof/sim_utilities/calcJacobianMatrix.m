function E2 = calcJacobianMatrix(q)

if length(q) == 6

E2 = [   cos(q(1) + q(2) + q(3)) + cos(q(1) + q(2) + q(3) + q(4)) + cos(q(1) + q(2)) + cos(q(1)),   cos(q(1) + q(2) + q(3)) + cos(q(1) + q(2) + q(3) + q(4)) + cos(q(1) + q(2)),   cos(q(1) + q(2) + q(3)) + cos(q(1) + q(2) + q(3) + q(4)),  cos(q(1) + q(2) + q(3) + q(4)), 1, 0;
       - sin(q(1) + q(2) + q(3)) - sin(q(1) + q(2) + q(3) + q(4)) - sin(q(1) + q(2)) - sin(q(1)), - sin(q(1) + q(2) + q(3)) - sin(q(1) + q(2) + q(3) + q(4)) - sin(q(1) + q(2)), - sin(q(1) + q(2) + q(3)) - sin(q(1) + q(2) + q(3) + q(4)), -sin(q(1) + q(2) + q(3) + q(4)), 0, 1];
 
elseif length(q) == 7
    
E2 =    [   cos(q(1) + q(2) + q(3)) + cos(q(1) + q(2) + q(3) + q(4)) + cos(q(1) + q(2)) + cos(q(1)),   cos(q(1) + q(2) + q(3)) + cos(q(1) + q(2) + q(3) + q(4)) + cos(q(1) + q(2)),   cos(q(1) + q(2) + q(3)) + cos(q(1) + q(2) + q(3) + q(4)),  cos(q(1) + q(2) + q(3) + q(4)), 0, 1, 0;
          - sin(q(1) + q(2) + q(3)) - sin(q(1) + q(2) + q(3) + q(4)) - sin(q(1) + q(2)) - sin(q(1)), - sin(q(1) + q(2) + q(3)) - sin(q(1) + q(2) + q(3) + q(4)) - sin(q(1) + q(2)), - sin(q(1) + q(2) + q(3)) - sin(q(1) + q(2) + q(3) + q(4)), -sin(q(1) + q(2) + q(3) + q(4)), 0, 0, 1];
 
elseif length(q) == 5
    
E2 =   [   cos(q(1) + q(2)) + cos(q(1)),  cos(q(1) + q(2)), 0, 1, 0;
         - sin(q(1) + q(2)) - sin(q(1)), -sin(q(1) + q(2)), 0, 0, 1];
   
elseif length(q) == 4
    
E2 = [   cos(q(1) + q(2)) + cos(q(1)),  cos(q(1) + q(2)), 1, 0;
       - sin(q(1) + q(2)) - sin(q(1)), -sin(q(1) + q(2)), 0, 1];
   
end
end