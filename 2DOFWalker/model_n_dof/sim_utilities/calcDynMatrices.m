function [D,C,G] = calcDynMatrices(q,q_dot)

if length(q) == 6
D = [ (12*cos(q(2) + q(3) + q(4)))/25 + (27*cos(q(2) + q(3)))/25 + (12*cos(q(3) + q(4)))/25 + (42*cos(q(2)))/25 + (27*cos(q(3)))/25 + (12*cos(q(4)))/25 + 627/250, (6*cos(q(2) + q(3) + q(4)))/25 + (27*cos(q(2) + q(3)))/50 + (12*cos(q(3) + q(4)))/25 + (21*cos(q(2)))/25 + (27*cos(q(3)))/25 + (12*cos(q(4)))/25 + 783/500, (6*cos(q(2) + q(3) + q(4)))/25 + (27*cos(q(2) + q(3)))/50 + (6*cos(q(3) + q(4)))/25 + (27*cos(q(3)))/50 + (12*cos(q(4)))/25 + 93/125, (6*cos(q(2) + q(3) + q(4)))/25 + (6*cos(q(3) + q(4)))/25 + (6*cos(q(4)))/25 + 111/500, - (27*cos(q(1) + q(2) + q(3)))/50 - (6*cos(q(1) + q(2) + q(3) + q(4)))/25 - (21*cos(q(1) + q(2)))/25 - (24*cos(q(1)))/25, (27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25 + (21*sin(q(1) + q(2)))/25 + (24*sin(q(1)))/25;
  (6*cos(q(2) + q(3) + q(4)))/25 + (27*cos(q(2) + q(3)))/50 + (12*cos(q(3) + q(4)))/25 + (21*cos(q(2)))/25 + (27*cos(q(3)))/25 + (12*cos(q(4)))/25 + 783/500,                                                                                 (12*cos(q(3) + q(4)))/25 + (27*cos(q(3)))/25 + (12*cos(q(4)))/25 + 783/500,                                                             (6*cos(q(3) + q(4)))/25 + (27*cos(q(3)))/50 + (12*cos(q(4)))/25 + 93/125,                                  (6*cos(q(3) + q(4)))/25 + (6*cos(q(4)))/25 + 111/500,                     - (27*cos(q(1) + q(2) + q(3)))/50 - (6*cos(q(1) + q(2) + q(3) + q(4)))/25 - (21*cos(q(1) + q(2)))/25,                     (27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25 + (21*sin(q(1) + q(2)))/25;
                        (6*cos(q(2) + q(3) + q(4)))/25 + (27*cos(q(2) + q(3)))/50 + (6*cos(q(3) + q(4)))/25 + (27*cos(q(3)))/50 + (12*cos(q(4)))/25 + 93/125,                                                                                   (6*cos(q(3) + q(4)))/25 + (27*cos(q(3)))/50 + (12*cos(q(4)))/25 + 93/125,                                                                                                           (12*cos(q(4)))/25 + 93/125,                                                            (6*cos(q(4)))/25 + 111/500,                                                - (27*cos(q(1) + q(2) + q(3)))/50 - (6*cos(q(1) + q(2) + q(3) + q(4)))/25,                                                (27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25;
                                                                       (6*cos(q(2) + q(3) + q(4)))/25 + (6*cos(q(3) + q(4)))/25 + (6*cos(q(4)))/25 + 111/500,                                                                                                       (6*cos(q(3) + q(4)))/25 + (6*cos(q(4)))/25 + 111/500,                                                                                                           (6*cos(q(4)))/25 + 111/500,                                                                               111/500,                                                                                   -(6*cos(q(1) + q(2) + q(3) + q(4)))/25,                                                                                  (6*sin(q(1) + q(2) + q(3) + q(4)))/25;
                                    - (27*cos(q(1) + q(2) + q(3)))/50 - (6*cos(q(1) + q(2) + q(3) + q(4)))/25 - (21*cos(q(1) + q(2)))/25 - (24*cos(q(1)))/25,                                                       - (27*cos(q(1) + q(2) + q(3)))/50 - (6*cos(q(1) + q(2) + q(3) + q(4)))/25 - (21*cos(q(1) + q(2)))/25,                                                            - (27*cos(q(1) + q(2) + q(3)))/50 - (6*cos(q(1) + q(2) + q(3) + q(4)))/25,                                                -(6*cos(q(1) + q(2) + q(3) + q(4)))/25,                                                                                                                      6/5,                                                                                                                      0;
                                      (27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25 + (21*sin(q(1) + q(2)))/25 + (24*sin(q(1)))/25,                                                         (27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25 + (21*sin(q(1) + q(2)))/25,                                                              (27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25,                                                 (6*sin(q(1) + q(2) + q(3) + q(4)))/25,                                                                                                                        0,                                                                                                                    6/5];
     
    
C = [                       - (27*q_dot(2)*sin(q(2) + q(3)))/50 - (27*q_dot(3)*sin(q(2) + q(3)))/50 - (6*q_dot(3)*sin(q(3) + q(4)))/25 - (6*q_dot(4)*sin(q(3) + q(4)))/25 - (21*q_dot(2)*sin(q(2)))/25 - (27*q_dot(3)*sin(q(3)))/50 - (6*q_dot(4)*sin(q(4)))/25 - (6*sin(q(2) + q(3) + q(4))*q_dot(2))/25 - (6*sin(q(2) + q(3) + q(4))*q_dot(3))/25 - (6*sin(q(2) + q(3) + q(4))*q_dot(4))/25, - (27*q_dot(1)*sin(q(2) + q(3)))/50 - (27*q_dot(2)*sin(q(2) + q(3)))/50 - (27*q_dot(3)*sin(q(2) + q(3)))/50 - (6*q_dot(3)*sin(q(3) + q(4)))/25 - (6*q_dot(4)*sin(q(3) + q(4)))/25 - (21*q_dot(1)*sin(q(2)))/25 - (21*q_dot(2)*sin(q(2)))/25 - (27*q_dot(3)*sin(q(3)))/50 - (6*q_dot(4)*sin(q(4)))/25 - (6*sin(q(2) + q(3) + q(4))*q_dot(1))/25 - (6*sin(q(2) + q(3) + q(4))*q_dot(2))/25 - (6*sin(q(2) + q(3) + q(4))*q_dot(3))/25 - (6*sin(q(2) + q(3) + q(4))*q_dot(4))/25, - (27*q_dot(1)*sin(q(2) + q(3)))/50 - (27*q_dot(2)*sin(q(2) + q(3)))/50 - (6*q_dot(1)*sin(q(3) + q(4)))/25 - (27*q_dot(3)*sin(q(2) + q(3)))/50 - (6*q_dot(2)*sin(q(3) + q(4)))/25 - (6*q_dot(3)*sin(q(3) + q(4)))/25 - (6*q_dot(4)*sin(q(3) + q(4)))/25 - (27*q_dot(1)*sin(q(3)))/50 - (27*q_dot(2)*sin(q(3)))/50 - (27*q_dot(3)*sin(q(3)))/50 - (6*q_dot(4)*sin(q(4)))/25 - (6*sin(q(2) + q(3) + q(4))*q_dot(1))/25 - (6*sin(q(2) + q(3) + q(4))*q_dot(2))/25 - (6*sin(q(2) + q(3) + q(4))*q_dot(3))/25 - (6*sin(q(2) + q(3) + q(4))*q_dot(4))/25, -((6*sin(q(2) + q(3) + q(4)) + 6*sin(q(3) + q(4)) + 6*sin(q(4)))*(q_dot(1) + q_dot(2) + q_dot(3) + q_dot(4)))/25, 0, 0;
                                                                                                                                                 (27*q_dot(1)*sin(q(2) + q(3)))/50 - (6*q_dot(3)*sin(q(3) + q(4)))/25 - (6*q_dot(4)*sin(q(3) + q(4)))/25 + (21*q_dot(1)*sin(q(2)))/25 - (27*q_dot(3)*sin(q(3)))/50 - (6*q_dot(4)*sin(q(4)))/25 + (6*sin(q(2) + q(3) + q(4))*q_dot(1))/25,                                                                                                                                                                                                                                                                                                                                               - (6*q_dot(3)*sin(q(3) + q(4)))/25 - (6*q_dot(4)*sin(q(3) + q(4)))/25 - (27*q_dot(3)*sin(q(3)))/50 - (6*q_dot(4)*sin(q(4)))/25,                                                                                                                                                                                                                                                                                     - (6*q_dot(1)*sin(q(3) + q(4)))/25 - (6*q_dot(2)*sin(q(3) + q(4)))/25 - (6*q_dot(3)*sin(q(3) + q(4)))/25 - (6*q_dot(4)*sin(q(3) + q(4)))/25 - (27*q_dot(1)*sin(q(3)))/50 - (27*q_dot(2)*sin(q(3)))/50 - (27*q_dot(3)*sin(q(3)))/50 - (6*q_dot(4)*sin(q(4)))/25,                             -((6*sin(q(3) + q(4)) + 6*sin(q(4)))*(q_dot(1) + q_dot(2) + q_dot(3) + q_dot(4)))/25, 0, 0;
                                                                                                                                                 (27*q_dot(1)*sin(q(2) + q(3)))/50 + (6*q_dot(1)*sin(q(3) + q(4)))/25 + (6*q_dot(2)*sin(q(3) + q(4)))/25 + (27*q_dot(1)*sin(q(3)))/50 + (27*q_dot(2)*sin(q(3)))/50 - (6*q_dot(4)*sin(q(4)))/25 + (6*sin(q(2) + q(3) + q(4))*q_dot(1))/25,                                                                                                                                                                                                                                                                                                                    (6*q_dot(1)*sin(q(3) + q(4)))/25 + (6*q_dot(2)*sin(q(3) + q(4)))/25 + (27*q_dot(1)*sin(q(3)))/50 + (27*q_dot(2)*sin(q(3)))/50 - (6*q_dot(4)*sin(q(4)))/25,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         -(6*q_dot(4)*sin(q(4)))/25,                                                    -(6*sin(q(4))*(q_dot(1) + q_dot(2) + q_dot(3) + q_dot(4)))/25, 0, 0;
                                                                                                                                                                                       (6*q_dot(1)*sin(q(3) + q(4)))/25 + (6*q_dot(2)*sin(q(3) + q(4)))/25 + (6*q_dot(1)*sin(q(4)))/25 + (6*q_dot(2)*sin(q(4)))/25 + (6*q_dot(3)*sin(q(4)))/25 + (6*sin(q(2) + q(3) + q(4))*q_dot(1))/25,                                                                                                                                                                                                                                                                                                                      (6*q_dot(1)*sin(q(3) + q(4)))/25 + (6*q_dot(2)*sin(q(3) + q(4)))/25 + (6*q_dot(1)*sin(q(4)))/25 + (6*q_dot(2)*sin(q(4)))/25 + (6*q_dot(3)*sin(q(4)))/25,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  (6*sin(q(4))*(q_dot(1) + q_dot(2) + q_dot(3)))/25,                                                                                                                0, 0, 0;
 q_dot(1)*((27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25 + (21*sin(q(1) + q(2)))/25 + (24*sin(q(1)))/25) + (6*q_dot(4)*sin(q(1) + q(2) + q(3) + q(4)))/25 + q_dot(3)*((27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25) + q_dot(2)*((27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25 + (21*sin(q(1) + q(2)))/25),                                                                                                          (6*q_dot(4)*sin(q(1) + q(2) + q(3) + q(4)))/25 + q_dot(3)*((27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25) + q_dot(1)*((27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25 + (21*sin(q(1) + q(2)))/25) + q_dot(2)*((27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25 + (21*sin(q(1) + q(2)))/25),                                                                                                                                                                                                                                      (6*q_dot(4)*sin(q(1) + q(2) + q(3) + q(4)))/25 + q_dot(1)*((27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25) + q_dot(2)*((27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25) + q_dot(3)*((27*sin(q(1) + q(2) + q(3)))/50 + (6*sin(q(1) + q(2) + q(3) + q(4)))/25),                                (6*sin(q(1) + q(2) + q(3) + q(4))*(q_dot(1) + q_dot(2) + q_dot(3) + q_dot(4)))/25, 0, 0;
 (6*q_dot(4)*cos(q(1) + q(2) + q(3) + q(4)))/25 + q_dot(3)*((27*cos(q(1) + q(2) + q(3)))/50 + (6*cos(q(1) + q(2) + q(3) + q(4)))/25) + q_dot(2)*((27*cos(q(1) + q(2) + q(3)))/50 + (6*cos(q(1) + q(2) + q(3) + q(4)))/25 + (21*cos(q(1) + q(2)))/25) + q_dot(1)*((27*cos(q(1) + q(2) + q(3)))/50 + (6*cos(q(1) + q(2) + q(3) + q(4)))/25 + (21*cos(q(1) + q(2)))/25 + (24*cos(q(1)))/25),                                                                                                          (6*q_dot(4)*cos(q(1) + q(2) + q(3) + q(4)))/25 + q_dot(3)*((27*cos(q(1) + q(2) + q(3)))/50 + (6*cos(q(1) + q(2) + q(3) + q(4)))/25) + q_dot(1)*((27*cos(q(1) + q(2) + q(3)))/50 + (6*cos(q(1) + q(2) + q(3) + q(4)))/25 + (21*cos(q(1) + q(2)))/25) + q_dot(2)*((27*cos(q(1) + q(2) + q(3)))/50 + (6*cos(q(1) + q(2) + q(3) + q(4)))/25 + (21*cos(q(1) + q(2)))/25),                                                                                                                                                                                                                                      (6*q_dot(4)*cos(q(1) + q(2) + q(3) + q(4)))/25 + q_dot(1)*((27*cos(q(1) + q(2) + q(3)))/50 + (6*cos(q(1) + q(2) + q(3) + q(4)))/25) + q_dot(2)*((27*cos(q(1) + q(2) + q(3)))/50 + (6*cos(q(1) + q(2) + q(3) + q(4)))/25) + q_dot(3)*((27*cos(q(1) + q(2) + q(3)))/50 + (6*cos(q(1) + q(2) + q(3) + q(4)))/25),                                (6*cos(q(1) + q(2) + q(3) + q(4))*(q_dot(1) + q_dot(2) + q_dot(3) + q_dot(4)))/25, 0, 0];



G = [ - (26487*sin(q(1) + q(2) + q(3)))/5000 - (2943*sin(q(1) + q(2) + q(3) + q(4)))/1250 - (20601*sin(q(1) + q(2)))/2500 - (5886*sin(q(1)))/625;
                        - (26487*sin(q(1) + q(2) + q(3)))/5000 - (2943*sin(q(1) + q(2) + q(3) + q(4)))/1250 - (20601*sin(q(1) + q(2)))/2500;
                                                        - (26487*sin(q(1) + q(2) + q(3)))/5000 - (2943*sin(q(1) + q(2) + q(3) + q(4)))/1250;
                                                                                                -(2943*sin(q(1) + q(2) + q(3) + q(4)))/1250;
                                                                                                                                          0;
                                                                                                                                   2943/250];
 
 
                                  
                                  
                                  
                                  
                                  
                                  
elseif length(q) == 4
    
   D = [                  (12*cos(q(2)))/25 + 141/250, (6*cos(q(2)))/25 + 111/500, - (6*cos(q(1) + q(2)))/25 - (9*cos(q(1)))/25, (6*sin(q(1) + q(2)))/25 + (9*sin(q(1)))/25;
                   (6*cos(q(2)))/25 + 111/500,                    111/500,                     -(6*cos(q(1) + q(2)))/25,                    (6*sin(q(1) + q(2)))/25;
 - (6*cos(q(1) + q(2)))/25 - (9*cos(q(1)))/25,   -(6*cos(q(1) + q(2)))/25,                                          3/5,                                          0;
   (6*sin(q(1) + q(2)))/25 + (9*sin(q(1)))/25,    (6*sin(q(1) + q(2)))/25,                                            0,                                        3/5];



 C = [                                                               -(6*q_dot(2)*sin(q(2)))/25,       -(6*sin(q(2))*(q_dot(1) + q_dot(2)))/25, 0, 0;
                                                                (6*q_dot(1)*sin(q(2)))/25,                                             0, 0, 0;
 (6*q_dot(2)*sin(q(1) + q(2)))/25 + q_dot(1)*((6*sin(q(1) + q(2)))/25 + (9*sin(q(1)))/25), (6*sin(q(1) + q(2))*(q_dot(1) + q_dot(2)))/25, 0, 0;
 q_dot(1)*((6*cos(q(1) + q(2)))/25 + (9*cos(q(1)))/25) + (6*q_dot(2)*cos(q(1) + q(2)))/25, (6*cos(q(1) + q(2))*(q_dot(1) + q_dot(2)))/25, 0, 0];


G =  [- (2943*sin(q(1) + q(2)))/1250 - (8829*sin(q(1)))/2500;
                          -(2943*sin(q(1) + q(2)))/1250;
                                                      0;
                                               2943/500];
end
 
end
 
 
 