function Links = simKinematics(q,robotData,Base)

Origin = [0;0];
l1 = robotData(5);
l2 = robotData(6);

R1_0 = [sin(q(1)) -cos(q(1)) 0;
        cos(q(1)) sin(q(1))  0;
        0          0           1];

R2_1 = [ cos(q(2)) sin(q(2)) 0;
        -sin(q(2)) cos(q(2)) 0;
        0           0          1];
   
    
R2_0 = R1_0*R2_1;

rp1_1 = [l1; 0; 0];
rp2_2 = [l2; 0; 0];
rp1_0 = R1_0 * rp1_1;
rp2_0 = R2_0 * rp2_2;


% Base = [q(3);q(4)]; %no fixed base
Links(1,1,:) = Base(1) + [Origin(1) rp1_0(1)]; %x of link 1 begin/end
Links(1,2,:) = Base(2) + [Origin(2) rp1_0(2)]; %y of link 1 begin/end

Links(2,1,:) = Links(1,1,2) + [Origin(1) rp2_0(1)]; %x of link 2 begin/end
Links(2,2,:) = Links(1,2,2) + [Origin(2) rp2_0(2)]; %y of link 2 begin/end
             %link 1, x, end
             %link 1, y, end
end