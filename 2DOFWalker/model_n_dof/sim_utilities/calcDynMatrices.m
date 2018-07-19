function [D,C,G,E2] = calcDynMatrices(q,q_dot)


if length(q) == 7 %5-link robot
elseif length(q) == 6 %4-link robot
elseif length(q) == 5 %3-link robot
    
    DM3Link_par1;
%     DM3Link_par2;
    
elseif length(q) == 4 %2-link robot
    
%     DM2Link_par1; %TODO %make this functions
    DM2Link_par2;
end

end