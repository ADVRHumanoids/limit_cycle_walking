function [torque,y] = controllerWalker(q,q_dot, D,C,G,offset_leg, offset_waist)

    n_link = length(q)-2;
    
    n_unactuated_variable = 1;
    n_base_variables = 2;
    n_controlled_variables = length(q) - n_base_variables - n_unactuated_variable; %-base z1 z2 // -underactuated var
    B = [zeros(1,n_controlled_variables); eye(n_controlled_variables)];
    

   a_leg = [-2.27 3.26 3.11 1.89];
   a_waist = [0.512 0.073 0.035 -0.819];
        
%==========finite time controller & partial linearization==================
    y = zeros(n_controlled_variables, 1);


        y(1) = q(1) - (offset_leg - q(1) - q(2)); %leg virtual constraint
        y(2) = offset_waist * (q(3) +  pi/8); %q(1) + q(3) + 2 * pi; %waist virtual constraint
        
        h_dq = [2 1 0;
                0 0 1];

% %         as long as there are no velocities in the y, I can write h_dq_dq like this
        h_dq_dq = [zeros(length(y), n_link) h_dq];




%     thd(1) = pi/8;
%     
%     th(1) = q(1);
%     th(2) = - (pi - q(1) - q(2));
%     th(3) = q(3);




% 
%     y(1) = th(2)-(-th(1)+(a_leg(1) + a_leg(2)*th(1) + a_leg(3)*th(1)^2+a_leg(4)*th(1)^3)*(th(1)-thd(1))*(th(1)+thd(1)));%leg virtual constraint
%     y(2) = th(3)-(a_waist(1) + a_waist(2)*th(1) + a_waist(3)*th(1)^2 + a_waist(4)*th(1)^3); %waist virtual constraint

    %h_dq = jacobian(y,q)
    %Controller based on Bezier curves for virtual constraints.
%     h_dq = [ 
%     a_leg(2)*thd(1)^2 - 3*a_leg(2)*q(1)^2 - 4*a_leg(3)*q(1)^3 - 5*a_leg(4)*q(1)^4 - 2*a_leg(1)*q(1) + 2*a_leg(3)*q(1)*thd(1)^2 + 3*a_leg(4)*q(1)^2*thd(1)^2 + 2, 1, 0;
%     - a_waist(2) - 2*a_waist(3)*q(1) - 3*a_waist(4)*q(1)^2, 0, 1;
%     ];
% 
%     h_dq_dq = [ 
%     -2*q_dot(1)*(a_leg(1) + 3*a_leg(2)*q(1) + 6*a_leg(3)*q(1)^2 + 10*a_leg(4)*q(1)^3 - a_leg(3)*thd(1)^2 - 3*a_leg(4)*q(1)*thd(1)^2), 0, 0, a_leg(2)*thd(1)^2 - 3*a_leg(2)*q(1)^2 - 4*a_leg(3)*q(1)^3 - 5*a_leg(4)*q(1)^4 - 2*a_leg(1)*q(1) + 2*a_leg(3)*q(1)*thd(1)^2 + 3*a_leg(4)*q(1)^2*thd(1)^2 + 2, 1, 0;
%     -q_dot(1)*(2*a_waist(3) + 6*a_waist(4)*q(1)), 0, 0, - a_waist(2) - 2*a_waist(3)*q(1) - 3*a_waist(4)*q(1)^2, 0, 1;
%     ];






    Fx = (inv(D)*(-C* q_dot(1:n_link) - G));
    Gx = inv(D) * B;
    %>>(see partialLinearization for clarification)
    h = y;
    Lfh = h_dq * q_dot(1:n_link);
    L2fh  =  h_dq_dq * [q_dot(1:n_link); Fx];
    LgLfh =  h_dq * Gx;
    

    
    v = finiteTime_stabilizer(h,Lfh);
%     v = -PD_controller(h,Lfh); %another working version of the stabilizer
%     v = 5; % does't go to 5
    u = inv(LgLfh) * (v - L2fh);

    torque = [0;u];
        

end



%% ========================================================================
% syms q1 q2 q1_dot q2_dot
% % 
% q = [q1; q2];
% q_dot = [q1_dot; q2_dot];
% h = - (q(1) - (anglePi - q(1) - q(2))); %%output of the system
% h_dq = jacobian(h,q);  % h_dq = functionalDerivative(h,q)';
% Lfh = h_dq * q_dot;
% h_dq_dq = [jacobian(Lfh, q) h_dq]; % h_dq_dq = [functionalDerivative(Lfh,q)'  h_dq];