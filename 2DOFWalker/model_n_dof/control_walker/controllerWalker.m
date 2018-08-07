function [torque,y] = controllerWalker(q,q_dot, D,C,G)

    n_link = length(q)-2;
    
    n_unactuated_variable = 1;
    n_base_variables = 2;
    n_controlled_variables = length(q) - n_base_variables - n_unactuated_variable; %-base z1 z2 // -underactuated var
    B = [zeros(1,n_controlled_variables); eye(n_controlled_variables)];
    


        
%==========finite time controller & partial linearization==================
    y = zeros(n_controlled_variables, 1);
    
    if n_link == 3
        
        y(1) = q(1) - (-pi - q(1) - q(2));
        y(2) = q(1) + q(3) + 2 * pi;

        h_dq = [2 1 0;
                1 0 1];
        
        %as long as there are no velocities in the y, I can write h_dq_dq like this
        h_dq_dq = [zeros(length(y), n_link) h_dq];
        
    elseif n_link == 2
        
        y(1) = q(2) - pi/2; % q goes to 5
%         y(1) = q(1) - (-pi - q(1) - q(2)); %not working properly
%         y(1) = q(1) - (-pi - q(1) - q(2));
        h_dq = [0 1]; %>> h_dq = jacobian(h,q);
%         h_dq = [2 1];
        
        %as long as there are no velocities in the y, I can write h_dq_dq like this
        h_dq_dq = [zeros(length(y), n_link) h_dq];
        
    end


    %>>(see partialLinearization for clarification)
    h = y;
    Lfh = h_dq * q_dot(1:n_link);
    L2fh  = -h_dq_dq(:, end-n_controlled_variables:end)* inv(D) * C* q_dot(1:n_link) - h_dq_dq(:, end-n_controlled_variables:end) * inv(D) * G;
    LgLfh =  h_dq * inv(D) * B;

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