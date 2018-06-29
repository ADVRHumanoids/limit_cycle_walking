function [q_Ddot_after, q_dot_after, q_after] = integrator(dt,D,C,G, tau, q_dot_before, q_before)

    q_Ddot_after =  D \ (tau - C*(q_dot_before) - G);
    q_dot_after = q_dot_before + dt * q_Ddot_after;
    q_after = q_before + dt * q_dot_after; 

end