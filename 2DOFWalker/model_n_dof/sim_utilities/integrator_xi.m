function [xi_Ddot_after, xi_dot_after, xi_after] = integrator_xi(dt, w,xi_Ddot_before,xi_dot_before,xi_before,D)
   
    xi_DDdot_after =  w;
    xi_Ddot_after = xi_Ddot_before + dt * xi_DDdot_after;
    xi_dot_after = xi_dot_before + dt * xi_Ddot_after;
    xi_after = xi_before + inv(D(1,1)) * dt * xi_dot_after;

end