function v = PD_controller(y, y_dot)
    

    epsilon = .1;
    Kd = 1;
    Kp = 10;
    
    v = (1/epsilon * Kd * y_dot) + (1/epsilon * Kp * y);
    
end