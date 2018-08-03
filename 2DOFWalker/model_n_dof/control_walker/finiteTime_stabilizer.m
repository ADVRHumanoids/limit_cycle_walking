function v = finiteTime_stabilizer(y, y_dot)
    
    alpha = 0.9;
    epsilon = 0.1;
    
    x1 = y;
    x2 = epsilon.*y_dot;

    phi = x1 + (1/(2 - alpha)).*sign(x2).*abs(x2).^(2-alpha);
    psi = -sign(x2).*abs(x2).^alpha - sign(phi) .* abs(phi).^(alpha/(2 - alpha));

    v = 1/epsilon^2 .* psi;
    
end