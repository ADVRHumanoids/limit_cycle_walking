r = [0; 2];
com_i = [1; 2];
theta_steer = pi/2;

theta_heading = 0;
R_heading = [cos(theta_heading), - sin(theta_heading); sin(theta_heading), cos(theta_heading)];
R_steering = [cos(theta_steer), - sin(theta_steer); sin(theta_steer), cos(theta_steer)];

R_steer_half = [cos(theta_steer/2), - sin(theta_steer/2); sin(theta_steer/2), cos(theta_steer/2)];
o = com_i + R_heading * r;

com_f = R_steering * (com_i - o) + o;

cord = 2 * norm(r) * sin(theta_steer/2);
com_f1 = com_i + R_heading * R_steer_half * cord;

plot(com_i(1), com_i(2), 'x'); hold on;
plot(com_f(1), com_f(2), 'o'); hold on;
plot([com_i(1),com_f1(1)], [com_i(2), com_f1(2)]);

h = circle(o(1),o(2),r(2));

function h = circle(x,y,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit);
    xlim([-2,4])
    ylim([1,7])
    pbaspect([1 1 1])
    hold off
end