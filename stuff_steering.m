% r = [0; 2];
% com_i = [1; 2];
% theta_steer = pi/2;
% 
% theta_heading = 0;
% R_heading = [cos(theta_heading), - sin(theta_heading); sin(theta_heading), cos(theta_heading)];
% R_steering = [cos(theta_steer), - sin(theta_steer); sin(theta_steer), cos(theta_steer)];
% 
% R_steer_half = [cos(theta_steer/2), - sin(theta_steer/2); sin(theta_steer/2), cos(theta_steer/2)];
% o = com_i + R_heading * r;
% 
% com_f = R_steering * (com_i - o) + o;
% 
% cord = 2 * norm(r) * sin(theta_steer/2);
% com_f1 = com_i + R_heading * R_steer_half * cord;
% 
% plot(com_i(1), com_i(2), 'x'); hold on;
% plot(com_f(1), com_f(2), 'o'); hold on;
% plot([com_i(1),com_f1(1)], [com_i(2), com_f1(2)]);
% 
% h = circle(o(1),o(2),r(2));
% 
% function h = circle(x,y,r)
%     hold on
%     th = 0:pi/50:2*pi;
%     xunit = r * cos(th) + x;
%     yunit = r * sin(th) + y;
%     h = plot(xunit, yunit);
%     xlim([-2,4])
%     ylim([1,7])
%     pbaspect([1 1 1])
%     hold off
% end
%------------------------------------------------------------------------
t_max = 200;
n_chuncks = 10;
time = 1:t_max;
zmp_y = ones(1,t_max);
size_chunck = t_max/n_chuncks;
j = 1;
for i = 0:size_chunck:t_max
    vec(j) = i;
    j = j+1;
end

for k = 1:length(vec)-1
    if mod(k,2)
        zmp_y(vec(k)+1:vec(k+1)) = -0.1;
    else
        zmp_y(vec(k)+1:vec(k+1)) =  0.1;
    end
end



theta = pi/10;
R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

zmp_y = [zeros(1,length(zmp_y)); zmp_y];
for k = 1:length(zmp_y)
    if (k > 80 && k < 101)
        theta = pi/10;
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        zmp_y(:,k) = R * zmp_y(:,k);
    elseif (k > 100 && k < 121)
        theta = pi/8;
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        zmp_y(:,k) = R * zmp_y(:,k);
    elseif (k > 120 && k < 141)
        theta = pi/6;
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        zmp_y(:,k) = R * zmp_y(:,k);
    elseif (k > 140 && k < 161)
        theta = pi/4;
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        zmp_y(:,k) = R * zmp_y(:,k);
    elseif (k > 160 && k < 181)
        theta = pi/4;
        R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        zmp_y(:,k) = R * zmp_y(:,k);
    else
        zmp_y(:,k) = zmp_y(:,k);
    end
end


plot(time, zmp_y);
ylim([-0.2,0.2]);

% 
%  if (k > 100 && k < 110)
%         zmp_y(:,k) = R * zmp_y(:,k);
%     elseif (k > 100 && k < 110)
%         theta = theta + pi/10;
%         R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
%         zmp_y(:,k) = R * zmp_y(:,k);
%     elseif (k > 110 && k < 120)
%         theta = theta + pi/10;
%         R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
%         zmp_y(:,k) = R * zmp_y(:,k);
%     elseif (k > 120 && k < 130)
%         theta = theta + pi/10;
%         R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
%         zmp_y(:,k) = R * zmp_y(:,k);
%     elseif (k > 130 && k < 140)
%         theta = theta + pi/10;
%         R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
%         zmp_y(:,k) = R * zmp_y(:,k);
%     else
%             zmp_y(:,k) = zmp_y(:,k);
