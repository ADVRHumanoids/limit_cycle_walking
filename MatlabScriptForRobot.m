%figure(1);
%plot(foot_trajectory');

%% zmp
figure(2);
plot(zmp');
hold on; plot(zmp_ref', 'r--');
plot(com_trajectory')

plot(zmp_y(20:end)');

plot(shifted');

plot(1 + foot_trajectory(3,:)');
%% phase LAND 1/FLIGHT 0
figure(3);
plot(landed_left);
hold on; plot(landed_right, 'r--');

%% force sensors
figure(4);
hold on; plot(abs(ft_right(3,:))', 'r--')
plot(abs(ft_left(3,:))')

%% impact detector
figure(5);
plot(impact_detector');
hold on;plot(1 +foot_trajectory(3,:)','r');

%% zmp and foot trajectory
hold on; plot(zmp_ref')
plot(zmp')
hold on;plot(1 +foot_trajectory(3,:)','r');

%% zmp and phase LAND and FLIGHT
hold on; plot(zmp_ref')
plot(0.1*landed_left,'k--');
plot(0.1*landed_right, 'r--');

%% com trajectory
plot(com_trajectory')