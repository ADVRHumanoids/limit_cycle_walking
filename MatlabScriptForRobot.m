%figure(1);
%plot(foot_trajectory');

figure(2);
plot(zmp');
hold on; plot(zmp_ref', 'r--');

figure(3);
plot(landed_left);
hold on; plot(landed_right, 'r--');

figure(4);
hold on; plot(abs(ft_right(3,:))', 'r--')
plot(abs(ft_left(3,:))')

figure(5);
plot(impact_detector');
hold on;plot(1 +foot_trajectory(3,:)','r');

hold on; plot(zmp_ref')
plot(zmp')
hold on;plot(1 +foot_trajectory(3,:)','r');


hold on; plot(zmp_ref')
plot(0.1*landed_left,'k--');
 plot(0.1*landed_right, 'r--');
 
 plot(com_trajectory')