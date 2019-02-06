%figure(1);
%plot(foot_trajectory');

figure(2);
plot(zmp');
hold on; plot(zmp_ref', 'r--');

figure(3);
plot(landed_left);
hold on; plot(landed_rigth, 'r--');

figure(4);
hold on; plot(abs(ft_rigth(3,:))', 'r--')
plot(abs(ft_left(3,:))')

figure(5);
plot(impact_detector');
hold on;plot(foot_trajectory(3,:)','r');