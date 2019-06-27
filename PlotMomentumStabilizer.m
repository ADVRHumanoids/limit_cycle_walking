figure()

%% pitch
figure()
plot(imu_z_waist(1,:)'); hold on;
plot(imu_z_world(1,:)'); hold on;
figure()
plot(imu_orient(1,:)'); hold on;

%% roll
figure()
plot(imu_z_waist(2,:)'); plot(CoMStabilizer_zmp_measured(1:2,:)'); hold on;
plot(CoMStabilizer_zmp_ref(1,:)', '--'); hold on;


plot(imu_z_world(2,:)'); hold on;
figure()
plot(imu_orient(2,:)'); hold on;

figure()
plot(CoMStabilizer_zmp_measured(1:2,:)'); hold on;
plot(CoMStabilizer_zmp_ref(1:2,:)', '--'); hold on;
legend('x_{meas}', 'y_{meas}','x_{cmd}', 'y_{cmd}'); hold on;



plot(CoMStabilizer_com_pos_desired'); hold on;

%%

figure()
plot(FT_l_leg_ft_wrench(1,:)'); hold on;
plot(FT_r_leg_ft_wrench(1,:)'); hold on;

figure()
plot(FT_l_leg_ft_wrench(2,:)'); hold on;
plot(FT_r_leg_ft_wrench(2,:)'); hold on;

figure()
plot(FT_l_leg_ft_wrench(3,:)'); hold on;
plot(FT_r_leg_ft_wrench(3,:)'); hold on;

figure()
plot(FT_l_leg_ft_wrench(4,:)'); hold on;
plot(FT_r_leg_ft_wrench(4,:)'); hold on;

figure()
plot(FT_l_leg_ft_wrench(5,:)'); hold on;
plot(FT_r_leg_ft_wrench(5,:)'); hold on;

figure()
plot(FT_l_leg_ft_wrench(6,:)'); hold on;
plot(FT_r_leg_ft_wrench(6,:)'); hold on;
