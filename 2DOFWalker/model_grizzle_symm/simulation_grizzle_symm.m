% close all; clear all; clc;
% 
% %==================simulation model of a 2 link walker extended========================
% syms m1 m2 
% syms vc1_0 vc2_0
% syms w01_0 w12_0 w12_1
% syms R1_0 R2_1 R2_0
% syms rp1_0 rc1_0
% syms rp2_0 rc2_0 
% syms rp1_1 rc1_1 
% syms rp2_2 rc2_2
% syms l1 lc1 l2 lc2
% syms q1(t) q2(t) q1_dot(t) q2_dot(t) q1_Ddot(t) q2_Ddot(t)
% syms z1(t) z2(t) z1_dot(t) z2_dot(t) z1_Ddot(t) z2_Ddot(t)
% syms I1 I2
% syms g
% syms theta1 theta2 theta3 theta4 theta5
% 
% syms m I l lc
% 
% 
% 
% GeneralizedCoordinates = [q1(t),q2(t),z1(t),z2(t)];
% first_derivative_names = [q1_dot(t), q2_dot(t), z1_dot(t), z2_dot(t)];
% second_derivative_names = [q1_Ddot(t), q2_Ddot(t), z1_Ddot(t), z2_Ddot(t)];
% dimGC = length(GeneralizedCoordinates);
% 
% d_GeneralizedCoordinates = diff_t(GeneralizedCoordinates, GeneralizedCoordinates, first_derivative_names);
% Dd_GeneralizedCoordinates = diff_t(d_GeneralizedCoordinates, d_GeneralizedCoordinates, second_derivative_names);  
% 
% 
% l = subs(l,1);
% lc = subs(lc,0.8);
% m = subs(m,0.3);
% I = subs(I,0.03);
% g = subs(g,9.81);
% %======================
% m1 = m;
% m2 = m;
% I1 = I;
% I2 = I;
% l1 = l;
% l2 = l;
% lc1 = l-lc;
% lc2 = lc;
% %======================
% kinematics_grizzle_symm;
% Lagrange_grizzle_symm;
% DynamicEquations = [simplify(First_eq), simplify(Second_eq), simplify(Third_eq), simplify(Fourth_eq)];
% dynamics_grizzle_symm;
% %======================
% 
% 
% 
% %======================
% q1(t) = subs(q1(t),q1,pi);
% q1_dot(t) = subs(q1_dot(t),q1_dot,0);
% q1_Ddot(t) = subs(q1_Ddot(t),q1_Ddot,0);
% 
% q2(t) = subs(q2(t),q2,pi);
% q2_dot(t) = subs(q2_dot(t),q2_dot,0);
% q2_Ddot(t) = subs(q2_Ddot(t),q2_Ddot,0);
% 
% z1(t) = subs(z1(t),z1,0);
% z1_dot(t) = subs(z1_dot(t),z1_dot,0);
% z1_Ddot(t) = subs(z1_Ddot(t),z1_Ddot,0);
% 
% z2(t) = subs(z2(t),z2,0);
% z2_dot(t) = subs(z2_dot(t),z2_dot,0);
% z2_Ddot(t) = subs(z2_Ddot(t),z2_Ddot,0);
% %======================
% symbolicVar = [GeneralizedCoordinates,...
%                d_GeneralizedCoordinates,...
%                Dd_GeneralizedCoordinates];
% numericVar = [q1(t), q2(t),z1(t),z2(t),...
%              q1_dot(t), q2_dot(t),z1_dot(t),z2_dot(t),...
%              q1_Ddot(t), q2_Ddot(t),z1_Ddot(t),z2_Ddot(t)];
% 
% symD = D;
% symC = C;
% symG = G;
% 
% 
% 
% q = [q1(t); 
%      q2(t);
%      z1(t);
%      z2(t)];
%  
% q_dot = [q1_dot(t); 
%          q2_dot(t);
%          z1_dot(t);
%          z2_dot(t)];
% 
% q_Ddot = [q1_Ddot(t); 
%           q2_Ddot(t);
%           z1_Ddot(t);
%           z2_Ddot(t)];
%          
% 
% 
% 
% 
% time_record = 0;
% q_record = zeros(4,1);
% 
% j = 0;
% F = zeros(4,1);
% time = 0;
% dt = 0.01;
% 
% Origin = [0;0];
% Base = [0;0];
% 
% 
% figure(1);  
% Link1 = plot(0,0,'LineWidth',2); grid on; hold on;
% Link2 = plot(0,0,'LineWidth',2); grid on; hold on;
% xlim([-3 3]);
% ylim([-3 3]);
% 
% figure(2);
% for loop = 1:size(q,1)
% plot_q(loop) = plot(0,0); hold on;
% xlim([0 10]);
% ylim([-10 10]);
% end
% 
% disp('push a button'); pause;
% 
%  while 1
%      
% j = j + 1;
% time = (j-1)*dt;
%     
%     
% [D,C,G] = updatateDynMatrices(symD,symC,symG,symbolicVar,numericVar);
% 
% q = double(q);
% q_dot = double(q_dot);
% % q_Ddot = double(q_Ddot);
% 
% q_Ddot =  D \ (F - C*(q_dot) - G);
% q_dot = q_dot + dt * q_Ddot;
% q = q + dt * q_dot; 
% 
% numericVar = [q; q_dot; q_Ddot].';
% 
% for i  = 1:length(q)
%     if q(i) >= 2*pi
%         q(i) = q(i) - 2*pi;
%     end
% end
% %======================
% 
% % q1(t) = q(1);
% % q1_dot(t) = q_dot(1);
% % q1_Ddot(t) = q_Ddot(1);
% % 
% % q2(t) = q(2);
% % q2_dot(t) = q_dot(2);
% % q2_Ddot(t) = q_Ddot(2);
% % 
% % z1(t) = q(3);
% % z1_dot(t) = q_dot(3);
% % z1_Ddot(t) = q_Ddot(3);
% % 
% % z2(t) = q(4);
% % z2_dot(t) = q_dot(4);
% % z2_Ddot(t) = q_Ddot(4);
% %======================
% %======================
% R2_0 = [cos(q(2)) -sin(q(2))  0;     
%         sin(q(2))  cos(q(2))  0; 
%         0                0      1];
% 
% R1_2 = [-cos(q(1))  sin(q(1))  0;
%          sin(q(1))  cos(q(1))  0;
%         0            0           1]; 
% 
% R1_0 = R2_0*R1_2;
% 
% rp1_1 = [l1; 0; 0];
% rp2_2 = [l2; 0; 0];
% 
% rp1_0 = R1_0 * rp1_1;
% rp2_0 = R2_0 * rp2_2;
% 
% %======================
% %======================
% 
% 
% 
% xLink2 = Base(1) + [Origin(1) rp2_0(1)];
% yLink2 = Base(2) + [Origin(2) rp2_0(2)];
% 
% xLink1 = rp2_0(1) + [Origin(1) rp1_0(1)];
% yLink1 = rp2_0(2) + [Origin(2) rp1_0(2)];
%      
% set(Link1,'xdata',yLink1,'ydata',xLink1);
% set(Link2,'xdata',yLink2,'ydata',xLink2);
% %       if t>1
% %           F = 10;
% %       end
% %        if t>2
% %           F = 0;
% %       end    
% time_record = sym([time_record time]);
% q_record = [q_record q];
% 
% for loop = 1:size(q,1)
% set(plot_q(loop),'xdata',time_record,'ydata',q_record(loop,:));
% end
% 
% drawnow;
% end
% 
% % 
% % q1(t) = subs(q1(t),q1,q(1));
% % q1_dot(t) = subs(q1_dot(t),q1_dot,q_dot(1));
% 
% % q1_Ddot(t) = subs(q1_Ddot(t),q1_Ddot,q_Ddot(1));
% % 
% % q2(t) = subs(q2(t),q2,q(2));
% % q2_dot(t) = subs(q2_dot(t),q2_dot,q_dot(2));
% % q2_Ddot(t) = subs(q2_Ddot(t),q2_Ddot,q_Ddot(2));
% % 
% % z1(t) = subs(z1(t),z1,q(3));
% % z1_dot(t) = subs(z1_dot(t),z1_dot,q_dot(3));
% % z1_Ddot(t) = subs(z1_Ddot(t),z1_Ddot,q_Ddot(3));
% % 
% % z2(t) = subs(z2(t),z2,q(4));
% % z2_dot(t) = subs(z2_dot(t),z2_dot,q_dot(4));
% % z2_Ddot(t) = subs(z2_Ddot(t),z2_Ddot,q_Ddot(4));


close all; clear all; clc;

%==================simulation model of a 2 link walker extended========================
syms m1 m2 
syms vc1_0 vc2_0
syms w01_0 w12_0 w12_1
syms R1_0 R2_1 R2_0
syms rp1_0 rc1_0
syms rp2_0 rc2_0 
syms rp1_1 rc1_1 
syms rp2_2 rc2_2
syms l1 lc1 l2 lc2
syms q1(t) q2(t) q1_dot(t) q2_dot(t) q1_Ddot(t) q2_Ddot(t)
syms z1(t) z2(t) z1_dot(t) z2_dot(t) z1_Ddot(t) z2_Ddot(t)
syms I1 I2
syms g

syms m I l lc



GeneralizedCoordinates = [q1(t),q2(t),z1(t),z2(t)];
first_derivative_names = [q1_dot(t), q2_dot(t), z1_dot(t), z2_dot(t)];
second_derivative_names = [q1_Ddot(t), q2_Ddot(t), z1_Ddot(t), z2_Ddot(t)];
dimGC = length(GeneralizedCoordinates);

d_GeneralizedCoordinates = diff_t(GeneralizedCoordinates, GeneralizedCoordinates, first_derivative_names);
Dd_GeneralizedCoordinates = diff_t(d_GeneralizedCoordinates, d_GeneralizedCoordinates, second_derivative_names);  

l = subs(l,1);
lc = subs(lc,0.8);
m = subs(m,0.3);
I = subs(I,0.03);
g = subs(g,9.81);
%======================
m1 = m;
m2 = m;
I1 = I;
I2 = I;
l1 = l;
l2 = l;
lc1 = l-lc;
lc2 = lc;
%======================
kinematics_grizzle_symm;
Lagrange_grizzle_symm;
DynamicEquations = [simplify(First_eq), simplify(Second_eq), simplify(Third_eq), simplify(Fourth_eq)];
dynamics_grizzle_symm;
%======================
q1(t) = subs(q1(t),q1,pi);
q1_dot(t) = subs(q1_dot(t),q1_dot,0);
q1_Ddot(t) = subs(q1_Ddot(t),q1_Ddot,0);

q2(t) = subs(q2(t),q2,0);
q2_dot(t) = subs(q2_dot(t),q2_dot,0);
q2_Ddot(t) = subs(q2_Ddot(t),q2_Ddot,0);

z1(t) = subs(z1(t),z1,0);
z1_dot(t) = subs(z1_dot(t),z1_dot,0);
z1_Ddot(t) = subs(z1_Ddot(t),z1_Ddot,0);

z2(t) = subs(z2(t),z2,0);
z2_dot(t) = subs(z2_dot(t),z2_dot,0);
z2_Ddot(t) = subs(z2_Ddot(t),z2_Ddot,0);


%======================
symbolicVar = [GeneralizedCoordinates,...
               d_GeneralizedCoordinates,...
               Dd_GeneralizedCoordinates];
numericVar = [q1(t), q2(t),z1(t),z2(t),...
             q1_dot(t), q2_dot(t),z1_dot(t),z2_dot(t),...
             q1_Ddot(t), q2_Ddot(t),z1_Ddot(t),z2_Ddot(t)];

symD = D;
symC = C;
symG = G;



q = [q1(t); 
     q2(t);
     z1(t);
     z2(t)];
 
q_dot = [q1_dot(t); 
         q2_dot(t);
         z1_dot(t);
         z2_dot(t)];

q_Ddot = [q1_Ddot(t); 
          q2_Ddot(t);
          z1_Ddot(t);
          z2_Ddot(t)];
         


time_record = 0;
q_record = zeros(length(q),1);

j = 0;
F = zeros(length(q),1);
time = 0;
dt = 0.05;

Origin = [0;0];
Base = [0;0];


fig1 = figure(1);
set(fig1,'Position',[2400, 400,560,420])

Link1 = plot(0,0,'LineWidth',2); grid on; hold on;
Link2 = plot(0,0,'LineWidth',2); grid on; hold on;
xlim([-3 3]);
ylim([-3 3]);


fig2 = figure(2);
set(fig2,'Position',[3000, 400,560,420])

for loop = 1:size(q,1)
plot_q(loop) = plot(0,0); hold on;
xlim([0 100]);
ylim([-2 10]);
end

disp('push a button'); pause;

 while 1
     
j = j + 1;
time = (j-1)*dt;
    
    
[D,C,G] = updatateDynMatrices(symD,symC,symG,symbolicVar,numericVar);

q = double(q);
q_dot = double(q_dot);
% q_Ddot = double(q_Ddot);

q_Ddot =  D \ (F - C*(q_dot) - G);
q_dot = q_dot + dt * q_Ddot;
q = q + dt * q_dot; 

numericVar = [q; q_dot; q_Ddot].';

for i  = 1:length(q)
    if q(i) >= 2*pi
        q(i) = q(i) - 2*pi;
    end
end


R2_0 = [cos(q(2)) -sin(q(2))  0;   
        sin(q(2))  cos(q(2))  0; 
        0                0      1];
    
R1_2 = [-cos(q(1))  sin(q(1))  0;
         sin(q(1))  cos(q(1))  0;
        0            0           1]; 
    
R1_0 = R2_0*R1_2;

rp1_1 = [l1; 0; 0];
rp2_2 = [l2; 0; 0];
rp1_0 = R1_0 * rp1_1;
rp2_0 = R2_0 * rp2_2;
%======================

% Base = [q(3);q(4)]; %no fixed base


 xLink2 = Base(1) + [Origin(1) rp2_0(1)];
 yLink2 = Base(2) + [Origin(2) rp2_0(2)];

 xLink1 = xLink2(2) + [Origin(1) rp1_0(1)];
 yLink1 = yLink2(2) + [Origin(2) rp1_0(2)];

set(Link1,'xdata',yLink1,'ydata',xLink1);
set(Link2,'xdata',yLink2,'ydata',xLink2);
%       if t>1
%           F = 10;
%       end
%        if t>2
%           F = 0;
%       end
buffer = 1000;
time_record = sym([time_record time]);
q_record = [q_record q];

if size(time_record,2) >= buffer
    time_record = sym([time_record(:,2:end) time]);
    q_record = [q_record(:,2:end) q];
end

for loop = 1:size(q,1)
set(plot_q(loop),'xdata',time_record,'ydata',q_record(loop,:));
end

drawnow;
end