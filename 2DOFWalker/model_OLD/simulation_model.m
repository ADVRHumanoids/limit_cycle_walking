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
syms I1 I2
syms g

syms m I l lc



GeneralizedCoordinates = [q1(t),q2(t)];
first_derivative_names = [q1_dot(t), q2_dot(t)];
second_derivative_names = [q1_Ddot(t), q2_Ddot(t)];
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
kinematics;
Lagrange;
DynamicEquations = [simplify(First_eq), simplify(Second_eq)];
dynamics;
%======================



%======================
q1(t) = subs(q1(t),q1,0);
q1_dot(t) = subs(q1_dot(t),q1_dot,0);
q1_Ddot(t) = subs(q1_Ddot(t),q1_Ddot,0);

q2(t) = subs(q2(t),q2,pi/4);
q2_dot(t) = subs(q2_dot(t),q2_dot,0);
q2_Ddot(t) = subs(q2_Ddot(t),q2_Ddot,0);

%======================
symbolicVar = [GeneralizedCoordinates,...
               d_GeneralizedCoordinates,...
               Dd_GeneralizedCoordinates];
numericVar = [q1(t), q2(t),...
             q1_dot(t), q2_dot(t),...
             q1_Ddot(t), q2_Ddot(t)];

symD = D;
symC = C;
symG = G;



q = [q1(t); 
     q2(t)];
 
q_dot = [q1_dot(t); 
         q2_dot(t)];

q_Ddot = [q1_Ddot(t); 
          q2_Ddot(t)];
         


time_record = 0;
q_record = zeros(length(q),1);

j = 0;
F = zeros(length(q),1);
time = 0;
dt = 0.05;

Origin = [0;0];
Base = [0;0];


figure(1);  
Link1 = plot(0,0,'LineWidth',2); grid on; hold on;
Link2 = plot(0,0,'LineWidth',2); grid on; hold on;
xlim([-3 3]);
ylim([-3 3]);

figure(2);
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


R1_0 = [sin(q(1)) -cos(q(1)) 0;
        cos(q(1)) sin(q(1))  0;
        0          0           1];

R2_1 = [ cos(q(2)) sin(q(2)) 0;
        -sin(q(2)) cos(q(2)) 0;
        0           0          1];
    
R2_0 = R1_0*R2_1;

rp1_1 = [l1; 0; 0];
rp2_2 = [l2; 0; 0];
rp1_0 = R1_0 * rp1_1;
rp2_0 = R2_0 * rp2_2;

%======================


% kinematics_grizzle_symm;

xLink1 = [Base(1) rp1_0(1)]; %
yLink1 = [Base(2) rp1_0(2)];

xLink2 = rp1_0(1) + [Base(1) rp2_0(1)]; %
yLink2 = rp1_0(2) + [Base(2) rp2_0(2)];

set(Link1,'xdata',xLink1,'ydata',yLink1);
set(Link2,'xdata',xLink2,'ydata',yLink2);
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