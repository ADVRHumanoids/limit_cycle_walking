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
syms alfa
syms m I l lc

nlink = 2;

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
kinematics_extended;
Lagrange_extended;
DynamicEquations = [simplify(First_eq), simplify(Second_eq), simplify(Third_eq), simplify(Fourth_eq)];
dynamics_extended;
impact_extended;
%======================
AngleSlope = -pi/4; 

                      %pos / vel / acc
startingParameters = [pi/18,   0,    0,...  %q1
                      pi/4,     0,    0,...  %q2
                      0,      0,    0,...  %z1
                      0,      0,    0];    %z2
%======================pi/18
q1(t) = subs(q1(t),q1,startingParameters(1));
q1_dot(t) = subs(q1_dot(t),q1_dot,startingParameters(2));
q1_Ddot(t) = subs(q1_Ddot(t),q1_Ddot,startingParameters(3));

q2(t) = subs(q2(t),q2,startingParameters(4));
q2_dot(t) = subs(q2_dot(t),q2_dot,startingParameters(5));
q2_Ddot(t) = subs(q2_Ddot(t),q2_Ddot,startingParameters(6));

z1(t) = subs(z1(t),z1,startingParameters(7));
z1_dot(t) = subs(z1_dot(t),z1_dot,startingParameters(8));
z1_Ddot(t) = subs(z1_Ddot(t),z1_Ddot,startingParameters(9));

z2(t) = subs(z2(t),z2,startingParameters(10));
z2_dot(t) = subs(z2_dot(t),z2_dot,startingParameters(11));
z2_Ddot(t) = subs(z2_Ddot(t),z2_Ddot,startingParameters(12));


%======================
symbolicVar = [GeneralizedCoordinates,...
               d_GeneralizedCoordinates,...
               Dd_GeneralizedCoordinates];
numericVar = [q1(t), q2(t),z1(t),z2(t),...
             q1_dot(t), q2_dot(t),z1_dot(t),z2_dot(t),...
             q1_Ddot(t), q2_Ddot(t),z1_Ddot(t),z2_Ddot(t)];

     %pi/6 
G = subs(G, alfa, double(subs(alfa,AngleSlope)));
alfa = double(subs(alfa,AngleSlope));
symD = D;
symC = C;
symG = G;
symdeltaq = deltaq;
symdeltaqDot = deltaqDot;




q = double([q1(t); 
     q2(t);
     z1(t);
     z2(t)]);
 
q_dot = double([q1_dot(t); 
         q2_dot(t);
         z1_dot(t);
         z2_dot(t)]);

q_Ddot = double([q1_Ddot(t); 
          q2_Ddot(t);
          z1_Ddot(t);
          z2_Ddot(t)]);
         



Origin = [0;0];
Base = [0;0];
%=======================starting kinematics================================

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

xLink1 = Base(1) + [Origin(1) rp1_0(1)]; %
yLink1 = Base(2) + [Origin(2) rp1_0(2)];

xLink2 = xLink1(2) + [Origin(1) rp2_0(1)]; %
yLink2 = yLink1(2) + [Origin(2) rp2_0(2)];


%==========================================================================
fig1 = figure(1);
% set(fig1,'Position',[2400, 400,560,420])

Link1 = plot(0,0,'LineWidth',2); grid on; hold on;
Link2 = plot(0,0,'LineWidth',2); grid on; hold on;

xlim([-3 3]);
ylim([-3 3]);

xLineTerrain = xlim;
yLineTerrain = tan(alfa) * (xLineTerrain);
LineTerrain = plot(xLineTerrain,yLineTerrain);
%===========================================
set(Link1,'xdata',xLink1,'ydata',yLink1);
set(Link2,'xdata',xLink2,'ydata',yLink2);
%===========================================
fig2 = figure(2);
% set(fig2,'Position',[3000, 400,560,420])
for loop = 1:size(q,1)
plot_q(loop) = plot(0,0); hold on;
xlim([0 100]);
ylim([-2 10]);
end

disp('push a button to continue'); pause;
flag = 0;
time_record = 0;
q_record = zeros(length(q),1);

j = 0;
F = zeros(length(q),1);
time = 0;
dt = 0.005;

 while 1
     
j = j + 1;
time = (j-1)*dt;
    
    
[D,C,G] = updatateDynMatrices(symD,symC,symG,symbolicVar,numericVar);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`

q = double(q);

q_dot = double(q_dot);
% q_Ddot = double(q_Ddot);
q_old = q;
q_dot_old = q_dot;

q_Ddot =  D \ (F - C*(q_dot) - G);
q_dot = q_dot + dt * q_Ddot;
q = q + dt * q_dot; 


for i  = 1:length(q)
    if q(i) >= 2*pi
        q(i) = q(i) - 2*pi;
    end
end

yLineTerrain = tan(alfa) * xLink2(2);
%===========impact switch===========
if yLink2(2) <= yLineTerrain && flag == 0
flag = 1;

deltaqDot = double(subs(symdeltaqDot,symbolicVar,numericVar));
q(1:2) = deltaq * q_old(1:2) - [2*alfa;0];
q_dot(1:2) = deltaqDot * q_dot_old(1:2);

Base = [xLink2(2);
        yLineTerrain];
end
%===================================

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


% Base = [q(3);q(4)]; %no fixed base
xLink1 = Base(1) + [Origin(1) rp1_0(1)]; %
yLink1 = Base(2) + [Origin(2) rp1_0(2)];

xLink2 = xLink1(2) + [Origin(1) rp2_0(1)]; %
yLink2 = yLink1(2) + [Origin(2) rp2_0(2)];

numericVar = [q; q_dot; q_Ddot].';


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
    time_record = [time_record(:,2:end) time];
    q_record = [q_record(:,2:end) q];
end

for loop = 1:size(q,1)
set(plot_q(loop),'xdata',time_record,'ydata',q_record(loop,:));
end

drawnow;
end