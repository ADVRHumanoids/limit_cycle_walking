close all; clear all; clc;
addpath(genpath('/home/francesco/advr-superbuild/external/limit_cycle_walking'));
% addpath(genpath('2DOFWalker'));
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

robotData(1) = m1;
robotData(2) = m2;
robotData(3) = I1;
robotData(4) = I2;
robotData(5) = l1;
robotData(6) = l2;
robotData(7) = lc1;
robotData(8) = lc2;

%======================
kinematics_extended;
Lagrange_extended;
DynamicEquations = [simplify(First_eq), simplify(Second_eq), simplify(Third_eq), simplify(Fourth_eq)];
dynamics_extended;
impact_extended;
%======================
AngleSlope = -pi/10;%-pi/4; 

                      %pos / vel / acc
startingParameters = [pi/18,  0,    0,...  %q1
                      3*pi/4, 0,    0,...  %q2      3*pi/4,    50,    0,...  %q2
                      0,       0,    0,...  %z1
                      0,       0,    0];    %z2
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
symdeltaqDotBar = deltaqDotBar;



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
robotData = eval(robotData); 
Links = simKinematics(q,robotData,Base); %update kinematics
%=======
%=======
set_plot;
%=======
%=======
j = 0;
F = zeros(length(q),1);
time = 0;
dt = 0.005;

 while 1
     
j = j + 1;
time = (j-1)*dt;
    
[D,C,G] = updatateDynMatrices(symD,symC,symG,symbolicVar,numericVar);
q = double(q);
q_dot = double(q_dot);
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
q_old = q;
q_dot_old = q_dot;
Links_old = Links;

q_Ddot =  D \ (F - C*(q_dot) - G);
q_dot = q_dot + dt * q_Ddot;
q = q + dt * q_dot; 


for i  = 1:length(q)
    if q(i) >= 2*pi
        q(i) = q(i) - 2*pi;
    end
end

yLineTerrain = double(tan(alfa) * Links(2,1,2));
% E2 = double(subs(E2,symbolicVar,numericVar));

Links = simKinematics(q,robotData,Base); %update kinematics


numericVar = [q; q_dot; q_Ddot].';


%===========impact switch===========
if Links(2,2,2) <= yLineTerrain && Links_old(2,2,2) > yLineTerrain %* yLink2_old(2) < 0 
flag_plot = 1;

%===================================
% E2 = double(subs(E2,symbolicVar,numericVar));
deltaF2 = double(subs(deltaF2,symbolicVar,numericVar));
deltaqDotBar = double(subs(deltaqDotBar,symbolicVar,numericVar)); %velocity before relabeling
% deltaqDot = double(subs(deltaqDot,symbolicVar,numericVar)); %velocity after relabeling TODO
%===================================
% deltaqDotBar = double(subs(symdeltaqDotBar,symbolicVar,numericVar));
q(1:2) = deltaq * q(1:2)- [2*alfa;0]; %q_old(1:2)
q_dot(1:2) = [eye(nlink,2) zeros(nlink,2)] * deltaqDotBar * q_dot(1:2); %q_dot_old(1:2)
% q_dot(1:2) = [R zeros(nlink,2)] * qdotF(1:4);

Base = [Links(2,1,2);
        yLineTerrain];

Links = simKinematics(q,robotData,Base);

% F2 = deltaF2 * q_dot(1:2);
% delete(handleQuiver);
end
%===================================

%============plot F2=======================================================
% if flag_plot == 1
% figure(1); handleQuiver = quiver(Links(2,1,2),Links(2,2,2),F2(1),F2(2));
% flag_plot = 0;
% end
%==========================================================================
%==========
%==========
update_plot
%==========
%==========
 end

 % matA = [ D -E2.';
%      E2  zeros(2)]; 
% matB = [D * q; zeros(2,1)];
% qdotF = linsolve(matA,matB);  %[q_dot+ ; F2]