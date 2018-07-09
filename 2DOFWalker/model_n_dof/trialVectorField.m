%% plot Zero Dynamics
% clear all;
% clc;
%======================real parameters=====================================
syms y(t)
link_length = 1;
com_position = 0.8; %0.8
mass = 0.3; %0.3
inertia = 0.03;

l1 = link_length;
l2 = link_length;
m1 = mass;
m2 = mass;
lc1 = 1-com_position;
lc2 = com_position;
I1 = inertia;
I2 = inertia;
g = 9.81;



eqn = (2*g*sin(y)*(l1*m2 + lc1*m1 + lc2*m2))/(2*m2*l1^2 + 3*m2*l1*lc2 + 2*m1*lc1^2 + m2*lc2^2 + 2*I1 + I2) == -diff(y,2);
% eqn = (g*sin(y)*(l1*m2 + lc1*m1 + lc2*m2))/(m2*l1^2 + 2*m2*l1*lc2 + m1*lc1^2 + m2*lc2^2 + I1 + I2) == -diff(y,2);

V = odeToVectorField(eqn); %try to make this work for QUIVER!


% M = matlabFunction(V,'vars', {'t','Y'});
% interval = [0 20];
% y0 = [1 1];
% ySol = ode45(M,interval,y0);
% tValues = linspace(0,20,100);
% yValues = deval(ySol,tValues,1);
% plot(tValues,yValues)


syms x y
u = x;
% v =  -(327*sin(y))/58;
v = -(1962*sin(y))/271;
limits = [-10:0.5:10];



startingPoints= [0 1;];
%                  1 0;...
%                  1 1;...
%                  2 0;...
%                  -3 2;];
h = VectorField(u,v, limits, 'streamline','startingPoints', startingPoints, 'normalize','scale',0.6);
% h = VectorField(u,v, limits, 'normalize','scale',0.6, 'streamline','startingPoints', startingPoints);
% VectorField(limits, u,v, 'normalize'); %'startingPoints', startingPoints

