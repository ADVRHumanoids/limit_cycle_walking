clear all; close all; clc
%==================simulation model of a 2 link walker extended========================

parent_tree = [0];
n_link = length(parent_tree);
flagSim = 1;


link_length = 1;
com_position = 0.5; %0.8
mass = 1; %0.3
inertia = [0.083 0.083];

link_length = link_length * ones(1,length(parent_tree));
com_position = [1-com_position, com_position * ones(1,length(parent_tree)-1)];
m = mass * ones(1,length(parent_tree));
% I = inertia * ones(1,length(parent_tree));
I = inertia;
g = 9.81;

%==========================================================================
robotData = struct('n_link',n_link,'link_length',link_length, 'com_position',com_position, 'mass',m, 'inertia',I,'gravity', g, 'flagSim', flagSim);

startingParameters = [0,   0,    0;...  %q1     %pi/18
                      0,  0,    0;...  %q2      3*pi/4,    50,    0,...  %q2
                      0,       0,    0;];  %z1
%                       0,       0,    0;
%                        0,       0,    0;
%                         0,       0,    0;
%                          0,       0,    0;
%                           0,       0,    0;
%                            0,       0,    0;];    %z2
%                   
%=======================starting kinematics================================
q = startingParameters(1:n_link+2,1);
q_dot = startingParameters(1:n_link+2,2);
q_Ddot = startingParameters(1:n_link+2,3);
Base = [0;0];
[Links,kinematics] = KinematicsLinks(q,parent_tree,robotData);
linkPosition = kinematics.linksPosition;

%===========================
fig1 = figure(1);
set(fig1,'position', [32   548   560   420]);
for i = 1:n_link
Link(i) = plot(0,0,'LineWidth',2); grid on; hold on;
linkPlot(i) = plot(0,0,'o');
end
lims = 5;
xlim([-lims lims]);
ylim([-lims lims]);
%===========================
for time = 1:0.01:10


% j = pi/2 *time/10;
ind = zeros(n_link,1);
ind(1) = pi*time/10;%
% ind(2) = 0;%pi*time/10;
% ind(3) = 0;%pi*time/10;
% ind(4) = pi/2*time/10;
% ind(5) = 0;%pi/2*time/10;
ind(2) = 0;%z1
ind(3) = 0;%z2

for i =  1:n_link
q(i) = ind(i);
end
[Links,kinematics] = KinematicsLinks(q,parent_tree,robotData);
linkPosition = kinematics.linksPosition;


for i = 1:n_link
set(Link(i),'xdata',Links(i,1,:),'ydata',Links(i,2,:));
set(linkPlot(i),'xdata',linkPosition(1,:,i),'ydata',linkPosition(2,:,i));
end    


drawnow;

end
