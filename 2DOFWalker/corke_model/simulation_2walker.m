clear all; clc;
Folder = cd;
addpath(genpath(fullfile(Folder, '..')));
% addpath(genpath('/home/francesco/Documents/MATLAB/Add-Ons/Toolboxes/Robotics Toolbox for MATLAB'));
% startup_rvc;
twoLinkWalkerModel;
n_link = 2;
n_link_extended = n_link + 2;
alfa = 0;
MatrixVelocityRelabel = inv(flip(tril(ones(n_link)))) *tril(ones(n_link));

                        %pos / vel / acc
startingParameters = [ 0,   0,    0;...  %z1    pi/18
                       0,   0,    0;...  %z2    pi/18
                       
                       0,   0,    0;...  %q1    pi/18
                       pi/4,   0,    0;...  %q2     -pi/18
                       0,   0,    0;...  %q3     3*pi/4
                       0,       0,    0];    %z2

         
q = startingParameters(1:n_link_extended,1);
q_dot = startingParameters(1:n_link_extended,2);
q_Ddot = startingParameters(1:n_link_extended,3);

% q(1) = pi/2 + startingParameters(1);
% q(2) = - startingParameters(2);
[Tp2, T_all] = twoLinkWalkerExt.fkine(q);
Base = [0 0 0];

for i = 3:n_link_extended
cartesian_pos(:,i-2) = T_all(i) * Base';
end

plotLinks = cat(2, Base', cartesian_pos);
plotLinks = plotLinks(2:end,:);

for i = 1:n_link
    for j = 1:n_link
Links(i,:,j) = plotLinks(:,i+j-1);
    end
end


yLineTerrain = double(tan(alfa) * Links(n_link,1,2));
yLineTerrain_old = yLineTerrain;

alfa = 0;
j = 0;
F = zeros(length(q),1);
time = 0;
dt = 0.005; %0.001

%~~~~~~~~~~~~
set_plot;
%~~~~~~~~~~~~
disp('push a button to continue'); pause;
while 1
     
j = j + 1;
time = (j-1)*dt;

D = twoLinkWalkerExt.inertia(q');
C = twoLinkWalkerExt.coriolis(q', q_dot');
G = twoLinkWalkerExt.gravload(q')';



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
q_old = q;
q_dot_old = q_dot;
Links_old = Links;

q_Ddot =  D \ (F - C*(q_dot) - G);
q_dot = q_dot + dt * q_Ddot;
q = q + dt * q_dot;
q(1:2) = 0;


for i  = 1:length(q)
    if q(i) >= 2*pi
        q(i) = q(i) - 2*pi;
    end
end

yLineTerrain_old = yLineTerrain;
yLineTerrain = double(tan(alfa) * Links(n_link,1,2));



%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
[Tp2, T_all] = twoLinkWalkerExt.fkine(q);
Base = [0 0 0];

for i = 3:n_link_extended
cartesian_pos(:,i-2) = T_all(i) * Base';
end

plotLinks = cat(2, Base', cartesian_pos);
plotLinks = plotLinks(2:end,:);

for i = 1:n_link
    for j = 1:n_link
Links(i,:,j) = plotLinks(:,i+j-1);
    end
end
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%===========impact switch===========
% if Links(n_link,2,2) <= yLineTerrain && Links_old(n_link,2,2) > yLineTerrain_old
% % flag_plot = 1;
% % 
% %====================impact model===============
% E2 = twoLinkWalkerExt.jacobe(q);
% deltaF2 = -inv(E2 * inv(D) * E2.') * E2 * [eye(n_link); zeros(2,n_link)];
% deltaqDotBar = inv(D) * E2.' * deltaF2 + [eye(n_link); zeros(2,n_link)];
% %===================================
% q_old = q;
% q(1) = -(pi -q_old(1) -q_old(2) -q_old(3) -q_old(4)); %- 2*alfa; 
% q(2) = -q_old(4);
% q(3) = -q_old(3);
% q(4) = -q_old(2);
% %==================check========================
% q_dot_check1 = deltaqDotBar * q_dot(1:n_link);
% %===============================================
% % q_dot(1:n_link) = [eye(n_link) zeros(n_link,2)] * deltaqDotBar * q_dot(1:n_link); %q
% 
% q_dot = deltaqDotBar * q_dot(1:n_link);
% q_dot(1:n_link) = MatrixVelocityRelabel*q_dot(1:n_link);
% % ==================check=======================
% numericVar = cat(3,q,q_dot,q_Ddot);
% 
% E2 = double(subs(symE2,symbolicVar,numericVar));
% q_dot_check2 = q_dot;
% q_dot_check2(5:6) = 0;
% p2_check2 = E2*q_dot_check2;
% %===============================================
% Base = [Links(n_link,1,2);
%         yLineTerrain];
% 
% numericVar = cat(3,q,q_dot,q_Ddot);   
% phi = Base + double(subs(symPhi,symbolicVar,numericVar));
% set(p2plot,'xdata',phi(1),'ydata',phi(2));
% 
% Links = simKinematics_n(q,parent_tree,robotData,Base); %update kinematics
% 
% % % F2 = deltaF2 * q_dot(1:2);
% % % delete(handleQuiver); 
% end
%===================================
%==========
%==========
update_plot
%==========
%==========
 end