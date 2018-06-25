twoLinkWalkerModel;
n_link = 2;
alfa = 0;
                        %pos / vel / acc
startingParameters = [ pi/18,   0,    0;...  %q1    pi/18
                      -pi/18,   0,    0;...  %q2     -pi/18
                       3*pi/4,  0,    0;...  %q3     3*pi/4,
                       pi/4,    0,    0;...  %q4      pi/4,
%                      0,       0,    0;...  %q5
                       0,       0,    0;...  %z1
                       0,       0,    0];    %z2

         
q = startingParameters(1:n_link,1);
q_dot = startingParameters(1:n_link,2);
q_Ddot = startingParameters(1:n_link,3);


[Tp2, T_all] = twoLinkWalker.fkine(q);
Base = [0 0 0]; 
points = T_all * Base';
%link,axis,begin/end
Links(1,:,1) = Base;
Links(1,:,2) = points(:,1);
Links(2,:,1) = points(:,1);
Links(1,:,2) = points(:,2);



yLineTerrain = double(tan(alfa) * Links(n_link,1,2));
yLineTerrain_old = yLineTerrain;

alfa = 0;
j = 0;
F = zeros(length(q),1);
time = 0;
dt = 0.005; %0.001

while 1
     
j = j + 1;
time = (j-1)*dt;

D = twoLinkWalker.inertia(q);
C = twoLinkWalker.coriolis(q, q_dot);
G = twoLinkWalker.coriolis(q);


% [D,C,G] = updatateDynMatrices(symD,symC,symG,symbolicVar,numericVar);
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

yLineTerrain_old = yLineTerrain;
yLineTerrain = double(tan(alfa) * Links(n_link,1,2));
Links = simKinematics_n(q,parent_tree,robotData,Base);  %update kinematics

numericVar = cat(3,q,q_dot,q_Ddot);


%===========impact switch===========
if Links(n_link,2,2) <= yLineTerrain && Links_old(n_link,2,2) > yLineTerrain_old
% flag_plot = 1;
% 
%====================impact model===============
phi = Base + double(subs(symPhi,symbolicVar,numericVar));
set(p2plot,'xdata',phi(1),'ydata',phi(2));
E2 = double(subs(symE2,symbolicVar,numericVar));
deltaF2 = -inv(E2 * inv(D) * E2.') * E2 * [eye(n_link); zeros(2,n_link)];
deltaqDotBar = inv(D) * E2.' * deltaF2 + [eye(n_link); zeros(2,n_link)];
%===================================
q_old = q;
q(1) = -(pi -q_old(1) -q_old(2) -q_old(3) -q_old(4)); %- 2*alfa; 
q(2) = -q_old(4);
q(3) = -q_old(3);
q(4) = -q_old(2);
%==================check========================
q_dot_check1 = deltaqDotBar * q_dot(1:n_link);
%===============================================
% q_dot(1:n_link) = [eye(n_link) zeros(n_link,2)] * deltaqDotBar * q_dot(1:n_link); %q

q_dot = deltaqDotBar * q_dot(1:n_link);
q_dot(1:n_link) = MatrixVelocityRelabel*q_dot(1:n_link);
% ==================check=======================
numericVar = cat(3,q,q_dot,q_Ddot);

E2 = double(subs(symE2,symbolicVar,numericVar));
q_dot_check2 = q_dot;
q_dot_check2(5:6) = 0;
p2_check2 = E2*q_dot_check2;
%===============================================
Base = [Links(n_link,1,2);
        yLineTerrain];

numericVar = cat(3,q,q_dot,q_Ddot);   
phi = Base + double(subs(symPhi,symbolicVar,numericVar));
set(p2plot,'xdata',phi(1),'ydata',phi(2));

Links = simKinematics_n(q,parent_tree,robotData,Base); %update kinematics

% % F2 = deltaF2 * q_dot(1:2);
% % delete(handleQuiver); 
end
%===================================
%==========
%==========
update_plot
%==========
%==========
 end