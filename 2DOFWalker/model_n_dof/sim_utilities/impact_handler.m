function [q_After,q_dot_After] = impact_handler(q_Before,q_dot_Before,relabelingMatrices,fileName)
%% impact model + relabeling
% for the dynamic model:
% D*q_Ddot + C*q_dot + G = B*u + F_implusive,
% if I integrate over a small period of time I get 
% D*q_dot(t+) - D*q_dot(t-) = integral{from t- to t+}(F_impulsive)
% I discard C,G,B because are NOT impulsive term

% Also, my hypothesis is that after the impact there is NO rebound or slip.
% Basically this means that the cartesian velocity of the swing leg after
% the impact is zero.
% So: E2 * q_dot(t+) = 0;
% 
% My conditions are, then:
% D*q_dot(t+) - D*q_dot(t-) = F2;
% E2 * q_dot(t+) - 0;

% F2 is he projection of F_implusive to the joint space:
% F2 = E2' * F_implusive (E2 is the jacobian of the swing leg)

q_After = q_Before;
n_link = length(q_Before)-2;
%====================impact model===============
[D,~,~,E2] = calcDynMatrices(q_Before,q_dot_Before,fileName);
deltaF2 = -inv(E2 * inv(D) * E2.') * E2 * [eye(n_link); zeros(2,n_link)];
deltaqDotBar = inv(D) * E2.' * deltaF2 + [eye(n_link); zeros(2,n_link)];
%===================================
q_After(1:n_link) = relabelingMatrices.MatrixRelabel*q_Before(1:n_link) - relabelingMatrices.piMatrix;
%==================check relabeling========================
% q_dot_check1 = deltaqDotBar * q_dot_Before(1:n_link);
%===============================================
q_dot_After = deltaqDotBar * q_dot_Before(1:n_link); %impact model changes velocities
% q_dot_After = q_dot_Before; %without impact model
q_dot_After(1:n_link) = relabelingMatrices.MatrixRelabel*q_dot_After(1:n_link);
% ==================check relabeling=======================
% [~,~,~,E2] = calcDynMatrices(q_After,q_dot_After,fileName);
% q_dot_check2 = q_dot_After;
% q_dot_check2(end-1:end) = 0;
% p2_check2 = E2*q_dot_check2;
%===============================================