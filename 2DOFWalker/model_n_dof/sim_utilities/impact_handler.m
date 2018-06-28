function [q_After,q_dot_After] = impact_handler(q_Before,q_dot_Before)

q_After = q_Before;
n_link = length(q_Before)-2;
% ===============relabeling matrices==============
piMatrix = pi*[1; zeros(n_link-1,1)];
MatrixRelabel = inv(flip(tril(ones(n_link)))) *tril(ones(n_link));
%====================impact model===============
[D,~,~] = calcDynMatrices(q_Before,q_dot_Before);
E2 = calcJacobianMatrix(q_Before);
deltaF2 = -inv(E2 * inv(D) * E2.') * E2 * [eye(n_link); zeros(2,n_link)];
deltaqDotBar = inv(D) * E2.' * deltaF2 + [eye(n_link); zeros(2,n_link)];
%===================================
q_After(1:n_link) = MatrixRelabel*q_Before(1:n_link) - piMatrix;
%==================check========================
% q_dot_check1 = deltaqDotBar * q_dot_Before(1:n_link);
%===============================================
q_dot_After = deltaqDotBar * q_dot_Before(1:n_link);
q_dot_After(1:n_link) = MatrixRelabel*q_dot_After(1:n_link);
% ==================check=======================
% % numericVar = cat(3,q,q_dot,q_Ddot);
% E2 = calcJacobianMatrix(q_After);
% q_dot_check2 = q_dot_After;
% q_dot_check2(end-1:end) = 0;
% p2_check2 = E2*q_dot_check2;
%===============================================