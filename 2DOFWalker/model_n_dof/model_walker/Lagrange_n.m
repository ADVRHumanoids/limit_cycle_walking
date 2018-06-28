function eqMotion = Lagrange_n(kinematics, generalizedVariables, robotData)



% if robotData.flagSym == 1
    K_terms = sym(zeros(robotData.n_link,1));
    P_terms = sym(zeros(robotData.n_link,1));
    rp_abs_fromOrigin = sym(zeros(3,1,robotData.n_link));
    
% else
    
%     K_terms = zeros(robotData.n_link,1);
%     P_terms = zeros(robotData.n_link,1);
%     rp_abs_fromOrigin = zeros(3,1,robotData.n_link);

% end

g = robotData.gravity;

qe =  generalizedVariables.qe;
qe_dot =  generalizedVariables.qe_dot;
qe_Ddot =  generalizedVariables.qe_Ddot;
z = generalizedVariables.qe(end-1:end);

parent_tree = kinematics.parent_tree;
w_abs = kinematics.angularVelocity_absolute;
rp_abs = kinematics.positionLink_absolute;
rc_abs = kinematics.positionCoM_absolute;
vc_abs = kinematics.velocityCoM_absolute;

m = robotData.mass;
I = robotData.inertia;



for i = 1:length(parent_tree)
    
    K_terms(i) =   1/2 * m(i) * vc_abs(:,:,i).'*vc_abs(:,:,i) ...
                 + 1/2 * w_abs(:,:,i).' * I(i) * w_abs(:,:,i);
end

K = sum(K_terms);


%for the potential energy

rp_abs_fromOrigin(:,:,1) = rp_abs(:,:,1);
for i = 2:length(parent_tree)
rp_abs_fromOrigin(:,:,i) = rp_abs_fromOrigin(:,:,parent_tree(i)) + rp_abs(:,:,i);
end


rp_abs_fromOrigin = cat(3,[0;0;0],rp_abs_fromOrigin);

for i = 1:length(parent_tree)
P_terms(i,:) = m(i) * g * (z(2) + rp_abs_fromOrigin(2,:,i) + rc_abs(2,:,i));
end

P = sum(P_terms);


L = K - P;

%==========================================================================
dL_q_dot = functionalDerivative(L,qe_dot);
dL_q_dot_dt = diff_t(dL_q_dot,[qe,qe_dot], [qe_dot, qe_Ddot]);
dL_q = functionalDerivative(L,qe);
eqMotion = dL_q_dot_dt - dL_q;
%==========================================================================
end