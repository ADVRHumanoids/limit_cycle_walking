K_terms = sym(zeros(length(parent_tree),1));
for i = 1:length(parent_tree)
    
    K_terms(i) =   1/2 * m(i) * vc_abs(:,:,i).'*vc_abs(:,:,i) ...
                 + 1/2 * w_abs(:,:,i).' * I(i) * w_abs(:,:,i);
end

K = sum(K_terms);




% P =   m(1) * g * (z(2) + rc_rel(1) * cos(q(1) + alfa) ) ...
%     + m(2) * g * (z(2) + l1 * cos(q(1)) + rc_rel(2) * cos(q(1) + q(2) + alfa) );


% P_terms = sym(zeros(length(parent_tree),1));

%for the potential energy
rp_abs_fromOrigin = sym(zeros(3,1,length(parent_tree)));
rp_abs_fromOrigin(:,:,1) = rp_abs(:,:,1);
for i = 2:length(parent_tree)
rp_abs_fromOrigin(:,:,i) = rp_abs_fromOrigin(:,:,parent_tree(i)) + rp_abs(:,:,i);
end


rp_abs_fromOrigin = cat(3,[0;0;0],rp_abs_fromOrigin);

P_terms = sym(zeros(length(parent_tree),1));
for i = 1:length(parent_tree)
P_terms(i,:) = m(i) * g * (z(2) + rp_abs_fromOrigin(2,:,i) + rc_abs(2,:,i));
end

P = sum(P_terms);
% P =  m(1) * g * (z(2) + rc_rel(1) * cos(q(1) + alfa) ) ... 
%    + m(2) * g * (z(2) + rc_rel(2) * (cos(q(1) + q(2) + alfa))) + rp_rel(1) * cos(q(1)) ...
%    + m(3) * g * (z(2) + rc_rel(3) * (cos(q(1) + q(2) + q(3) + alfa))) + rp_rel(1) * cos(q(1)) + rp_rel(2) * cos(q(2))...

% a = m(1) * g * (z(2) + rc_abs(2,:,1)) + ...
% b = m(2) * g * (z(2) + rc_abs(2,:,2) + rp_abs(2,:,1)
% c = m(3) * g * (z(2) + rc_abs(2,:,3) + rp_abs(2,:,1) + rp_abs(2,:,2)


% P =   m1 * g * (z2(t) + lc1 * cos(q1(t) + alfa) ) ...
%       + m2 * g * (z2(t) + l1 * cos(q1(t)) + lc2 * cos(q1(t) + q2(t) + alfa) );
  
% P =   m1 * g * (lc1 * cos(q1(t))) ...
%       + m2 * g * (l1 * cos(q1(t)) + lc2 * cos(q1(t) + q2(t)));

L = K - P;
%==========================================================================
% dL_q_dot = functionalDerivative(L,q1_dot(t));
% dL1_dt = diff_t(dL_q_dot,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
% dL_q = functionalDerivative(L,q1(t));
% First_eq = dL1_dt - dL_q;
%==========================================================================
% dL_q2dot = functionalDerivative(L,q2_dot(t));
% dL2_dt = diff_t(dL_q2dot,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
% dL_q2 = functionalDerivative(L,q2(t));
% Second_eq = dL2_dt - dL_q2;
%==========================================================================
dL_q_dot = functionalDerivative(L,qe_dot);
dL_q_dot_dt = diff_t(dL_q_dot,[qe,qe_dot], [qe_dot, qe_Ddot]);
dL_q = functionalDerivative(L,qe);
eqMotion = dL_q_dot_dt - dL_q;
%==========================================================================
