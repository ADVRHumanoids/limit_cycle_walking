m = sym('m',[length(parent_tree),1]);
I = sym('I',[length(parent_tree),1]);

K_terms = sym(zeros(length(parent_tree),1));
for i = 1:length(parent_tree)
    
    K_terms(i) =   1/2 * m(i) * vc_abs(:,:,i).'*vc_abs(:,:,i) ...
                 + 1/2 * w_abs(:,:,i).' * I(i) * w_abs(:,:,i);
end

K = sum(K_terms);




% P =   m(1) * g * (z(2) + rc_rel(1) * cos(q(1) + alfa) ) ...
%     + m(2) * g * (z(2) + l1 * cos(q(1)) + rc_rel(2) * cos(q(1) + q(2) + alfa) );

%angles stack
% angle_matrix_stack = sym(zeros(length(parent_tree),1));
% angle_matrix_stack(1) = q(1);
% for i = 2:length(parent_tree)
%     angle_matrix_stack(i) = q(i) + angle_matrix_stack(i-1);
% end

%full link terms
p_full_link_term = sym(zeros(length(parent_tree),1));
for i = 1:length(parent_tree)
p_full_link_term(i) = rp_rel(1,i) * cos(q(i));
end

%link stack
link_stack = sym(zeros(length(parent_tree),1));
link_stack(1) = 0;
for i = 2:length(parent_tree)
    link_stack(2) = link_stack(parent_tree(2)) + p_full_link_term(parent_tree(2));
    link_stack(3) = link_stack(parent_tree(3)) + p_full_link_term(parent_tree(3));
    link_stack(4) = link_stack(parent_tree(4)) + p_full_link_term(parent_tree(4));
    link_stack(5) = link_stack(parent_tree(5)) + p_full_link_term(parent_tree(5));
end

%mass link
p_last_link_term = sym(zeros(length(parent_tree),1));
for i = 1:length(parent_tree)
p_last_link_term(i) = rc_rel(1,i) * cos(w_abs(i) + alfa);
end

P_terms = sym(zeros(length(parent_tree),1));
P_terms(1) = m(1) * g * p_last_link_term(1);
for i = 2:length(parent_tree)
P_terms(i) = m(i) * g * (link_stack(i) + p_last_link_term(i));
end

% P =  m(1) * g * (z(2) + rc_rel(1) * cos(q(1) + alfa) ) ... 
%    + m(2) * g * (z(2) + rc_rel(2) * (cos(q(1) + q(2) + alfa))) + rp_rel(1) * cos(q(1)) ...
%    + m(3) * g * (z(2) + rc_rel(3) * (cos(q(1) + q(2) + q(3) + alfa))) + rp_rel(1) * cos(q(1)) + rp_rel(2) * cos(q(2))...

% m(1) * g * (z(2) + rc_abs(2,:,1)) + ...
% m(2) * g * (z(2) + rc_abs(2,:,2) + rp_rel(1) * cos(q(1)) +...
% m(3) * g * (z(2) + rc_abs(2,:,3) + rp_rel(1) * cos(q(1)) + rp_rel(2) * cos(q(2)) +...

P = sum(P_terms);

% P =   m1 * g * (lc1 * cos(q1(t))) ...
%       + m2 * g * (l1 * cos(q1(t)) + lc2 * cos(q1(t) + q2(t)));

L = K - P;
%==========================================================================
dL_q1dot = functionalDerivative(L,q1_dot(t));
dL1_dt = diff_t(dL_q1dot,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
dL_q1 = functionalDerivative(L,q1(t));
First_eq = dL1_dt - dL_q1;
%==========================================================================
dL_q2dot = functionalDerivative(L,q2_dot(t));
dL2_dt = diff_t(dL_q2dot,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
dL_q2 = functionalDerivative(L,q2(t));
Second_eq = dL2_dt - dL_q2;
%==========================================================================






%==========================================================================
dL_z1dot = functionalDerivative(L,z1_dot(t));
dL3_dt = diff_t(dL_z1dot,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
dL_z1 = functionalDerivative(L,z1(t));
Third_eq = dL3_dt - dL_z1;
%==========================================================================
dL_z2dot = functionalDerivative(L,z2_dot(t));
dL4_dt = diff_t(dL_z2dot,[GeneralizedCoordinates,d_GeneralizedCoordinates], [first_derivative_names, second_derivative_names]);
dL_z2 = functionalDerivative(L,z2(t));
Fourth_eq = dL4_dt - dL_z2;