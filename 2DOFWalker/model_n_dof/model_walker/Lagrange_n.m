K_terms = sym(zeros(length(parent_tree),1));
for i = 1:length(parent_tree)
    
    K_terms(i) =   1/2 * m(i) * vc_abs(:,:,i).'*vc_abs(:,:,i) ...
                 + 1/2 * w_abs(:,:,i).' * I(i) * w_abs(:,:,i);
end

K = sum(K_terms);


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


L = K - P;

%==========================================================================
dL_q_dot = functionalDerivative(L,qe_dot);
dL_q_dot_dt = diff_t(dL_q_dot,[qe,qe_dot], [qe_dot, qe_Ddot]);
dL_q = functionalDerivative(L,qe);
eqMotion = dL_q_dot_dt - dL_q;
%==========================================================================
