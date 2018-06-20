%=============================impact model=================================
phi = sum(reshape(rp_abs,3,length(parent_tree)),2);
phi = phi(1:2);

p2 = z.' + phi(1:2);

E2 = sym(zeros(size(phi,1), dim_qe));

for j = 1:size(phi,1)
  for i = 1:dim_qe
        E2(j,i) = functionalDerivative(p2(j),qe(i));
  end
end 

%too slow
% deltaF2 = -inv(E2 * inv(D) * E2.') * E2 * [eye(n_link); zeros(2)];
% deltaqDotBar = inv(D) * E2.' * deltaF2 + [eye(n_link); zeros(2)];