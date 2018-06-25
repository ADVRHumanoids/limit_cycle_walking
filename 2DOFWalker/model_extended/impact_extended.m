%=============================impact model=================================
phi_link1 = rp1_0(1:2); %planar
phi_link2 = rp1_0(1:2) + rp2_0(1:2);  %planar

p1 = [z1(t);z2(t)] + phi_link1;
p2 = [z1(t);z2(t)] + phi_link2;

E1 = sym(zeros(size(phi_link1,1), dimGC));
E2 = sym(zeros(size(phi_link2,1), dimGC));

for j = 1:size(phi_link2,1)
  for i = 1:dimGC
        E1(j,i) = functionalDerivative(p1(j),GeneralizedCoordinates(i));
  end
end 

for j = 1:size(phi_link2,1)
  for i = 1:dimGC
        E2(j,i) = functionalDerivative(p2(j),GeneralizedCoordinates(i));
  end
end 

deltaF2 = - inv(E2 * inv(D) * E2.') * E2 * [eye(n_link); zeros(2)];

deltaqDotBar = inv(D) * E2.' * deltaF2 + [eye(n_link); zeros(2)];

% R = [-1 0; %[pi;0]
%      0 -1];
%  
% % R = eye(2);
% deltaq = R;
% deltaqDot = [R zeros(n_link,2)] * deltaqDotBar;