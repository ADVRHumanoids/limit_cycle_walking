%=============================impact model=================================
phi = rp1_0(1:2) + rp2_0(1:2); 
p2 = [z1(t);z2(t)] + phi;

E2 = sym(zeros(size(phi,1), dimGC));

for j = 1:size(phi,1)
  for i = 1:dimGC
        E2(j,i) = functionalDerivative(p2(j),GeneralizedCoordinates(i));
  end
end 

deltaF2 = -inv(E2 * inv(D) * E2.') * E2 * [eye(nlink); zeros(2)];

deltaqDotBar = inv(D) * E2.' * deltaF2 + [eye(nlink); zeros(2)];

R = [-1 0; %[pi;0]
     0 -1];
 
% R = eye(2);
deltaq = R;
deltaqDot = [R zeros(nlink,2)] * deltaqDotBar;