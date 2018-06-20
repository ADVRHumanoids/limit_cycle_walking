dimGC = size(qe,2);

zeroTerms_D = [q1_dot(t), q2_dot(t), g];
D = sym(zeros(dimGC));
for i = 1:dimGC
    for j = 1:dimGC
        D(i,j) = calcDynModMatrices(eqMotion(i), zeroTerms_D, qe_Ddot(j));
    end
end
%=============================getting C matrix=============================
% zeroTerms_C = [q1_Ddot(t), q2_Ddot(t), g];
% C = sym(zeros(dimGC));
% for i = 1:dimGC
%     for j = 1:dimGC
%         C(i,j) = calcDynModMatrices(DynamicEquations(i), zeroTerms_C, d_GeneralizedCoordinates(j));
%     end
% end
%=============================getting G matrix=============================================
zeroTerms_G = [q1_Ddot(t), q2_Ddot(t), q1_dot(t), q2_dot(t), z1_Ddot(t), z2_Ddot(t), z1_dot(t), z2_dot(t)];
G = sym(zeros(dimGC,1));
for i = 1:dimGC
        G(i) = calcDynModMatrices(eqMotion(i), zeroTerms_G);
end
%=============================getting C matrix with Christoffel============
for i = 1:dimGC
    for j = 1:dimGC
        for k = 1:dimGC
            c(i,j,k) = 1/2 * (functionalDerivative(D(k,j), qe(i)) + ...
                              functionalDerivative(D(k,i), qe(j)) - ...
                              functionalDerivative(D(i,j), qe(k)));
        end
    end
end

C = sym(zeros(dimGC));

for j = 1:dimGC
    for k = 1:dimGC
        for i = 1:dimGC
            C(k,j) = C(k,j) + c(i,j,k)*qe_dot(i).';
        end
    end
end
