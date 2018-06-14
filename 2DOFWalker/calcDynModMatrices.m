function X = calcDynModMatrices(equation, zeroElements, mainFactor)


dim = length(zeroElements);

X_1 = subs(equation, zeroElements, zeros(1,dim));

    if nargin > 2
        X = 0;
        [coeffs_X_1,order] = coeffs(X_1, mainFactor);
        i = length(find(order~=1));
        while (i > 0)
            X = X + coeffs_X_1(1,i)*order(1,i)/mainFactor;
            i = i-1;
        end
    else
        X = X_1;
    end

end