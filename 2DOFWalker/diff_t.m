function d_equation = diff_t(equation, variables, d_variables)

syms t
dimVar = length(variables);
for i = 1:dimVar
    tosub_variables(i) = diff(variables(i),t);
end

d_equation = subs(diff(equation,t), tosub_variables, d_variables);
                                                   
end


