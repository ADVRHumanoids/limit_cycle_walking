function d_equation = diff_t(equation, variables, d_variables)
%% differentiates an equation with respect to time
% Given a vector of variables dependent on time and the vector holding the
% names of the variables AFTER differentiation, it differentiate the
% equation. 

%Example:
% diff_t((cos(q1 + a)+ q2), [q1 q2], [q1_dot q2_dot])

% It requires:
% >> equation to differentiate
% >> variables dependent on time
% >> names of the variables

    % author: Francesco Ruscelli
    % e-mail: francesco.ruscelli@iit.it
syms t
dimVar = length(variables);
for i = 1:dimVar
    tosub_variables(i) = diff(variables(i),t);
end

d_equation = subs(diff(equation,t), tosub_variables, d_variables);
                                                   
end


