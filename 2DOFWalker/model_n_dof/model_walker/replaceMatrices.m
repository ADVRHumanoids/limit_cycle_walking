function [D,C,G,E2,T] = replaceMatrices(symD, symC, symG, symE2, symT)
%=========================================
sizeE2 = flip(size(symE2));
charE2 = char(simplify(symE2));
newStr = strrep(charE2,'q1(t)','q(1)');
newStr = strrep(newStr,'q2(t)','q(2)');
newStr = strrep(newStr,'q3(t)','q(3)');
newStr = strrep(newStr,'q4(t)','q(4)');
newStr = strrep(newStr,'q5(t)','q(5)');

if contains(newStr,'matrix')
newStr([1:7, end:end]) = [];
end
E2 = str2sym(newStr);
E2 = reshape(E2,sizeE2).';
%=========================================
sizeD = flip(size(symD));
charD = char(simplify(symD));
newStr = strrep(charD,'q1(t)','q(1)');
newStr = strrep(newStr,'q2(t)','q(2)');
newStr = strrep(newStr,'q3(t)','q(3)');
newStr = strrep(newStr,'q4(t)','q(4)');
newStr = strrep(newStr,'q5(t)','q(5)');

newStr = strrep(newStr,'q1_dot(t)','q_dot(1)');
newStr = strrep(newStr,'q2_dot(t)','q_dot(2)');
newStr = strrep(newStr,'q3_dot(t)','q_dot(3)');
newStr = strrep(newStr,'q4_dot(t)','q_dot(4)');
newStr = strrep(newStr,'q5_dot(t)','q_dot(5)');

if contains(newStr,'matrix')
newStr([1:7, end:end]) = [];
end
D = str2sym(newStr);
D = reshape(D,sizeD).';
%=========================================
sizeC = flip(size(symC));
charC = char(simplify(symC));

newStr = strrep(charC,'q1(t)','q(1)');
newStr = strrep(newStr,'q2(t)','q(2)');
newStr = strrep(newStr,'q3(t)','q(3)');
newStr = strrep(newStr,'q4(t)','q(4)');
newStr = strrep(newStr,'q5(t)','q(5)');

newStr = strrep(newStr,'q1_dot(t)','q_dot(1)');
newStr = strrep(newStr,'q2_dot(t)','q_dot(2)');
newStr = strrep(newStr,'q3_dot(t)','q_dot(3)');
newStr = strrep(newStr,'q4_dot(t)','q_dot(4)');
newStr = strrep(newStr,'q5_dot(t)','q_dot(5)');

if contains(newStr,'matrix')
newStr([1:7, end:end]) = [];
end
C = str2sym(newStr);
C = reshape(C,sizeC).';

%=========================================
sizeG = flip(size(symG));
charG = char(simplify(symG));

newStr = strrep(charG,'q1(t)','q(1)');
newStr = strrep(newStr,'q2(t)','q(2)');
newStr = strrep(newStr,'q3(t)','q(3)');
newStr = strrep(newStr,'q4(t)','q(4)');
newStr = strrep(newStr,'q5(t)','q(5)');



if contains(newStr,'matrix')
newStr([1:7, end:end]) = [];
end
G = str2sym(newStr);
G = reshape(G,sizeG).';
%=========================================
sizeT = flip(size(symT));
charT = char(simplify(symT));

newStr = strrep(charT,'q1(t)','q(1)');
newStr = strrep(newStr,'q2(t)','q(2)');
newStr = strrep(newStr,'q3(t)','q(3)');
newStr = strrep(newStr,'q4(t)','q(4)');
newStr = strrep(newStr,'q5(t)','q(5)');

newStr = strrep(newStr,'q1_dot(t)','q_dot(1)');
newStr = strrep(newStr,'q2_dot(t)','q_dot(2)');
newStr = strrep(newStr,'q3_dot(t)','q_dot(3)');
newStr = strrep(newStr,'q4_dot(t)','q_dot(4)');
newStr = strrep(newStr,'q5_dot(t)','q_dot(5)');

T = str2sym(newStr);
T = reshape(T,sizeT).';
%====================================================

end


% 
% for i = 1:length(symq)
%     charq{i} = char(symq(i));
% end
% for i = 1:length(symq)
%     charq_dot{i} = char(symq_dot(i));
% end
% for i = 1:length(symq)
%     charq_Ddot{i} = char(symq_Ddot(i));
% end