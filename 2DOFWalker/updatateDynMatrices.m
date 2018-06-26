function [D,C,G] = updatateDynMatrices(symD,symC,symG,symbolicVar,numericVar)

D = double(subs(symD,symbolicVar,numericVar));
C = double(subs(symC,symbolicVar,numericVar));
G = double(subs(symG,symbolicVar,numericVar));  

end