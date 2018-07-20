function [generalizedVariables,generalizedVariablesExtended] = autogen_robotVariables(parent_tree)

n_link = length(parent_tree);
fileName = sprintf('generalizedVariables_%glink', n_link);
startingFolder = what('model_n_dof');
checkFile = fullfile(startingFolder.path, fileName);
if exist(checkFile, 'file') ~= 2
   autogenGeneralizedVariables(fileName, n_link); 
end

run(fileName);

qe = [q, z];
qe_dot = [q_dot, z_dot];
qe_Ddot = [q_Ddot, z_Ddot];

generalizedVariablesExtended.qe = qe;
generalizedVariablesExtended.qe_dot = qe_dot;
generalizedVariablesExtended.qe_Ddot = qe_Ddot;


generalizedVariables.q = q;
generalizedVariables.q_dot = q_dot;
generalizedVariables.q_Ddot = q_Ddot;
end
%==========================================================================

function autogenGeneralizedVariables(baseFileName, n_link)

    FileName = sprintf('%s.m', baseFileName);
    startingFolder = what('model_walker');
    fullFileName = fullfile(startingFolder.path, FileName);
    % Open a new file.
    fileID = fopen(fullFileName, 'wt');
    % Load it up.
    
    %==================================================
    
    fprintf(fileID, 'syms g I l lc m ...\n');
    fprintf(fileID, 'q%d(t) ', 1:n_link);
    fprintf(fileID, '...\n');
    fprintf(fileID, 'q%d_dot(t) ', 1:n_link);
    fprintf(fileID, '...\n');
    fprintf(fileID, 'q%d_Ddot(t) ', 1:n_link);
    fprintf(fileID, '...\n');
    fprintf(fileID, 'z%d(t) ', 1:2);
    fprintf(fileID, '...\n');
    fprintf(fileID, 'z%d_dot(t) ', 1:2);
    fprintf(fileID, '...\n');
    fprintf(fileID, 'z%d_Ddot(t) ', 1:2);
    fprintf(fileID, '\n\n');
    
    %==================================================
    
    fprintf(fileID, 'q = [ ');
    fprintf(fileID, 'q%d(t) ', 1:n_link);
    fprintf(fileID, '];\n');

    fprintf(fileID, 'q_dot = [ ');
    fprintf(fileID, 'q%d_dot(t) ', 1:n_link);
    fprintf(fileID, '];\n');
    
    fprintf(fileID, 'q_Ddot = [ ');
    fprintf(fileID, 'q%d_Ddot(t) ', 1:n_link);
    fprintf(fileID, '];\n');
    
    %=================================================
    fprintf(fileID, '\n');
    
    fprintf(fileID, 'z = [ ');
    fprintf(fileID, 'z%d(t) ', 1:2);
    fprintf(fileID, '];\n');

    fprintf(fileID, 'z_dot = [ ');
    fprintf(fileID, 'z%d_dot(t) ', 1:2);
    fprintf(fileID, '];\n');
    
    fprintf(fileID, 'z_Ddot = [ ');
    fprintf(fileID, 'z%d_Ddot(t) ', 1:2);
    fprintf(fileID, '];\n');  
    
    
end