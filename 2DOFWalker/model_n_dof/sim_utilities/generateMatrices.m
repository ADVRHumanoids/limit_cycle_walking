function generateMatrices(model, fileName)
%% This function auto-generates a file with the dynamic matrices and energy of the robot walker for simulation
    %this function will autogenerate a file with the Dynamic Matrices in:
    %>>.../sim_utilities/dynMatrices
    
    % author: Francesco Ruscelli
    % e-mail: francesco.ruscelli@iit.it  
    
%==========================================================================
%%>> autogenerate a file name based on the file which are already there
% 
%     i = 1;
%     fileName = sprintf('DM%dLink_par%d', length(D),i);
%     startingFolder = what('dynMatrices');
%     checkFile = fullfile(startingFolder.path, fileName);
%     while exist(checkFile, 'file') == 2
%         previousPar = sprintf('_par%s',num2str(i));
%         nextPar = sprintf('_par%s',num2str(i+1));
%         checkFile = strrep(checkFile, previousPar, nextPar);
%         i = i+1;
%         fileName = strrep(fileName, previousPar, nextPar);
%     end


        
    autogenfile_DM(fileName, model.dynamicMatricesExtended, model.robotData);
    autogenfile_energy(fileName,model.dynamics, model.robotData);

    %add success return %TODO
    %======================================================================
end