function generateDynamicMatrices(model, fileName)
%% generate dynamic matrices file for simulation
    %this function will autogenerate a file with the Dynamic Matrices in:
    %>>.../sim_utilities/dynMatrices
    
%%>> autogenerate a file name based on the file which are already there

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
    

    %add success return
    %======================================================================
end