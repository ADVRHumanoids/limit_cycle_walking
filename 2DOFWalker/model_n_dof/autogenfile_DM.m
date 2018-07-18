function autogenfile_DM(baseFileName, matrices)
    

    
    FileName = sprintf('%s.m', baseFileName);
    startingFolder = what('dynMatrices');
    fullFileName = fullfile(startingFolder.path, FileName);
    % Open a new file.
    fileID = fopen(fullFileName, 'wt');
    % Load it up.
    for i = 1:length(matrices)
        fill_autogenFile(matrices(i),fileID)   %<---------------
    end
    % Close the file.
    fclose(fileID);
    % Open the file in the editor.
    edit(fullFileName);

end

function fill_autogenFile(matrix,fileID)
    
    matrixName = fieldnames(matrix);
    MatrixCell = remodelMatrix(matrix);
    fprintf(fileID, '%s = [ \n', matrixName{1});
    for i = 1:length(MatrixCell)
    fprintf(fileID, '%s;\n',MatrixCell{i});
    end
    fprintf(fileID, '];');
    fprintf(fileID, '\n\n');
    
end

function MatrixCell = remodelMatrix(matrix)

MyFieldNames = fieldnames(matrix);
tempMatrix = getfield(matrix,MyFieldNames{1});

charMatrix = cell(size(tempMatrix,1),1);
tempCell = cell(size(tempMatrix,1),1);
MatrixCell = cell(size(tempMatrix,1),1);

    for i = 1:length(tempMatrix)
         charMatrix{i} = char(simplify(tempMatrix(i,:)));
         tempCell{i} = strrep(charMatrix{i},'q1(t)','q(1)');
         tempCell{i} = strrep(tempCell{i},'q2(t)','q(2)');
         tempCell{i} = strrep(tempCell{i},'q3(t)','q(3)');
         tempCell{i} = strrep(tempCell{i},'q4(t)','q(4)');
         tempCell{i} = strrep(tempCell{i},'q5(t)','q(5)');
         tempStr = tempCell{i};
         tempStr([1:7, end:end]) = [];
         tempStr = replace(tempStr, '[[' , '');
         tempStr = replace(tempStr, ']]' , '' );
         MatrixCell{i} = tempStr;
    end
    
end
