function autogenfile_DM(baseFileName, matrices, robotData)
    
    matrixName = fieldnames(matrices);

    FileName = sprintf('%s.m', baseFileName);
    startingFolder = what('dynMatrices');
    fullFileName = fullfile(startingFolder.path, FileName);
    % Open a new file.
    fileID = fopen(fullFileName, 'wt');
    % Load it up.
    fprintf(fileID, '%%Dynamic matrices for a %g-link walker.\n', (size(matrices.(matrixName{1}),2)-2));
    fprintf(fileID, '%%Robot Parameters: \n\n');
    fprintf(fileID, '%%Number of links: %g\n',robotData.n_link);
    fprintf(fileID, '%%Length of links: '); fprintf(fileID, '%g  ',robotData.link_length);
    fprintf(fileID, '\n%%Position of CoM: '); fprintf(fileID,'%g  ',robotData.com_position);
    fprintf(fileID, '\n%%Mass of links: '); fprintf(fileID,'%g  ',robotData.mass);
    fprintf(fileID, '\n%%Inertia of links: '); fprintf(fileID,'%g  ',robotData.inertia);
    fprintf(fileID, '\n%%Gravity: %g\n\n',robotData.gravity);
    for i = 1:numel(fieldnames(matrices))
        fill_autogenFile(matrices.(matrixName{i}),matrixName{i},fileID)   %<---------------
    end
    % Close the file.
    fclose(fileID);
    % Open the file in the editor.
    edit(fullFileName);

end

function fill_autogenFile(matrix,matrixName,fileID)
    

    MatrixCell = remodelMatrix(matrix);
    fprintf(fileID, '%s = [ \n', matrixName);
    for i = 1:length(MatrixCell)
    fprintf(fileID, '%s;\n',MatrixCell{i});
    end
    fprintf(fileID, '];');
    fprintf(fileID, '\n\n');
    
end

function MatrixCell = remodelMatrix(matrix)


tempMatrix = matrix;

charMatrix = cell(size(tempMatrix,1),1);
tempCell = cell(size(tempMatrix,1),1);
MatrixCell = cell(size(tempMatrix,1),1);

    for i = 1:size(tempMatrix,1)
        charMatrix{i} = char(simplify(tempMatrix(i,:)));
        tempCell{i} = strrep(charMatrix{i},'q1(t)','q(1)');
        tempCell{i} = strrep(tempCell{i},'q2(t)','q(2)');
        tempCell{i} = strrep(tempCell{i},'q3(t)','q(3)');
        tempCell{i} = strrep(tempCell{i},'q4(t)','q(4)');
        tempCell{i} = strrep(tempCell{i},'q5(t)','q(5)');

        tempCell{i} = strrep(tempCell{i},'q1_dot(t)','q_dot(1)');
        tempCell{i} = strrep(tempCell{i},'q2_dot(t)','q_dot(2)');
        tempCell{i} = strrep(tempCell{i},'q3_dot(t)','q_dot(3)');
        tempCell{i} = strrep(tempCell{i},'q4_dot(t)','q_dot(4)');
        tempCell{i} = strrep(tempCell{i},'q5_dot(t)','q_dot(5)');
        tempStr = tempCell{i};
        if contains(tempStr,'matrix')
            tempStr([1:7, end:end]) = [];
        end
        tempStr = replace(tempStr, '[[' , '');
        tempStr = replace(tempStr, ']]' , '' );
        MatrixCell{i} = tempStr;
    end
    
end
