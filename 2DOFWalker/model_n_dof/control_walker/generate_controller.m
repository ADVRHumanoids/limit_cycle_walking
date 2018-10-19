clear all; clc;

syms a01 a11 a21 a31 a02 a12 a22 a32
syms D C G 
syms th th1d
syms q1 q2 q3 q1_dot q2_dot q3_dot

a_leg = [a02 a12 a22 a32];
a_waist = [a01 a11 a21 a31];

q=[q1;q2;q3];
dq=[q1_dot;q2_dot;q3_dot];

    th(1) = q(1);
    th(2) = - (pi - q(1) - q(2));
    th(3) = q(3);


thd(1) = th1d;

%output function h
y = [th(2)-(-th(1)+(a_leg(1) + a_leg(2)*th(1) + a_leg(3)*th(1)^2+a_leg(4)*th(1)^3)*(th(1)-thd(1))*(th(1)+thd(1)));%leg virtual constraint
                                  th(3)-(a_waist(1) + a_waist(2)*th(1) + a_waist(3)*th(1)^2 + a_waist(4)*th(1)^3)]; %waist virtual constraint

                         
h_dq = jacobian(y,q); 
h_dq_dq = [jacobian(h_dq * dq, q), h_dq]; %RIGHT


%==========================================================================
FileName = sprintf('templateController.m');

    startingFolder = what('control_walker');
    fullFileName = fullfile(startingFolder.path, FileName);
    % Open a new file.
    fileID = fopen(fullFileName, 'wt');
    % Load it up.
    fprintf(fileID, '%%Controller based on Bezier curves for virtual constraints.\n');
    

    h_dq = substitute_output(h_dq, 'h_dq', fileID);
    h_dq_dq = substitute_output(h_dq_dq, 'h_dq_dq', fileID);
        

    % Close the file.
    fclose(fileID);
    % Open the file in the editor.
    edit(fullFileName);
    
    
    
function output = substitute_output(input, inputName, fileID)

    %substitute q1 q2 q3 and Bezier parameters for a 3 link walker
    %
    output_cell = cell(size(input,1),1);
    
    for i = 1:size(output_cell,1)
        output_cell{i} = char(simplify(input(i,:)));
    end

    output_cell = strrep(output_cell,'q1_dot','q_dot(1)');
    output_cell = strrep(output_cell,'q2_dot','q_dot(2)');
    output_cell = strrep(output_cell,'q3_dot','q_dot(3)');
    
    output_cell = strrep(output_cell,'q1','q(1)');
    output_cell = strrep(output_cell,'q2','q(2)');
    output_cell = strrep(output_cell,'q3','q(3)');

    output_cell = strrep(output_cell,'a01','a_waist(1)');
    output_cell = strrep(output_cell,'a11','a_waist(2)');
    output_cell = strrep(output_cell,'a21','a_waist(3)');
    output_cell = strrep(output_cell,'a31','a_waist(4)');

    output_cell = strrep(output_cell,'a02','a_leg(1)');
    output_cell = strrep(output_cell,'a12','a_leg(2)');
    output_cell = strrep(output_cell,'a22','a_leg(3)');
    output_cell = strrep(output_cell,'a32','a_leg(4)');

    output_cell = strrep(output_cell,'th1d','thd(1)');
    

    output_cell = replace(output_cell, 'matrix([[' , '');
    output_cell = replace(output_cell, ']])' , '' );
    
    output = output_cell;
    

        
    fprintf(fileID, '%s = [ \n', inputName);
    
    for i = 1:length(output)
        fprintf(fileID, '%s;\n', output{i});
    end
    
    fprintf(fileID, '];');
    fprintf(fileID, '\n\n');
end