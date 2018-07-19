%% robot walker tree
% flagSim ---> 1 or 0; defines if model is for simulation or not
% parent_tree ---> parent tree defining the links of the robot
% swing_leg ---> swing leg of the robot

% this script will create a full model (also extended with floating base)
% and will generate dynamic matrices file in .../sim_utilities/dyMatrices for the
% simulation.

close all; clear all; clc;

Folder = cd;
addpath(genpath(fullfile(Folder, '..')));
%==========================================================================
%user defined ...

flagSimulation = 1;
% parent_tree = [0 1 1];
% parent_tree = [0 1];
parent_tree = [0 1 2 3];
swing_leg = 3;
waist = []; %needed for the relabeling

% parameterSet = 1;
% link_length = [1 1 0.5];
% com_position = [0.8 0.8 0.5/2]; %0.8
% m = [0.3 0.3 1]; %0.3
% I = [0.03 0.03 0.1];
% g = 9.81;

parameterSet = 2;
link_length = [1 1 1 1];
com_position = [0.2 0.8 0.8 0.8];
m = [0.3 0.3 0.3 0.3];
I = [0.03 0.03 0.03 0.03];
g = 9.81;


%check if file with specified parameter set already exists

fileName = ['DM',num2str(length(parent_tree)),'Link_par', num2str(parameterSet)];
checkFile = what('sim_utilities/dynMatrices');
%========================generate model and files==========================
robotData = setRobotParameters(link_length,com_position,m,I,g,parent_tree,flagSimulation);

if isempty(find(~cellfun('isempty',strfind(checkFile.m, fileName)), 1)) || ~flagSimulation
    
    if ~flagSimulation
        disp('Creating model of the robot...');
    end
    if flagSimulation
        disp('Dynamic matrices for simulation not found. Starting autogeneration...');
    end
    
    robotData.flagSimulation = 0; %first, I have to create matrices, so I put simulation to zero
    [generalizedVariables,generalizedVariablesExtended] = autogen_robotVariables(parent_tree);
    model = generateModel(generalizedVariables,generalizedVariablesExtended,parent_tree,swing_leg,robotData);
    if flagSimulation
        robotData.flagSimulation = 1;
        generateDynamicMatrices(model,fileName);
    end
    disp('Finished');
end
%==========================================================================












