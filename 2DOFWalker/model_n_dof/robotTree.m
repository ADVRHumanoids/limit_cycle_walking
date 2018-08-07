%% robot walker tree
% flagSim ---> 1 or 0; defines if model is for simulation or not
% parent_tree ---> parent tree defining the links of the robot
% swing_leg ---> swing leg of the robot

% this script will create a full model (also extended with floating base)
% and will generate dynamic matrices file in .../sim_utilities/dyMatrices for the
% simulation.

    % author: Francesco Ruscelli
    % e-mail: francesco.ruscelli@iit.it
    
close all; clear all; clc;

Folder = cd;
addpath(genpath(fullfile(Folder, '..')));
%==========================================================================
%user defined ...
%change to 1 if this file is used for simulation (in simulation_model_n)
%change to 0 if this file is used to generate symbolic model
flagSimulation = 1; 

%define the parent tree of the robot walker
parent_tree = [0 1];
% parent_tree = [0 1];
% parent_tree = [0 1 2 3];

%choose which leg is the initial swinging leg
swing_leg = 2;

%choose which link is the waist (if there is no waist, leave it an empty array [])
waist = []; %needed for the relabeling


%physical parameters of the robot walker:
%(link length, com position, mass, inertia and gravity)
parameterSet = 1;
link_length = [1 1 0.5];
com_position = [0.8 0.2 0.5/2]; %0.8
m = [0.3 0.3 1]; %0.3
I = [0.03 0.03 0.1];
g = 9.81;

% parameterSet = 2;
% link_length = [1 1 1];
% com_position = [0.8 0.2 0.8];
% m = [0.3 0.3 0.3];
% I = [0.03 0.03 0.03];
% g = 9.81;



%==========================================================================
%autocheck if file with specified parameter set already exists
fileName = ['DM',num2str(length(parent_tree)),'Link_par', num2str(parameterSet)];
checkFile = what('sim_utilities/dynMatrices');
%========================generate model and files==========================
setRobotParameters(link_length,com_position,m,I,g,parent_tree,flagSimulation); %global variable robotData
robotData = getRobotData;

if isempty(find(~cellfun('isempty',strfind(checkFile.m, fileName)), 1)) || ~flagSimulation
    
    if ~flagSimulation
        disp('Creating model of the robot...');
    end
    if flagSimulation
        disp('Dynamic matrices for simulation not found. Starting autogeneration...');
    end
    

    robotData.flagSimulation = 0; %first, I have to create matrices, so I put simulation to zero
    [generalizedVariables,generalizedVariablesExtended] = autogen_robotVariables(parent_tree);
    model = generateModel(generalizedVariables,generalizedVariablesExtended,swing_leg);
    
    if flagSimulation
        robotData.flagSimulation = 1;
        generateDynamicMatrices(model,fileName);
    end
    
    disp('Finished');
end
%==========================================================================












