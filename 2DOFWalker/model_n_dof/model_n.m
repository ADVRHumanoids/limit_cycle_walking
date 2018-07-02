close all; clear all; clc;

Folder = cd;
addpath(genpath(fullfile(Folder, '..')));
%==================dynamic model of a n link walker========================
robotTree;
%======================symbolic parameters=================================

% link_length = sym('l',[length(parent_tree),1]).';
% com_position = sym('lc',[length(parent_tree),1]).';
% m = sym('m',[length(parent_tree),1]);
% I = sym('I',[length(parent_tree),1]);

%======================real parameters=====================================
link_length = 1;
com_position = 0.5; %0.8
mass = 1; %0.3
inertia = 0.083;

link_length = link_length * ones(1,length(parent_tree));
com_position = [1-com_position, com_position * ones(1,length(parent_tree)-1)];
m = mass * ones(1,length(parent_tree));
I = inertia * ones(1,length(parent_tree));
g = 9.81;

%==========================================================================
robotData = struct('n_link',n_link,'link_length',link_length, 'com_position',com_position, 'mass',m, 'inertia',I,'gravity', g, 'flagSim', flagSim);


[D,C,G,kinematics,T] = createModelWalker(parent_tree,generalizedVariables,robotData);
%======================
% save('dynMatricesExtendedSymbolic.mat','D','C','G');
%=============================check========================================
% D_dot = diff_t(D,[qe,qe_dot], [qe_dot,qe_Ddot]);
% 
% N = simplify(D_dot - 2*C);
% ThisShouldbeZero = simplify(qe_dot*N*qe_dot.');
%==========================================================================
%==========================================================================
E2 = kinematics.jacobian;

[matD, matC, matG, matE2, matT] = replaceMatrices(D, C, G, E2, T);
%when needed, to put in --> calcDynMatrices, calcJacobianMatrix
% save('dynMatricesExtended.mat','matD','matC','matG', 'matE2');