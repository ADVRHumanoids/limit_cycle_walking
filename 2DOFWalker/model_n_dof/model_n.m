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


[D_ext,C_ext,G_ext,kinematics_ext,dynamics_ext] = createModelExtendedWalker(parent_tree,generalizedVariablesExtended,robotData);

[D,C,G,kinematics,dynamics] = createModelWalker(parent_tree,generalizedVariables,robotData);
%======================
% save('dynMatricesExtendedSymbolic.mat','D','C','G');
%=============================check========================================
% D_dot_ext = diff_t(D_ext,[qe,qe_dot], [qe_dot,qe_Ddot]);
% 
% N_ext = simplify(D_dot_ext - 2*C_ext);
% ThisShouldbeZero_ext = simplify(qe_dot*N_ext*qe_dot.');
% %===========================check========================================
% D_dot = diff_t(D,[q,q_dot], [q_dot,q_Ddot]);
% 
% N = simplify(D_dot - 2*C);
% ThisShouldbeZero = simplify(q_dot*N*q_dot.');
%==========================================================================
E2_ext = kinematics_ext.jacobian;
E2 = kinematics.jacobian;
[matD_ext, matC_ext, matG_ext, matE2_ext] = replaceMatricesExtended(D_ext, C_ext, G_ext, E2_ext);
[matD, matC, matG, matE2, matT] = replaceMatrices(D, C, G, E2, dynamics.mechanicalEnergy);

K_dot = diff_t(dynamics.KineticEnergy,[q,q_dot], [q_dot, q_Ddot]);
%when needed, to put in --> calcDynMatrices, calcJacobianMatrix
% save('dynMatricesExtended.mat','matD','matC','matG', 'matE2');