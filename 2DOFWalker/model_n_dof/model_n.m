close all; clear all; clc;

Folder = cd;
addpath(genpath(fullfile(Folder, '..')));
%==================dynamic model of a n link walker========================
robotTree;
[D,C,G,kinematics] = createModelWalker(parent_tree,generalizedVariables,robotData);
%======================
% save('dynMatricesExtendedSymbolic.mat','D','C','G');
%=============================check========================================
D_dot = diff_t(D,[qe,qe_dot], [qe_dot,qe_Ddot]);

N = simplify(D_dot - 2*C);
ThisShouldbeZero = simplify(qe_dot*N*qe_dot.');
%==========================================================================
%==========================================================================
% E2 = kinematics.jacobian;
% [matD, matC, matG, matE2] = replaceMatrices(D, C, G, E2);
% %when needed, to put in --> calcDynMatrices, calcJacobianMatrix
% save('dynMatricesExtended.mat','matD','matC','matG', 'matE2');