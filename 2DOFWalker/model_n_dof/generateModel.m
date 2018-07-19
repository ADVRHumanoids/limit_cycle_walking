function model = generateModel(generalizedVariables,generalizedVariablesExtended,parent_tree,swing_leg,robotData)
%% dynamic model of a n link walker
[D_ext,C_ext,G_ext,kinematics_ext,dynamics_ext] = createModelExtendedWalker(parent_tree,generalizedVariablesExtended,robotData);
[D,C,G,kinematics,dynamics] = createModelWalker(parent_tree,generalizedVariables,robotData);
%======================
% save('dynMatricesExtendedSymbolic.mat','D','C','G');
%=============================check========================================
% D_dot_ext = diff_t(D_ext,[generalizedVariablesExtended.qe,generalizedVariablesExtended.qe_dot], [generalizedVariablesExtended.qe_dot,generalizedVariablesExtended.qe_Ddot]);
% 
% N_ext = simplify(D_dot_ext - 2*C_ext);
% ThisShouldbeZero_ext = simplify(generalizedVariablesExtended.qe_dot*N_ext*generalizedVariablesExtended.qe_dot.');
% %===========================check========================================
% D_dot = diff_t(D,[generalizedVariables.q,generalizedVariables.q_dot], [generalizedVariables.q_dot,generalizedVariables.q_Ddot]);
% 
% N = simplify(D_dot - 2*C);
% ThisShouldbeZero = simplify(generalizedVariables.q_dot*N*generalizedVariables.q_dot.');
%====================derivative of kinetic energy==========================
% K_dot = diff_t(dynamics.KineticEnergy,[q,q_dot], [q_dot, q_Ddot]);
%==========================================================================
E2_ext = kinematics_ext.jacobian(:,:,swing_leg);
E2 = kinematics.jacobian(:,:,swing_leg);


%==========================================================================
model.kinematics_ext = kinematics_ext;
model.kinematics = kinematics;
model.dynamics_ext = dynamics_ext;
model.dynamics = dynamics;
model.dynamicMatrices.D = D;
model.dynamicMatrices.C = C;
model.dynamicMatrices.G = G;
model.dynamicMatrices.E2 = E2;
model.dynamicMatricesExtended.D = D_ext;
model.dynamicMatricesExtended.C = C_ext;
model.dynamicMatricesExtended.G = G_ext;
model.dynamicMatricesExtended.E2 = E2_ext;
model.robotData = robotData;


model_n_created = 1;
end
