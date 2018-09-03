function [mechanicalEnergy,kineticEnergy,potentialEnergy,kineticEnergy_dot] = calcEnergy(q,q_dot,q_Ddot,fileName)

fileName = sprintf('%s_energy.m', fileName);
%>> files are in:
%>> /home/francesco/advr-superbuild/external/limit_cycle_walking/2DOFWalker/model_n_dof/sim_utilities/dynMatrices
run(fileName)
end