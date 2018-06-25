clear all; clc;
% addpath(genpath('/home/francesco/Documents/MATLAB/Add-Ons/Toolboxes/Robotics Toolbox for MATLAB'));
startup_rvc;

clear L

a = [1,1];

L(1) = Revolute('d', 0, 'a', a(1), 'alpha', 0, ...
    'I', [0, 0.35, 0], ...
    'm', 0.3);

L(2) = Revolute('d', 0, 'a', a(2), 'alpha', 0, ...
    'I', [0, 0.35, 0], ...
    'm', 0.3);


qz = [0 pi/2];


twoLinkWalker = SerialLink(L, 'name', 'twoLinkWalker', ...
                            'configs', {'qz', qz});

clear L
