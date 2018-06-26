clear all; clc
clear L

a = [1,1];

L_base(1) = Prismatic('theta', 0, 'alpha', -pi/2);% 0 -pi/2
L_base(2) = Prismatic('theta', -pi/2, 'alpha', pi/2); %-pi/2 pi/2



base = SerialLink(L_base, 'name', 'base'); %'base', troty(pi/2)

L(1) = Revolute('d', 0, 'a', a(1), 'alpha',0, ...
                'I', [0, 0.35, 0], ...
                'm', 0.3);

L(2) = Revolute('d', 0, 'a', a(2), 'alpha', pi/2, ...
                'I', [0, 0.35, 0], ...
                'm', 0.3);


L_base(1).qlim = [0 1];
L_base(2).qlim = [0 1];

qz = [0 0];


twoLinkWalker = SerialLink(L, 'name', 'twoLinkWalker');
                        
twoLinkWalkerExt = SerialLink([base,twoLinkWalker], 'name', 'twoLinkWalkerExt');                 

clear L
% twoLinkWalker.teach;
% base.teach;
% twoLinkWalkerExt.teach