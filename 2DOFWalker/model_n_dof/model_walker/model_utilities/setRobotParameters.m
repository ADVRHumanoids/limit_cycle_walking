function setRobotParameters(link_length,com_position,m,I,g,parent_tree,flagSimulation)
%% returns struct of parameters of the robot
% If flagSimulation is set to 1, this function uses symbolic values

% Parameters:
% >> n_link
% >> link_length
% >> com_position
% >> mass
% >> inertia
% >> gravity

    % author: Francesco Ruscelli
    % e-mail: francesco.ruscelli@iit.it
    
    global robotData;
    n_link = length(parent_tree);

    if ~flagSimulation

        link_length = sym('l',[n_link,1]).';
        com_position = sym('lc',[n_link,1]).';
        m = sym('m',[n_link,1]);
        I = sym('I',[n_link,1]);
        g = sym('g');

    end
    

    link_length = link_length(1:n_link);
    com_position = com_position(1:n_link);
    m = m(1:n_link);
    I = I(1:n_link);
%     if length(link_length) ~= length(parent_tree) || length(com_position) ~= length(parent_tree) || length(m) ~= length(parent_tree) || length(I) ~= length(parent_tree)
%     error('Number of link is not compatible with robot parameters.')
%     end
    
    robotData = struct('parent_tree',parent_tree,'n_link',n_link,'link_length',link_length, 'com_position',com_position, 'mass',m, 'inertia',I,'gravity', g);   
end