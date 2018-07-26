function relabelingMatrices = getRelabelingMatrices(parent_tree, waist)
%% relabeling matrices for the robot walker
% get relabeling matrices from the parent tree

    % author: Francesco Ruscelli
    % e-mail: francesco.ruscelli@iit.it
    
    n_link = length(parent_tree);
    relabelingMatrices.piMatrix = pi*[1; zeros(n_link-1,1)];
    relabelingMatrices.MatrixRelabel = zeros(n_link,n_link);
    relabelingMatrices.MatrixRelabel(1,:) = 1;

    waist_ind = waist;
    if isempty(waist) && length(parent_tree) ~= length(unique(parent_tree))
        error('Given this specific parent tree, there must be a waist');
    end

    if ~isempty(waist)
        duplicates = find(parent_tree == parent_tree(waist));
        nextLinkAfterWaist = duplicates(duplicates~=waist);
        relabelingMatrices.piMatrix(waist) = pi;
        relabelingMatrices.MatrixRelabel(1,waist) = 0;
    else
        waist_ind = 0;
    end


    j = 0;

    for i = 2:n_link
        if i ~= waist_ind

            if relabelingMatrices.MatrixRelabel(i,n_link-j) == 0 
                if (n_link - j) == waist
                    j = j + 1;
                end
                relabelingMatrices.MatrixRelabel(i,n_link-j) = -1;           
            end
            j = j + 1;

        else

            relabelingMatrices.MatrixRelabel(waist_ind, waist_ind) = 1;
            relabelingMatrices.MatrixRelabel(waist_ind, nextLinkAfterWaist) = -1;
        end

    end
end


% [~, ind] = unique(parent_tree);
% duplicate_ind = setdiff(1:length(parent_tree), ind);
% duplicate_value = parent_tree(duplicate_ind);