% ===============relabeling matrices==============
piMatrix = pi*[1; zeros(n_link-1,1)];
MatrixRelabel = zeros(n_link,n_link);
MatrixRelabel(1,:) = 1;
free_link = 0;

%identify free link
for i = 1:n_link
    if parent_tree(i) ~= i-1
        free_link = i-1;
    end
end

if free_link ~= 0
    piMatrix(free_link) = pi;
    MatrixRelabel(1,free_link) = 0;
end


for i = 2:n_link
    if i ~= free_link
       MatrixRelabel(i,end-j)
    end
    
end

% if i ~= free_link
%         MatrixRelabel(i,end-i+2) = -1;
%     else
%         for i = free_link:n_link
%             MatrixRelabel(free_link,i) = -1;
%         end 
%     end

% MatrixRelabel = inv(flip(tril(ones(n_link)))) *tril(ones(n_link));
