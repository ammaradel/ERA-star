function [index_neighbours] = get_free_neighbours(i_current,j_current, M, obstacle_value)

length=size(M,1);
width=size(M,2);

index_neighbours = get_neighbours(i_current,j_current, length, width);

id_neighbours=(index_neighbours(:,2)-1)*size(M,1)+index_neighbours(:,1);

index_neighbours=index_neighbours(M(id_neighbours)~=obstacle_value,:);

end

