function [ index_neighbours ] = get_neighbours(i_current,j_current, length, width)

index_neighbours=[0 1; 0 -1; -1 0; 1 0; -1 1; 1 1; -1 -1; 1 -1];
index_neighbours=index_neighbours+[ones(8,1)*i_current ones(8,1)*j_current];
Ind=find(index_neighbours(:,1)>0 & index_neighbours(:,1)<=length & index_neighbours(:,2)>0 & index_neighbours(:,2)<=width);
index_neighbours=index_neighbours(Ind,:);

end

