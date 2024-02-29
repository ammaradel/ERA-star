% Partial dijkstra: we stop when S(i_goal,j_goal)==-1

function [map_with_path,path,D,iter,cost,fail,t] = Astar_anglesCases4(map,Nb_iter_max,i_start,j_start,i_goal,j_goal)

%i_start=950;
%j_start=350;
%i_goal=350;
%j_goal=980;
%Nb_iter_max=2000;
path_color=255;
obstacle=100;
M=double(map);
lambda=10;
length=size(M,1);
width=size(M,2);
iter=0;
cost=0;
lateral_cost=1;
diagonal_cost=sqrt(2);
%figure;hold on
i_current=i_start;
j_current=j_start;
%path=[];
map_with_path=map;
tBreak=1+1/(length+width);
sub2ind1 = @(u)(u(:,2)-1)*length+u(:,1);
ind2sub1 = @(k)[rem(k-1, length)+1; (k - rem(k-1, length) - 1)/length + 1];
fail=0;
IndStart=sub2ind1([i_start,j_start]);
Iopen = [IndStart];
Iclosed=[];
%Mcheck=[];

P0_x_sup1=[2*sqrt(2),2,2*sqrt(2)-1;  2,0,0; 2*sqrt(2),sqrt(2),2*sqrt(2)-1];
P0_x_1=[2*sqrt(2),2,sqrt(2);  2,0,0; 2*sqrt(2),sqrt(2),sqrt(2)];
P0_45_y_inf_xm2=[2,2-sqrt(2),0; 2,0,0; 2*sqrt(2), 2*sqrt(2)-1,2*sqrt(2)-2];
P0_45_y_eq_xm1=[2,2-sqrt(2),0; 2,0,0; 2*sqrt(2), 2*sqrt(2)-1,sqrt(2)];
P45=[2,2-sqrt(2),0; 2,0,2-sqrt(2); 2*sqrt(2),2,2];
P_45_90_x_inf_ym2=[2*sqrt(2)-2,0,0; 2*sqrt(2)-1,0,2-sqrt(2); 2*sqrt(2),2,2];
P_45_90_x_eq_ym1=[sqrt(2),0,0; 2*sqrt(2)-1,0,2-sqrt(2); 2*sqrt(2),2,2];

P90_y_sup1=rot90(P0_x_sup1);
P90_y_1=rot90(P0_x_1);
P90_135_x_inf_ym2=rot90(P0_45_y_inf_xm2);
P90_135_x_eq_ym1=rot90(P0_45_y_eq_xm1);
P135=rot90(P45);
P_135_180_y_inf_xm2=rot90(P_45_90_x_inf_ym2);
P_135_180_y_eq_xm1=rot90(P_45_90_x_eq_ym1);

P180_x_sup1=rot90(P90_y_sup1);
P180_x_m1=rot90(P0_x_1,2);
P180_225_y_inf_xm2=rot90(P0_45_y_inf_xm2,2);
P180_225_y_eq_xm1=rot90(P0_45_y_eq_xm1,2);
P225=rot90(P45,2);
P_225_270_x_inf_ym2=rot90(P_45_90_x_inf_ym2,2);
P_225_270_x_eq_ym1=rot90(P_45_90_x_eq_ym1,2);

P270_y_sup1=rot90(P90_y_sup1,2);
P270_y_1=rot90(P90_y_1,2);
P270_315_x_inf_ym2=rot90(P90_135_x_inf_ym2,2);
P270_315_x_eq_ym1=rot90(P90_135_x_eq_ym1,2);
P315=rot90(P135,2);
P_315_360_y_inf_xm2=rot90(P_135_180_y_inf_xm2,2);
P_315_360_y_eq_xm1=rot90(P_135_180_y_eq_xm1,2);


tic

% sub2ind1 = @(u)(u(:,2)-1)*length+u(:,1);
% ind2sub1 = @(k)[rem(k-1, length)+1; (k - rem(k-1, length) - 1)/length + 1];
% 
sub2ind3 = @(u)(u(:,2)-1)*3+u(:,1);

I = [i_start,j_start];
Id=0;

%Initialize the distance to Inf, except for the boundary conditions.

D = zeros(length,width)+Inf;
Predecessors=zeros(length,width)+Inf;
D(i_start,j_start) = 0;

%Initialize the state to 0 (unexplored), 
%S = zeros(length,width);
%S(i_start,j_start) = -1;

%The first step of each iteration of the method is to pop  from stack the element  with smallest current distance .
while (D(i_goal,j_goal)==Inf & iter<Nb_iter_max & ~isempty(I)) 
    % Removed from standard Dijkstra: [tmp,j] = sort(D(sub2ind1(I))); j = j(1);
    %
    [m,ind]=min(Id);
    i = I(ind,:); I(ind,:) = []; Id(ind,:) = [];
    %We update its state  to be dead (-1).
    %S(i(1),i(2)) = -1;
    %Retrieve the list of non dead neighbors.
    x=i_goal-i(1);
    y=j_goal-i(2);
    
    if y>=1,
        
        if y<=x-2,  matP=P0_45_y_inf_xm2;
        elseif x<0,
            if y>=-x+2, matP=P90_135_x_inf_ym2;
            elseif y<=-x-2, matP=P_135_180_y_inf_xm2;
            elseif y==-x+1, matP=P90_135_x_eq_ym1;
            elseif y==-x, matP=P135;
            elseif y==-x-1, matP=P_135_180_y_eq_xm1;
            
            end;
        elseif (x>0 & y>=x+2), matP=P_45_90_x_inf_ym2;
        elseif y==x+1,  matP=P_45_90_x_eq_ym1;
        elseif x==0, matP=P90_y_sup1;
        elseif y==x, matP=P45;
        elseif y==x-1, matP=P0_45_y_eq_xm1;
        end;
        
     elseif y<=-1,
        
        if y>=x+2,  matP=P180_225_y_inf_xm2;
        
        elseif (x<0 & y<=x-2), matP=P_225_270_x_inf_ym2;
        elseif x>0,
            if y<=-x-2, matP=P270_315_x_inf_ym2;
            elseif y>=-x+2, matP=P_315_360_y_inf_xm2;
            elseif -y==x+1, matP=P270_315_x_eq_ym1;
            elseif -y==x, matP=P315;
            elseif -y==x-1, matP=P_315_360_y_eq_xm1;
            
            end;
            
         elseif y==x-1,  matP=P_225_270_x_eq_ym1;
         elseif x==0, matP=P270_y_sup1;
        elseif y==x, matP=P225;
        elseif y==x+1, matP=P180_225_y_eq_xm1;
        end;   
  
    
    elseif y==0,
        
      if x>1, matP=P0_x_sup1;
        
        elseif x<-1, matP=P180_x_sup1; 
        elseif x==1, matP=P0_x_1; 
        elseif x==-1, matP=P180_x_m1;
        elseif x==0, continue;
      end
    
    elseif (y==1 & x==0), matP=P90_y_1;
    elseif (y==-1 & x==0), matP=P270_y_1;
end
    
     J = get_free_Infinity_neighbours(i(1),i(2), map, obstacle, D);
    
%J1 = J(D(sub2ind1(J))==Inf,:);
I = [I; J];

%Update neighbor values. For each neightbor  of , perform the update, assuming the length of the edge between  and  is .
index_i=sub2ind1(i);

for j=J'
    Predecessors(j(1),j(2))=index_i;

	D(j(1),j(2)) = D(i(1),i(2))+matP(2-j(2)+i(2),j(1)-i(1)+2);
   
    Id=[Id;D(j(1),j(2))];
    
end
    
    
    
    
    
iter=iter+1;
%if ~mod(iter,5000), 
 %   iter, toc, 
  %  size(I,1) 
   %figure(1),imagesc(D), hold on, 
%end

end

if D(i_goal,j_goal)~=Inf
    %D(D==Inf)=-Inf;
    disp('***********Path reconstruction:**************')
  i_current=i_goal;
    j_current=j_goal;
    i_start;
    j_start;
    path=[i_goal,j_goal];
    cost=0;
    while (i_current~=i_start | j_current~=j_start) 
        
        index_successor=Predecessors(i_current,j_current);
        if index_successor ~= Inf
        new=ind2sub1(index_successor);
        i_new=new(1);
        j_new=new(2);
        %cost=cost+lateral_cost; 
        if ((i_current==i_new) | (j_current==j_new)), cost=cost+lateral_cost; else cost=cost+diagonal_cost; end
        i_current=i_new;
        j_current=j_new;
        % Addition to path:
        path=[i_current, j_current; path];
        % Addition to the map:
        map_with_path(i_current, j_current)=path_color;
        else deadlock=1, break, end
    end
%     save -ascii path_test path
% 	kjk
else disp('Path not found'), path=[], fail=1,
end
iter

t=toc
end
