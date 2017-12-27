function [idx_front2,idx_front1,idx_rear1,idx_rear2] = get_neighboursCars2(mycar, othercars)

nr_cars = othercars.n;
id = [];
xcoord = [];

for i=1:nr_cars
    if strcmp(othercars.car{i,1}.ctrlmode,'normal')
        id =[id,i];
        x = othercars.car{i,1}.pos(1);
        xcoord =[xcoord,x];
    end
end

nr_cars = size(id,2);
%--- my car position------
xcoord = [xcoord,mycar.pos(1)];
id = [id, nr_cars +1];
%--------------------------

[xtmp,itmp] = sort(xcoord,'descend');
xorder = id(itmp);
%idx_mycar = find(xtmp==mycar.pos(1));
idx_mycar = find(itmp==(nr_cars +1));

if idx_mycar == 1
    idx_front2 = 0;
    idx_front1 = 0;
    idx_rear1  = xorder(idx_mycar +1);
    idx_rear2  = xorder(idx_mycar +2);
elseif idx_mycar == 2
    idx_front2 = 0;
    idx_front1 = xorder(idx_mycar -1);
    idx_rear1  = xorder(idx_mycar +1);
    idx_rear2  = xorder(idx_mycar +2);
elseif idx_mycar == length(xorder)
    idx_front2 = xorder(idx_mycar -2);
    idx_front1 = xorder(idx_mycar -1);
    idx_rear1  = 0;
    idx_rear2  = 0;
elseif idx_mycar == (length(xorder)-1)
    idx_front2 = xorder(idx_mycar -2);
    idx_front1 = xorder(idx_mycar -1);
    idx_rear1  = xorder(idx_mycar +1);
    idx_rear2  = 0;
else
    idx_front2 = xorder(idx_mycar -2);
    idx_front1 = xorder(idx_mycar -1);
    idx_rear1  = xorder(idx_mycar +1);
    idx_rear2  = xorder(idx_mycar +2);    
end

%-- for debug-------
%fprintf('idx: %d, %d, %d, %d \n', idx_front2,idx_front1,idx_rear1,idx_rear2);
%-------------------
end


