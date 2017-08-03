function [idx_front,idx_rear] = get_neighboursCars(mycar, othercars)

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

%--- my car position------
xcoord = [xcoord,mycar.pos(1)];
id = [id, nr_cars +1];
%--------------------------

[xtmp,itmp] = sort(xcoord,'descend');
xorder = id(itmp);
idx_mycar = find(xtmp==mycar.pos(1));

if idx_mycar==1
    idx_front = 0;
    idx_rear  = xorder(idx_mycar +1);
elseif idx_mycar==length(xorder)
    idx_front = xorder(idx_mycar -1);
    idx_rear  = 0;
else
    idx_front = xorder(idx_mycar -1);
    idx_rear  = xorder(idx_mycar +1);
end


end


