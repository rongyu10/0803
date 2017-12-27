function othercars = update_othercars_along_traj(othercars,sim)
% updating othercars position along prescribed trajectories


for i = 1:othercars.n
    
    if (isfield(othercars,'info'))&&(isfield(othercars.info{i},'type'))&&(strcmp(othercars.info{i}.type,'plaza')) % inside the "plaza"
        % UPDATING othrecars locations along the trajectories IN 'plaza'
        othercars.car{i}.pos ...
            = updata_pos_along_traj(othercars.car{i}.pos, othercars.car{i}.vel, sim.T, othercars.traj{i});         
    else
        othercars.car{i}.pos ...
            = update_pos(othercars.car{i}.pos, othercars.car{i}.vel, sim.T);         
    end
    
    othercars.car{i}.bd ...
        = get_carshape(othercars.car{i}.pos ...
        , othercars.car{i}.W, othercars.car{i}.H);
end

end

function next_pos = updata_pos_along_traj(pos,vel,T,routeData)
% get next position at a distance of 'velocity*time' along the trajectory 

  dlength = vel(1)*T;
  curr_idx = current_indexX(pos,routeData);
  
  [next_pos, ~] = nextSTEP_pos(pos, routeData, curr_idx, dlength);
  
end

function idx_x = current_indexX(pos,routeData)
% get current index on trajectory data based on X coordinate (FASTER version)

data    = routeData.route;
n_data  = size(data,1)-1;
curr_x  = pos(1);
%-----
xdata = data(1:end-1,1);
one   = ones(n_data,1);
diff_x = xdata - one*curr_x;
%-----

idx_x = find(diff_x >=0, 1); % first positive data is current index 

if isempty(idx_x) 
   idx_x = n_data;
end

end

%{
function idx_x = current_indexX(pos,routeData)

data    = routeData.route;
n_data  = size(data,1);
curr_x  = pos(1);

idx_x = 0;
for i=1:n_data-1
  idx_x = i;
  if curr_x < data(i,1)
     break
  end
end

end
%}

function [next_pos, next_idx_x] = nextSTEP_pos(pos, routeData, curr_idx, dlength)

 data    = routeData.route;
 n_data  = size(data,1);
 seg_len = routeData.seg_len;
 seg_ang = routeData.seg_ang;
 %------------------
 next_pos = zeros(1,3);
 %----------------------------------------------
 diffx   = data(curr_idx+1, 1) - pos(1);
 diffy   = data(curr_idx+1, 2) - pos(2);
 length1 = sqrt(diffx*diffx + diffy*diffy);
 if dlength < length1
    radian = seg_ang(curr_idx,1)*pi/180;
    next_idx_x = curr_idx;
    next_pos(1) = pos(1) + dlength*cos(radian);
    next_pos(2) = pos(2) + dlength*sin(radian);    
    next_pos(3) = seg_ang(curr_idx,1);            
    return
 end
 if (curr_idx ==(n_data-1))
    next_idx_x  = n_data;
    next_pos(1) = data(end,1) + dlength - length1;
    next_pos(2) = data(end,2);
    next_pos(3) = 0.0;
    return
 end
 %---------------------------------------------
 
 tmp_length = length1;
 for i = curr_idx+1:n_data-1
     next_idx_x = i;
     tmp_length = tmp_length + seg_len(i);
     if dlength < tmp_length
        break
     end
 end
 
 length2 = tmp_length - dlength;
 
 radian = seg_ang(next_idx_x,1)*pi/180;
 next_pos(1) = data(next_idx_x+1, 1) - length2 * cos(radian);
 next_pos(2) = data(next_idx_x+1, 2) - length2 * sin(radian);    
 next_pos(3) = seg_ang(next_idx_x,1);
 return
end

