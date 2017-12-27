function othercars = respawn_othercars_tollplaza(othercars,road, sim,varargin)
% RESPAWN OTHERCARS outside the course 
% made by kumano
persistent first_flag respawn_flag respawn_time respawn_interval
%----------------------
if (nargin == 4)
    min_intervalTime = varargin{1};
else
    min_intervalTime = 1.8; % 1.5
end
%----------------------
if isempty(first_flag)
    first_flag   = true;
    respawn_flag = false;
end

%----- OUTSIDE the simulator--
for i = 1:othercars.n
    if othercars.car{i}.pos(1) > road.xmax
        othercars.plot(i) = 0;
    end
end
%----------------------------

%--- Set "respawn switch"------------
xpos = othercars.car{othercars.n}.pos(1);
if first_flag && (xpos > road.xmin)
   first_flag = false;
   respawn_flag = true;
   respawn_time = sim.sec;
   respawn_interval = rand() + min_intervalTime; % interval time of respawn
end
%------------------------------------

%----- Reset start position for respawn ----
if (respawn_flag)&&((sim.sec - respawn_time)> respawn_interval)
   idx_outside = find(othercars.plot==0);
   
   if ~isempty(idx_outside)
       ndata = length(idx_outside); 
       idx_tmp = my_randsample(ndata,1);
       idx_car = idx_outside(idx_tmp);
       %-------------------
       laneidx = othercars.car{idx_car}.save.lane_idx;
       othercars.car{idx_car}.pos = get_posintrack(road.track{1}, 1, 0, laneidx, 0); % respawn from 2nd lane (no offset)
       othercars.car{idx_car}.vel = othercars.car{i}.save.vel;
       othercars.plot(idx_car) = 1;       
       %------------------
       respawn_time = sim.sec;
       respawn_interval = rand() + min_intervalTime; % interval time of respawn
   end
end
%------------------------------------------

end

