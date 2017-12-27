function othercars = addcars_tollplaza_exitSide(othercars, track, nr_cars, idx_openGate,varargin)
% ADD CARS for Exit-Side Toll Plaza simulation

X_LEAD     = 5*10^3;

if (nargin == 5)
   X_INTERVAL = varargin{1};
else
   X_INTERVAL = 20*10^3;
end

if isempty(idx_openGate)
   N_GATE = track.nr_lane;
else
   N_GATE = length(idx_openGate);
end

%--------------------
for carid = 1:nr_cars
       if isempty(idx_openGate)
          laneidx= my_randsample(N_GATE , 1);
       else
          idx_tmp  = my_randsample(N_GATE, 1);
          laneidx  = idx_openGate(idx_tmp);           
       end
       tmppos = get_posintrack(track, 1, 0, laneidx, 0);
       x = X_LEAD - (carid -1)*X_INTERVAL;
       y = tmppos(2);
       carpos = [x y 0];
      segidx = 1;
      othercars = add_othercars(othercars, carpos, [10000 0], 'normal',1, laneidx, segidx); % 10000 mm/s = 36 km/h
end
%--------------------

%---- for plot othercars------
plot_list = zeros(othercars.n,1);

for i = 1:nr_cars
    plot_list(i)  = 1;
end
othercars.plot = plot_list;
%-----------------------------

end


function othercars = add_othercars(othercars, pos, vel, ctrlmode, track_idx,lane_idx,seg_idx)
if nargin == 3
    ctrlmode = 'normal';
end
othercars.n = othercars.n + 1;
othercars.car{othercars.n}.pos = pos;
othercars.car{othercars.n}.vel = vel;
othercars.car{othercars.n}.ctrlmode = ctrlmode;
othercars.car{othercars.n}.paths = [];

othercars.car{othercars.n}.save.pos = pos; 
othercars.car{othercars.n}.save.vel = vel;
othercars.car{othercars.n}.save.track_idx = track_idx;
othercars.car{othercars.n}.save.lane_idx  = lane_idx;
othercars.car{othercars.n}.save.seg_idx   = seg_idx;
end



