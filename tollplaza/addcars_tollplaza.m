function othercars = addcars_tollplaza(othercars, track, nr_cars)
% ADD CARS IN RANDOM POSTIONS
VERBOSE = 0;



for carid = 1:nr_cars
    if carid <= 10
       laneidx= 1;
    else
       laneidx= 3;
    end
    
    segidx = mod(carid-1,track.nr_seg)+1;
    seglen = track.seg{segidx}.d;
    randoffset = seglen*rand*0.5;
    %randoffset = seglen*0.5;
    
    carpos = get_posintrack(track, segidx, randoffset, laneidx, 0);
    othercars = add_othercars(othercars, carpos, [30000 0], 'normal',1, laneidx, segidx); % 10000 mm/s = 36 km/h
end


%---- setting for extra cars
for carid = nr_cars+1:othercars.MAX_NRCAR
    tmp = mod(carid,2);
    if tmp == 0
       laneidx= 1;
    else
       laneidx= 3;
    end
    carpos = get_posintrack(track, 1, 0, laneidx, 0);
    othercars = add_othercars(othercars, carpos, [30000 0], 'normal',1, laneidx, 1); % 10000 mm/s = 36 km/h
end
othercars.n = nr_cars;


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